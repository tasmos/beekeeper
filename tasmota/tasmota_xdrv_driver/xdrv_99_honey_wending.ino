/*
  xdrv_99_honey_vending_machine.ino - Coin Counter with €5 Detection
  ===================================================================
  Counts pulses from CH-926 coin acceptor and calculates monetary value.
  Notifies when €5 is reached (honey available!).
  
  Pulse mapping:
  - 10c → 1 pulse
  - 20c → 2 pulses
  - 50c → 5 pulses
  - 1€  → 10 pulses
  - 2€  → 20 pulses
*/

#ifdef USE_HONEY_WENDING_MACHINE

#define XDRV_99  99

#warning **** Honey Vending Machine Driver (€5 Detection) is included... ****

#include <ESP8266WebServer.h>  // ESP32: use <WebServer.h>

ESP8266WebServer server(80);

#ifndef HONEY_WENDING_GPIO
  #define HONEY_WENDING_GPIO  14   // GPIO14 = D5 NodeMCU
#endif

#define COIN_TIMEOUT_MS      200   // 200ms between pulses (coins pulse very fast!)
#define TARGET_AMOUNT_CENTS  500   // €5.00 = 500 cents
#define PULSE_DEBOUNCE_MS    5     // 5ms debounce between pulses
#define HONEY_BOX_COUNT      15    // Total number of honey boxes
#define HONEY_SETTINGS_INDEX 20    // Settings storage index for status
#define HONEY_TIMESTAMP_INDEX 21   // Settings storage index for timestamps
#define HONEY_PUBLISH_INTERVAL 300000  // 5 minutes in milliseconds

// Machine state
struct {
  volatile uint32_t pulse_count;       // pulses in current coin
  volatile uint32_t last_pulse_ms;     // timestamp of last pulse
  volatile uint32_t first_pulse_ms;    // timestamp of first pulse in coin
  
  uint32_t total_cents;                // total money inserted (cents)
  uint32_t last_coin_cents;            // value of last detected coin
  uint32_t coins_detected;             // total number of coins
  
  bool honey_available;                // €5 reached?
  bool initialized;
  bool box_status[HONEY_BOX_COUNT];    // true = honey available, false = empty
  uint32_t box_last_changed[HONEY_BOX_COUNT];  // Unix timestamp of last status change
  uint32_t last_publish_ms;            // Last time we published to MQTT
  bool discovery_sent;                 // Home Assistant discovery config sent?
  uint32_t device_id;                  // Unique device ID (chip ID)

} vending;


// ========== FORWARD DECLARATIONS ==========
void HoneyVending_SaveStatus(void);
void HoneyVending_LoadStatus(void);
void HoneyVending_SetStatus(uint8_t box_id, bool has_honey);
bool HoneyVending_GetStatus(uint8_t box_id);
void HoneyVending_ToggleStatus(uint8_t box_id);
void CentsToEuroString(uint32_t cents, char* buffer, size_t len);
void HoneyVending_DisplayValues(void);
void HoneyVending_PublishBoxMQTT(uint8_t box_id);
void HoneyVending_PublishAllBoxesMQTT(void);
void HoneyVending_PublishSystemMQTT(void);
void HoneyVending_PublishDiscovery(void);


// ========== MQTT FUNCTIONS ==========

// Publish Home Assistant MQTT Discovery configuration
void HoneyVending_PublishDiscovery(void) {
  if (!MqttIsConnected()) return;
  
  char topic[128];
  char device_name[64];
  char unique_id[64];
  
  // Get device name from Tasmota settings
  snprintf(device_name, sizeof(device_name), "%s", SettingsText(SET_DEVICENAME));
  
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Publishing Home Assistant Discovery config..."));
  
  // Publish discovery config for each box (binary_sensor)
  for (int i = 1; i <= HONEY_BOX_COUNT; i++) {
    snprintf(unique_id, sizeof(unique_id), "honey_%04X_box_%d", (unsigned int)vending.device_id, i);
    snprintf(topic, sizeof(topic), "homeassistant/binary_sensor/%s_box%d/config", device_name, i);
    
    Response_P(PSTR("{"
      "\"name\":\"Honey Box %d\","
      "\"unique_id\":\"%s\","
      "\"state_topic\":\"beekeeper_%04X/honey/all_boxes/box_%d\","
      "\"value_template\":\"{{ value_json.status }}\","
      "\"payload_on\":\"available\","
      "\"payload_off\":\"not_available\","
      "\"device_class\":\"occupancy\","
      "\"icon\":\"mdi:beehive\","
      "\"device\":{"
        "\"identifiers\":[\"%s_honey_%04X\"],"
        "\"name\":\"Honey Vending %04X\","
        "\"model\":\"Custom Tasmota Driver\","
        "\"manufacturer\":\"Beekeeper\""
      "}"
    "}"), i, unique_id, (unsigned int)vending.device_id, i, device_name, 
         (unsigned int)vending.device_id, (unsigned int)vending.device_id);
    
    MqttPublish(topic, true);  // retained = true
  }
  
  // Publish discovery config for summary sensor
  snprintf(topic, sizeof(topic), "homeassistant/sensor/%s_boxes_available/config", device_name);
  Response_P(PSTR("{"
    "\"name\":\"Honey Boxes Available\","
    "\"unique_id\":\"honey_%04X_boxes_available\","
    "\"state_topic\":\"beekeeper_%04X/honey/summary\","
    "\"value_template\":\"{{ value_json.available }}\","
    "\"unit_of_measurement\":\"boxes\","
    "\"icon\":\"mdi:beehive-outline\","
    "\"device\":{"
      "\"identifiers\":[\"%s_honey_%04X\"],"
      "\"name\":\"Honey Vending %04X\","
      "\"model\":\"Custom Tasmota Driver\","
      "\"manufacturer\":\"Beekeeper\""
    "}"
  "}"), (unsigned int)vending.device_id, (unsigned int)vending.device_id, 
       device_name, (unsigned int)vending.device_id, (unsigned int)vending.device_id);
  MqttPublish(topic, true);
  
  // Publish discovery config for system status
  snprintf(topic, sizeof(topic), "homeassistant/sensor/%s_vending_total/config", device_name);
  Response_P(PSTR("{"
    "\"name\":\"Honey Vending Total\","
    "\"unique_id\":\"honey_%04X_vending_total\","
    "\"state_topic\":\"beekeeper_%04X/honey/system\","
    "\"value_template\":\"{{ value_json.total }}\","
    "\"icon\":\"mdi:currency-eur\","
    "\"device\":{"
      "\"identifiers\":[\"%s_honey_%04X\"],"
      "\"name\":\"Honey Vending %04X\","
      "\"model\":\"Custom Tasmota Driver\","
      "\"manufacturer\":\"Beekeeper\""
    "}"
  "}"), (unsigned int)vending.device_id, (unsigned int)vending.device_id,
       device_name, (unsigned int)vending.device_id, (unsigned int)vending.device_id);
  MqttPublish(topic, true);
  
  vending.discovery_sent = true;
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Home Assistant Discovery config published (ID: %04X)"), 
    (unsigned int)vending.device_id);
}

// Publish individual box status to custom MQTT topic with retained
void HoneyVending_PublishBoxMQTT(uint8_t box_id) {
  if (box_id < 1 || box_id > HONEY_BOX_COUNT) return;
  if (!MqttIsConnected()) return;
  
  bool has_honey = vending.box_status[box_id - 1];
  uint32_t timestamp = vending.box_last_changed[box_id - 1];
  
  // Format timestamp as ISO 8601 string
  char time_str[32];
  if (timestamp > 0) {
    TIME_T tm;
    BreakTime(timestamp, tm);
    snprintf(time_str, sizeof(time_str), "%04d-%02d-%02dT%02d:%02d:%02dZ", 
             tm.year + 1970, tm.month, tm.day_of_month, tm.hour, tm.minute, tm.second);
  } else {
    snprintf(time_str, sizeof(time_str), "1970-01-01T00:00:00Z");
  }
  
  // Build JSON payload
  Response_P(PSTR("{"
    "\"id\":%d,"
    "\"status\":\"%s\","
    "\"timestamp\":\"%s\""
  "}"),
    box_id,
    has_honey ? "available" : "not_available",
    time_str
  );
  
  // Publish to beekeeper_<ID>/honey/all_boxes/box_N with retained=true
  char topic[64];
  snprintf(topic, sizeof(topic), "beekeeper_%04X/honey/all_boxes/box_%d", 
           (unsigned int)vending.device_id, box_id);
  
  MqttPublish(topic, true);  // retained = true
  
  AddLog(LOG_LEVEL_DEBUG, PSTR("VENDING: Published Box %d to %s"), box_id, topic);
}

// Publish all boxes status
void HoneyVending_PublishAllBoxesMQTT(void) {
  if (!MqttIsConnected()) return;
  
  AddLog(LOG_LEVEL_DEBUG, PSTR("VENDING: Publishing all boxes to MQTT..."));
  
  // Publish each box individually to retained topics
  for (int i = 1; i <= HONEY_BOX_COUNT; i++) {
    HoneyVending_PublishBoxMQTT(i);
  }
  
  // Also publish summary
  int available_count = 0;
  int empty_count = 0;
  for (int i = 0; i < HONEY_BOX_COUNT; i++) {
    if (vending.box_status[i]) {
      available_count++;
    } else {
      empty_count++;
    }
  }
  
  Response_P(PSTR("{"
    "\"total\":%d,"
    "\"available\":%d,"
    "\"empty\":%d"
  "}"),
    HONEY_BOX_COUNT, available_count, empty_count);
  
  char topic[64];
  snprintf(topic, sizeof(topic), "beekeeper_%04X/honey/summary", (unsigned int)vending.device_id);
  MqttPublish(topic, true);  // retained
  
  vending.last_publish_ms = millis();
  AddLog(LOG_LEVEL_DEBUG, PSTR("VENDING: All boxes published"));
}

// Publish system status (coin counter, total, etc.)
void HoneyVending_PublishSystemMQTT(void) {
  if (!MqttIsConnected()) return;
  
  char total_str[16];
  char target_str[16];
  CentsToEuroString(vending.total_cents, total_str, sizeof(total_str));
  CentsToEuroString(TARGET_AMOUNT_CENTS, target_str, sizeof(target_str));
  
  Response_P(PSTR("{"
    "\"total_cents\":%lu,"
    "\"total\":\"%s\","
    "\"target_cents\":%d,"
    "\"target\":\"%s\","
    "\"coins_detected\":%lu,"
    "\"honey_available\":%d"
  "}"),
    (unsigned long)vending.total_cents,
    total_str,
    TARGET_AMOUNT_CENTS,
    target_str,
    (unsigned long)vending.coins_detected,
    vending.honey_available ? 1 : 0
  );
  
  char topic[64];
  snprintf(topic, sizeof(topic), "beekeeper_%04X/honey/system", (unsigned int)vending.device_id);
  MqttPublish(topic, true);  // retained
  
  AddLog(LOG_LEVEL_DEBUG, PSTR("VENDING: System status published"));
}


// ========== PERSISTENT STORAGE FUNCTIONS ==========

// Save box status to Settings (persistent across reboots)
void HoneyVending_SaveStatus(void) {
  uint16_t packed = 0;
  for (int i = 0; i < HONEY_BOX_COUNT; i++) {
    if (vending.box_status[i]) {
      packed |= (1 << i);
    }
  }
  
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "H%04X", packed);  // Prefix with 'H' to mark as valid
  
  SettingsUpdateText(HONEY_SETTINGS_INDEX, buffer);
  
  // Save timestamps (format: "T:timestamp1,timestamp2,timestamp3,...")
  char ts_buffer[256];
  int offset = 0;
  offset += snprintf(ts_buffer + offset, sizeof(ts_buffer) - offset, "T:");
  
  for (int i = 0; i < HONEY_BOX_COUNT; i++) {
    if (i > 0) {
      offset += snprintf(ts_buffer + offset, sizeof(ts_buffer) - offset, ",");
    }
    offset += snprintf(ts_buffer + offset, sizeof(ts_buffer) - offset, "%lu", 
                      (unsigned long)vending.box_last_changed[i]);
  }
  
  SettingsUpdateText(HONEY_TIMESTAMP_INDEX, ts_buffer);
  SettingsSave(1);
  
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Status saved '%s' (0x%04X)"), buffer, packed);
  AddLog(LOG_LEVEL_DEBUG, PSTR("VENDING: Timestamps saved"));
}

// Load box status from Settings
void HoneyVending_LoadStatus(void) {
  const char* stored = SettingsText(HONEY_SETTINGS_INDEX);
  uint16_t packed = 0;
  bool valid_data = false;
  
  if (stored && strlen(stored) >= 5 && stored[0] == 'H') {
    // Valid data starts with 'H'
    packed = (uint16_t)strtol(stored + 1, nullptr, 16);
    valid_data = true;
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Loaded status '%s' (0x%04X)"), stored, packed);
  } else {
    // First run - initialize all boxes as EMPTY (safer default)
    packed = 0x0000;  // All bits clear = all boxes empty
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: No saved status (found: '%s'), initializing all boxes as EMPTY"), 
      stored ? stored : "null");
    
    // Save the initial state
    for (int i = 0; i < HONEY_BOX_COUNT; i++) {
      vending.box_status[i] = false;  // All empty
      vending.box_last_changed[i] = 0;  // No timestamp
    }
    HoneyVending_SaveStatus();
    return;
  }

  // Unpack the loaded data
  for (int i = 0; i < HONEY_BOX_COUNT; i++) {
    vending.box_status[i] = (packed & (1 << i)) != 0;
  }
  
  // Load timestamps
  const char* ts_stored = SettingsText(HONEY_TIMESTAMP_INDEX);
  if (ts_stored && strlen(ts_stored) > 2 && ts_stored[0] == 'T' && ts_stored[1] == ':') {
    // Parse timestamps
    const char* ptr = ts_stored + 2;  // Skip "T:"
    for (int i = 0; i < HONEY_BOX_COUNT; i++) {
      vending.box_last_changed[i] = strtoul(ptr, nullptr, 10);
      ptr = strchr(ptr, ',');
      if (ptr) {
        ptr++;  // Move past comma
      } else {
        break;
      }
    }
    AddLog(LOG_LEVEL_DEBUG, PSTR("VENDING: Timestamps loaded"));
  } else {
    // No timestamps saved yet, initialize to 0
    for (int i = 0; i < HONEY_BOX_COUNT; i++) {
      vending.box_last_changed[i] = 0;
    }
    AddLog(LOG_LEVEL_DEBUG, PSTR("VENDING: No timestamps found, initialized to 0"));
  }
  
  // Log loaded state
  int available_count = 0;
  for (int i = 0; i < HONEY_BOX_COUNT; i++) {
    if (vending.box_status[i]) available_count++;
  }
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Loaded %d/%d boxes as AVAILABLE"), available_count, HONEY_BOX_COUNT);
}

// Set status for a specific box (1-15)
void HoneyVending_SetStatus(uint8_t box_id, bool has_honey) {
  if (box_id < 1 || box_id > HONEY_BOX_COUNT) {
    AddLog(LOG_LEVEL_ERROR, PSTR("VENDING: Invalid box ID %d (must be 1-%d)"), box_id, HONEY_BOX_COUNT);
    return;
  }
  
  vending.box_status[box_id - 1] = has_honey;
  vending.box_last_changed[box_id - 1] = Rtc.utc_time;  // Save current UTC time
  HoneyVending_SaveStatus();  // Persist to flash  
  
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Box %d → %s (timestamp: %lu)"), 
    box_id, has_honey ? "AVAILABLE" : "EMPTY", (unsigned long)Rtc.utc_time);
  
  // Publish to MQTT immediately when box changes
  HoneyVending_PublishBoxMQTT(box_id);
  
  // Also update summary
  int available_count = 0;
  for (int i = 0; i < HONEY_BOX_COUNT; i++) {
    if (vending.box_status[i]) available_count++;
  }
  
  Response_P(PSTR("{\"total\":%d,\"available\":%d,\"empty\":%d}"),
    HONEY_BOX_COUNT, available_count, HONEY_BOX_COUNT - available_count);
  
  char topic[64];
  snprintf(topic, sizeof(topic), "beekeeper_%04X/honey/summary", (unsigned int)vending.device_id);
  MqttPublish(topic, true);
}

// Get status for a specific box (1-15)
bool HoneyVending_GetStatus(uint8_t box_id) {
  if (box_id < 1 || box_id > HONEY_BOX_COUNT) {
    return false;
  }
  return vending.box_status[box_id - 1];
}

// Get timestamp for a specific box (1-15)
uint32_t HoneyVending_GetTimestamp(uint8_t box_id) {
  if (box_id < 1 || box_id > HONEY_BOX_COUNT) {
    return 0;
  }
  return vending.box_last_changed[box_id - 1];
}

// Toggle status for a specific box (1-15)
void HoneyVending_ToggleStatus(uint8_t box_id) {
  if (box_id < 1 || box_id > HONEY_BOX_COUNT) {
    AddLog(LOG_LEVEL_ERROR, PSTR("VENDING: Invalid box ID %d (must be 1-%d)"), box_id, HONEY_BOX_COUNT);
    return;
  }
  
  bool new_status = !vending.box_status[box_id - 1];
  HoneyVending_SetStatus(box_id, new_status);
}


// ========== COMMAND FUNCTIONS ==========

void CmndVendingToggle(void) {
  if (XdrvMailbox.data_len > 0) {
    uint8_t box_id = atoi(XdrvMailbox.data);
    if (box_id >= 1 && box_id <= HONEY_BOX_COUNT) {
      HoneyVending_ToggleStatus(box_id);
      
      bool new_status = HoneyVending_GetStatus(box_id);
      uint32_t timestamp = HoneyVending_GetTimestamp(box_id);
      
      Response_P(PSTR("{\"Box\":%d,\"Status\":\"%s\",\"Timestamp\":%lu}"), 
        box_id, new_status ? "AVAILABLE" : "EMPTY", (unsigned long)timestamp);
      return;
    }
  }
  ResponseCmndError();
}

void CmndVendingSet(void) {
  if (XdrvMailbox.data_len > 0) {
    char* space_pos = strchr(XdrvMailbox.data, ' ');
    if (space_pos != nullptr) {
      *space_pos = '\0';
      uint8_t box_id = atoi(XdrvMailbox.data);
      uint8_t status = atoi(space_pos + 1);
      
      if (box_id >= 1 && box_id <= HONEY_BOX_COUNT) {
        HoneyVending_SetStatus(box_id, status != 0);
        ResponseCmndDone();
        return;
      }
    }
  }
  ResponseCmndError();
}

void CmndVendingBoxStatus(void) {
  Response_P(PSTR("{\"Boxes\":["));
  
  for (int i = 1; i <= HONEY_BOX_COUNT; i++) {
    bool has_honey = HoneyVending_GetStatus(i);
    uint32_t timestamp = HoneyVending_GetTimestamp(i);
    
    if (i > 1) ResponseAppend_P(PSTR(","));
    ResponseAppend_P(PSTR("{\"id\":%d,\"status\":\"%s\",\"timestamp\":%lu}"), 
      i, has_honey ? "AVAILABLE" : "EMPTY", (unsigned long)timestamp);
  }
  
  ResponseAppend_P(PSTR("]}"));
}

void CmndVendingStatus(void) {
  char total_str[16];
  char target_str[16];
  CentsToEuroString(vending.total_cents, total_str, sizeof(total_str));
  CentsToEuroString(TARGET_AMOUNT_CENTS, target_str, sizeof(target_str));
  
  Response_P(PSTR("{\"Total\":\"%s\",\"Target\":\"%s\",\"Coins\":%d,\"HoneyAvailable\":%d,\"DeviceID\":\"%04X\"}"),
    total_str, target_str, vending.coins_detected, vending.honey_available ? 1 : 0, 
    (unsigned int)vending.device_id);
}

void CmndVendingDisplay(void) {
  HoneyVending_DisplayValues();
  ResponseCmndDone();
}

void CmndVendingReset(void) {
  vending.total_cents = 0;
  vending.coins_detected = 0;
  vending.honey_available = false;
  vending.pulse_count = 0;
  
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: System reset - ready for new customer"));
  
  // Publish system status after reset
  HoneyVending_PublishSystemMQTT();
  
  ResponseCmndDone();
}

void CmndVendingTest(void) {
  if (XdrvMailbox.data_len > 0) {
    uint32_t test_pulses = atoi(XdrvMailbox.data);
    if (test_pulses > 0) {
      vending.pulse_count = test_pulses;
      vending.last_pulse_ms = millis() - COIN_TIMEOUT_MS - 100;
      
      AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Test mode - simulating %lu pulses"), 
        (unsigned long)test_pulses);
      ResponseCmndDone();
      return;
    }
  }
  ResponseCmndError();
}

void CmndVendingDebug(void) {
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: === Debug Info ==="));
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Device ID: %04X"), (unsigned int)vending.device_id);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Current pulse count: %lu"), (unsigned long)vending.pulse_count);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Last pulse: %lu ms ago"), 
    vending.last_pulse_ms > 0 ? (millis() - vending.last_pulse_ms) : 0);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: GPIO%d state: %d"), HONEY_WENDING_GPIO, digitalRead(HONEY_WENDING_GPIO));
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Current UTC time: %lu"), (unsigned long)Rtc.utc_time);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Last publish: %lu ms ago"), millis() - vending.last_publish_ms);
  ResponseCmndDone();
}

void CmndVendingRawSettings(void) {
  const char* stored = SettingsText(HONEY_SETTINGS_INDEX);
  const char* ts_stored = SettingsText(HONEY_TIMESTAMP_INDEX);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Raw settings value: '%s'"), stored ? stored : "null");
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Raw timestamps value: '%s'"), ts_stored ? ts_stored : "null");
  
  Response_P(PSTR("{\"RawSettings\":\"%s\",\"RawTimestamps\":\"%s\"}"), 
    stored ? stored : "null", ts_stored ? ts_stored : "null");
}

void CmndVendingClearAll(void) {
  for (int i = 0; i < HONEY_BOX_COUNT; i++) {
    vending.box_status[i] = false;
    vending.box_last_changed[i] = Rtc.utc_time;
  }
  HoneyVending_SaveStatus();
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: All boxes cleared to EMPTY"));
  
  // Publish updated status
  HoneyVending_PublishAllBoxesMQTT();
  
  ResponseCmndDone();
}

void CmndVendingFillAll(void) {
  for (int i = 0; i < HONEY_BOX_COUNT; i++) {
    vending.box_status[i] = true;
    vending.box_last_changed[i] = Rtc.utc_time;
  }
  HoneyVending_SaveStatus();
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: All boxes set to AVAILABLE"));
  
  // Publish updated status
  HoneyVending_PublishAllBoxesMQTT();
  
  ResponseCmndDone();
}

void CmndVendingPublish(void) {
  // Manual MQTT publish command
  HoneyVending_PublishAllBoxesMQTT();
  HoneyVending_PublishSystemMQTT();
  ResponseCmndDone();
}

void CmndVendingDiscovery(void) {
  // Manually trigger Home Assistant discovery
  HoneyVending_PublishDiscovery();
  ResponseCmndDone();
}

// *** Command definitions - MUST BE AFTER function declarations ***
const char kHoneyVendingCommands[] PROGMEM = 
  "Vending|"  // Prefix for all commands
  "Status|Toggle|Set|BoxStatus|Display|Reset|Test|Debug|RawSettings|ClearAll|FillAll|Publish|Discovery";

void (* const HoneyVendingCommand[])(void) PROGMEM = {
  &CmndVendingStatus, &CmndVendingToggle, &CmndVendingSet, &CmndVendingBoxStatus,
  &CmndVendingDisplay, &CmndVendingReset, &CmndVendingTest, &CmndVendingDebug,
  &CmndVendingRawSettings, &CmndVendingClearAll, &CmndVendingFillAll, &CmndVendingPublish,
  &CmndVendingDiscovery
};


// ========== DYNAMIC HTML GENERATION ==========

void HoneyVending_ShowWebButton(void) {
  WSContentSend_P(PSTR(
    "<style>"
    ".honey-grid{display:grid;grid-template-columns:repeat(5,1fr);gap:5px;margin:10px 0;}"
    ".honey-grid button{padding:8px 4px;font-size:0.9rem;min-width:50px;display:flex;flex-direction:column;align-items:center;line-height:1.2;}"
    ".honey-available{background-color:#4CAF50;color:white;}"
    ".honey-empty{background-color:#f44336;color:white;}"
    ".box-date{font-size:0.65rem;opacity:0.9;margin-top:2px;}"
    "</style>"
    "<script>"
    "function toggleBox(id){"
      "fetch('/cm?cmnd=VendingToggle '+id)"
      ".then(r=>r.json())"
      ".then(data=>{"
        "console.log('Toggle result:',data);"
        "setTimeout(()=>location.reload(),300);"
      "})"
      ".catch(e=>console.error('Toggle failed:',e));"
    "}"
    "</script>"
    "<p><b>🍯 Honey Vending Machine (ID: %04X)</b></p>"
    "<div class='honey-grid'>"
  ), (unsigned int)vending.device_id);
  
  // Generate buttons with current status and timestamp
  for (int i = 1; i <= HONEY_BOX_COUNT; i++) {
    bool has_honey = HoneyVending_GetStatus(i);
    uint32_t timestamp = HoneyVending_GetTimestamp(i);
    const char* css_class = has_honey ? "honey-available" : "honey-empty";
    const char* icon = has_honey ? "🍯 " : "";
    
    // Format date/time
    char date_str[32] = "";
    if (timestamp > 0) {
      TIME_T tm;
      BreakTime(timestamp, tm);
      snprintf(date_str, sizeof(date_str), "📅 %02d/%02d %02d:%02d", 
               tm.day_of_month, tm.month, tm.hour, tm.minute);
    } else {
      snprintf(date_str, sizeof(date_str), "📅 --/-- --:--");
    }
    
    WSContentSend_P(PSTR(
      "<button id='box%d' class='%s' onclick='toggleBox(%d)'>"
      "<span>%sBox %d</span>"
      "<span class='box-date'>%s</span>"
      "</button>"
    ), i, css_class, i, icon, i, date_str);
  }
  
  WSContentSend_P(PSTR("</div>"));
}


// ========== USER LOOP ==========

void userLoop() {
  server.handleClient();  // always handle HTTP requests
}


// ========== ISR AND COIN DETECTION ==========

// ISR - count pulses with debouncing
void IRAM_ATTR HoneyVending_PulseISR(void) {
  uint32_t now = millis();
  
  // Simple debounce - ignore pulses within 5ms
  if (vending.pulse_count > 0 && (now - vending.last_pulse_ms) < PULSE_DEBOUNCE_MS) {
    return;
  }
  
  // Track first pulse of this coin
  if (vending.pulse_count == 0) {
    vending.first_pulse_ms = now;
  }
  
  vending.pulse_count++;
  vending.last_pulse_ms = now;
}

// Convert pulse count to cents
uint32_t PulsesToCents(uint32_t pulses) {
  switch(pulses) {
    case 1:  return 10;   // 10 cents
    case 2:  return 20;   // 20 cents
    case 5:  return 50;   // 50 cents
    case 10: return 100;  // 1 euro
    case 20: return 200;  // 2 euros
    default: return 0;    // unknown
  }
}

// Format cents as euro string (e.g., 550 → "€5.50")
void CentsToEuroString(uint32_t cents, char* buffer, size_t len) {
  uint32_t euros = cents / 100;
  uint32_t cents_part = cents % 100;
  snprintf(buffer, len, "€%lu.%02lu", (unsigned long)euros, (unsigned long)cents_part);
}

// Display current and required values (for 7-segment display)
void HoneyVending_DisplayValues(void) {
  char current_str[16];
  char required_str[16];
  uint32_t remaining_cents = 0;
  
  CentsToEuroString(vending.total_cents, current_str, sizeof(current_str));
  CentsToEuroString(TARGET_AMOUNT_CENTS, required_str, sizeof(required_str));
  
  if (vending.total_cents < TARGET_AMOUNT_CENTS) {
    remaining_cents = TARGET_AMOUNT_CENTS - vending.total_cents;
  }
  
  char remaining_str[16];
  CentsToEuroString(remaining_cents, remaining_str, sizeof(remaining_str));
  
  AddLog(LOG_LEVEL_INFO, PSTR("DISPLAY: Current=%s Required=%s Remaining=%s"), 
    current_str, required_str, remaining_str);
  
  // TODO: Send to 7-segment display here
  // Example: Send current value in cents to display driver
  // Display_ShowValue(vending.total_cents);
}

// Honey available notification and auto-reset
void HoneyVending_HoneyAvailable(void) {
  char total_str[16];
  CentsToEuroString(vending.total_cents, total_str, sizeof(total_str));
  
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: ╔════════════════════════════════════╗"));
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: ║  🍯🍯 HONEY IS AVAILABLE! 🍯        ║"));
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: ║  Target €5.00 reached!             ║"));
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: ║  Total: %s                         ║"), total_str);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: ╚════════════════════════════════════╝"));
  
  // Publish honey available event to MQTT
  HoneyVending_PublishSystemMQTT();
  
  // TODO: Add actions here:
  // - Trigger relay to dispense honey
  // - Send MQTT notification
  // - Activate buzzer/LED
  // - Display "DISPENSE" on 7-segment
  
  // Auto-reset after notification
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Auto-resetting for next customer..."));
  
  vending.total_cents = 0;
  vending.coins_detected = 0;
  vending.honey_available = false;
  vending.pulse_count = 0;
  
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: System ready - waiting for coins..."));
}

// Init
void HoneyVending_Init(void) {
  memset(&vending, 0, sizeof(vending));
  
  // Get unique device ID from ESP chip ID
  #ifdef ESP8266
    vending.device_id = ESP.getChipId() & 0xFFFF;  // Use lower 16 bits
  #else  // ESP32
    uint64_t chip_id = ESP.getEfuseMac();
    vending.device_id = (uint32_t)(chip_id & 0xFFFF);  // Use lower 16 bits
  #endif
  
  int8_t pin = HONEY_WENDING_GPIO;
  
  pinMode(pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin), HoneyVending_PulseISR, FALLING);
  
  vending.initialized = true;
  
  // *** LOAD PERSISTENT STATUS FROM FLASH ***
  HoneyVending_LoadStatus();

  // Disable relay web buttons using ExecuteCommand - TOGGLE ON/OFF
  // Backlog Template {"NAME":"Honey","GPIO":[0,0,0,0,0,0,0,0,0,0,0,0,0,0],"FLAG":0,"BASE":18};Module 0;Restart 1
  
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Initialized on GPIO%d"), pin);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Device ID: %04X"), (unsigned int)vending.device_id);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Target amount: €5.00"));
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Waiting for coins..."));
}

// Check for completed coin insertion
void HoneyVending_Every100ms(void) {
  if (!vending.initialized) return;
  
  static uint32_t last_check = 0;
  uint32_t now = millis();
  
  // Check every 100ms
  if (now - last_check < 100) return;
  last_check = now;
  
  // If we have pulses and timeout has passed, process the coin
  if (vending.pulse_count > 0 && 
      (now - vending.last_pulse_ms) >= COIN_TIMEOUT_MS) {
    
    uint32_t pulses = vending.pulse_count;
    uint32_t duration_ms = vending.last_pulse_ms - vending.first_pulse_ms;
    uint32_t coin_value = PulsesToCents(pulses);
    
    // Log pulse timing info
    AddLog(LOG_LEVEL_DEBUG, PSTR("VENDING: Pulse train complete: %lu pulses in %lu ms (%.1f pulses/sec)"),
      (unsigned long)pulses, 
      (unsigned long)duration_ms,
      duration_ms > 0 ? (pulses * 1000.0 / duration_ms) : 0.0);
    
    if (coin_value > 0) {
      // Valid coin detected!
      vending.total_cents += coin_value;
      vending.last_coin_cents = coin_value;
      vending.coins_detected++;
      
      char coin_str[16];
      char total_str[16];
      CentsToEuroString(coin_value, coin_str, sizeof(coin_str));
      CentsToEuroString(vending.total_cents, total_str, sizeof(total_str));
      
      AddLog(LOG_LEVEL_INFO, PSTR("VENDING: *** COIN DETECTED: %s (pulses=%lu) ***"), 
        coin_str, (unsigned long)pulses);
      AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Total: %s"), total_str);
      
      // Display current values (for 7-segment display)
      HoneyVending_DisplayValues();
      
      // Publish coin event to MQTT
      HoneyVending_PublishSystemMQTT();
      
      // Check if we reached €5
      if (!vending.honey_available && vending.total_cents >= TARGET_AMOUNT_CENTS) {
        vending.honey_available = true;
        HoneyVending_HoneyAvailable();  // Call separate method
      }
      
    } else {
      // Unknown pulse count
      AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Unknown coin (%lu pulses) - ignoring"), 
        (unsigned long)pulses);
    }
    
    // Reset pulse counter for next coin
    vending.pulse_count = 0;
  }
}

// Periodic MQTT publish (every 5 minutes)
void HoneyVending_Every250ms(void) {
  if (!vending.initialized) return;
  if (!MqttIsConnected()) return;
  
  static uint32_t last_periodic_check = 0;
  uint32_t now = millis();
  
  // Check every 250ms for smoother timing
  if (now - last_periodic_check < 250) return;
  last_periodic_check = now;
  
  // Publish every 5 minutes
  if ((now - vending.last_publish_ms) >= HONEY_PUBLISH_INTERVAL) {
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Periodic MQTT publish (5 min timer)"));
    HoneyVending_PublishAllBoxesMQTT();
    HoneyVending_PublishSystemMQTT();
  }
}

// Handle MQTT connection/reconnection
void HoneyVending_MqttConnected(void) {
  if (!vending.initialized) return;
  
  // Publish Home Assistant discovery config first
  if (!vending.discovery_sent) {
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Sending Home Assistant Discovery..."));
    HoneyVending_PublishDiscovery();
  }
  
  // Publish all box status when MQTT connects/reconnects
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: MQTT connected, publishing initial status..."));
  HoneyVending_PublishAllBoxesMQTT();
  HoneyVending_PublishSystemMQTT();
}


// Dispatcher
bool Xdrv99(uint32_t function) {
  bool result = false;
  
  switch (function) {
    case FUNC_INIT:
      HoneyVending_Init();
      break;
      
    case FUNC_EVERY_100_MSECOND:
      HoneyVending_Every100ms();
      break;
      
    case FUNC_EVERY_250_MSECOND:
      HoneyVending_Every250ms();
      break;
      
    case FUNC_COMMAND:
      result = DecodeCommand(kHoneyVendingCommands, HoneyVendingCommand);
      break;
      
    case FUNC_WEB_ADD_MAIN_BUTTON:
      HoneyVending_ShowWebButton();
      break;
      
    case FUNC_MQTT_INIT:
      HoneyVending_MqttConnected();
      break;
  }
  
  return result;
}

#endif  // USE_HONEY_WENDING_MACHINE
