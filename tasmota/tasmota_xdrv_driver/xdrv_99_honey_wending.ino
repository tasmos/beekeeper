/*
  xdrv_99_honey_wending.ino - Coin Counter with Per-Box Pricing
  ======================================================================
  Counts pulses from CH-926 coin acceptor and calculates monetary value.
  Each box can have its own price. Select a box, insert coins, get honey!
  
  Pulse mapping:
  - 10c → 1 pulse
  - 20c → 2 pulses
  - 50c → 5 pulses
  - 1€  → 10 pulses
  - 2€  → 20 pulses

  ======================================================================
  TASMOTA CONSOLE COMMANDS (prefix: Vending)
  ======================================================================
  VendingStatus          - Show coin counter, selected box, total inserted
  VendingToggle <id>     - Toggle box 1..N between AVAILABLE / EMPTY
  VendingSet <id> <0|1>  - Set box status directly (0=EMPTY, 1=AVAILABLE)
  VendingBoxStatus       - List all boxes with status, price, timestamp
  VendingDisplay         - Print current/required/remaining amounts to log
  VendingReset           - Reset coin counter and deselect box (new customer)
  VendingTest <pulses>   - Simulate a coin pulse train for testing
  VendingDebug           - Print extended debug info to log
  VendingRawSettings     - Dump raw settings strings from flash
  VendingClearAll        - Set all boxes to EMPTY
  VendingFillAll         - Set all boxes to AVAILABLE
  VendingPublish         - Force publish all boxes + system to MQTT
  VendingDiscovery       - Re-send Home Assistant MQTT discovery config
  VendingBoxCount <n>    - Set number of active boxes (1–30)
  VendingSetPrice <id> <cents>  - Set price for box (e.g. "VendingSetPrice 3 750" = €7.50)
  VendingSelectBox <id>  - Select box for vending (starts coin counter); 0 = deselect
  VendingUnlock <id>     - Manually unlock a box solenoid for UNLOCK_DURATION_MS
  VendingLock <id>       - Manually lock a box solenoid
  VendingLockAll         - Lock all box solenoids immediately
  VendingMotor COIN_ACCEPT  - Rotate coin gate 45° LEFT  (accept coin to collection)
  VendingMotor COIN_REJECT  - Rotate coin gate 45° RIGHT (return coin to customer)
  VendingMotor RESET        - Return coin gate to home position
  VendingLCDWrite <row> <text>  - Write text to LCD row (0=top, 1=bottom)
  VendingLCDClear           - Clear the LCD display
  VendingButtons            - Read MCP23017 and log any pressed buttons
  ======================================================================

  I2C BUS (GPIO21=SDA, GPIO22=SCL) — shared devices:
  ├── LCD 1602  (default I2C address: 0x27 or 0x3F via PCF8574 backpack)
  └── MCP23017  (default I2C address: 0x20, all address pins A0-A2 to GND)
      ├── GPA0..GPA7 = buttons 1..8   (pins pulled HIGH internally, active LOW)
      └── GPB0..GPB7 = buttons 9..16  (pins pulled HIGH internally, active LOW)
*/

#ifdef USE_HONEY_WENDING_MACHINE

#define XDRV_99  99

#warning **** Honey Vending Machine Driver (Per-Box Pricing) is included... ****

#include <WebServer.h>
#include <Wire.h>               // I2C bus (shared: LCD + MCP23017)
#include <LiquidCrystal_I2C.h> // LCD 1602 via PCF8574 I2C backpack

WebServer server(80);

#define HONEY_WENDING_GPIO  32   // GPIO32 = input-only pin (coin acceptor pulse)

// 74HC595 Shift Register Pins (for solenoid lock control)
#define SHIFT_REG_DATA_PIN  23   // GPIO23 = MOSI (SER/DS pin on 74HC595)
#define SHIFT_REG_CLOCK_PIN 18   // GPIO18 = SCK  (SRCLK pin on 74HC595)
#define SHIFT_REG_LATCH_PIN  4   // GPIO4  = SS   (RCLK/Latch pin on 74HC595)

// Shift Register Configuration
#define NUM_SHIFT_REGISTERS  4   // Number of cascaded 74HC595 chips (4 = 32 outputs for 30 boxes)
#define UNLOCK_DURATION_MS   60000 // 60 seconds (1 minute) unlock time
#define SOLENOID_ACTIVE_HIGH true // true = HIGH unlocks, false = LOW unlocks

#define COIN_TIMEOUT_MS      200   // 200ms between pulses (coins pulse very fast!)
#define DEFAULT_PRICE_CENTS  500   // Default €5.00 = 500 cents per box
#define PULSE_DEBOUNCE_MS    5     // 5ms debounce between pulses
#define MAX_HONEY_BOX_COUNT  30    // Maximum number of honey boxes (compile-time limit)
#define DEFAULT_HONEY_BOX_COUNT 15 // Default number of boxes
#define HONEY_SETTINGS_INDEX 20    // Settings storage index for box status
#define HONEY_TIMESTAMP_INDEX 21   // Settings storage index for timestamps
#define HONEY_BOX_COUNT_INDEX 22   // Settings storage index for box count
#define HONEY_PRICE_INDEX 23       // Settings storage index for box prices
#define HONEY_PUBLISH_INTERVAL 300000  // 5 minutes in milliseconds

// ========== DRV8825 STEPPER MOTOR (Coin Gate) ==========
#define MOTOR_STEP_PIN    25   // GPIO25 = DRV8825 STEP
#define MOTOR_DIR_PIN     26   // GPIO26 = DRV8825 DIR
#define MOTOR_ENABLE_PIN  27   // GPIO27 = DRV8825 ENABLE (active LOW)

#define MOTOR_STEPS_PER_REV  200   // 1.8° per step (full-step mode)
#define MOTOR_STEP_DELAY_US  2000  // 2ms between steps (slow = more torque)
// 45° = 200 steps/360° * 45° = 25 steps
#define MOTOR_45DEG_STEPS    ((MOTOR_STEPS_PER_REV * 45) / 360)

typedef enum {
  COIN_ACCEPT = 0,   // Rotate 45° LEFT  (accept coin, guide to collection)
  COIN_REJECT = 1,   // Rotate 45° RIGHT (reject coin, return to customer)
  MOTOR_RESET = 2    // Return to home position
} MotorAction_t;

// Track position offset from home (signed, in steps; + = right, - = left)
static int16_t motor_position_offset = 0;

// ========== LCD 1602 + MCP23017 (I2C, shared bus) ==========
#define I2C_SDA_PIN       21     // GPIO21 = SDA (shared by LCD and MCP23017)
#define I2C_SCL_PIN       22     // GPIO22 = SCL (shared by LCD and MCP23017)

#⚠️ If LCD stays blank after flashing, change LCD_I2C_ADDR from 0x27 to 0x3F — both are common for PCF8574 backpacks.
#define LCD_I2C_ADDR      0x27   // PCF8574 backpack address (try 0x3F if 0x27 fails)
#define LCD_COLS          16     // LCD 1602 = 16 columns
#define LCD_ROWS          2      // LCD 1602 = 2 rows

#define MCP23017_I2C_ADDR 0x20   // MCP23017 address (A0=A1=A2=GND → 0x20)
// MCP23017 register map
#define MCP23017_IODIRA   0x00   // Direction register GPA  (1=input, 0=output)
#define MCP23017_IODIRB   0x01   // Direction register GPB  (1=input, 0=output)
#define MCP23017_GPPUA    0x0C   // Pull-up enable register GPA (1=pull-up on)
#define MCP23017_GPPUB    0x0D   // Pull-up enable register GPB (1=pull-up on)
#define MCP23017_GPIOA    0x12   // GPIO read register GPA  (buttons 1–8)
#define MCP23017_GPIOB    0x13   // GPIO read register GPB  (buttons 9–16)
// Buttons are ACTIVE LOW: pressed = bit 0, released = bit 1 (pull-ups enabled)

// LCD object — constructed with (I2C address, columns, rows)
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

// Machine state
struct {
  volatile uint32_t pulse_count;       // pulses in current coin
  volatile uint32_t last_pulse_ms;     // timestamp of last pulse
  volatile uint32_t first_pulse_ms;    // timestamp of first pulse in coin
  
  uint32_t total_cents;                // total money inserted (cents)
  uint32_t last_coin_cents;            // value of last detected coin
  uint32_t coins_detected;             // total number of coins
  
  bool honey_available;                // Target price reached?
  bool initialized;
  uint8_t box_count;                   // Current number of active boxes (configurable)
  uint8_t selected_box_id;             // Currently selected box for vending (0 = none)
  bool box_status[MAX_HONEY_BOX_COUNT];    // true = honey available, false = empty
  uint32_t box_price[MAX_HONEY_BOX_COUNT]; // price in cents for each box
  uint32_t box_last_changed[MAX_HONEY_BOX_COUNT];  // Unix timestamp of last status change
  uint32_t last_publish_ms;            // Last time we published to MQTT
  bool discovery_sent;                 // Home Assistant discovery config sent?
  uint32_t device_id;                  // Unique device ID (chip ID)
  
  // Shift register state for solenoid locks
  uint32_t shift_reg_state[NUM_SHIFT_REGISTERS]; // Current state of shift register outputs
  uint32_t unlock_start_ms;            // Timestamp when unlock started
  uint8_t unlocked_box_id;             // Which box is currently unlocked (0 = none)

  // LCD + MCP23017 state
  bool lcd_initialized;                // true once LCD init succeeded
  bool mcp_initialized;                // true once MCP23017 init succeeded
  uint16_t last_button_state;          // previous GPA+GPB reading (for edge detection)

} vending;


// ========== FORWARD DECLARATIONS ==========
void HoneyVending_SaveStatus(void);
void HoneyVending_LoadStatus(void);
void HoneyVending_SaveBoxCount(void);
void HoneyVending_LoadBoxCount(void);
void HoneyVending_SavePrices(void);
void HoneyVending_LoadPrices(void);
void HoneyVending_SetBoxCount(uint8_t count);
void HoneyVending_SetStatus(uint8_t box_id, bool has_honey);
void HoneyVending_SetPrice(uint8_t box_id, uint32_t price_cents);
uint32_t HoneyVending_GetPrice(uint8_t box_id);
bool HoneyVending_GetStatus(uint8_t box_id);
void HoneyVending_ToggleStatus(uint8_t box_id);
void HoneyVending_SelectBox(uint8_t box_id);
void CentsToEuroString(uint32_t cents, char* buffer, size_t len);
void HoneyVending_DisplayValues(void);
void HoneyVending_PublishBoxMQTT(uint8_t box_id);
void HoneyVending_PublishAllBoxesMQTT(void);
void HoneyVending_PublishSystemMQTT(void);
void HoneyVending_PublishDiscovery(void);

// Shift Register / Lock Control Functions
void ShiftReg_Init(void);
void ShiftReg_Write(uint32_t* data, uint8_t num_registers);
void ShiftReg_SetBit(uint8_t bit_position, bool state);
void UnlockBoxKey(uint8_t box_id);
void LockBoxKey(uint8_t box_id);
void LockAllBoxes(void);
void CheckUnlockTimeout(void);

// Motor / Coin Gate Functions
void Motor_Init(void);
void Motor_DoAction(MotorAction_t action);

// LCD Functions
void LCD_Init(void);
void LCD_WriteText(uint8_t row, uint8_t col, const char* text);
void LCD_Clear(void);

// MCP23017 / Button Functions
void MCP23017_Init(void);
uint16_t MCP23017_ReadButtons(void);
void HoneyVending_PrintPressedButtons(void);



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
  for (int i = 1; i <= vending.box_count; i++) {
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
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Home Assistant Discovery config published (ID: %04X, Boxes: %d)"), 
    (unsigned int)vending.device_id, vending.box_count);
}

// Publish individual box status to custom MQTT topic with retained
void HoneyVending_PublishBoxMQTT(uint8_t box_id) {
  if (box_id < 1 || box_id > vending.box_count) return;
  if (!MqttIsConnected()) return;
  
  bool has_honey = vending.box_status[box_id - 1];
  uint32_t timestamp = vending.box_last_changed[box_id - 1];
  uint32_t price_cents = vending.box_price[box_id - 1];
  
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
  
  char price_str[16];
  CentsToEuroString(price_cents, price_str, sizeof(price_str));
  
  // Build JSON payload
  Response_P(PSTR("{"
    "\"id\":%d,"
    "\"status\":\"%s\","
    "\"price_cents\":%lu,"
    "\"price\":\"%s\","
    "\"timestamp\":\"%s\""
  "}"),
    box_id,
    has_honey ? "available" : "not_available",
    (unsigned long)price_cents,
    price_str,
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
  for (int i = 1; i <= vending.box_count; i++) {
    HoneyVending_PublishBoxMQTT(i);
  }
  
  // Also publish summary
  int available_count = 0;
  int empty_count = 0;
  for (int i = 0; i < vending.box_count; i++) {
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
    vending.box_count, available_count, empty_count);
  
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
  
  uint32_t target_cents = 0;
  if (vending.selected_box_id > 0 && vending.selected_box_id <= vending.box_count) {
    target_cents = vending.box_price[vending.selected_box_id - 1];
  }
  CentsToEuroString(target_cents, target_str, sizeof(target_str));
  
  Response_P(PSTR("{"
    "\"total_cents\":%lu,"
    "\"total\":\"%s\","
    "\"target_cents\":%lu,"
    "\"target\":\"%s\","
    "\"selected_box\":%d,"
    "\"coins_detected\":%lu,"
    "\"honey_available\":%d,"
    "\"box_count\":%d"
  "}"),
    (unsigned long)vending.total_cents,
    total_str,
    (unsigned long)target_cents,
    target_str,
    vending.selected_box_id,
    (unsigned long)vending.coins_detected,
    vending.honey_available ? 1 : 0,
    vending.box_count
  );
  
  char topic[64];
  snprintf(topic, sizeof(topic), "beekeeper_%04X/honey/system", (unsigned int)vending.device_id);
  MqttPublish(topic, true);  // retained
  
  AddLog(LOG_LEVEL_DEBUG, PSTR("VENDING: System status published"));
}


// ========== PERSISTENT STORAGE FUNCTIONS ==========

// Save box count to Settings
void HoneyVending_SaveBoxCount(void) {
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "C%02d", vending.box_count);
  
  SettingsUpdateText(HONEY_BOX_COUNT_INDEX, buffer);
  SettingsSave(1);
  
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Box count saved: %d"), vending.box_count);
}

// Load box count from Settings
void HoneyVending_LoadBoxCount(void) {
  const char* stored = SettingsText(HONEY_BOX_COUNT_INDEX);
  
  if (stored && strlen(stored) >= 3 && stored[0] == 'C') {
    uint8_t count = (uint8_t)atoi(stored + 1);
    if (count >= 1 && count <= MAX_HONEY_BOX_COUNT) {
      vending.box_count = count;
      AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Loaded box count: %d"), vending.box_count);
      return;
    }
  }
  
  // Default
  vending.box_count = DEFAULT_HONEY_BOX_COUNT;
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: No saved box count, using default: %d"), vending.box_count);
  HoneyVending_SaveBoxCount();
}

// Save box prices to Settings
void HoneyVending_SavePrices(void) {
  // Save prices (format: "P:price1,price2,price3,...")
  char price_buffer[512];
  int offset = 0;
  offset += snprintf(price_buffer + offset, sizeof(price_buffer) - offset, "P:");
  
  for (int i = 0; i < vending.box_count; i++) {
    if (i > 0) {
      offset += snprintf(price_buffer + offset, sizeof(price_buffer) - offset, ",");
    }
    offset += snprintf(price_buffer + offset, sizeof(price_buffer) - offset, "%lu", 
                      (unsigned long)vending.box_price[i]);
  }
  
  SettingsUpdateText(HONEY_PRICE_INDEX, price_buffer);
  SettingsSave(1);
  
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Prices saved"));
  AddLog(LOG_LEVEL_DEBUG, PSTR("VENDING: Price data: %s"), price_buffer);
}

// Load box prices from Settings
void HoneyVending_LoadPrices(void) {
  const char* price_stored = SettingsText(HONEY_PRICE_INDEX);
  
  if (price_stored && strlen(price_stored) > 2 && price_stored[0] == 'P' && price_stored[1] == ':') {
    // Parse prices
    const char* ptr = price_stored + 2;  // Skip "P:"
    for (int i = 0; i < vending.box_count; i++) {
      uint32_t price = strtoul(ptr, nullptr, 10);
      vending.box_price[i] = (price > 0) ? price : DEFAULT_PRICE_CENTS;
      
      ptr = strchr(ptr, ',');
      if (ptr) {
        ptr++;  // Move past comma
      } else {
        // No more prices, use default for remaining boxes
        for (int j = i + 1; j < vending.box_count; j++) {
          vending.box_price[j] = DEFAULT_PRICE_CENTS;
        }
        break;
      }
    }
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Prices loaded from memory"));
  } else {
    // No prices saved yet, initialize all to default
    for (int i = 0; i < vending.box_count; i++) {
      vending.box_price[i] = DEFAULT_PRICE_CENTS;
    }
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: No prices found, initialized to default €5.00"));
    HoneyVending_SavePrices();
  }
}

// Set box count (1-30)
void HoneyVending_SetBoxCount(uint8_t count) {
  if (count < 1 || count > MAX_HONEY_BOX_COUNT) {
    AddLog(LOG_LEVEL_ERROR, PSTR("VENDING: Invalid box count %d (must be 1-%d)"), 
           count, MAX_HONEY_BOX_COUNT);
    return;
  }
  
  uint8_t old_count = vending.box_count;
  vending.box_count = count;
  
  // If increasing count, initialize new boxes as EMPTY with default price
  if (count > old_count) {
    for (int i = old_count; i < count; i++) {
      vending.box_status[i] = false;
      vending.box_last_changed[i] = Rtc.utc_time;
      vending.box_price[i] = DEFAULT_PRICE_CENTS;
    }
  }
  
  HoneyVending_SaveBoxCount();
  HoneyVending_SaveStatus();
  HoneyVending_SavePrices();
  
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Box count changed from %d to %d"), old_count, count);
  
  // Republish everything to MQTT
  vending.discovery_sent = false;
  if (MqttIsConnected()) {
    HoneyVending_CleanupOldBoxes(old_count, count); 
    HoneyVending_PublishDiscovery();
    HoneyVending_PublishAllBoxesMQTT();
    HoneyVending_PublishSystemMQTT();
  }
}

void HoneyVending_CleanupOldBoxes(uint8_t old_count, uint8_t new_count) {
  if (new_count >= old_count) return;
  if (!MqttIsConnected()) return;
  
  char topic[128];
  char device_name[64];
  snprintf(device_name, sizeof(device_name), "%s", SettingsText(SET_DEVICENAME));
  
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Cleaning up boxes %d-%d from HA"), 
         new_count + 1, old_count);
  
  for (int i = new_count + 1; i <= old_count; i++) {
    snprintf(topic, sizeof(topic), 
             "homeassistant/binary_sensor/%s_box%d/config", device_name, i);
    MqttPublish(topic, true);
  }
}

// Save box status to Settings
void HoneyVending_SaveStatus(void) {
  uint32_t packed = 0;
  for (int i = 0; i < vending.box_count; i++) {
    if (vending.box_status[i]) {
      packed |= (1 << i);
    }
  }
  
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "H%08lX", (unsigned long)packed);
  
  SettingsUpdateText(HONEY_SETTINGS_INDEX, buffer);
  
  // Save timestamps
  char ts_buffer[512];
  int offset = 0;
  offset += snprintf(ts_buffer + offset, sizeof(ts_buffer) - offset, "T:");
  
  for (int i = 0; i < vending.box_count; i++) {
    if (i > 0) {
      offset += snprintf(ts_buffer + offset, sizeof(ts_buffer) - offset, ",");
    }
    offset += snprintf(ts_buffer + offset, sizeof(ts_buffer) - offset, "%lu", 
                      (unsigned long)vending.box_last_changed[i]);
  }
  
  SettingsUpdateText(HONEY_TIMESTAMP_INDEX, ts_buffer);
  SettingsSave(1);
  
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Status saved '%s' (0x%08lX)"), buffer, (unsigned long)packed);
}

// Load box status from Settings
void HoneyVending_LoadStatus(void) {
  const char* stored = SettingsText(HONEY_SETTINGS_INDEX);
  uint32_t packed = 0;
  bool valid_data = false;
  
  if (stored && strlen(stored) >= 5 && stored[0] == 'H') {
    packed = (uint32_t)strtoul(stored + 1, nullptr, 16);
    valid_data = true;
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Loaded status '%s' (0x%08lX)"), stored, (unsigned long)packed);
  } else {
    packed = 0x00000000;
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: No saved status, initializing all boxes as EMPTY"));
    
    for (int i = 0; i < vending.box_count; i++) {
      vending.box_status[i] = false;
      vending.box_last_changed[i] = 0;
    }
    HoneyVending_SaveStatus();
    return;
  }

  for (int i = 0; i < vending.box_count; i++) {
    vending.box_status[i] = (packed & (1 << i)) != 0;
  }
  
  // Load timestamps
  const char* ts_stored = SettingsText(HONEY_TIMESTAMP_INDEX);
  if (ts_stored && strlen(ts_stored) > 2 && ts_stored[0] == 'T' && ts_stored[1] == ':') {
    const char* ptr = ts_stored + 2;
    for (int i = 0; i < vending.box_count; i++) {
      vending.box_last_changed[i] = strtoul(ptr, nullptr, 10);
      ptr = strchr(ptr, ',');
      if (ptr) {
        ptr++;
      } else {
        break;
      }
    }
  } else {
    for (int i = 0; i < vending.box_count; i++) {
      vending.box_last_changed[i] = 0;
    }
  }
  
  int available_count = 0;
  for (int i = 0; i < vending.box_count; i++) {
    if (vending.box_status[i]) available_count++;
  }
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Loaded %d/%d boxes as AVAILABLE"), 
         available_count, vending.box_count);
}

// Set status for a specific box (1-N)
void HoneyVending_SetStatus(uint8_t box_id, bool has_honey) {
  if (box_id < 1 || box_id > vending.box_count) {
    AddLog(LOG_LEVEL_ERROR, PSTR("VENDING: Invalid box ID %d (must be 1-%d)"), 
           box_id, vending.box_count);
    return;
  }
  
  vending.box_status[box_id - 1] = has_honey;
  vending.box_last_changed[box_id - 1] = Rtc.utc_time;
  HoneyVending_SaveStatus();
  
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Box %d → %s (timestamp: %lu)"), 
    box_id, has_honey ? "AVAILABLE" : "EMPTY", (unsigned long)Rtc.utc_time);
  
  HoneyVending_PublishBoxMQTT(box_id);
  
  int available_count = 0;
  for (int i = 0; i < vending.box_count; i++) {
    if (vending.box_status[i]) available_count++;
  }
  
  Response_P(PSTR("{\"total\":%d,\"available\":%d,\"empty\":%d}"),
    vending.box_count, available_count, vending.box_count - available_count);
  
  char topic[64];
  snprintf(topic, sizeof(topic), "beekeeper_%04X/honey/summary", (unsigned int)vending.device_id);
  MqttPublish(topic, true);
}

// Set price for a specific box (1-N)
void HoneyVending_SetPrice(uint8_t box_id, uint32_t price_cents) {
  if (box_id < 1 || box_id > vending.box_count) {
    AddLog(LOG_LEVEL_ERROR, PSTR("VENDING: Invalid box ID %d (must be 1-%d)"), 
           box_id, vending.box_count);
    return;
  }
  
  if (price_cents < 10 || price_cents > 10000) {
    AddLog(LOG_LEVEL_ERROR, PSTR("VENDING: Invalid price %lu cents (must be 10-10000)"), 
           (unsigned long)price_cents);
    return;
  }
  
  char old_price_str[16];
  char new_price_str[16];
  CentsToEuroString(vending.box_price[box_id - 1], old_price_str, sizeof(old_price_str));
  CentsToEuroString(price_cents, new_price_str, sizeof(new_price_str));
  
  vending.box_price[box_id - 1] = price_cents;
  HoneyVending_SavePrices();
  
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Box %d price: %s → %s"), 
    box_id, old_price_str, new_price_str);
  
  HoneyVending_PublishBoxMQTT(box_id);
}

// Get price for a specific box (1-N)
uint32_t HoneyVending_GetPrice(uint8_t box_id) {
  if (box_id < 1 || box_id > vending.box_count) {
    return DEFAULT_PRICE_CENTS;
  }
  return vending.box_price[box_id - 1];
}

// Get status for a specific box (1-N)
bool HoneyVending_GetStatus(uint8_t box_id) {
  if (box_id < 1 || box_id > vending.box_count) {
    return false;
  }
  return vending.box_status[box_id - 1];
}

// Get timestamp for a specific box (1-N)
uint32_t HoneyVending_GetTimestamp(uint8_t box_id) {
  if (box_id < 1 || box_id > vending.box_count) {
    return 0;
  }
  return vending.box_last_changed[box_id - 1];
}

// Toggle status for a specific box (1-N) - triggers price modal if going from EMPTY to AVAILABLE
void HoneyVending_ToggleStatus(uint8_t box_id) {
  if (box_id < 1 || box_id > vending.box_count) {
    AddLog(LOG_LEVEL_ERROR, PSTR("VENDING: Invalid box ID %d (must be 1-%d)"), 
           box_id, vending.box_count);
    return;
  }
  
  bool new_status = !vending.box_status[box_id - 1];
  HoneyVending_SetStatus(box_id, new_status);
}

// Select a box for vending operation
void HoneyVending_SelectBox(uint8_t box_id) {
  if (box_id < 1 || box_id > vending.box_count) {
    AddLog(LOG_LEVEL_ERROR, PSTR("VENDING: Invalid box ID %d (must be 1-%d)"), 
           box_id, vending.box_count);
    return;
  }
  
  if (!vending.box_status[box_id - 1]) {
    AddLog(LOG_LEVEL_ERROR, PSTR("VENDING: Box %d is EMPTY - cannot select for vending"), box_id);
    return;
  }
  
  vending.selected_box_id = box_id;
  uint32_t target_price = vending.box_price[box_id - 1];
  
  char price_str[16];
  CentsToEuroString(target_price, price_str, sizeof(price_str));
  
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Selected Box %d (Price: %s)"), box_id, price_str);
  
  // Reset coin counter for new transaction
  vending.total_cents = 0;
  vending.coins_detected = 0;
  vending.honey_available = false;
  
  HoneyVending_PublishSystemMQTT();
  HoneyVending_DisplayValues();
}


// ========== COMMAND FUNCTIONS ==========

void CmndVendingBoxCount(void) {
  if (XdrvMailbox.data_len > 0) {
    uint8_t new_count = atoi(XdrvMailbox.data);
    if (new_count >= 1 && new_count <= MAX_HONEY_BOX_COUNT) {
      HoneyVending_SetBoxCount(new_count);
      Response_P(PSTR("{\"BoxCount\":%d,\"Max\":%d}"), vending.box_count, MAX_HONEY_BOX_COUNT);
      return;
    } else {
      ResponseCmndError();
      return;
    }
  }
  
  Response_P(PSTR("{\"BoxCount\":%d,\"Max\":%d}"), vending.box_count, MAX_HONEY_BOX_COUNT);
}

void CmndVendingToggle(void) {
  if (XdrvMailbox.data_len > 0) {
    uint8_t box_id = atoi(XdrvMailbox.data);
    if (box_id >= 1 && box_id <= vending.box_count) {
      HoneyVending_ToggleStatus(box_id);
      
      bool new_status = HoneyVending_GetStatus(box_id);
      uint32_t timestamp = HoneyVending_GetTimestamp(box_id);
      uint32_t price = HoneyVending_GetPrice(box_id);
      
      char price_str[16];
      CentsToEuroString(price, price_str, sizeof(price_str));
      
      Response_P(PSTR("{\"Box\":%d,\"Status\":\"%s\",\"Price\":\"%s\",\"PriceCents\":%lu,\"Timestamp\":%lu}"), 
        box_id, new_status ? "AVAILABLE" : "EMPTY", price_str, (unsigned long)price, (unsigned long)timestamp);
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
      
      if (box_id >= 1 && box_id <= vending.box_count) {
        HoneyVending_SetStatus(box_id, status != 0);
        ResponseCmndDone();
        return;
      }
    }
  }
  ResponseCmndError();
}

void CmndVendingSetPrice(void) {
  if (XdrvMailbox.data_len > 0) {
    char* space_pos = strchr(XdrvMailbox.data, ' ');
    if (space_pos != nullptr) {
      *space_pos = '\0';
      uint8_t box_id = atoi(XdrvMailbox.data);
      uint32_t price_cents = atoi(space_pos + 1);
      
      if (box_id >= 1 && box_id <= vending.box_count) {
        HoneyVending_SetPrice(box_id, price_cents);
        
        char price_str[16];
        CentsToEuroString(price_cents, price_str, sizeof(price_str));
        
        Response_P(PSTR("{\"Box\":%d,\"Price\":\"%s\",\"PriceCents\":%lu}"), 
          box_id, price_str, (unsigned long)price_cents);
        return;
      }
    }
  }
  ResponseCmndError();
}

void CmndVendingSelectBox(void) {
  if (XdrvMailbox.data_len > 0) {
    uint8_t box_id = atoi(XdrvMailbox.data);
    if (box_id >= 1 && box_id <= vending.box_count) {
      if (vending.box_status[box_id - 1]) {
        HoneyVending_SelectBox(box_id);
        
        uint32_t price = HoneyVending_GetPrice(box_id);
        char price_str[16];
        CentsToEuroString(price, price_str, sizeof(price_str));
        
        Response_P(PSTR("{\"SelectedBox\":%d,\"Price\":\"%s\",\"PriceCents\":%lu}"), 
          box_id, price_str, (unsigned long)price);
        return;
      } else {
        Response_P(PSTR("{\"Error\":\"Box %d is EMPTY\"}"), box_id);
        return;
      }
    }
  }
  
  // Deselect (box_id = 0)
  vending.selected_box_id = 0;
  vending.total_cents = 0;
  vending.coins_detected = 0;
  vending.honey_available = false;
  
  Response_P(PSTR("{\"SelectedBox\":0,\"Status\":\"Deselected\"}"));
}

void CmndVendingBoxStatus(void) {
  Response_P(PSTR("{\"Boxes\":["));
  
  for (int i = 1; i <= vending.box_count; i++) {
    bool has_honey = HoneyVending_GetStatus(i);
    uint32_t timestamp = HoneyVending_GetTimestamp(i);
    uint32_t price = HoneyVending_GetPrice(i);
    
    char price_str[16];
    CentsToEuroString(price, price_str, sizeof(price_str));
    
    if (i > 1) ResponseAppend_P(PSTR(","));
    ResponseAppend_P(PSTR("{\"id\":%d,\"status\":\"%s\",\"price\":\"%s\",\"price_cents\":%lu,\"timestamp\":%lu}"), 
      i, has_honey ? "AVAILABLE" : "EMPTY", price_str, (unsigned long)price, (unsigned long)timestamp);
  }
  
  ResponseAppend_P(PSTR("]}"));
}

void CmndVendingStatus(void) {
  char total_str[16];
  char target_str[16];
  CentsToEuroString(vending.total_cents, total_str, sizeof(total_str));
  
  uint32_t target_cents = 0;
  if (vending.selected_box_id > 0) {
    target_cents = vending.box_price[vending.selected_box_id - 1];
  }
  CentsToEuroString(target_cents, target_str, sizeof(target_str));
  
  Response_P(PSTR("{\"Total\":\"%s\",\"Target\":\"%s\",\"SelectedBox\":%d,\"Coins\":%d,\"HoneyAvailable\":%d,\"BoxCount\":%d,\"DeviceID\":\"%04X\"}"),
    total_str, target_str, vending.selected_box_id, vending.coins_detected, 
    vending.honey_available ? 1 : 0, vending.box_count, (unsigned int)vending.device_id);
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
  vending.selected_box_id = 0;
  
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: System reset - ready for new customer"));
  
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
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Box count: %d (max: %d)"), vending.box_count, MAX_HONEY_BOX_COUNT);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Selected box: %d"), vending.selected_box_id);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Current pulse count: %lu"), (unsigned long)vending.pulse_count);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Last pulse: %lu ms ago"), vending.last_pulse_ms > 0 ? (millis() - vending.last_pulse_ms) : 0);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: GPIO%d state: %d"), HONEY_WENDING_GPIO, digitalRead(HONEY_WENDING_GPIO));
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Current UTC time: %lu"), (unsigned long)Rtc.utc_time);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: LCD init: %d  MCP23017 init: %d"), vending.lcd_initialized, vending.mcp_initialized);
  ResponseCmndDone();
}

void CmndVendingRawSettings(void) {
  const char* stored = SettingsText(HONEY_SETTINGS_INDEX);
  const char* ts_stored = SettingsText(HONEY_TIMESTAMP_INDEX);
  const char* count_stored = SettingsText(HONEY_BOX_COUNT_INDEX);
  const char* price_stored = SettingsText(HONEY_PRICE_INDEX);
  
  Response_P(PSTR("{\"RawSettings\":\"%s\",\"RawTimestamps\":\"%s\",\"RawBoxCount\":\"%s\",\"RawPrices\":\"%s\"}"), 
    stored ? stored : "null", 
    ts_stored ? ts_stored : "null", 
    count_stored ? count_stored : "null",
    price_stored ? price_stored : "null");
}

void CmndVendingClearAll(void) {
  for (int i = 0; i < vending.box_count; i++) {
    vending.box_status[i] = false;
    vending.box_last_changed[i] = Rtc.utc_time;
  }
  HoneyVending_SaveStatus();
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: All %d boxes cleared to EMPTY"), vending.box_count);
  
  HoneyVending_PublishAllBoxesMQTT();
  
  ResponseCmndDone();
}

void CmndVendingFillAll(void) {
  for (int i = 0; i < vending.box_count; i++) {
    vending.box_status[i] = true;
    vending.box_last_changed[i] = Rtc.utc_time;
  }
  HoneyVending_SaveStatus();
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: All %d boxes set to AVAILABLE"), vending.box_count);
  
  HoneyVending_PublishAllBoxesMQTT();
  
  ResponseCmndDone();
}

void CmndVendingPublish(void) {
  HoneyVending_PublishAllBoxesMQTT();
  HoneyVending_PublishSystemMQTT();
  ResponseCmndDone();
}

void CmndVendingDiscovery(void) {
  HoneyVending_PublishDiscovery();
  ResponseCmndDone();
}

// Lock control commands
void CmndVendingUnlock(void) {
  if (XdrvMailbox.data_len > 0) {
    uint8_t box_id = atoi(XdrvMailbox.data);
    if (box_id >= 1 && box_id <= vending.box_count) {
      UnlockBoxKey(box_id);
      Response_P(PSTR("{\"Box\":%d,\"Status\":\"Unlocked\",\"Duration\":%d}"), 
        box_id, UNLOCK_DURATION_MS);
      return;
    }
  }
  ResponseCmndError();
}

void CmndVendingLock(void) {
  if (XdrvMailbox.data_len > 0) {
    uint8_t box_id = atoi(XdrvMailbox.data);
    if (box_id >= 1 && box_id <= vending.box_count) {
      LockBoxKey(box_id);
      Response_P(PSTR("{\"Box\":%d,\"Status\":\"Locked\"}"), box_id);
      return;
    }
  }
  ResponseCmndError();
}

void CmndVendingLockAll(void) {
  LockAllBoxes();
  Response_P(PSTR("{\"Status\":\"All boxes locked\",\"Count\":%d}"), vending.box_count);
}

// Motor command: VendingMotor COIN_ACCEPT | COIN_REJECT | RESET
void CmndVendingMotor(void) {
  if (XdrvMailbox.data_len > 0) {
    if (strcasecmp(XdrvMailbox.data, "COIN_ACCEPT") == 0) {
      Motor_DoAction(COIN_ACCEPT);
      Response_P(PSTR("{\"Motor\":\"COIN_ACCEPT\",\"Steps\":%d,\"Offset\":%d}"),
        MOTOR_45DEG_STEPS, motor_position_offset);
      return;
    } else if (strcasecmp(XdrvMailbox.data, "COIN_REJECT") == 0) {
      Motor_DoAction(COIN_REJECT);
      Response_P(PSTR("{\"Motor\":\"COIN_REJECT\",\"Steps\":%d,\"Offset\":%d}"),
        MOTOR_45DEG_STEPS, motor_position_offset);
      return;
    } else if (strcasecmp(XdrvMailbox.data, "RESET") == 0) {
      Motor_DoAction(MOTOR_RESET);
      Response_P(PSTR("{\"Motor\":\"RESET\",\"Offset\":%d}"), motor_position_offset);
      return;
    }
  }
  // Usage hint on bad input
  Response_P(PSTR("{\"Error\":\"Usage: VendingMotor COIN_ACCEPT|COIN_REJECT|RESET\"}"));
}

// LCD command: VendingLCDWrite <row> <text>
// Row 0 = top line, row 1 = bottom line (16 chars max, auto-padded with spaces)
// Example: VendingLCDWrite 0 Insert coin
// Example: VendingLCDWrite 1 Box 3 = E5.00
void CmndVendingLCDWrite(void) {
  if (XdrvMailbox.data_len > 0) {
    char* space_pos = strchr(XdrvMailbox.data, ' ');
    if (space_pos != nullptr) {
      *space_pos = '\0';
      uint8_t row = (uint8_t)atoi(XdrvMailbox.data);
      const char* text = space_pos + 1;
      if (row < LCD_ROWS) {
        LCD_WriteText(row, 0, text);
        Response_P(PSTR("{\"LCD\":\"OK\",\"Row\":%d,\"Text\":\"%s\"}"), row, text);
        return;
      }
    }
  }
  Response_P(PSTR("{\"Error\":\"Usage: VendingLCDWrite <row 0|1> <text>\"}"));
}

// LCD clear command: VendingLCDClear
void CmndVendingLCDClear(void) {
  LCD_Clear();
  ResponseCmndDone();
}

// Button read command: VendingButtons
// Reads MCP23017 GPA+GPB and logs every pressed button to console
void CmndVendingButtons(void) {
  HoneyVending_PrintPressedButtons();
  uint16_t state = MCP23017_ReadButtons();
  Response_P(PSTR("{\"Buttons\":\"0x%04X\",\"GPA\":\"0x%02X\",\"GPB\":\"0x%02X\"}"),
    state, (uint8_t)(state & 0xFF), (uint8_t)((state >> 8) & 0xFF));
}


// Command definitions
const char kHoneyVendingCommands[] PROGMEM = 
  "Vending|"
  "Status|Toggle|Set|BoxStatus|Display|Reset|Test|Debug|RawSettings|ClearAll|FillAll|Publish|Discovery|BoxCount|SetPrice|SelectBox|Unlock|Lock|LockAll|Motor|LCDWrite|LCDClear|Buttons";

void (* const HoneyVendingCommand[])(void) PROGMEM = {
  &CmndVendingStatus, &CmndVendingToggle, &CmndVendingSet, &CmndVendingBoxStatus,
  &CmndVendingDisplay, &CmndVendingReset, &CmndVendingTest, &CmndVendingDebug,
  &CmndVendingRawSettings, &CmndVendingClearAll, &CmndVendingFillAll, &CmndVendingPublish,
  &CmndVendingDiscovery, &CmndVendingBoxCount, &CmndVendingSetPrice, &CmndVendingSelectBox,
  &CmndVendingUnlock, &CmndVendingLock, &CmndVendingLockAll, &CmndVendingMotor,
  &CmndVendingLCDWrite, &CmndVendingLCDClear, &CmndVendingButtons
};


// ========== DYNAMIC HTML GENERATION ==========

void HoneyVending_ShowWebButton(void) {
  // Inject CSS and JavaScript
  WSContentSend_P(PSTR(
    "<style>"
    ".honey-grid{display:grid;grid-template-columns:repeat(5,1fr);gap:8px;margin:10px 0;}"
    ".honey-grid button{padding:10px 6px;font-size:0.85rem;min-width:50px;display:flex;flex-direction:column;align-items:center;line-height:1.3;border:none;cursor:pointer;border-radius:5px;}"
    ".honey-available{background-color:#4CAF50;color:white;}"
    ".honey-empty{background-color:#f44336;color:white;}"
    ".box-number{font-weight:bold;font-size:0.95rem;margin-bottom:3px;}"
    ".box-price{font-size:0.8rem;font-weight:bold;margin:3px 0;}"
    ".box-date{font-size:0.7rem;opacity:0.9;margin-top:3px;}"
    "</style>"
    "<style>"
    "#priceModal{display:none;position:fixed;z-index:1000;left:0;top:0;width:100%%;height:100%%;background-color:rgba(0,0,0,0.4);}"
    ".modal-content{background-color:#fefefe;color:#000;margin:10%% auto;padding:20px;border:1px solid #888;width:90%%;max-width:400px;border-radius:8px;}"
    ".modal-header{font-size:1.2rem;font-weight:bold;margin-bottom:15px;}"
    ".modal-body{margin:15px 0;}"
    ".modal-input{width:100%%;padding:8px;margin:5px 0;font-size:1rem;box-sizing:border-box;}"
    ".modal-buttons{display:flex;gap:10px;margin-top:15px;}"
    ".modal-buttons button{flex:1;padding:10px;font-size:1rem;cursor:pointer;border:none;border-radius:5px;}"
    ".btn-save{background-color:#4CAF50;color:white;}"
    ".btn-cancel{background-color:#f44336;color:white;}"
    "</style>"
    "<script>"
    "var boxPrices=%s;"
    "function toggleBox(id){"
      "var btn=document.getElementById('box'+id);"
      "var wasEmpty=btn.classList.contains('honey-empty');"
      "if(wasEmpty){"
        "showPriceModal(id);"
      "}else{"
        "fetch('/cm?cmnd=VendingToggle '+id)"
        ".then(r=>r.json())"
        ".then(data=>{"
          "console.log('Toggle result:',data);"
          "setTimeout(()=>location.reload(),300);"
        "})"
        ".catch(e=>console.error('Toggle failed:',e));"
      "}"
    "}"
    "function showPriceModal(boxId){"
      "var modal=document.getElementById('priceModal');"
      "var currentPrice=(boxPrices[boxId-1]/100).toFixed(2);"
      "document.getElementById('modalBoxId').textContent=boxId;"
      "document.getElementById('modalCurrentPrice').textContent=currentPrice;"
      "document.getElementById('modalNewPrice').value=currentPrice;"
      "modal.style.display='block';"
      "document.getElementById('modalNewPrice').focus();"
    "}"
    "function closePriceModal(){"
      "document.getElementById('priceModal').style.display='none';"
    "}"
    "function savePriceAndToggle(){"
      "var boxId=parseInt(document.getElementById('modalBoxId').textContent);"
      "var newPrice=parseFloat(document.getElementById('modalNewPrice').value);"
      "if(isNaN(newPrice)||newPrice<0.1||newPrice>100){"
        "alert('Invalid price! Must be between €0.10 and €100.00');"
        "return;"
      "}"
      "var priceCents=Math.round(newPrice*100);"
      "fetch('/cm?cmnd=VendingSetPrice '+boxId+' '+priceCents)"
      ".then(r=>r.json())"
      ".then(data=>{"
        "console.log('Price set:',data);"
        "return fetch('/cm?cmnd=VendingToggle '+boxId);"
      "})"
      ".then(()=>{"
        "setTimeout(()=>location.reload(),300);"
      "})"
      ".catch(e=>console.error('Save failed:',e));"
      "closePriceModal();"
    "}"
    "</script>"
  ), 
  "[]"  // We'll build this below
  );
  
  // Build JavaScript array of prices
  WSContentSend_P(PSTR("<script>boxPrices=["));
  for (int i = 0; i < vending.box_count; i++) {
    if (i > 0) WSContentSend_P(PSTR(","));
    WSContentSend_P(PSTR("%lu"), (unsigned long)vending.box_price[i]);
  }
  WSContentSend_P(PSTR("];</script>"));
  
  // Show coin counter if a box is selected
  if (vending.selected_box_id > 0) {
    char target_str[16];
    char total_str[16];
    uint32_t target_cents = vending.box_price[vending.selected_box_id - 1];
    CentsToEuroString(target_cents, target_str, sizeof(target_str));
    CentsToEuroString(vending.total_cents, total_str, sizeof(total_str));
    
    WSContentSend_P(PSTR(
      "<div style='background-color:#FFF3CD;border:1px solid #FFD700;padding:10px;margin:10px 0;border-radius:5px;'>"
      "<b>🎯 Selected Box:</b> %d | <b>Target:</b> %s | <b>Inserted:</b> %s | <b>Coins:</b> %d"
      "</div>"
    ), vending.selected_box_id, target_str, total_str, vending.coins_detected);
  }
  
  // Main header
  WSContentSend_P(PSTR(
    "<p><b>🍯 Honey Vending Machine (ID: %04X, Boxes: %d/%d)</b></p>"
    "<div class='honey-grid'>"
  ), (unsigned int)vending.device_id, vending.box_count, MAX_HONEY_BOX_COUNT);
  
  // Generate grid buttons with box number, price, and date
  for (int i = 1; i <= vending.box_count; i++) {
    bool has_honey = HoneyVending_GetStatus(i);
    uint32_t timestamp = HoneyVending_GetTimestamp(i);
    uint32_t price_cents = HoneyVending_GetPrice(i);
    
    const char* css_class = has_honey ? "honey-available" : "honey-empty";
    const char* icon = has_honey ? "🍯 " : "";
    
    char price_str[16];
    CentsToEuroString(price_cents, price_str, sizeof(price_str));
    
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
      "<span class='box-number'>%sBox %d</span>"
      "<span class='box-price'>💰 %s</span>"
      "<span class='box-date'>%s</span>"
      "</button>"
    ), i, css_class, i, icon, i, price_str, date_str);
  }
  
  WSContentSend_P(PSTR("</div>"));
  
  // Price editing modal
  WSContentSend_P(PSTR(
    "<div id='priceModal'>"
    "<div class='modal-content'>"
    "<div class='modal-header'>💰 Set Price for Box <span id='modalBoxId'>1</span></div>"
    "<div class='modal-body'>"
    "<p><b>Current Price:</b> €<span id='modalCurrentPrice'>5.00</span></p>"
    "<p><b>New Price (€):</b></p>"
    "<input type='number' id='modalNewPrice' class='modal-input' min='0.10' max='100' step='0.01' value='5.00'>"
    "</div>"
    "<div class='modal-buttons'>"
    "<button class='btn-save' onclick='savePriceAndToggle()'>💾 Save</button>"
    "<button class='btn-cancel' onclick='closePriceModal()'>❌ Cancel</button>"
    "</div>"
    "</div>"
    "</div>"
  ));
}


// ========== USER LOOP ==========

void userLoop() {
  server.handleClient();
}


// ========== ISR AND COIN DETECTION ==========

void IRAM_ATTR HoneyVending_PulseISR(void) {
  uint32_t now = millis();
  
  if (vending.pulse_count > 0 && (now - vending.last_pulse_ms) < PULSE_DEBOUNCE_MS) {
    return;
  }
  
  if (vending.pulse_count == 0) {
    vending.first_pulse_ms = now;
  }
  
  vending.pulse_count++;
  vending.last_pulse_ms = now;
}

uint32_t PulsesToCents(uint32_t pulses) {
  switch(pulses) {
    case 1:  return 10;
    case 2:  return 20;
    case 5:  return 50;
    case 10: return 100;
    case 20: return 200;
    default: return 0;
  }
}

void CentsToEuroString(uint32_t cents, char* buffer, size_t len) {
  uint32_t euros = cents / 100;
  uint32_t cents_part = cents % 100;
  snprintf(buffer, len, "€%lu.%02lu", (unsigned long)euros, (unsigned long)cents_part);
}

void HoneyVending_DisplayValues(void) {
  char current_str[16];
  char required_str[16];
  uint32_t remaining_cents = 0;
  
  CentsToEuroString(vending.total_cents, current_str, sizeof(current_str));
  
  uint32_t target_cents = 0;
  if (vending.selected_box_id > 0 && vending.selected_box_id <= vending.box_count) {
    target_cents = vending.box_price[vending.selected_box_id - 1];
  }
  CentsToEuroString(target_cents, required_str, sizeof(required_str));
  
  if (vending.total_cents < target_cents) {
    remaining_cents = target_cents - vending.total_cents;
  }
  
  char remaining_str[16];
  CentsToEuroString(remaining_cents, remaining_str, sizeof(remaining_str));
  
  AddLog(LOG_LEVEL_INFO, PSTR("DISPLAY: Current=%s Required=%s Remaining=%s"), 
    current_str, required_str, remaining_str);
}

void HoneyVending_HoneyAvailable(void) {
  if (vending.selected_box_id > 0) {
    UnlockBoxKey(vending.selected_box_id);
    char total_str[16];
    CentsToEuroString(vending.total_cents, total_str, sizeof(total_str));
    
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: ╔════════════════════════════════════╗"));
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: ║  🍯🍯 HONEY IS AVAILABLE! 🍯        ║"));
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: ║  Box %d - Target reached!          ║"), vending.selected_box_id);
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: ║  Total: %s                         ║"), total_str);
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: ╚════════════════════════════════════╝"));
    
    HoneyVending_PublishSystemMQTT();
    
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Auto-resetting for next customer..."));
    
    // Mark box as dispensed (set to EMPTY)
    HoneyVending_SetStatus(vending.selected_box_id, false);
    // reset the vending machine
    CmndVendingReset();
  }
}

// ========== SHIFT REGISTER / LOCK CONTROL FUNCTIONS ==========

// Initialize shift register GPIO pins
void ShiftReg_Init(void) {
  pinMode(SHIFT_REG_DATA_PIN, OUTPUT);
  pinMode(SHIFT_REG_CLOCK_PIN, OUTPUT);
  pinMode(SHIFT_REG_LATCH_PIN, OUTPUT);
  
  // Initialize all outputs to locked state (all LOW or HIGH depending on config)
  for (int i = 0; i < NUM_SHIFT_REGISTERS; i++) {
    vending.shift_reg_state[i] = SOLENOID_ACTIVE_HIGH ? 0x00000000 : 0xFFFFFFFF;
  }
  
  ShiftReg_Write(vending.shift_reg_state, NUM_SHIFT_REGISTERS);
  
  AddLog(LOG_LEVEL_INFO, PSTR("SHIFT_REG: Initialized %d registers on GPIO D=%d C=%d L=%d"), 
    NUM_SHIFT_REGISTERS, SHIFT_REG_DATA_PIN, SHIFT_REG_CLOCK_PIN, SHIFT_REG_LATCH_PIN);
  AddLog(LOG_LEVEL_INFO, PSTR("SHIFT_REG: All locks engaged (active %s)"), 
    SOLENOID_ACTIVE_HIGH ? "HIGH" : "LOW");
}

// Write data to shift register chain
void ShiftReg_Write(uint32_t* data, uint8_t num_registers) {
  digitalWrite(SHIFT_REG_LATCH_PIN, LOW);
  
  // Shift out data MSB first, starting from last register in chain
  for (int reg = num_registers - 1; reg >= 0; reg--) {
    for (int bit = 7; bit >= 0; bit--) {
      digitalWrite(SHIFT_REG_CLOCK_PIN, LOW);
      
      // Only use the lower 8 bits of each uint32_t for one 74HC595
      bool bit_value = (data[reg] >> bit) & 0x01;
      digitalWrite(SHIFT_REG_DATA_PIN, bit_value ? HIGH : LOW);
      
      digitalWrite(SHIFT_REG_CLOCK_PIN, HIGH);
    }
  }
  
  // Latch the data to outputs
  digitalWrite(SHIFT_REG_LATCH_PIN, HIGH);
  digitalWrite(SHIFT_REG_LATCH_PIN, LOW);
}

// Set individual bit in shift register
void ShiftReg_SetBit(uint8_t bit_position, bool state) {
  if (bit_position >= (NUM_SHIFT_REGISTERS * 8)) {
    AddLog(LOG_LEVEL_ERROR, PSTR("SHIFT_REG: Invalid bit position %d (max %d)"), 
      bit_position, (NUM_SHIFT_REGISTERS * 8) - 1);
    return;
  }
  
  uint8_t register_index = bit_position / 8;
  uint8_t bit_index = bit_position % 8;
  
  if (state) {
    vending.shift_reg_state[register_index] |= (1 << bit_index);
  } else {
    vending.shift_reg_state[register_index] &= ~(1 << bit_index);
  }
  
  ShiftReg_Write(vending.shift_reg_state, NUM_SHIFT_REGISTERS);
}

// Unlock a specific box (activate solenoid)
void UnlockBoxKey(uint8_t box_id) {
  if (box_id < 1 || box_id > vending.box_count) {
    AddLog(LOG_LEVEL_ERROR, PSTR("KEY: Invalid box ID %d (must be 1-%d)"), box_id, vending.box_count);
    return;
  }
  
  if (box_id > (NUM_SHIFT_REGISTERS * 8)) {
    AddLog(LOG_LEVEL_ERROR, PSTR("KEY: Box %d exceeds shift register capacity (%d outputs)"), box_id, NUM_SHIFT_REGISTERS * 8);
    return;
  }
  
  // Lock any previously unlocked box first
  if (vending.unlocked_box_id > 0) {
    LockBoxKey(vending.unlocked_box_id);
  }
  
  uint8_t bit_position = box_id - 1; // Box 1 = bit 0, Box 2 = bit 1, etc.
  ShiftReg_SetBit(bit_position, SOLENOID_ACTIVE_HIGH);
  
  vending.unlocked_box_id = box_id;
  vending.unlock_start_ms = millis();
  
  AddLog(LOG_LEVEL_INFO, PSTR("KEY: ✓ Unlocked Box %d (bit %d, duration %dms)"), box_id, bit_position, UNLOCK_DURATION_MS);
}

// Lock a specific box (deactivate solenoid)
void LockBoxKey(uint8_t box_id) {
  if (box_id < 1 || box_id > vending.box_count) {
    return;
  }
  
  uint8_t bit_position = box_id - 1;
  ShiftReg_SetBit(bit_position, !SOLENOID_ACTIVE_HIGH);
  
  if (vending.unlocked_box_id == box_id) {
    vending.unlocked_box_id = 0;
    vending.unlock_start_ms = 0;
  }
  
  AddLog(LOG_LEVEL_INFO, PSTR("KEY: ✗ Locked Box %d (bit %d)"), box_id, bit_position);
}

// Lock all boxes (emergency/reset)
void LockAllBoxes(void) {
  for (int i = 0; i < NUM_SHIFT_REGISTERS; i++) {
    vending.shift_reg_state[i] = SOLENOID_ACTIVE_HIGH ? 0x00000000 : 0xFFFFFFFF;
  }
  
  ShiftReg_Write(vending.shift_reg_state, NUM_SHIFT_REGISTERS);
  
  vending.unlocked_box_id = 0;
  vending.unlock_start_ms = 0;
  
  AddLog(LOG_LEVEL_INFO, PSTR("KEY: All boxes locked"));
}

// Check if unlock timeout has expired and auto-lock
void CheckUnlockTimeout(void) {
  if (vending.unlocked_box_id > 0) {
    uint32_t elapsed = millis() - vending.unlock_start_ms;
    
    if (elapsed >= UNLOCK_DURATION_MS) {
      AddLog(LOG_LEVEL_INFO, PSTR("KEY: Timeout reached for Box %d (%lu ms)"), vending.unlocked_box_id, (unsigned long)elapsed);
      LockBoxKey(vending.unlocked_box_id);
    }
  }
}


// ========== DRV8825 STEPPER MOTOR FUNCTIONS ==========

// Initialize DRV8825 GPIO pins
void Motor_Init(void) {
  pinMode(MOTOR_STEP_PIN,   OUTPUT);
  pinMode(MOTOR_DIR_PIN,    OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN, HIGH); // Disabled at startup (active LOW)
  motor_position_offset = 0;
  AddLog(LOG_LEVEL_INFO, PSTR("MOTOR: DRV8825 initialized on STEP=%d DIR=%d EN=%d"),
    MOTOR_STEP_PIN, MOTOR_DIR_PIN, MOTOR_ENABLE_PIN);
}

// Rotate motor by given number of steps in given direction
// clockwise=true → RIGHT, clockwise=false → LEFT
static void Motor_Rotate(uint16_t steps, bool clockwise) {
  if (steps == 0) return;
  digitalWrite(MOTOR_DIR_PIN,    clockwise ? HIGH : LOW);
  digitalWrite(MOTOR_ENABLE_PIN, LOW);  // Enable driver
  delayMicroseconds(10);                // Setup time for DIR signal
  for (uint16_t s = 0; s < steps; s++) {
    digitalWrite(MOTOR_STEP_PIN, HIGH);
    delayMicroseconds(MOTOR_STEP_DELAY_US);
    digitalWrite(MOTOR_STEP_PIN, LOW);
    delayMicroseconds(MOTOR_STEP_DELAY_US);
  }
  digitalWrite(MOTOR_ENABLE_PIN, HIGH); // Disable driver (saves power, reduces heat)
}

// Perform a named coin gate action
void Motor_DoAction(MotorAction_t action) {
  switch (action) {
    case COIN_ACCEPT:
      // 45° LEFT (counter-clockwise) — guides coin to collection tray
      Motor_Rotate(MOTOR_45DEG_STEPS, false);
      motor_position_offset -= MOTOR_45DEG_STEPS;
      AddLog(LOG_LEVEL_INFO, PSTR("MOTOR: COIN_ACCEPT — rotated 45° LEFT (%d steps, offset=%d)"),
        MOTOR_45DEG_STEPS, motor_position_offset);
      break;

    case COIN_REJECT:
      // 45° RIGHT (clockwise) — returns coin to customer
      Motor_Rotate(MOTOR_45DEG_STEPS, true);
      motor_position_offset += MOTOR_45DEG_STEPS;
      AddLog(LOG_LEVEL_INFO, PSTR("MOTOR: COIN_REJECT — rotated 45° RIGHT (%d steps, offset=%d)"),
        MOTOR_45DEG_STEPS, motor_position_offset);
      break;

    case MOTOR_RESET: {
      // Return to home by reversing the current offset
      if (motor_position_offset == 0) {
        AddLog(LOG_LEVEL_INFO, PSTR("MOTOR: RESET — already at home position"));
        return;
      }
      bool go_left = (motor_position_offset > 0); // went right → go back left
      uint16_t steps_back = (uint16_t)(motor_position_offset < 0
                                        ? -motor_position_offset
                                        :  motor_position_offset);
      Motor_Rotate(steps_back, !go_left);
      AddLog(LOG_LEVEL_INFO, PSTR("MOTOR: RESET — returned %d steps %s to home"),
        steps_back, go_left ? "LEFT" : "RIGHT");
      motor_position_offset = 0;
      break;
    }
  }
}


// ========== LCD 1602 FUNCTIONS (I2C, GPIO21=SDA, GPIO22=SCL) ==========

// Initialize LCD 1602 over shared I2C bus.
// Called once from HoneyVending_Init().
// ⚠ If the display stays blank, swap LCD_I2C_ADDR between 0x27 and 0x3F —
//   different PCF8574 backpack batches use different addresses.
void LCD_Init(void) {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); // Start shared I2C bus (also used by MCP23017)
  lcd.init();                            // Initialize LCD controller
  lcd.backlight();                       // Turn on backlight
  lcd.clear();                           // Blank the screen

  vending.lcd_initialized = true;

  AddLog(LOG_LEVEL_INFO, PSTR("LCD: Initialized 16x2 at I2C 0x%02X (SDA=%d SCL=%d)"),
    LCD_I2C_ADDR, I2C_SDA_PIN, I2C_SCL_PIN);

  // Startup splash — visible for the first few seconds
  LCD_WriteText(0, 0, "  Honey Vending ");
  LCD_WriteText(1, 0, "   Starting...  ");
}

// Write a text string to the LCD at a specific row and column.
//
// Parameters:
//   row  — 0 = top line, 1 = bottom line
//   col  — starting column 0–15
//   text — null-terminated string; auto-padded with spaces to overwrite old text
//
// Examples from other functions:
//   LCD_WriteText(0, 0, "Select a box");
//   LCD_WriteText(1, 0, "Box 3 = E5.00");
//
//   char buf[17];
//   snprintf(buf, sizeof(buf), "Total: %s", total_str);
//   LCD_WriteText(1, 0, buf);
void LCD_WriteText(uint8_t row, uint8_t col, const char* text) {
  if (!vending.lcd_initialized) {
    AddLog(LOG_LEVEL_ERROR, PSTR("LCD: Not initialized — call LCD_Init() first"));
    return;
  }
  if (row >= LCD_ROWS || col >= LCD_COLS) {
    AddLog(LOG_LEVEL_ERROR, PSTR("LCD: Invalid position row=%d col=%d (max row=%d col=%d)"),
      row, col, LCD_ROWS - 1, LCD_COLS - 1);
    return;
  }

  // Build a space-padded string to fill the remaining columns on the row.
  // This overwrites any leftover characters from a previous longer string.
  uint8_t available = LCD_COLS - col;
  char padded[LCD_COLS + 1];
  memset(padded, ' ', LCD_COLS);   // fill with spaces
  padded[LCD_COLS] = '\0';
  strncpy(padded, text, available);// copy text into the buffer
  padded[available] = '\0';        // ensure null terminator

  lcd.setCursor(col, row);
  lcd.print(padded);

  AddLog(LOG_LEVEL_DEBUG, PSTR("LCD: Row%d Col%d → \"%s\""), row, col, padded);
}

// Clear the entire LCD screen (both rows, all columns)
void LCD_Clear(void) {
  if (!vending.lcd_initialized) return;
  lcd.clear();
  AddLog(LOG_LEVEL_INFO, PSTR("LCD: Screen cleared"));
}


// ========== MCP23017 BUTTON FUNCTIONS (I2C, GPIO21=SDA, GPIO22=SCL) ==========

// Low-level helper: write one byte to an MCP23017 internal register
static void MCP23017_WriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MCP23017_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Low-level helper: read one byte from an MCP23017 internal register
static uint8_t MCP23017_ReadReg(uint8_t reg) {
  Wire.beginTransmission(MCP23017_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)MCP23017_I2C_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// Initialize MCP23017: all 16 pins as inputs with internal pull-ups enabled.
// Wire each button between a pin and GND — pressed = LOW (active LOW).
// Called once from HoneyVending_Init().
void MCP23017_Init(void) {
  // Wire.begin() already called by LCD_Init() — safe to call again (no-op)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // IODIRA / IODIRB = 0xFF → all 16 pins are inputs
  MCP23017_WriteReg(MCP23017_IODIRA, 0xFF);
  MCP23017_WriteReg(MCP23017_IODIRB, 0xFF);

  // GPPUA / GPPUB = 0xFF → enable internal pull-ups on all 16 pins
  // Un-pressed pin reads HIGH; pressed pin reads LOW
  MCP23017_WriteReg(MCP23017_GPPUA, 0xFF);
  MCP23017_WriteReg(MCP23017_GPPUB, 0xFF);

  vending.mcp_initialized  = true;
  vending.last_button_state = 0xFFFF; // all pins HIGH = all released

  AddLog(LOG_LEVEL_INFO, PSTR("MCP23017: Initialized at I2C 0x%02X — 16 inputs with pull-ups (SDA=%d SCL=%d)"),
    MCP23017_I2C_ADDR, I2C_SDA_PIN, I2C_SCL_PIN);
}

// Read all 16 button inputs from MCP23017 in one call.
//
// Returns a 16-bit value:
//   bits  0–7  = GPA0..GPA7  →  buttons  1–8
//   bits  8–15 = GPB0..GPB7  →  buttons  9–16
//
// Bit = 0 means PRESSED  (active LOW, pull-up pulls unused pin HIGH)
// Bit = 1 means RELEASED
//
// Usage from any other function:
//   uint16_t state = MCP23017_ReadButtons();
//   if (!(state & (1 << 0)))  { /* button 1 (GPA0) pressed */ }
//   if (!(state & (1 << 8)))  { /* button 9 (GPB0) pressed */ }
//   if (!(state & (1 << 15))) { /* button 16 (GPB7) pressed */ }
uint16_t MCP23017_ReadButtons(void) {
  if (!vending.mcp_initialized) {
    AddLog(LOG_LEVEL_ERROR, PSTR("MCP23017: Not initialized — call MCP23017_Init() first"));
    return 0xFFFF; // return all released so caller logic is safe
  }
  uint8_t gpa = MCP23017_ReadReg(MCP23017_GPIOA); // buttons 1–8
  uint8_t gpb = MCP23017_ReadReg(MCP23017_GPIOB); // buttons 9–16
  return (uint16_t)(gpa | ((uint16_t)gpb << 8));
}

// Read MCP23017 and print every currently-pressed button to the Tasmota log.
// Also writes the pressed button number onto LCD row 1.
//
// Call this directly from any function when you need to inspect button state:
//   HoneyVending_PrintPressedButtons();
//
// For non-logging use, call MCP23017_ReadButtons() and test bits yourself:
//   uint16_t raw = MCP23017_ReadButtons();
//   if (!(raw & (1 << 2))) { /* button 3 is down */ }
void HoneyVending_PrintPressedButtons(void) {
  if (!vending.mcp_initialized) {
    AddLog(LOG_LEVEL_ERROR, PSTR("BUTTONS: MCP23017 not initialized"));
    return;
  }

  uint16_t state = MCP23017_ReadButtons();
  bool any_pressed = false;

  AddLog(LOG_LEVEL_INFO, PSTR("BUTTONS: Raw state GPA=0x%02X GPB=0x%02X (bit 0=pressed, 1=released)"),
    (uint8_t)(state & 0xFF), (uint8_t)((state >> 8) & 0xFF));

  // Walk all 16 bits; a LOW bit means that button is pressed
  for (uint8_t i = 0; i < 16; i++) {
    if (!(state & (1 << i))) {
      uint8_t button_number = i + 1;              // 1-indexed for user display
      const char* port      = (i < 8) ? "GPA" : "GPB";
      uint8_t     pin_num   = i % 8;

      AddLog(LOG_LEVEL_INFO, PSTR("BUTTONS: *** Button %d PRESSED (%s%d) ***"),
        button_number, port, pin_num);

      // Show on LCD bottom row
      char lcd_buf[LCD_COLS + 1];
      snprintf(lcd_buf, sizeof(lcd_buf), "Button %d pressed ", button_number);
      LCD_WriteText(1, 0, lcd_buf);

      any_pressed = true;
    }
  }

  if (!any_pressed) {
    AddLog(LOG_LEVEL_INFO, PSTR("BUTTONS: No buttons currently pressed"));
  }
}


// ========== INIT AND LOOP ==========

void HoneyVending_Init(void) {
  memset(&vending, 0, sizeof(vending));
  
  #ifdef ESP8266
    vending.device_id = ESP.getChipId() & 0xFFFF;
  #else
    uint64_t chip_id = ESP.getEfuseMac();
    vending.device_id = (uint32_t)(chip_id & 0xFFFF);
  #endif
  
  int8_t pin = HONEY_WENDING_GPIO;
  
  pinMode(pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin), HoneyVending_PulseISR, FALLING);
  
  vending.initialized = true;
  
  HoneyVending_LoadBoxCount();
  HoneyVending_LoadStatus();
  HoneyVending_LoadPrices();
  
  // Initialize shift register for solenoid locks
  ShiftReg_Init();
  
  // Initialize stepper motor for coin gate
  Motor_Init();

  // Initialize LCD 1602 and MCP23017 on shared I2C bus (GPIO21=SDA, GPIO22=SCL)
  LCD_Init();
  MCP23017_Init();

  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Initialized on GPIO%d"), pin);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Device ID: %04X"), (unsigned int)vending.device_id);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Box count: %d (max: %d)"), vending.box_count, MAX_HONEY_BOX_COUNT);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Waiting for box selection..."));
}

void HoneyVending_Every100ms(void) {
  if (!vending.initialized) return;
  
  static uint32_t last_check = 0;
  uint32_t now = millis();
  
  if (now - last_check < 100) return;
  last_check = now;
  
  // Check for unlock timeout and auto-lock
  CheckUnlockTimeout();
  
  if (vending.pulse_count > 0 && 
      (now - vending.last_pulse_ms) >= COIN_TIMEOUT_MS) {
    
    uint32_t pulses = vending.pulse_count;
    uint32_t duration_ms = vending.last_pulse_ms - vending.first_pulse_ms;
    uint32_t coin_value = PulsesToCents(pulses);
    
    AddLog(LOG_LEVEL_DEBUG, PSTR("VENDING: Pulse train complete: %lu pulses in %lu ms"),
      (unsigned long)pulses, (unsigned long)duration_ms);
    
    if (coin_value > 0) {
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

      // Update LCD row 1 with running total each time a coin is inserted
      char lcd_buf[LCD_COLS + 1];
      snprintf(lcd_buf, sizeof(lcd_buf), "Total: %s", total_str);
      LCD_WriteText(1, 0, lcd_buf);
      
      HoneyVending_DisplayValues();
      HoneyVending_PublishSystemMQTT();
      
      // Check if we reached the target price for selected box
      if (vending.selected_box_id > 0 && !vending.honey_available) {
        uint32_t target_price = vending.box_price[vending.selected_box_id - 1];
        if (vending.total_cents >= target_price) {
          vending.honey_available = true;
          HoneyVending_HoneyAvailable();
        }
      }
      
    } else {
      AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Unknown coin (%lu pulses) - ignoring"), 
        (unsigned long)pulses);
    }
    
    vending.pulse_count = 0;
  }

  // Poll MCP23017 for button presses every 100ms using edge detection.
  // Only fires on the moment a button transitions from released to pressed.
  if (vending.mcp_initialized) {
    uint16_t current_state  = MCP23017_ReadButtons();
    // newly_pressed: bit was 1 (released) last time, is 0 (pressed) now
    uint16_t newly_pressed  = (~current_state) & vending.last_button_state;
    if (newly_pressed) {
      for (uint8_t i = 0; i < 16; i++) {
        if (newly_pressed & (1 << i)) {
          uint8_t button_number = i + 1;
          const char* port      = (i < 8) ? "GPA" : "GPB";
          uint8_t     pin_num   = i % 8;
          AddLog(LOG_LEVEL_INFO, PSTR("BUTTONS: *** Button %d PRESSED (%s%d) ***"),
            button_number, port, pin_num);
          // Mirror on LCD row 1
          char lcd_buf[LCD_COLS + 1];
          snprintf(lcd_buf, sizeof(lcd_buf), "Button %d pressed ", button_number);
          LCD_WriteText(1, 0, lcd_buf);
        }
      }
    }
    vending.last_button_state = current_state; // save for next edge detection cycle
  }
}

void HoneyVending_Every250ms(void) {
  if (!vending.initialized) return;
  if (!MqttIsConnected()) return;
  
  static uint32_t last_periodic_check = 0;
  uint32_t now = millis();
  
  if (now - last_periodic_check < 250) return;
  last_periodic_check = now;
  
  if ((now - vending.last_publish_ms) >= HONEY_PUBLISH_INTERVAL) {
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Periodic MQTT publish (5 min timer)"));
    HoneyVending_PublishAllBoxesMQTT();
    HoneyVending_PublishSystemMQTT();
  }
}

void HoneyVending_MqttConnected(void) {
  if (!vending.initialized) return;
  
  if (!vending.discovery_sent) {
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Sending Home Assistant Discovery..."));
    HoneyVending_PublishDiscovery();
  }
  
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: MQTT connected, publishing initial status..."));
  HoneyVending_PublishAllBoxesMQTT();
  HoneyVending_PublishSystemMQTT();
}


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
