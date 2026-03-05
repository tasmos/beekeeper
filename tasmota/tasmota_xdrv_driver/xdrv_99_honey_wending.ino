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
  WORKFLOW STATE MACHINE
  ======================================================================
  1. Button pressed        → Box selected (MQTT: box_selected:<n>)
                             60-second idle timeout starts
  2. No action in 60s      → Auto-reset (MQTT: idle)
  3. Coin inserted         → 2-minute timeout starts after EACH coin
                             (MQTT: coin_inserted:<cents>)
  4. No coin in 2 minutes  → Motor COIN_REJECT, reset (MQTT: rejecting → idle)
  5. Cancel button (#16)   → Motor COIN_REJECT if coins present, reset
  6. Target price reached  → Solenoid unlocked 500ms (MQTT: vending_unlocked:<n>)
                             Box marked EMPTY, machine resets (MQTT: idle)
  ======================================================================

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
  ├── LCD 2004  (default I2C address: 0x27 or 0x3F via PCF8574 backpack)
  └── MCP23017  (default I2C address: 0x20, all address pins A0-A2 to GND)
      ├── GPA0..GPA7 = buttons 1..8   (pins pulled HIGH internally, active LOW)
      └── GPB0..GPB7 = buttons 9..16  (pins pulled HIGH internally, active LOW)

  MQTT TOPICS
  ══════════════════════════════════════════════════════════════════════
  beekeeper_<ID>/honey/machine_status   – workflow state (not retained)
    {"machine_status":"<state>","selected_box":<n>,"total_cents":<c>}
    States: idle | box_selected:<n> | coin_inserted:<cents>
            vending_unlocked:<n> | rejecting
*/

#ifdef USE_HONEY_WENDING_MACHINE

#define XDRV_99  99

#warning **** Honey Vending Machine Driver (Per-Box Pricing) is included... ****

#include <WebServer.h>
#include <Wire.h>               // I2C bus (shared: LCD + MCP23017)
#include <LiquidCrystal_I2C.h> // LCD 2004 via PCF8574 I2C backpack

WebServer server(80);

#define HONEY_WENDING_GPIO  27   // GPIO27 = input-only pin (coin acceptor pulse)

// 74HC595 Shift Register Pins (for solenoid lock control)
#define SHIFT_REG_DATA_PIN  23   // GPIO23 = MOSI (SER/DS pin on 74HC595)
#define SHIFT_REG_CLOCK_PIN 18   // GPIO18 = SCK  (SRCLK pin on 74HC595)
#define SHIFT_REG_LATCH_PIN  5   // GPIO5  = SS   (RCLK/Latch pin on 74HC595)

// Shift Register Configuration
#define NUM_SHIFT_REGISTERS  4   // Number of cascaded 74HC595 chips (4 = 32 outputs for 30 boxes)
#define UNLOCK_DURATION_MS   100  // FIX 1: was 'true' (= 1ms), now correct 100ms
#define SOLENOID_ACTIVE_HIGH true // FIX 2: was 'false', now HIGH = unlocked, LOW = locked

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
#define MOTOR_ENABLE_PIN  33   // GPIO33 = DRV8825 ENABLE (active LOW)

#define MOTOR_STEPS_PER_REV  200   // 1.8° per step (full-step mode)
#define MOTOR_STEP_DELAY_US  2000  // 2ms between steps (slow = more torque)

typedef enum {
  COIN_ACCEPT = 0,   // Rotate 45° LEFT  (accept coin, guide to collection)
  COIN_REJECT = 1,   // Rotate 45° RIGHT (reject coin, return to customer)
  MOTOR_RESET = 2    // Return to home position
} MotorAction_t;

// Track position offset from home (signed, in steps; + = right, - = left)
static int16_t motor_position_offset = 0;

// ========== LCD 2004 + MCP23017 (I2C, shared bus) ==========
#define I2C_SDA_PIN       21     // GPIO21 = SDA (shared by LCD and MCP23017)
#define I2C_SCL_PIN       22     // GPIO22 = SCL (shared by LCD and MCP23017)

// NOTE: #⚠️ If LCD stays blank after flashing, change LCD_I2C_ADDR from 0x27 to 0x3F — both are common for PCF8574 backpacks.
#define LCD_I2C_ADDR      0x27   // PCF8574 backpack address (try 0x3F if 0x27 fails)
#define LCD_COLS          20     // LCD 2004 = 20 columns
#define LCD_ROWS          4      // LCD 2004 = 4 rows

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


// ======================================================================
// [WORKFLOW] STATE MACHINE CONFIGURATION
// ======================================================================
#define CANCEL_BUTTON_ID              6      // Button number that triggers cancel/coin-reject
#define WORKFLOW_BOX_IDLE_TIMEOUT_MS  60000   // 1 min:  box selected, no coin → auto-reset
#define WORKFLOW_COIN_TIMEOUT_MS      120000  // 2 min:  last coin inserted, no more → coin_reject + reset
#define WORKFLOW_VENDING_UNLOCK_MS    100     // 100 ms: solenoid unlock duration when dispensing

// [WORKFLOW] Machine state enum
typedef enum {
  MACHINE_IDLE = 0,       // No box selected, waiting for customer
  MACHINE_BOX_SELECTED,   // Box selected, waiting for first coin (1-min timeout)
  MACHINE_COIN_INSERTED,  // At least one coin inserted (2-min timeout after each coin)
  MACHINE_VENDING,        // Target price reached — solenoid active for 500ms
  MACHINE_REJECTING       // Coin-reject motor running, then auto-reset
} MachineState_t;
// ======================================================================


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

  // [WORKFLOW] State machine fields
  MachineState_t machine_state;        // Current workflow state
  uint32_t state_entered_ms;           // millis() when current state was entered

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
void CentsToLCDString(uint32_t cents, char* buffer, size_t len);   
void HoneyVending_UpdateLCD(void);                                 

// MCP23017 / Button Functions
void MCP23017_Init(void);
uint16_t MCP23017_ReadButtons(void);
void HoneyVending_PrintPressedButtons(void);

// [WORKFLOW] State Machine Functions
void HoneyVending_PublishMachineStatus(const char* status_str);
void HoneyVending_SetMachineState(MachineState_t new_state);
void HoneyVending_WorkflowReset(void);
void HoneyVending_WorkflowCancel(void);
void HoneyVending_WorkflowSelectBox(uint8_t box_id);
void HoneyVending_WorkflowCheckTimeouts(void);


// ========== MQTT FUNCTIONS ==========

void HoneyVending_PublishDiscovery(void) {
  if (!MqttIsConnected()) return;
  
  char topic[128];
  char device_name[64];
  char unique_id[64];
  
  snprintf(device_name, sizeof(device_name), "%s", SettingsText(SET_DEVICENAME));
  
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Publishing Home Assistant Discovery config..."));
  
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
    
    MqttPublish(topic, true);
  }
  
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

void HoneyVending_PublishBoxMQTT(uint8_t box_id) {
  if (box_id < 1 || box_id > vending.box_count) return;
  if (!MqttIsConnected()) return;
  
  bool has_honey = vending.box_status[box_id - 1];
  uint32_t timestamp = vending.box_last_changed[box_id - 1];
  uint32_t price_cents = vending.box_price[box_id - 1];
  
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
  
  char topic[64];
  snprintf(topic, sizeof(topic), "beekeeper_%04X/honey/all_boxes/box_%d", 
           (unsigned int)vending.device_id, box_id);
  
  MqttPublish(topic, true);
  
  AddLog(LOG_LEVEL_DEBUG, PSTR("VENDING: Published Box %d to %s"), box_id, topic);
}

void HoneyVending_PublishAllBoxesMQTT(void) {
  if (!MqttIsConnected()) return;
  
  for (int i = 1; i <= vending.box_count; i++) {
    HoneyVending_PublishBoxMQTT(i);
  }
  
  int available_count = 0;
  int empty_count = 0;
  for (int i = 0; i < vending.box_count; i++) {
    if (vending.box_status[i]) available_count++;
    else empty_count++;
  }
  
  Response_P(PSTR("{\"total\":%d,\"available\":%d,\"empty\":%d}"),
    vending.box_count, available_count, empty_count);
  
  char topic[64];
  snprintf(topic, sizeof(topic), "beekeeper_%04X/honey/summary", (unsigned int)vending.device_id);
  MqttPublish(topic, true);
  
  vending.last_publish_ms = millis();
}

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
    (unsigned long)vending.total_cents, total_str,
    (unsigned long)target_cents, target_str,
    vending.selected_box_id,
    (unsigned long)vending.coins_detected,
    vending.honey_available ? 1 : 0,
    vending.box_count
  );
  
  char topic[64];
  snprintf(topic, sizeof(topic), "beekeeper_%04X/honey/system", (unsigned int)vending.device_id);
  MqttPublish(topic, true);
}


// ======================================================================
// [WORKFLOW] STATE MACHINE FUNCTIONS
// ======================================================================

void HoneyVending_PublishMachineStatus(const char* status_str) {
  if (!MqttIsConnected()) return;

  Response_P(PSTR("{\"machine_status\":\"%s\",\"selected_box\":%d,\"total_cents\":%lu}"),
    status_str, vending.selected_box_id, (unsigned long)vending.total_cents);

  char topic[64];
  snprintf(topic, sizeof(topic), "beekeeper_%04X/honey/machine_status",
    (unsigned int)vending.device_id);
  MqttPublish(topic, false);

  AddLog(LOG_LEVEL_DEBUG, PSTR("WORKFLOW: Published machine_status: %s"), status_str);
}

void HoneyVending_SetMachineState(MachineState_t new_state) {
  vending.machine_state   = new_state;
  vending.state_entered_ms = millis();

  char status_str[48];

  switch (new_state) {
    case MACHINE_IDLE:
      snprintf(status_str, sizeof(status_str), "idle");
      break;
    case MACHINE_BOX_SELECTED:
      snprintf(status_str, sizeof(status_str), "box_selected:%d", vending.selected_box_id);
      break;
    case MACHINE_COIN_INSERTED:
      snprintf(status_str, sizeof(status_str), "coin_inserted:%lu",
        (unsigned long)vending.total_cents);
      break;
    case MACHINE_VENDING:
      UnlockBoxKey(vending.selected_box_id);
      snprintf(status_str, sizeof(status_str), "vending_unlocked:%d", vending.selected_box_id);
      AddLog(LOG_LEVEL_INFO,
        PSTR("WORKFLOW: *** HONEY DISPENSING — Box %d unlocked for %d ms ***"),
        vending.selected_box_id, WORKFLOW_VENDING_UNLOCK_MS);
      break;
    case MACHINE_REJECTING:
      snprintf(status_str, sizeof(status_str), "rejecting");
      break;
    default:
      snprintf(status_str, sizeof(status_str), "unknown");
      break;
  }

  HoneyVending_PublishMachineStatus(status_str);
  AddLog(LOG_LEVEL_INFO, PSTR("WORKFLOW: State → %s"), status_str);
}

void HoneyVending_WorkflowReset(void) {
  LockAllBoxes();

  vending.total_cents     = 0;
  vending.coins_detected  = 0;
  vending.honey_available = false;
  vending.pulse_count     = 0;
  vending.selected_box_id = 0;

  HoneyVending_SetMachineState(MACHINE_IDLE);
  HoneyVending_PublishSystemMQTT();
  HoneyVending_UpdateLCD();

  AddLog(LOG_LEVEL_INFO, PSTR("WORKFLOW: Machine reset — now idle"));
}

void HoneyVending_WorkflowCancel(void) {
  if (vending.machine_state == MACHINE_IDLE) {
    AddLog(LOG_LEVEL_INFO, PSTR("WORKFLOW: Cancel pressed but already idle — ignoring"));
    return;
  }
  if (vending.machine_state == MACHINE_VENDING) {
    AddLog(LOG_LEVEL_INFO, PSTR("WORKFLOW: Cancel pressed during vending — ignoring"));
    return;
  }

  AddLog(LOG_LEVEL_INFO,
    PSTR("WORKFLOW: Cancel triggered (state=%d, inserted=%lu cents)"),
    vending.machine_state, (unsigned long)vending.total_cents);

  HoneyVending_SetMachineState(MACHINE_REJECTING);

  if (vending.total_cents > 0) {
    AddLog(LOG_LEVEL_INFO, PSTR("WORKFLOW: Rejecting coins — running motor COIN_REJECT"));
    Motor_DoAction(COIN_REJECT);
  } else {
    AddLog(LOG_LEVEL_INFO, PSTR("WORKFLOW: No coins to reject — resetting directly"));
  }

  HoneyVending_WorkflowReset();
}

void HoneyVending_WorkflowSelectBox(uint8_t box_id) {
  if (vending.machine_state != MACHINE_IDLE) {
    AddLog(LOG_LEVEL_INFO,
      PSTR("WORKFLOW: Box %d press ignored — machine not idle (state=%d)"),
      box_id, vending.machine_state);
    return;
  }
  if (box_id < 1 || box_id > vending.box_count) {
    AddLog(LOG_LEVEL_INFO, PSTR("WORKFLOW: Invalid box ID %d"), box_id);
    return;
  }
  if (!vending.box_status[box_id - 1]) {
    AddLog(LOG_LEVEL_INFO, PSTR("WORKFLOW: Box %d is EMPTY — cannot select"), box_id);
    LCD_WriteText(2, 0, "  Box is EMPTY!     ");
    return;
  }

  HoneyVending_SelectBox(box_id);
  HoneyVending_SetMachineState(MACHINE_BOX_SELECTED);

  AddLog(LOG_LEVEL_INFO,
    PSTR("WORKFLOW: Box %d selected — 60-second coin-wait timer started"), box_id);
}

void HoneyVending_WorkflowCheckTimeouts(void) {
  if (vending.machine_state == MACHINE_IDLE ||
      vending.machine_state == MACHINE_REJECTING) return;

  uint32_t elapsed = millis() - vending.state_entered_ms;

  switch (vending.machine_state) {
    case MACHINE_BOX_SELECTED:
      if (elapsed >= WORKFLOW_BOX_IDLE_TIMEOUT_MS) {
        AddLog(LOG_LEVEL_INFO, PSTR("WORKFLOW: 60-second idle timeout — resetting"));
        HoneyVending_WorkflowReset();
      }
      break;
    case MACHINE_COIN_INSERTED:
      if (elapsed >= WORKFLOW_COIN_TIMEOUT_MS) {
        AddLog(LOG_LEVEL_INFO, PSTR("WORKFLOW: 2-minute coin timeout — rejecting"));
        HoneyVending_WorkflowCancel();
      }
      break;
    case MACHINE_VENDING:
      if (elapsed >= WORKFLOW_VENDING_UNLOCK_MS) {
        uint8_t dispensed_box = vending.selected_box_id;
        AddLog(LOG_LEVEL_INFO,
          PSTR("WORKFLOW: Dispensing complete — Box %d marked EMPTY, resetting"), dispensed_box);
        HoneyVending_SetStatus(dispensed_box, false);
        HoneyVending_WorkflowReset();
      }
      break;
    default:
      break;
  }
}


// ========== PERSISTENT STORAGE FUNCTIONS ==========

void HoneyVending_SaveBoxCount(void) {
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "C%02d", vending.box_count);
  SettingsUpdateText(HONEY_BOX_COUNT_INDEX, buffer);
  SettingsSave(1);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Box count saved: %d"), vending.box_count);
}

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
  vending.box_count = DEFAULT_HONEY_BOX_COUNT;
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: No saved box count, using default: %d"), vending.box_count);
  HoneyVending_SaveBoxCount();
}

void HoneyVending_SavePrices(void) {
  char price_buffer[512];
  int offset = 0;
  offset += snprintf(price_buffer + offset, sizeof(price_buffer) - offset, "P:");
  for (int i = 0; i < vending.box_count; i++) {
    if (i > 0) offset += snprintf(price_buffer + offset, sizeof(price_buffer) - offset, ",");
    offset += snprintf(price_buffer + offset, sizeof(price_buffer) - offset, "%lu", 
                      (unsigned long)vending.box_price[i]);
  }
  SettingsUpdateText(HONEY_PRICE_INDEX, price_buffer);
  SettingsSave(1);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Prices saved"));
}

void HoneyVending_LoadPrices(void) {
  const char* price_stored = SettingsText(HONEY_PRICE_INDEX);
  if (price_stored && strlen(price_stored) > 2 && price_stored[0] == 'P' && price_stored[1] == ':') {
    const char* ptr = price_stored + 2;
    for (int i = 0; i < vending.box_count; i++) {
      uint32_t price = strtoul(ptr, nullptr, 10);
      vending.box_price[i] = (price > 0) ? price : DEFAULT_PRICE_CENTS;
      ptr = strchr(ptr, ',');
      if (ptr) { ptr++; }
      else {
        for (int j = i + 1; j < vending.box_count; j++) vending.box_price[j] = DEFAULT_PRICE_CENTS;
        break;
      }
    }
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Prices loaded from memory"));
  } else {
    for (int i = 0; i < vending.box_count; i++) vending.box_price[i] = DEFAULT_PRICE_CENTS;
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: No prices found, initialized to default €5.00"));
    HoneyVending_SavePrices();
  }
}

void HoneyVending_SetBoxCount(uint8_t count) {
  if (count < 1 || count > MAX_HONEY_BOX_COUNT) {
    AddLog(LOG_LEVEL_ERROR, PSTR("VENDING: Invalid box count %d (must be 1-%d)"), count, MAX_HONEY_BOX_COUNT);
    return;
  }
  uint8_t old_count = vending.box_count;
  vending.box_count = count;
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
  for (int i = new_count + 1; i <= old_count; i++) {
    snprintf(topic, sizeof(topic), "homeassistant/binary_sensor/%s_box%d/config", device_name, i);
    MqttPublish(topic, true);
  }
}

void HoneyVending_SaveStatus(void) {
  uint32_t packed = 0;
  for (int i = 0; i < vending.box_count; i++) {
    if (vending.box_status[i]) packed |= (1 << i);
  }
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "H%08lX", (unsigned long)packed);
  SettingsUpdateText(HONEY_SETTINGS_INDEX, buffer);

  char ts_buffer[512];
  int offset = 0;
  offset += snprintf(ts_buffer + offset, sizeof(ts_buffer) - offset, "T:");
  for (int i = 0; i < vending.box_count; i++) {
    if (i > 0) offset += snprintf(ts_buffer + offset, sizeof(ts_buffer) - offset, ",");
    offset += snprintf(ts_buffer + offset, sizeof(ts_buffer) - offset, "%lu", 
                      (unsigned long)vending.box_last_changed[i]);
  }
  SettingsUpdateText(HONEY_TIMESTAMP_INDEX, ts_buffer);
  SettingsSave(1);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Status saved '%s' (0x%08lX)"), buffer, (unsigned long)packed);
}

void HoneyVending_LoadStatus(void) {
  const char* stored = SettingsText(HONEY_SETTINGS_INDEX);
  uint32_t packed = 0;
  
  if (stored && strlen(stored) >= 5 && stored[0] == 'H') {
    packed = (uint32_t)strtoul(stored + 1, nullptr, 16);
    AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Loaded status '%s' (0x%08lX)"), stored, (unsigned long)packed);
  } else {
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
  
  const char* ts_stored = SettingsText(HONEY_TIMESTAMP_INDEX);
  if (ts_stored && strlen(ts_stored) > 2 && ts_stored[0] == 'T' && ts_stored[1] == ':') {
    const char* ptr = ts_stored + 2;
    for (int i = 0; i < vending.box_count; i++) {
      vending.box_last_changed[i] = strtoul(ptr, nullptr, 10);
      ptr = strchr(ptr, ',');
      if (ptr) ptr++; else break;
    }
  } else {
    for (int i = 0; i < vending.box_count; i++) vending.box_last_changed[i] = 0;
  }
  
  int available_count = 0;
  for (int i = 0; i < vending.box_count; i++) if (vending.box_status[i]) available_count++;
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Loaded %d/%d boxes as AVAILABLE"), available_count, vending.box_count);
}

void HoneyVending_SetStatus(uint8_t box_id, bool has_honey) {
  if (box_id < 1 || box_id > vending.box_count) {
    AddLog(LOG_LEVEL_ERROR, PSTR("VENDING: Invalid box ID %d (must be 1-%d)"), box_id, vending.box_count);
    return;
  }
  vending.box_status[box_id - 1] = has_honey;
  vending.box_last_changed[box_id - 1] = Rtc.utc_time;
  HoneyVending_SaveStatus();
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Box %d → %s (timestamp: %lu)"), 
    box_id, has_honey ? "AVAILABLE" : "EMPTY", (unsigned long)Rtc.utc_time);
  HoneyVending_PublishBoxMQTT(box_id);
  
  int available_count = 0;
  for (int i = 0; i < vending.box_count; i++) if (vending.box_status[i]) available_count++;
  Response_P(PSTR("{\"total\":%d,\"available\":%d,\"empty\":%d}"),
    vending.box_count, available_count, vending.box_count - available_count);
  char topic[64];
  snprintf(topic, sizeof(topic), "beekeeper_%04X/honey/summary", (unsigned int)vending.device_id);
  MqttPublish(topic, true);
}

void HoneyVending_SetPrice(uint8_t box_id, uint32_t price_cents) {
  if (box_id < 1 || box_id > vending.box_count) {
    AddLog(LOG_LEVEL_ERROR, PSTR("VENDING: Invalid box ID %d (must be 1-%d)"), box_id, vending.box_count);
    return;
  }
  if (price_cents < 10 || price_cents > 10000) {
    AddLog(LOG_LEVEL_ERROR, PSTR("VENDING: Invalid price %lu cents (must be 10-10000)"), (unsigned long)price_cents);
    return;
  }
  char old_price_str[16], new_price_str[16];
  CentsToEuroString(vending.box_price[box_id - 1], old_price_str, sizeof(old_price_str));
  CentsToEuroString(price_cents, new_price_str, sizeof(new_price_str));
  vending.box_price[box_id - 1] = price_cents;
  HoneyVending_SavePrices();
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Box %d price: %s → %s"), box_id, old_price_str, new_price_str);
  HoneyVending_PublishBoxMQTT(box_id);
}

uint32_t HoneyVending_GetPrice(uint8_t box_id) {
  if (box_id < 1 || box_id > vending.box_count) return DEFAULT_PRICE_CENTS;
  return vending.box_price[box_id - 1];
}

bool HoneyVending_GetStatus(uint8_t box_id) {
  if (box_id < 1 || box_id > vending.box_count) return false;
  return vending.box_status[box_id - 1];
}

uint32_t HoneyVending_GetTimestamp(uint8_t box_id) {
  if (box_id < 1 || box_id > vending.box_count) return 0;
  return vending.box_last_changed[box_id - 1];
}

void HoneyVending_ToggleStatus(uint8_t box_id) {
  if (box_id < 1 || box_id > vending.box_count) {
    AddLog(LOG_LEVEL_ERROR, PSTR("VENDING: Invalid box ID %d (must be 1-%d)"), box_id, vending.box_count);
    return;
  }
  HoneyVending_SetStatus(box_id, !vending.box_status[box_id - 1]);
}

void HoneyVending_SelectBox(uint8_t box_id) {
  if (box_id < 1 || box_id > vending.box_count) {
    AddLog(LOG_LEVEL_ERROR, PSTR("VENDING: Invalid box ID %d (must be 1-%d)"), box_id, vending.box_count);
    return;
  }
  if (!vending.box_status[box_id - 1]) {
    AddLog(LOG_LEVEL_ERROR, PSTR("VENDING: Box %d is EMPTY - cannot select for vending"), box_id);
    return;
  }
  vending.selected_box_id = box_id;
  char price_str[16];
  CentsToEuroString(vending.box_price[box_id - 1], price_str, sizeof(price_str));
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Selected Box %d (Price: %s)"), box_id, price_str);
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
    } else { ResponseCmndError(); return; }
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
  char total_str[16], target_str[16];
  CentsToEuroString(vending.total_cents, total_str, sizeof(total_str));
  uint32_t target_cents = 0;
  if (vending.selected_box_id > 0) target_cents = vending.box_price[vending.selected_box_id - 1];
  CentsToEuroString(target_cents, target_str, sizeof(target_str));
  Response_P(PSTR("{\"Total\":\"%s\",\"Target\":\"%s\",\"SelectedBox\":%d,\"Coins\":%d,\"HoneyAvailable\":%d,\"BoxCount\":%d,\"DeviceID\":\"%04X\"}"),
    total_str, target_str, vending.selected_box_id, vending.coins_detected, 
    vending.honey_available ? 1 : 0, vending.box_count, (unsigned int)vending.device_id);
}

void CmndVendingDisplay(void)  { HoneyVending_DisplayValues(); ResponseCmndDone(); }

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
      AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Test mode - simulating %lu pulses"), (unsigned long)test_pulses);
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
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Unlocked box: %d"), vending.unlocked_box_id);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Current pulse count: %lu"), (unsigned long)vending.pulse_count);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: GPIO%d state: %d"), HONEY_WENDING_GPIO, digitalRead(HONEY_WENDING_GPIO));
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Current UTC time: %lu"), (unsigned long)Rtc.utc_time);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: LCD init: %d  MCP23017 init: %d"), vending.lcd_initialized, vending.mcp_initialized);
  const char* state_names[] = {"IDLE","BOX_SELECTED","COIN_INSERTED","VENDING","REJECTING"};
  uint32_t state_elapsed = millis() - vending.state_entered_ms;
  AddLog(LOG_LEVEL_INFO, PSTR("WORKFLOW: State: %s  Elapsed in state: %lu ms"),
    state_names[vending.machine_state], (unsigned long)state_elapsed);
  ResponseCmndDone();
}

void CmndVendingRawSettings(void) {
  const char* stored     = SettingsText(HONEY_SETTINGS_INDEX);
  const char* ts_stored  = SettingsText(HONEY_TIMESTAMP_INDEX);
  const char* count_stored = SettingsText(HONEY_BOX_COUNT_INDEX);
  const char* price_stored = SettingsText(HONEY_PRICE_INDEX);
  Response_P(PSTR("{\"RawSettings\":\"%s\",\"RawTimestamps\":\"%s\",\"RawBoxCount\":\"%s\",\"RawPrices\":\"%s\"}"), 
    stored ? stored : "null", ts_stored ? ts_stored : "null", 
    count_stored ? count_stored : "null", price_stored ? price_stored : "null");
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

void CmndVendingPublish(void)   { HoneyVending_PublishAllBoxesMQTT(); HoneyVending_PublishSystemMQTT(); ResponseCmndDone(); }
void CmndVendingDiscovery(void) { HoneyVending_PublishDiscovery(); ResponseCmndDone(); }

void CmndVendingUnlock(void) {
  if (XdrvMailbox.data_len > 0) {
    uint8_t box_id = atoi(XdrvMailbox.data);
    if (box_id >= 1 && box_id <= vending.box_count) {
      UnlockBoxKey(box_id);
      Response_P(PSTR("{\"Box\":%d,\"Status\":\"Unlocked\",\"Duration\":%d}"), box_id, UNLOCK_DURATION_MS);
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

void CmndVendingMotor(void) {
  if (XdrvMailbox.data_len > 0) {
    if (strcasecmp(XdrvMailbox.data, "COIN_ACCEPT") == 0) {
      Motor_DoAction(COIN_ACCEPT);
      Response_P(PSTR("{\"Motor\":\"COIN_ACCEPT\",\"Steps\":%d,\"Offset\":%d}"), MOTOR_STEPS_PER_REV, motor_position_offset);
      return;
    } else if (strcasecmp(XdrvMailbox.data, "COIN_REJECT") == 0) {
      Motor_DoAction(COIN_REJECT);
      Response_P(PSTR("{\"Motor\":\"COIN_REJECT\",\"Steps\":%d,\"Offset\":%d}"), MOTOR_STEPS_PER_REV, motor_position_offset);
      return;
    } else if (strcasecmp(XdrvMailbox.data, "RESET") == 0) {
      Motor_DoAction(MOTOR_RESET);
      Response_P(PSTR("{\"Motor\":\"RESET\",\"Offset\":%d}"), motor_position_offset);
      return;
    }
  }
  Response_P(PSTR("{\"Error\":\"Usage: VendingMotor COIN_ACCEPT|COIN_REJECT|RESET\"}"));
}

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

void CmndVendingLCDClear(void) { LCD_Clear(); ResponseCmndDone(); }

void CmndVendingButtons(void) {
  HoneyVending_PrintPressedButtons();
  uint16_t state = MCP23017_ReadButtons();
  Response_P(PSTR("{\"Buttons\":\"0x%04X\",\"GPA\":\"0x%02X\",\"GPB\":\"0x%02X\"}"),
    state, (uint8_t)(state & 0xFF), (uint8_t)((state >> 8) & 0xFF));
}

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
      "if(wasEmpty){showPriceModal(id);}"
      "else{"
        "fetch('/cm?cmnd=VendingToggle '+id)"
        ".then(r=>r.json())"
        ".then(data=>{setTimeout(()=>location.reload(),300);})"
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
    "function closePriceModal(){document.getElementById('priceModal').style.display='none';}"
    "function savePriceAndToggle(){"
      "var boxId=parseInt(document.getElementById('modalBoxId').textContent);"
      "var newPrice=parseFloat(document.getElementById('modalNewPrice').value);"
      "if(isNaN(newPrice)||newPrice<0.1||newPrice>100){alert('Invalid price!');return;}"
      "var priceCents=Math.round(newPrice*100);"
      "fetch('/cm?cmnd=VendingSetPrice '+boxId+' '+priceCents)"
      ".then(r=>r.json())"
      ".then(data=>{return fetch('/cm?cmnd=VendingToggle '+boxId);})"
      ".then(()=>{setTimeout(()=>location.reload(),300);})"
      ".catch(e=>console.error('Save failed:',e));"
      "closePriceModal();"
    "}"
    "</script>"
  ), "[]");
  
  WSContentSend_P(PSTR("<script>boxPrices=["));
  for (int i = 0; i < vending.box_count; i++) {
    if (i > 0) WSContentSend_P(PSTR(","));
    WSContentSend_P(PSTR("%lu"), (unsigned long)vending.box_price[i]);
  }
  WSContentSend_P(PSTR("];</script>"));
  
  if (vending.selected_box_id > 0) {
    char target_str[16], total_str[16];
    uint32_t target_cents = vending.box_price[vending.selected_box_id - 1];
    CentsToEuroString(target_cents, target_str, sizeof(target_str));
    CentsToEuroString(vending.total_cents, total_str, sizeof(total_str));
    WSContentSend_P(PSTR(
      "<div style='background-color:#FFF3CD;border:1px solid #FFD700;padding:10px;margin:10px 0;border-radius:5px;'>"
      "<b>🎯 Selected Box:</b> %d | <b>Target:</b> %s | <b>Inserted:</b> %s | <b>Coins:</b> %d"
      "</div>"
    ), vending.selected_box_id, target_str, total_str, vending.coins_detected);
  }
  
  WSContentSend_P(PSTR(
    "<p><b>🍯 Honey Vending Machine (ID: %04X, Boxes: %d/%d)</b></p>"
    "<div class='honey-grid'>"
  ), (unsigned int)vending.device_id, vending.box_count, MAX_HONEY_BOX_COUNT);
  
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
void userLoop() { server.handleClient(); }


// ========== ISR AND COIN DETECTION ==========

void IRAM_ATTR HoneyVending_PulseISR(void) {
  uint32_t now = millis();
  if (vending.pulse_count > 0 && (now - vending.last_pulse_ms) < PULSE_DEBOUNCE_MS) return;
  if (vending.pulse_count == 0) vending.first_pulse_ms = now;
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
  snprintf(buffer, len, "€%lu.%02lu", (unsigned long)(cents / 100), (unsigned long)(cents % 100));
}

void HoneyVending_DisplayValues(void) {
  char current_str[16], required_str[16], remaining_str[16];
  CentsToEuroString(vending.total_cents, current_str, sizeof(current_str));
  uint32_t target_cents = 0;
  if (vending.selected_box_id > 0 && vending.selected_box_id <= vending.box_count)
    target_cents = vending.box_price[vending.selected_box_id - 1];
  CentsToEuroString(target_cents, required_str, sizeof(required_str));
  uint32_t remaining = (vending.total_cents < target_cents) ? (target_cents - vending.total_cents) : 0;
  CentsToEuroString(remaining, remaining_str, sizeof(remaining_str));
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
    HoneyVending_SetStatus(vending.selected_box_id, false);
    CmndVendingReset();
  }
}

// ========== SHIFT REGISTER / LOCK CONTROL FUNCTIONS ==========

void ShiftReg_Init(void) {
  pinMode(SHIFT_REG_DATA_PIN, OUTPUT);
  pinMode(SHIFT_REG_CLOCK_PIN, OUTPUT);
  pinMode(SHIFT_REG_LATCH_PIN, OUTPUT);
  
  // All LOW = all locked (SOLENOID_ACTIVE_HIGH true → LOW = locked)
  for (int i = 0; i < NUM_SHIFT_REGISTERS; i++) {
    vending.shift_reg_state[i] = 0x00000000;
  }
  ShiftReg_Write(vending.shift_reg_state, NUM_SHIFT_REGISTERS);
  
  AddLog(LOG_LEVEL_INFO, PSTR("SHIFT_REG: Initialized %d registers — all outputs LOW (locked)"), NUM_SHIFT_REGISTERS);
}

void ShiftReg_Write(uint32_t* data, uint8_t num_registers) {
  digitalWrite(SHIFT_REG_LATCH_PIN, LOW);
  for (int reg = num_registers - 1; reg >= 0; reg--) {
    for (int bit = 7; bit >= 0; bit--) {
      digitalWrite(SHIFT_REG_CLOCK_PIN, LOW);
      bool bit_value = (data[reg] >> bit) & 0x01;
      digitalWrite(SHIFT_REG_DATA_PIN, bit_value ? HIGH : LOW);
      digitalWrite(SHIFT_REG_CLOCK_PIN, HIGH);
    }
  }
  digitalWrite(SHIFT_REG_LATCH_PIN, HIGH);
  digitalWrite(SHIFT_REG_LATCH_PIN, LOW);
}

void ShiftReg_SetBit(uint8_t bit_position, bool state) {
  if (bit_position >= (NUM_SHIFT_REGISTERS * 8)) {
    AddLog(LOG_LEVEL_ERROR, PSTR("SHIFT_REG: Invalid bit position %d (max %d)"), 
      bit_position, (NUM_SHIFT_REGISTERS * 8) - 1);
    return;
  }
  uint8_t register_index = bit_position / 8;
  uint8_t bit_index = bit_position % 8;
  if (state) vending.shift_reg_state[register_index] |=  (1 << bit_index);
  else       vending.shift_reg_state[register_index] &= ~(1 << bit_index);
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

  // FIX 3: Clear ALL outputs first — ensures only ONE solenoid is ever HIGH at a time.
  // Previously this called LockBoxKey(previous) which only cleared one bit.
  // Since ShiftReg_SetBit uses |= (OR), bits from old calls accumulated,
  // causing multiple solenoids to fire. Zeroing all registers first prevents this.
  for (int i = 0; i < NUM_SHIFT_REGISTERS; i++) {
    vending.shift_reg_state[i] = 0x00000000;
  }

  uint8_t bit_position = box_id - 1; // Box 1 = bit 0, Box 2 = bit 1, etc.
  ShiftReg_SetBit(bit_position, SOLENOID_ACTIVE_HIGH); // set only this bit HIGH

  vending.unlocked_box_id = box_id;
  vending.unlock_start_ms = millis();

  AddLog(LOG_LEVEL_INFO, PSTR("KEY: ✓ Unlocked Box %d (bit %d, %dms)"), box_id, bit_position, UNLOCK_DURATION_MS);
}

void LockBoxKey(uint8_t box_id) {
  if (box_id < 1 || box_id > vending.box_count) return;
  uint8_t bit_position = box_id - 1;
  ShiftReg_SetBit(bit_position, !SOLENOID_ACTIVE_HIGH);
  if (vending.unlocked_box_id == box_id) {
    vending.unlocked_box_id = 0;
    vending.unlock_start_ms = 0;
  }
  AddLog(LOG_LEVEL_INFO, PSTR("KEY: ✗ Locked Box %d (bit %d)"), box_id, bit_position);
}

void LockAllBoxes(void) {
  for (int i = 0; i < NUM_SHIFT_REGISTERS; i++) {
    vending.shift_reg_state[i] = 0x00000000;
  }
  ShiftReg_Write(vending.shift_reg_state, NUM_SHIFT_REGISTERS);
  vending.unlocked_box_id = 0;
  vending.unlock_start_ms = 0;
  AddLog(LOG_LEVEL_INFO, PSTR("KEY: All boxes locked"));
}

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

void Motor_Init(void) {
  pinMode(MOTOR_STEP_PIN,   OUTPUT);
  pinMode(MOTOR_DIR_PIN,    OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
  motor_position_offset = 0;
  AddLog(LOG_LEVEL_INFO, PSTR("MOTOR: DRV8825 initialized on STEP=%d DIR=%d EN=%d"),
    MOTOR_STEP_PIN, MOTOR_DIR_PIN, MOTOR_ENABLE_PIN);
}

static void Motor_Rotate(uint16_t steps, bool clockwise) {
  if (steps == 0) return;
  digitalWrite(MOTOR_DIR_PIN,    clockwise ? HIGH : LOW);
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  delayMicroseconds(10);
  for (uint16_t s = 0; s < steps; s++) {
    digitalWrite(MOTOR_STEP_PIN, HIGH);
    delayMicroseconds(MOTOR_STEP_DELAY_US);
    digitalWrite(MOTOR_STEP_PIN, LOW);
    delayMicroseconds(MOTOR_STEP_DELAY_US);
  }
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
}

void Motor_DoAction(MotorAction_t action) {
  switch (action) {
    case COIN_ACCEPT:
      Motor_Rotate(MOTOR_STEPS_PER_REV, true);
      motor_position_offset += MOTOR_STEPS_PER_REV;
      AddLog(LOG_LEVEL_INFO, PSTR("MOTOR: COIN_ACCEPT — 360° CW (%d steps, offset=%d)"), MOTOR_STEPS_PER_REV, motor_position_offset);
      break;
    case COIN_REJECT:
      Motor_Rotate(MOTOR_STEPS_PER_REV, false);
      motor_position_offset -= MOTOR_STEPS_PER_REV;
      AddLog(LOG_LEVEL_INFO, PSTR("MOTOR: COIN_REJECT — 360° CCW (%d steps, offset=%d)"), MOTOR_STEPS_PER_REV, motor_position_offset);
      break;
    case MOTOR_RESET: {
      if (motor_position_offset == 0) { AddLog(LOG_LEVEL_INFO, PSTR("MOTOR: RESET — already at home")); return; }
      bool go_ccw = (motor_position_offset > 0);
      uint16_t steps_back = (uint16_t)(motor_position_offset < 0 ? -motor_position_offset : motor_position_offset);
      Motor_Rotate(steps_back, !go_ccw);
      AddLog(LOG_LEVEL_INFO, PSTR("MOTOR: RESET — returned %d steps %s to home"), steps_back, go_ccw ? "CCW" : "CW");
      motor_position_offset = 0;
      break;
    }
  }
}


// ========== LCD 2004 FUNCTIONS ==========

void LCD_Init(void) {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  vending.lcd_initialized = true;
  AddLog(LOG_LEVEL_INFO, PSTR("LCD: Initialized 20x4 at I2C 0x%02X (SDA=%d SCL=%d)"), LCD_I2C_ADDR, I2C_SDA_PIN, I2C_SCL_PIN);
  LCD_WriteText(0, 0, "  Honey Vending     ");
  LCD_WriteText(1, 0, "   Starting...      ");
}

void LCD_WriteText(uint8_t row, uint8_t col, const char* text) {
  if (!vending.lcd_initialized) return;
  if (row >= LCD_ROWS || col >= LCD_COLS) return;
  uint8_t available = LCD_COLS - col;
  char padded[LCD_COLS + 1];
  memset(padded, ' ', LCD_COLS);
  memcpy(padded, text, strnlen(text, available));
  padded[available] = '\0';
  lcd.setCursor(col, row);
  lcd.print(padded);
}

void LCD_Clear(void) {
  if (!vending.lcd_initialized) return;
  lcd.clear();
  AddLog(LOG_LEVEL_INFO, PSTR("LCD: Screen cleared"));
}

void CentsToLCDString(uint32_t cents, char* buffer, size_t len) {
  snprintf(buffer, len, "%lu,%02lu EUR", (unsigned long)(cents / 100), (unsigned long)(cents % 100));
}

void HoneyVending_UpdateLCD(void) {
  if (!vending.lcd_initialized) return;
  if (vending.selected_box_id == 0) {
    LCD_WriteText(0, 0, "   Honig Automat    ");
    LCD_WriteText(1, 0, "                    ");
    LCD_WriteText(2, 0, " Bitte Box waehlen  ");
    LCD_WriteText(3, 0, "   zum Starten...   ");
    return;
  }
  uint32_t price_cents    = vending.box_price[vending.selected_box_id - 1];
  uint32_t inserted_cents = vending.total_cents;
  uint32_t remaining      = (inserted_cents < price_cents) ? (price_cents - inserted_cents) : 0;
  char amount[14], line[LCD_COLS + 1];
  snprintf(line, sizeof(line), "Box %d selected     ", vending.selected_box_id);
  LCD_WriteText(0, 0, line);
  CentsToLCDString(price_cents, amount, sizeof(amount));
  snprintf(line, sizeof(line), "Total:    %s", amount);
  LCD_WriteText(1, 0, line);
  CentsToLCDString(inserted_cents, amount, sizeof(amount));
  snprintf(line, sizeof(line), "Inserted: %s", amount);
  LCD_WriteText(2, 0, line);
  CentsToLCDString(remaining, amount, sizeof(amount));
  snprintf(line, sizeof(line), "Remaining:%s", amount);
  LCD_WriteText(3, 0, line);
}


// ========== MCP23017 BUTTON FUNCTIONS ==========

static void MCP23017_WriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MCP23017_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

static uint8_t MCP23017_ReadReg(uint8_t reg) {
  Wire.beginTransmission(MCP23017_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)MCP23017_I2C_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void MCP23017_Init(void) {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  MCP23017_WriteReg(MCP23017_IODIRA, 0xFF);
  MCP23017_WriteReg(MCP23017_IODIRB, 0xFF);
  MCP23017_WriteReg(MCP23017_GPPUA,  0xFF);
  MCP23017_WriteReg(MCP23017_GPPUB,  0xFF);
  vending.mcp_initialized  = true;
  vending.last_button_state = 0xFFFF;
  AddLog(LOG_LEVEL_INFO, PSTR("MCP23017: Initialized at I2C 0x%02X — 16 inputs with pull-ups (SDA=%d SCL=%d)"),
    MCP23017_I2C_ADDR, I2C_SDA_PIN, I2C_SCL_PIN);
}

uint16_t MCP23017_ReadButtons(void) {
  if (!vending.mcp_initialized) return 0xFFFF;
  uint8_t gpa = MCP23017_ReadReg(MCP23017_GPIOA);
  uint8_t gpb = MCP23017_ReadReg(MCP23017_GPIOB);
  return (uint16_t)(gpa | ((uint16_t)gpb << 8));
}

void HoneyVending_PrintPressedButtons(void) {
  if (!vending.mcp_initialized) { AddLog(LOG_LEVEL_ERROR, PSTR("BUTTONS: MCP23017 not initialized")); return; }
  uint16_t state = MCP23017_ReadButtons();
  bool any_pressed = false;
  AddLog(LOG_LEVEL_INFO, PSTR("BUTTONS: Raw state GPA=0x%02X GPB=0x%02X"),
    (uint8_t)(state & 0xFF), (uint8_t)((state >> 8) & 0xFF));
  for (uint8_t i = 0; i < 16; i++) {
    if (!(state & (1 << i))) {
      uint8_t button_number = i + 1;
      AddLog(LOG_LEVEL_INFO, PSTR("BUTTONS: *** Button %d PRESSED (%s%d) ***"), button_number, (i < 8) ? "GPA" : "GPB", i % 8);
      char lcd_buf[LCD_COLS + 1];
      snprintf(lcd_buf, sizeof(lcd_buf), "Button %d pressed   ", button_number);
      LCD_WriteText(1, 0, lcd_buf);
      any_pressed = true;
    }
  }
  if (!any_pressed) AddLog(LOG_LEVEL_INFO, PSTR("BUTTONS: No buttons currently pressed"));
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
  
  vending.initialized      = true;
  vending.machine_state    = MACHINE_IDLE;
  vending.state_entered_ms = millis();
  
  HoneyVending_LoadBoxCount();
  HoneyVending_LoadStatus();
  HoneyVending_LoadPrices();
  
  ShiftReg_Init();
  Motor_Init();
  LCD_Init();
  MCP23017_Init();

  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Initialized on GPIO%d"), pin);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Device ID: %04X"), (unsigned int)vending.device_id);
  AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Box count: %d (max: %d)"), vending.box_count, MAX_HONEY_BOX_COUNT);
  AddLog(LOG_LEVEL_INFO, PSTR("WORKFLOW: State machine active — IDLE"));
  AddLog(LOG_LEVEL_INFO, PSTR("WORKFLOW: Cancel=#%d | Idle=%ds | CoinTimeout=%ds | VendingUnlock=%dms"),
    CANCEL_BUTTON_ID,
    WORKFLOW_BOX_IDLE_TIMEOUT_MS / 1000,
    WORKFLOW_COIN_TIMEOUT_MS / 1000,
    WORKFLOW_VENDING_UNLOCK_MS);
  
  LCD_WriteText(0, 0, "   Honig Automat    ");
  LCD_WriteText(1, 0, "                    ");
  LCD_WriteText(2, 0, " Bitte Box waehlen  ");
  LCD_WriteText(3, 0, "   zum Starten...   ");
}

void HoneyVending_Every100ms(void) {
  if (!vending.initialized) return;
  if (millis() < 3000) return;

  static uint32_t last_check = 0;
  uint32_t now = millis();
  if (now - last_check < 100) return;
  last_check = now;
  
  CheckUnlockTimeout();
  HoneyVending_WorkflowCheckTimeouts();
  
  if (vending.pulse_count > 0 && (now - vending.last_pulse_ms) >= COIN_TIMEOUT_MS) {
    uint32_t pulses    = vending.pulse_count;
    uint32_t coin_value = PulsesToCents(pulses);
    
    if (coin_value > 0) {
      vending.total_cents    += coin_value;
      vending.last_coin_cents = coin_value;
      vending.coins_detected++;
      
      char coin_str[16], total_str[16];
      CentsToEuroString(coin_value, coin_str, sizeof(coin_str));
      CentsToEuroString(vending.total_cents, total_str, sizeof(total_str));
      AddLog(LOG_LEVEL_INFO, PSTR("VENDING: *** COIN DETECTED: %s (pulses=%lu) ***"), coin_str, (unsigned long)pulses);
      AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Total: %s"), total_str);

      HoneyVending_UpdateLCD();
      HoneyVending_DisplayValues();
      HoneyVending_PublishSystemMQTT();

      if (vending.machine_state == MACHINE_BOX_SELECTED) {
        HoneyVending_SetMachineState(MACHINE_COIN_INSERTED);
      } else if (vending.machine_state == MACHINE_COIN_INSERTED) {
        vending.state_entered_ms = millis();
        char ms_str[40];
        snprintf(ms_str, sizeof(ms_str), "coin_inserted:%lu", (unsigned long)vending.total_cents);
        HoneyVending_PublishMachineStatus(ms_str);
      }
      
      if (vending.selected_box_id > 0 && !vending.honey_available) {
        uint32_t target_price = vending.box_price[vending.selected_box_id - 1];
        if (vending.total_cents >= target_price) {
          vending.honey_available = true;
          HoneyVending_SetMachineState(MACHINE_VENDING);
        }
      }
    } else {
      AddLog(LOG_LEVEL_INFO, PSTR("VENDING: Unknown coin (%lu pulses) - ignoring"), (unsigned long)pulses);
    }
    vending.pulse_count = 0;
  }

  if (vending.mcp_initialized) {
    static uint32_t button_last_ms[16] = {0};
    uint16_t current_state = MCP23017_ReadButtons();
    uint16_t newly_pressed = (~current_state) & vending.last_button_state;

    if (__builtin_popcount(newly_pressed) > 3) {
      vending.last_button_state = current_state;
      return;
    }

    if (newly_pressed) {
      for (uint8_t i = 0; i < 16; i++) {
        if (newly_pressed & (1 << i)) {
          if ((now - button_last_ms[i]) < 200) continue;
          button_last_ms[i] = now;

          uint8_t button_number = i + 1;
          AddLog(LOG_LEVEL_INFO, PSTR("BUTTONS: *** Button %d PRESSED (%s%d) ***"),
            button_number, (i < 8) ? "GPA" : "GPB", i % 8);

          if (button_number == CANCEL_BUTTON_ID) {
            AddLog(LOG_LEVEL_INFO, PSTR("WORKFLOW: Cancel button (#%d) pressed"), CANCEL_BUTTON_ID);
            HoneyVending_WorkflowCancel();
          } else {
            HoneyVending_WorkflowSelectBox(button_number);
          }

          char lcd_buf[LCD_COLS + 1];
          snprintf(lcd_buf, sizeof(lcd_buf), "Button %d pressed   ", button_number);
          LCD_WriteText(1, 0, lcd_buf);
        }
      }
    }
    vending.last_button_state = current_state;
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
  if (!vending.discovery_sent) HoneyVending_PublishDiscovery();
  HoneyVending_PublishAllBoxesMQTT();
  HoneyVending_PublishSystemMQTT();
  HoneyVending_PublishMachineStatus("idle");
}


bool Xdrv99(uint32_t function) {
  bool result = false;
  switch (function) {
    case FUNC_INIT:              HoneyVending_Init();       break;
    case FUNC_EVERY_100_MSECOND: HoneyVending_Every100ms(); break;
    case FUNC_EVERY_250_MSECOND: HoneyVending_Every250ms(); break;
    case FUNC_COMMAND:           result = DecodeCommand(kHoneyVendingCommands, HoneyVendingCommand); break;
    case FUNC_WEB_ADD_MAIN_BUTTON: HoneyVending_ShowWebButton(); break;
    case FUNC_MQTT_INIT:         HoneyVending_MqttConnected(); break;
  }
  return result;
}

#endif  // USE_HONEY_WENDING_MACHINE
