#include <Arduino.h>
#include <SPI.h>  // Required for all MCP2515 communication
#include <MCP2515.h>  

// Pin definitions (using Arduino pin numbers)
#define APPS_5V PIN_PC0
#define APPS_3V3 PIN_PC1
#define BRAKE PIN_PC3
#define START_BUTTON PIN_PC4
#define BRAKE_LIGHT PIN_PD2
#define BUZZER PIN_PD4
#define DRIVE_LED PIN_PD3

#define MOTOR_CAN  PIN_PB2
#define BMS_CAN    PIN_PB1
#define DEBUG_CAN  PIN_PB0

// Create MCP2515 Objects
MCP2515 motorCAN(MOTOR_CAN);
MCP2515 bmsCAN(BMS_CAN);
MCP2515 debugCAN(DEBUG_CAN);

// Constants
const uint16_t BRAKE_THRESHOLD = 204;  // 20% depression to consider brakes "depressed"
const uint16_t APPS_FAULTY_THRESHOLD = 102;  // 10% difference for fault
const uint32_t STARTIN_HOLD_TIME = 2000;  // 2 seconds
const uint32_t BUZZIN_TIME = 2000;        // 2 seconds
const uint32_t APPS_FAULT_TIME = 100;     // 100 ms
const bool FLIP_MOTOR = false;

// State enum
enum State {
  INIT,
  STARTIN,
  BUZZIN,
  DRIVE
};

// Global variables
State currentState = INIT;
uint32_t stateStartTime = 0;
uint32_t faultStartTime = 0;
bool wasFaulty = false;
int16_t torque = 0;
bool bms_ready = false;


void setup() {
  // Initialize pins
  pinMode(START_BUTTON, INPUT);
  pinMode(BRAKE_LIGHT, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(DRIVE_LED, OUTPUT);

  // Initial states
  digitalWrite(BRAKE_LIGHT, LOW);
  digitalWrite(BUZZER, LOW);
  digitalWrite(DRIVE_LED, LOW);

  // For analog reads, default reference (5V)

  SPI.begin();  // Must be called before any MCP2515 begin()  motorCAN.reset();
  
  // Motor
  motorCAN.reset();
  motorCAN.setBitrate(CAN_500KBPS, MCP_20MHZ);
  motorCAN.setNormalMode();          // ← no argument!

  // BMS
  bmsCAN.reset();
  bmsCAN.setBitrate(CAN_500KBPS, MCP_20MHZ);
  bmsCAN.setNormalMode();

  // Debug
  debugCAN.reset();
  debugCAN.setBitrate(CAN_500KBPS, MCP_20MHZ);
  debugCAN.setNormalMode();
}

void loop() {
  // Read inputs
  uint16_t apps5V = analogRead(APPS_5V);    // 0-1023
  uint16_t apps3V3 = analogRead(APPS_3V3);  // 0-~675 (for 3.3V max)
  uint16_t brake = analogRead(BRAKE);       // 0-1023, assuming 5V
  bool startButton = digitalRead(START_BUTTON) == HIGH;  // Assume active HIGH

  bool brakesPressed = brake > BRAKE_THRESHOLD;

  // Always handle brake light
  digitalWrite(BRAKE_LIGHT, brakesPressed ? HIGH : LOW);

  can_frame rxFrame;

  if (bmsCAN.readMessage(&rxFrame) == MCP2515::ERROR_OK) {
    if (rxFrame.can_id == 0x186040F3 && rxFrame.can_dlc >= 7 && rxFrame.data[6] == 0x50) {
      bms_ready = true;
    }
  }
  // State machine
  switch (currentState) {
    case INIT:
      torque = 0;
      // Transition to STARTIN if start button pushed and brakes depressed
      if (startButton && !brakesPressed) {
        currentState = STARTIN;
        stateStartTime = millis();
      }
      break;
    case STARTIN:
      torque = 0;
      digitalWrite(BUZZER, LOW);
      digitalWrite(DRIVE_LED, LOW);
      // Back to INIT if brakes released or button not held
      if (!startButton || !brakesPressed) {
        currentState = INIT;
      }
      else {
        // If held for >= 2s, to BUZZIN
        if (bms_ready && millis() - stateStartTime >= STARTIN_HOLD_TIME) {
          currentState = BUZZIN;
          stateStartTime = millis();
          bms_ready = false;
        }
      }
      break;
    case BUZZIN:
      torque = 0;
      digitalWrite(BUZZER, LOW);
      digitalWrite(DRIVE_LED, LOW);

      // After 2s, to DRIVE
      if (millis() - stateStartTime >= BUZZIN_TIME) {
        currentState = DRIVE;
        faultStartTime = 0;  // Reset fault timer
      }
      break;
    case DRIVE:
      digitalWrite(BUZZER, LOW);
      digitalWrite(DRIVE_LED, HIGH);

      // Check for APPS fault
      uint16_t appsDiff = abs(apps5V - (apps3V3 * 5 / 3.3));
      if (appsDiff > APPS_FAULTY_THRESHOLD) {
        if (!wasFaulty) {
          faultStartTime = millis();
          wasFaulty = true;
        }
        if (millis() - faultStartTime >= APPS_FAULT_TIME) {
          torque = 0;
          currentState = INIT;
          faultStartTime = 0;
          wasFaulty = false;
          break;  // Exit early
        }
      }
      else {
        faultStartTime = 0; // Reset fault timer
        wasFaulty = false; // Reset fault
      }

      // Calculate average pedal position
      int16_t avgPedal = (apps5V + (apps3V3 * 5 / 3.3)) / 2;

      // Calculate torque
      if (FLIP_MOTOR) {
        torque = -avgPedal * 16;  // -32768 to 0
      } else {
        torque = avgPedal * 16;   // 0 to 32767
      }

      can_frame torqueFrame;                      // Create a can_frame struct
      
      torqueFrame.can_id  = 0x201;                // Standard 11-bit ID
      torqueFrame.can_dlc = 8;                    // Always 8 bytes

      torqueFrame.data[0] = 0x90;                 // Command identifier
      torqueFrame.data[1] = static_cast<uint8_t>(torque & 0xFF);           // Low byte
      torqueFrame.data[2] = static_cast<uint8_t>((torque >> 8) & 0xFF);    // High byte
      torqueFrame.data[3] = 0;
      torqueFrame.data[4] = 0;
      torqueFrame.data[5] = 0;
      torqueFrame.data[6] = 0;
      torqueFrame.data[7] = 0;

      motorCAN.sendMessage(&torqueFrame);         // Send it (pass pointer to struct)
      break;
  }

  static uint32_t last_debug = 0;
  if (millis() - last_debug >= 50) {  // 20 Hz
    last_debug = millis();
    uint16_t appsDiff = abs(apps5V - (apps3V3 * 5 / 3.3));
    can_frame debugFrame;

    debugFrame.can_id  = 0x500;
    debugFrame.can_dlc = 8;

    debugFrame.data[0] = apps5V >> 8;
    debugFrame.data[1] = apps5V & 0xFF;
    debugFrame.data[2] = apps3V3 >> 8;
    debugFrame.data[3] = apps3V3 & 0xFF;
    debugFrame.data[4] = static_cast<uint8_t>(currentState);
    debugFrame.data[5] = static_cast<uint8_t>(appsDiff >> 8);
    debugFrame.data[6] = static_cast<uint8_t>(appsDiff & 0xFF);
    debugFrame.data[7] = wasFaulty ? 1 : 0;

    debugCAN.sendMessage(&debugFrame);
  }
}