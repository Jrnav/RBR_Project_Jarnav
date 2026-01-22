/**
 * @file main.cpp
 * @brief Red bird racing vehicle control system for electric vehicle using MCP2515 CAN controllers
 * 
 * This program reads accelerator and brake pedal positions, manages vehicle states,
 * anc communicates with motor and battery management systems over CAN bus
 * 
 * @author Arnav
 * @version 1.0
 * @date 2026-21-01
 */

#include <Arduino.h>
#include <MCP2515.h>  

/** 
 * @brief Pin definitions for sensors and outputs
 */
#define APPS_5V      PIN_PC0   ///< Main APPS (5V reference)
#define APPS_3V3     PIN_PC1   ///< Redundant APPS (3.3V reference)
#define BRAKE        PIN_PC3   ///< Brake sensor
#define START_BUTTON PIN_PC4   ///< Driver start button
#define BRAKE_LIGHT  PIN_PD2   ///< Brake light output
#define BUZZER       PIN_PD4   ///< Ready-to-drive buzzer
#define DRIVE_LED    PIN_PD3   ///< Drive mode indicator LED

#define MOTOR_CAN    PIN_PB2   ///< Motor CAN CS
#define BMS_CAN      PIN_PB1   ///< BMS CAN CS
#define DEBUG_CAN    PIN_PB0   ///< Debug CAN CS

/**
 * @brief CAN controller objects
 */
MCP2515 motorCAN(MOTOR_CAN);  ///< Motor torque transmission
MCP2515 bmsCAN(BMS_CAN);      ///< BMS permission reception
MCP2515 debugCAN(DEBUG_CAN);  ///< Telemetry output

/**
 * @brief Configuration constants
 */
const uint16_t BRAKE_THRESHOLD        = 204;      ///< Raw ADC threshold (~20%) for brakes depressed
const uint16_t APPS_FAULTY_THRESHOLD  = 102;      ///< Raw difference threshold for APPS fault
const uint32_t STARTIN_HOLD_TIME      = 2000;     ///< Hold time for STARTIN state (ms)
const uint32_t BUZZIN_TIME            = 2000;     ///< Buzzer duration (ms)
const uint32_t APPS_FAULT_TIME        = 100;      ///< APPS fault persistence time (ms)
const bool     FLIP_MOTOR             = false;    ///< Reverse torque sign if true

/**
 * @brief Vehicle state machine enumeration
 */
enum State {
    INIT,     ///< Idle / waiting
    STARTIN,  ///< Start button + brakes held
    BUZZIN,   ///< Ready-to-drive buzzer phase
    DRIVE     ///< Torque enabled
};

/**
 * @brief Global state variables
 */
State    currentState     = INIT;             ///< Current vehicle state
uint32_t stateStartTime   = 0;                ///< Timestamp of entering current state
uint32_t faultStartTime   = 0;                ///< Timestamp when APPS fault first detected
bool     wasFaulty        = false;            ///< APPS fault currently active
int16_t  torque           = 0;                ///< Current torque request
bool     bms_ready        = false;            ///< BMS permission received

/**
 * @brief Fixed points for pedal to torque mapping (piecewise linear)
 */
const uint16_t PEDAL_POINTS[5] = {0, 100, 200, 300, 1023};  ///< Pedal input points (0-1023)
const int16_t TORQUE_POINTS[5] = {0, 500, 1500, 3000, 32767}; ///< Corresponding torque outputs

/**
 * @brief Linear interpolation function for pedal to torque mapping
 * @param pedal The current pedal reading (0-1023)
 * @return Interpolated torque value
 */
int16_t interpolateTorque(uint16_t pedal) {
  for (int i = 0; i < 4; i++) {  // Loop through segments
    if (pedal <= PEDAL_POINTS[i + 1]) {
      // Linear interpolation between point i and i+1
      int16_t slope = (TORQUE_POINTS[i+1] - TORQUE_POINTS[i]) / (PEDAL_POINTS[i+1] - PEDAL_POINTS[i]);
      return TORQUE_POINTS[i] + (slope * (pedal - PEDAL_POINTS[i]));
    }
  }
  return TORQUE_POINTS[4];  // Max value if pedal > last point
}

/**
 * @brief Initializes pins, states and CAN controllers
 * @return void
 */
void setup() {
  // Initialize pins
  pinMode(APPS_5V, INPUT);
  pinMode(APPS_3V3, INPUT);
  pinMode(BRAKE, INPUT);
  pinMode(START_BUTTON, INPUT);
  pinMode(BRAKE_LIGHT, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(DRIVE_LED, OUTPUT);

  // Initial states
  digitalWrite(BRAKE_LIGHT, LOW);
  digitalWrite(BUZZER, LOW);
  digitalWrite(DRIVE_LED, LOW);

  // For analog reads, default reference (5V)
  
  // Initialize Motor CAN
  motorCAN.reset();
  motorCAN.setBitrate(CAN_500KBPS, MCP_20MHZ);
  motorCAN.setNormalMode();          // ← no argument!

  // Initialize BMS CAN
  bmsCAN.reset();
  bmsCAN.setBitrate(CAN_500KBPS, MCP_20MHZ);
  bmsCAN.setNormalMode();

  // Initialize Debug CAN
  debugCAN.reset();
  debugCAN.setBitrate(CAN_500KBPS, MCP_20MHZ);
  debugCAN.setNormalMode();
}

/**
 * @brief Main program loop: read sensors, run state machine, send CAN messages
 * @return void
 */
void loop() {
  // Read inputs
  uint16_t apps5V = analogRead(APPS_5V);    // 0-1023
  uint16_t apps3V3 = analogRead(APPS_3V3);  // 0-~675 (for 3.3V max)
  uint16_t brake = analogRead(BRAKE);       // 0-1023, assuming 5V
  bool startButton = digitalRead(START_BUTTON) == HIGH;  // Assume active HIGH

  bool brakesPressed = brake > BRAKE_THRESHOLD;
  digitalWrite(BRAKE_LIGHT, brakesPressed ? HIGH : LOW);

  // BMS permission check
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
      if (startButton && brakesPressed) {
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
      }
      break;
    case DRIVE:
      digitalWrite(BUZZER, LOW);
      digitalWrite(DRIVE_LED, HIGH);

      // Check for APPS fault
      uint16_t appsDiff = abs(apps5V - (apps3V3 * 50 / 33));
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
      int16_t avgPedal = (apps5V + (apps3V3 * 50 / 33)) / 2;

      // Improved torque calculation with interpolation
      torque = interpolateTorque(avgPedal);

      // Calculate torque
      if (FLIP_MOTOR) {
        torque = -avgPedal * 16;  // -32768 to 0
      } else {
        torque = avgPedal * 16;   // 0 to 32767
      }

      can_frame torqueFrame;                      // Create a can_frame struct
      
      torqueFrame.can_id  = 0x201;                // Standard 11-bit ID
      torqueFrame.can_dlc = 3;                    // Always 8 bytes

      torqueFrame.data[0] = 0x90;                 // Command identifier
      torqueFrame.data[1] = static_cast<uint8_t>(torque & 0xFF);           // Low byte
      torqueFrame.data[2] = static_cast<uint8_t>((torque >> 8) & 0xFF);    // High byte

      motorCAN.sendMessage(&torqueFrame);         // Send it (pass pointer to struct)
      break;
  }

  static uint32_t last_debug = 0;
  if (millis() - last_debug >= 50) {  // 20 Hz
    last_debug = millis();
    uint16_t appsDiff = abs((int16_t)apps5V - (int16_t)apps3V3 * 50 / 33) ;
    can_frame debugFrame;

    debugFrame.can_id  = 0x500;
    debugFrame.can_dlc = 8;

    debugFrame.data[0] = apps5V & 0xFF;
    debugFrame.data[1] = apps5V >> 8;
    debugFrame.data[2] = apps3V3 & 0xFF;
    debugFrame.data[3] = apps3V3 >> 8;
    debugFrame.data[4] = static_cast<uint8_t>(currentState);
    debugFrame.data[5] = static_cast<uint8_t>(appsDiff >> 8);
    debugFrame.data[6] = static_cast<uint8_t>(appsDiff & 0xFF);
    debugFrame.data[7] = wasFaulty ? 0x01 : 0x00;

    debugCAN.sendMessage(&debugFrame);
  }
}