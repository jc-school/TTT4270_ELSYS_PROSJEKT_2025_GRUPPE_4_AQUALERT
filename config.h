#ifndef CONFIG_H
#define CONFIG_H

// ==========================================================================
// ==                        Aqualert Configuration                        ==
// ==========================================================================
// This file contains configuration settings for both the Control Room and
// Net Pen ESP32 devices used in the Aqualert system.
// Ensure settings are consistent where necessary (e.g., MAC addresses, channel).
// ==========================================================================

#include <stdint.h> // Required for uint8_t type
#include "secrets.h" // Include sensitive data like WiFi credentials

// ==========================================================================
// ==                      Network Configuration                           ==
// ==========================================================================

// --- MAC Addresses ---
// NOTE: These addresses are programmatically set on the ESP32s.
//       They MUST be unique on your network.
//       Using Locally Administered Addresses (LAA) is recommended
//       (second least significant bit of the first byte is 1, e.g., 02:XX...).
const uint8_t NET_PEN_MAC[6]      = {0x02, 0xDE, 0xAD, 0xBE, 0xEF, 0x01};
const uint8_t CONTROL_ROOM_MAC[6] = {0x02, 0xDE, 0xAD, 0xBE, 0xEF, 0x02};

// --- WiFi & ESP-NOW Channel ---
// IMPORTANT: Both WiFi and ESP-NOW MUST use the same channel for reliable
//            communication, especially for the Control Room receiving ESP-NOW
//            packets while connected to WiFi.
const uint8_t WIFI_AND_ESP_NOW_CHANNEL = 6; // Channel 1-13 valid usually
// TODO: Automatically sync WiFi and ESP-NOW channels in both the Control Room and the Net Pen.

// --- WiFi Credentials ---
// Defined in secrets.h:
// const char* WIFI_SSID = "YourSSID";
// const char* WIFI_PASSWORD = "YourPassword";

// --- mDNS Hostname (Control Room) ---
// Used for accessing the Control Room web server via a friendly name on the
// local network (e.g., http://aqualert.local). Requires mDNS service on the
// client device (Bonjour on Apple, often pre-installed on Linux/Windows 10+).
const char* CONTROL_ROOM_HOSTNAME = "aqualert";


// ==========================================================================
// ==                       Device Defaults                                ==
// ==========================================================================

// Initial target brightness values (%)
const uint8_t DEFAULT_TARGET_BRIGHTNESS_MAIN      = 80;
const uint8_t DEFAULT_TARGET_BRIGHTNESS_EMERGENCY = 80;

// Default ramp rate (% change per second)
// const float DEFAULT_RAMP_RATE_PERCENTAGE_PER_SEC = 5.0f;


// ==========================================================================
// ==                       Timing Configuration (ms)                      ==
// ==========================================================================

// How often the Control Room sends SSE updates to connected web clients
const unsigned long CONTROL_ROOM_EVENT_INTERVAL_MS = 1000;

// How often the Control Room sends a heartbeat ping via SSE
const unsigned long CONTROL_ROOM_HEARTBEAT_INTERVAL_MS = 5000;

// How often the Net Pen reads sensors and sends data via ESP-NOW
const unsigned long NET_PEN_SEND_INTERVAL_MS = 1000;


// ==========================================================================
// ==           ESP-NOW Data Structures                                    ==
// ==========================================================================

// Structure: Net Pen -> Control Room (Sensor Readings)
typedef struct SensorData {
    // float   voltageMain;               // Volts (V)
    float   currentMain;               // Amperes (A)
    // float   voltageEmergency;          // Volts (V)
    // float   currentEmergency;          // Amperes (A)
    uint8_t actualBrightnessMain;      // Calculated/measured brightness (0-100%)
    uint8_t actualBrightnessEmergency; // Calculated/measured brightness (0-100%)
} SensorData_t; // Using _t suffix is a common convention

// Structure: Control Room -> Net Pen (Control Commands)
typedef struct ControlData {
    uint8_t targetBrightnessMain;      // Desired brightness (0-100%)
    uint8_t targetBrightnessEmergency; // Desired brightness (0-100%)
    // float   rampRatePercentPerSec;     // Speed of brightness change (%/sec)
} ControlData_t;


// ==========================================================================
// ==             Hardware Configuration (Net Pen Device)                  ==
// ==========================================================================

// --- Pin Assignments ---
// const int MAIN_LIGHT_PIN = XX;  // TODO: Define if main light control is added
const int EMERGENCY_LIGHT_PIN = 4; // GPIO pin for emergency light PWM output

// --- PWM Configuration ---
// const int MAIN_LIGHT_PWM_CHANNEL = XX;  // TODO: Define if main light control is added
const int EMERGENCY_LIGHT_PWM_CHANNEL = 0; // LEDC channel for emergency light (0-15)

const int PWM_FREQUENCY_HZ    = 500; // PWM frequency in Hertz (Hz)
const int PWM_RESOLUTION_BITS = 12;  // PWM resolution (8-16 bits typical)
constexpr int PWM_MAX_DUTY    = ((1 << PWM_RESOLUTION_BITS) - 1); // Max duty cycle value (e.g., 4095 for 12-bit or 255 for 8-bit)

// --- Sensor Related ---
// Current draw (Amperes) corresponding to 100% main light brightness. Used for estimation.
const float MAIN_LIGHT_100PCT_CURRENT_A = 0.210f;


// ==========================================================================
// ==            Control Logic Configuration (Net Pen Device)              ==
// ==========================================================================

// --- Brightness Ramping ---
// Value used for gamma correction to approximate human perception of brightness
const float BRIGHTNESS_GAMMA_CORRECTION = 2.2f;

// Approximate total time (ms) for emergency light ramp from 0% to 100% or vice-versa
const unsigned long EMERGENCY_LIGHT_RAMP_TIME_MS = 30000;

// Allowed difference (%) between actual and target brightness before ramping stops.
// Prevents oscillation if ramp calculation slightly misses the exact target.
const int RAMP_TARGET_TOLERANCE_PERCENT = 3;

// Brightness threshold (%) below which the main light is considered "off".
// Used for deciding whether to start ramping up from 0 or from the current level.
const int MAIN_LIGHT_OFF_THRESHOLD = 2;

#endif // CONFIG_H