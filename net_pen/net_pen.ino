// net_pen.ino START

// ==========================================================================
// ==                            Includes                                  ==
// ==========================================================================
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>      // Required for esp_wifi_set_mac
#include <esp32-hal-ledc.h> // For PWM control (ledc functions)
#include <driver/ledc.h>    // For ledc configuration structs/enums (if needed)
#include "Adafruit_INA3221.h" // For INA3221 power sensor
#include <Wire.h>          // For I2C communication (for INA3221)
#include "config.h"        // Include shared configuration

// ==========================================================================
// ==                      Global Variables                                ==
// ==========================================================================

// --- State Variables ---
SensorData_t currentSensorData;         // Holds the latest sensor readings to be sent
ControlData_t lastReceivedControlData;   // Stores the latest control commands received

// --- Hardware & Timing ---
const bool IS_PHYSICAL_HARDWARE_PRESENT = true; // Set to false to simulate sensors/PWM
Adafruit_INA3221 powerMonitor;          // INA3221 sensor object instance
unsigned long lastSensorSendTimeMillis = 0; // Timestamp for the last sensor data transmission
// unsigned long lastRampUpdateTimeMillis = 0; // Uncomment if needed for continuous ramping loop

// ==========================================================================
// ==                      Function Prototypes                             ==
// ==========================================================================

// --- Initialization ---
void initializeDefaultValues();
void initializePwmOutput();
void initializeEspNow();
void initializeIna3221Sensor();

// --- ESP-NOW Callbacks ---
struct esp_now_recv_info; // Forward declaration for callback signature
void onEspNowDataReceived(const esp_now_recv_info *macInfo, const uint8_t *incomingData, int dataLength);
void onEspNowDataSent(const uint8_t *peerMacAddress, esp_now_send_status_t status);

// --- Hardware Interface ---
void initializePwmOutput();
void initializeIna3221Sensor();
void applyPwmDutyEmergencyLight(int duty);
void turnOffEmergencyLight();
int calculateLogarithmicDutyFromPercent(float percentage);
int calculateLogarithmicPercentFromDuty(int duty);

// --- Sensor & Logic ---
void readSensorValues();
void manageEmergencyLightBasedOnMainLightStatus();
void rampEmergencyLightLogarithmic();
uint8_t mapFloatToUint8Range(float value, float inputMin, float inputMax, uint8_t outputMin, uint8_t outputMax);

// --- Communication ---
void sendSensorDataViaEspNow();


// ==========================================================================
// ==                         Setup Function                               ==
// ==========================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial port to connect (optional, good for debugging)
  Serial.println("\nNet Pen ESP32 Starting...");

  Serial.println("Initializing Default Values...");
  initializeDefaultValues();

  // Initialize hardware only if enabled
  if (IS_PHYSICAL_HARDWARE_PRESENT) {
    Serial.println("Initializing PWM Output...");
    initializePwmOutput();
    Serial.println("Initializing INA3221 Sensor...");
    initializeIna3221Sensor();
  } else {
      Serial.println("Physical hardware (PWM/INA3221) is disabled via IS_PHYSICAL_HARDWARE_PRESENT flag.");
  }

  Serial.println("Initializing ESP-NOW...");
  initializeEspNow();

  Serial.println("Setup Complete.");
}

// ==========================================================================
// ==                          Main Loop                                   ==
// ==========================================================================

void loop() {
  unsigned long currentMillis = millis();

  // Task 1: Periodically read sensor values and send them via ESP-NOW
  if (currentMillis - lastSensorSendTimeMillis >= NET_PEN_SEND_INTERVAL_MS) {
    lastSensorSendTimeMillis = currentMillis;

    // Read sensors and execute related logic (like managing emergency light)
    // This happens *before* sending the data so the sent data is fresh.
    if (IS_PHYSICAL_HARDWARE_PRESENT) {
      readSensorValues();
    }

    // Send the potentially updated sensor data
    sendSensorDataViaEspNow();
  }

  // Note: Ramping logic is currently triggered within readSensorValues/manageEmergencyLight.
  // If a continuous background ramp independent of sensor reads was needed,
  // it would involve checking `currentMillis - lastRampUpdateTimeMillis` here
  // and calling a dedicated ramp update function.
}

// ==========================================================================
// ==                   Initialization Functions                           ==
// ==========================================================================

/**
 * @brief Initializes global variables to default values defined in config.h.
 */
void initializeDefaultValues() {
  // Initialize Control Data Defaults (these are the initial targets before receiving commands)
  lastReceivedControlData.targetBrightnessMain = DEFAULT_TARGET_BRIGHTNESS_MAIN;
  lastReceivedControlData.targetBrightnessEmergency = DEFAULT_TARGET_BRIGHTNESS_EMERGENCY;
  // lastReceivedControlData.rampRatePercentPerSec = DEFAULT_RAMP_RATE_PERCENTAGE_PER_SEC; // Uncomment if used

  // Initialize Sensor Data Defaults (representing initial state)
  // currentSensorData.voltageMain = 0.0; // Uncomment if used
  currentSensorData.currentMain = 0.0;
  // currentSensorData.voltageEmergency = 0.0; // Uncomment if used
  // currentSensorData.currentEmergency = 0.0; // Uncomment if used
  currentSensorData.actualBrightnessMain = 0;      // Assume lights start off
  currentSensorData.actualBrightnessEmergency = 0; // Assume lights start off
}

/**
 * @brief Configures the LEDC peripheral for PWM output on the emergency light pin.
 */
void initializePwmOutput() {
  pinMode(EMERGENCY_LIGHT_PIN, OUTPUT); // Set pin as output

  // Configure LEDC timer with specified frequency and resolution
  // Note: ledcSetup is deprecated, use ledcAttach/ledcChangeFrequency etc. if needed
  // ledcSetup(EMERGENCY_LIGHT_PWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS); // Old way

  // Attach the GPIO pin to the LEDC channel
  // ledcAttachPin(EMERGENCY_LIGHT_PIN, EMERGENCY_LIGHT_PWM_CHANNEL); // Old way

  // Use newer API: Attach pin to driver, then configure channel and attach pin to channel
  ledcAttach(EMERGENCY_LIGHT_PIN, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcAttachChannel(EMERGENCY_LIGHT_PIN, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS, EMERGENCY_LIGHT_PWM_CHANNEL);

  ledcWrite(EMERGENCY_LIGHT_PWM_CHANNEL, 0); // Ensure light starts OFF

  Serial.printf("PWM Output Initialized: Pin %d, Channel %d, Freq %d Hz, Res %d bits\n",
                EMERGENCY_LIGHT_PIN, EMERGENCY_LIGHT_PWM_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
}

/**
 * @brief Initializes ESP-NOW communication, sets the MAC address, and adds the Control Room as a peer.
 */
void initializeEspNow() {
  // --- WiFi & MAC Setup ---
  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station (required for setting MAC)

  // Set the custom MAC Address for this Net Pen ESP32
  esp_err_t setMacResult = esp_wifi_set_mac(WIFI_IF_STA, NET_PEN_MAC);
  if (setMacResult != ESP_OK) {
    Serial.print("Error setting MAC address: ");
    Serial.println(esp_err_to_name(setMacResult));
  } else {
    // Verify the MAC address after setting it
    Serial.print("MAC Address set to: ");
    uint8_t currentMacAddress[6];
    esp_wifi_get_mac(WIFI_IF_STA, currentMacAddress);
    for (int i = 0; i < 6; ++i) {
      if (currentMacAddress[i] < 0x10) Serial.print("0"); // Add leading zero if needed
      Serial.print(currentMacAddress[i], HEX);
      if (i < 5) Serial.print(":");
    }
    Serial.println();
  }

  // --- ESP-NOW Channel & Init ---
  // Force ESP-NOW onto the configured channel. This ensures communication
  // even if WiFi is not used or connects on a different channel.
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
  esp_wifi_set_channel(WIFI_AND_ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
  Serial.printf("ESP-NOW channel forced to: %d\n", WIFI_AND_ESP_NOW_CHANNEL);


  // Initialize ESP-NOW service
  if (esp_now_init() != ESP_OK) {
    Serial.println("CRITICAL ERROR: ESP-NOW Initialization Failed");
    ESP.restart(); // Restart if ESP-NOW fails to initialize
    return;
  }
  Serial.println("ESP-NOW Initialized.");

  // --- Register Callbacks & Peer ---
  // Register ESP-NOW send and receive callback functions
  esp_now_register_send_cb(onEspNowDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(onEspNowDataReceived)); // Cast needed for signature

  // Configure and add the Control Room ESP32 as a peer
  esp_now_peer_info_t controlRoomPeerInfo;
  memset(&controlRoomPeerInfo, 0, sizeof(controlRoomPeerInfo)); // Zero initialize structure
  memcpy(controlRoomPeerInfo.peer_addr, CONTROL_ROOM_MAC, 6); // Set peer MAC address
  controlRoomPeerInfo.channel = WIFI_AND_ESP_NOW_CHANNEL;     // Use configured channel
  controlRoomPeerInfo.encrypt = false;                      // No encryption
  controlRoomPeerInfo.ifidx = WIFI_IF_STA;                  // Interface index

  if (esp_now_add_peer(&controlRoomPeerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer (Control Room). Sending will fail.");
  } else {
    Serial.printf("ESP-NOW Peer Added (Control Room). MAC: %02X:%02X:%02X:%02X:%02X:%02X, Channel: %d\n",
                  controlRoomPeerInfo.peer_addr[0], controlRoomPeerInfo.peer_addr[1], controlRoomPeerInfo.peer_addr[2],
                  controlRoomPeerInfo.peer_addr[3], controlRoomPeerInfo.peer_addr[4], controlRoomPeerInfo.peer_addr[5],
                  controlRoomPeerInfo.channel);
  }
}

/**
 * @brief Initializes the I2C communication and the INA3221 sensor.
 */
void initializeIna3221Sensor() {
  // Define I2C pins explicitly if they are non-standard for your board
   const int I2C_SDA_PIN = 8;
   const int I2C_SCL_PIN = 9;
   Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); // Initialize I2C with specified pins
   Serial.printf("I2C Initialized: SDA=%d, SCL=%d\n", I2C_SDA_PIN, I2C_SCL_PIN);

  // Attempt to initialize the INA3221 sensor at its I2C address
  const uint8_t INA3221_I2C_ADDRESS = 0x40; // Default address, check your module
  if (!powerMonitor.begin(INA3221_I2C_ADDRESS, &Wire)) {
    Serial.println("CRITICAL ERROR: INA3221 sensor not found at address 0x" + String(INA3221_I2C_ADDRESS, HEX));
    Serial.println("Check I2C wiring and sensor address.");
    // Halt execution as sensor readings are critical
    while (1) { delay(1000); }
  }

  Serial.println("INA3221 sensor initialized successfully.");

  // Configure sensor settings (optional, defaults may be okay)
  powerMonitor.setAveragingMode(INA3221_AVG_16_SAMPLES); // Set sample averaging

  // Configure shunt resistor values (must match hardware) for each channel used
  const float SHUNT_RESISTANCE_OHMS = 0.05f; // Example value from config.h comments
  // Assuming Ch0=Main Light, Ch1=Emergency Light (or unused), Ch2=Unused
  powerMonitor.setShuntResistance(0, SHUNT_RESISTANCE_OHMS); // Channel 1 (index 0)
  // powerMonitor.setShuntResistance(1, SHUNT_RESISTANCE_OHMS); // Channel 2 (index 1) - Uncomment if used
  // powerMonitor.setShuntResistance(2, SHUNT_RESISTANCE_OHMS); // Channel 3 (index 2) - Uncomment if used
  Serial.printf("INA3221 Shunt Resistance configured to %.3f Ohms (for used channels).\n", SHUNT_RESISTANCE_OHMS);

  // Optional: Configure power valid window (if using alerts)
  // powerMonitor.setPowerValidLimits(3.0f, 15.0f);
}


// ==========================================================================
// ==                       ESP-NOW Callbacks                              ==
// ==========================================================================

/**
 * @brief Callback function executed when ESP-NOW data is received.
 * @param macInfo Information about the received packet (includes sender MAC).
 * @param incomingData Pointer to the received data buffer.
 * @param dataLength Length of the received data.
 */
void onEspNowDataReceived(const esp_now_recv_info *macInfo, const uint8_t *incomingData, int dataLength) {
  // Check if the sender MAC matches the expected Control Room MAC
  if (memcmp(macInfo->src_addr, CONTROL_ROOM_MAC, 6) == 0) {
    // Check if the data length matches the expected ControlData structure size
    if (dataLength == sizeof(ControlData_t)) {
      // Copy received data into the global state variable
      memcpy(&lastReceivedControlData, incomingData, sizeof(lastReceivedControlData));

      // Print received control commands (optional, for debugging)
      Serial.print("ESP-NOW RX: Control Data Received: Target Main=");
      Serial.print(lastReceivedControlData.targetBrightnessMain);
      Serial.print("%, Target Emerg=");
      Serial.print(lastReceivedControlData.targetBrightnessEmergency);
      Serial.println("%");
      // Serial.print("%, Ramp Rate="); // Uncomment if used
      // Serial.println(lastReceivedControlData.rampRatePercentPerSec);

      // Clamp received brightness values to ensure they are within the valid 0-100 range
      lastReceivedControlData.targetBrightnessMain = constrain(lastReceivedControlData.targetBrightnessMain, 0, 100);
      lastReceivedControlData.targetBrightnessEmergency = constrain(lastReceivedControlData.targetBrightnessEmergency, 0, 100);

      // Clamp ramp rate if used
      // if (lastReceivedControlData.rampRatePercentPerSec <= 0) {
      //   lastReceivedControlData.rampRatePercentPerSec = 1.0; // Prevent division by zero or invalid rates
      // }

      // IMPORTANT: Immediately act on the received commands if hardware is present.
      // This makes the system more responsive than waiting for the next loop iteration.
      if (IS_PHYSICAL_HARDWARE_PRESENT) {
           Serial.println("-> Processing received commands immediately.");
           manageEmergencyLightBasedOnMainLightStatus(); // Update light state based on new targets
      }

    } else {
      Serial.printf("WARN: Received data from Control Room MAC, but incorrect size: %d bytes (Expected: %d bytes)\n", dataLength, sizeof(ControlData_t));
    }
  } else {
    // Log if data is received from an unexpected MAC address
    Serial.print("Ignoring received ESP-NOW data from unexpected MAC: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(macInfo->src_addr[i], HEX); if (i < 5) Serial.print(":");
    }
    Serial.printf(" (%d bytes)\n", dataLength);
  }
}

/**
 * @brief Callback function executed after attempting to send ESP-NOW data.
 * @param peerMacAddress MAC address of the recipient.
 * @param status Status of the send operation (ESP_NOW_SEND_SUCCESS or ESP_NOW_SEND_FAIL).
 */
void onEspNowDataSent(const uint8_t *peerMacAddress, esp_now_send_status_t status) {
  // Log only failures to reduce serial output noise
   if (status != ESP_NOW_SEND_SUCCESS) {
       Serial.print("!!! ESP-NOW Send Failure to MAC: ");
       for(int i=0; i<6; i++) { Serial.print(peerMacAddress[i], HEX); if(i<5) Serial.print(":"); }
       Serial.print(" Status Code: "); Serial.println(status);
   }
   // else { Serial.println("ESP-NOW Sensor data sent successfully."); } // Optional success message
}

// ==========================================================================
// ==               Hardware Interface Functions (PWM, Sensor)             ==
// ==========================================================================

/**
 * @brief Sets the PWM duty cycle for the emergency light LEDC channel.
 *        Updates the `currentSensorData.actualBrightnessEmergency` based on the applied duty.
 * @param duty The desired PWM duty cycle (0 to PWM_MAX_DUTY).
 */
void applyPwmDutyEmergencyLight(int duty) {
  int clampedDuty = constrain(duty, 0, PWM_MAX_DUTY); // Ensure duty is within valid PWM range

  // Write the duty cycle to the configured LEDC channel
  ledcWrite(EMERGENCY_LIGHT_PWM_CHANNEL, clampedDuty);

  // Update the globally stored actual brightness percentage based on the duty cycle applied
  // Use the inverse function to estimate perception
  currentSensorData.actualBrightnessEmergency = calculateLogarithmicPercentFromDuty(clampedDuty);

  // Debugging output (optional)
  // Serial.printf("  applyPwmDuty: Duty=%d -> Actual Brightness=%d%%\n", clampedDuty, currentSensorData.actualBrightnessEmergency);
}

/**
 * @brief Turns off the emergency light immediately by setting PWM duty to 0.
 */
void turnOffEmergencyLight() {
  // Only act if the light is currently perceived as on
  if (currentSensorData.actualBrightnessEmergency > 0) {
    applyPwmDutyEmergencyLight(0); // Set duty cycle to 0
    Serial.println("Turned off emergency light (Duty=0).");
  }
}

/**
 * @brief Calculates the PWM duty cycle corresponding to a desired brightness percentage,
 *        applying gamma correction for a more linear perceived brightness change.
 * @param percentage Desired brightness (0.0 to 100.0).
 * @return Corresponding PWM duty cycle (0 to PWM_MAX_DUTY).
 */
int calculateLogarithmicDutyFromPercent(float percentage) {
  float clampedPercent = constrain(percentage, 0.0f, 100.0f); // Clamp input percentage

  float normalizedBrightness = clampedPercent / 100.0f; // Normalize to 0.0 - 1.0
  // Apply gamma correction: brightness = normalized ^ gamma
  float correctedBrightness = pow(normalizedBrightness, BRIGHTNESS_GAMMA_CORRECTION);
  // Scale the corrected brightness to the PWM duty cycle range
  int dutyCycle = floor(correctedBrightness * PWM_MAX_DUTY);

  return constrain(dutyCycle, 0, PWM_MAX_DUTY); // Ensure duty is within valid range
}

/**
 * @brief Calculates the perceived brightness percentage corresponding to a given PWM duty cycle,
 *        reversing the gamma correction.
 * @param duty Current PWM duty cycle (0 to PWM_MAX_DUTY).
 * @return Estimated perceived brightness percentage (0 to 100).
 */
int calculateLogarithmicPercentFromDuty(int duty) {
  int clampedDuty = constrain(duty, 0, PWM_MAX_DUTY); // Clamp input duty cycle

  float normalizedDuty = (float)clampedDuty / (float)PWM_MAX_DUTY; // Normalize to 0.0 - 1.0
  // Apply inverse gamma correction: normalized = duty ^ (1/gamma)
  float correctedNormalizedBrightness = pow(normalizedDuty, 1.0f / BRIGHTNESS_GAMMA_CORRECTION);
  // Scale the corrected brightness back to percentage
  int percentage = floor(correctedNormalizedBrightness * 100.0f);

  return constrain(percentage, 0, 100); // Ensure percentage is within valid range
}

// ==========================================================================
// ==                      Sensor Reading & Logic                          ==
// ==========================================================================

/**
 * @brief Reads sensor values from INA3221 and triggers emergency light logic.
 *        Updates the `currentSensorData` structure.
 */
void readSensorValues() {
  // --- Read Main Light Sensor (INA3221 Channel 0) ---
  // currentSensorData.voltageMain = powerMonitor.getBusVoltage(0); // Uncomment if voltage needed
  currentSensorData.currentMain = powerMonitor.getCurrentAmps(0); // Read current in Amperes

  // Estimate main light brightness percentage based on current draw
  // Maps current from [0, MaxCurrent] to brightness [0, 100]
  currentSensorData.actualBrightnessMain = mapFloatToUint8Range(
      currentSensorData.currentMain,
      0.0f,                         // Minimum expected current (off)
      MAIN_LIGHT_100PCT_CURRENT_A,  // Current corresponding to 100% brightness (from config.h)
      0,                            // Minimum output brightness %
      100                           // Maximum output brightness %
  );

  // --- Read Emergency Light Sensors (INA3221 Channel 1, if used) ---
  // currentSensorData.voltageEmergency = powerMonitor.getBusVoltage(1);
  // currentSensorData.currentEmergency = powerMonitor.getCurrentAmps(1);
  // Note: `currentSensorData.actualBrightnessEmergency` is updated within `applyPwmDutyEmergencyLight`

  // --- Debug Output ---
  // Serial.printf("Sensor Read: Main Current=%.3f A -> Main Brightness=%d%%\n",
  //               currentSensorData.currentMain, currentSensorData.actualBrightnessMain);

  // --- Execute Control Logic ---
  // Decide and potentially actuate the emergency light based on the main light's status and targets
  manageEmergencyLightBasedOnMainLightStatus();

  // Final state after logic (useful for debugging)
  // Serial.printf(" State Post-Logic: Target M=%d A=%d | Target E=%d A=%d\n",
  //                lastReceivedControlData.targetBrightnessMain, currentSensorData.actualBrightnessMain,
  //                lastReceivedControlData.targetBrightnessEmergency, currentSensorData.actualBrightnessEmergency);
}

/**
 * @brief Implements the core logic for controlling the emergency light.
 *        Turns it on (ramping) if the main light fails, turns it off otherwise.
 */
void manageEmergencyLightBasedOnMainLightStatus() {
  // Determine the desired state of the main light based on the last command
  bool isMainLightSupposedToBeOn = lastReceivedControlData.targetBrightnessMain > MAIN_LIGHT_OFF_THRESHOLD;
  // Determine the actual state of the main light based on sensor readings
  bool isMainLightActuallyOff = currentSensorData.actualBrightnessMain <= MAIN_LIGHT_OFF_THRESHOLD;

  // Print current states for debugging understanding
  // Serial.printf("  ManageEmergencyLight: Main Target=%d%% (Supposed On: %s), Main Actual=%d%% (Actually Off: %s), Emerg Target=%d%%\n",
  //                lastReceivedControlData.targetBrightnessMain, isMainLightSupposedToBeOn ? "Yes" : "No",
  //                currentSensorData.actualBrightnessMain, isMainLightActuallyOff ? "Yes" : "No",
  //                lastReceivedControlData.targetBrightnessEmergency);

  // --- Decision Logic ---
  if (isMainLightSupposedToBeOn && isMainLightActuallyOff) {
    // Condition: Main light commanded ON, but sensor indicates it's OFF (below threshold)
    // This is the primary failure scenario where the emergency light is needed.
    Serial.println("  Condition: Main light failed (Commanded ON, but sensor shows OFF).");
    Serial.println("  -> Action: Ramping Emergency Light to target.");
    rampEmergencyLightLogarithmic(); // Activate and ramp emergency light to its target brightness
  } else {
    // Condition: Main light is operating as expected OR commanded off.
    // Includes:
    // 1. Main commanded ON, Main sensor ON -> Main light working fine.
    // 2. Main commanded OFF, Main sensor OFF -> Main light off as expected.
    // 3. Main commanded OFF, Main sensor ON -> Main light turning off, or sensor issue? (Still turn off emergency)
    // In all these non-failure cases, the emergency light should be OFF.
    // if (!isMainLightSupposedToBeOn) {
    //      Serial.println("  Condition: Main light commanded OFF.");
    // } else { // Implies isMainLightSupposedToBeOn && !isMainLightActuallyOff
    //      Serial.println("  Condition: Main light commanded ON and sensor shows ON.");
    // }
    // Serial.println("  -> Action: Ensuring Emergency Light is OFF.");
    turnOffEmergencyLight(); // Ensure emergency light is turned off
  }
}

/**
 * @brief Smoothly ramps the emergency light brightness from its current level
 *        to the `lastReceivedControlData.targetBrightnessEmergency` over a defined duration,
 *        using gamma correction and an easing function for perceived linearity.
 */
void rampEmergencyLightLogarithmic() {
    float startBrightnessPercent = (float)currentSensorData.actualBrightnessEmergency;
    float targetBrightnessPercent = (float)lastReceivedControlData.targetBrightnessEmergency;
    float brightnessDifferencePercent = startBrightnessPercent - targetBrightnessPercent;

    // Only ramp if the difference exceeds the tolerance threshold
    if (abs(brightnessDifferencePercent) > RAMP_TARGET_TOLERANCE_PERCENT) {

        // Define ramp parameters
        const float stepSizePercent = 0.1f; // Affects smoothness vs number of steps
        int numberOfSteps = (int)(abs(targetBrightnessPercent - startBrightnessPercent) / stepSizePercent);
        numberOfSteps = max(1, numberOfSteps); // Ensure at least one step

        // Calculate delay per step to achieve the target ramp duration
        unsigned long delayPerStepMillis = (EMERGENCY_LIGHT_RAMP_TIME_MS > 0 && numberOfSteps > 0)
                                            ? (EMERGENCY_LIGHT_RAMP_TIME_MS / numberOfSteps) : 0;

        bool isRampingDown = brightnessDifferencePercent > 0;

        Serial.printf("  Starting Ramp %s: From %.1f%% to %.1f%% (%d steps, %lu ms/step, total ~%lu ms)\n",
                       (isRampingDown ? "Down" : "Up"),
                       startBrightnessPercent, targetBrightnessPercent,
                       numberOfSteps, delayPerStepMillis, (unsigned long)numberOfSteps * delayPerStepMillis);

        unsigned long rampStartTime = millis();

        // Execute the ramp loop
        for (int step = 0; step <= numberOfSteps; step++) {
            // Calculate normalized progress 't' (0.0 to 1.0)
            float t = (float)step / (float)numberOfSteps;

            // Apply easing function (e.g., ease-in: starts slow)
            // cosine: 1 - cos(t * PI/2) gives ease-in
            // sine: sin(t * PI / 2) gives ease-out
            // smoothstep: t*t*(3 - 2*t) gives ease-in-out
            float easedT = (1.0f - cos(PI * t / 2.0f)); // Ease-in

            // Calculate the brightness for this step using linear interpolation combined with easing
            float currentRampBrightnessPercent = startBrightnessPercent + (targetBrightnessPercent - startBrightnessPercent) * easedT;

            // Calculate the required PWM duty for this brightness level
            int dutyCycle = calculateLogarithmicDutyFromPercent(currentRampBrightnessPercent);

            // Apply the duty cycle (this also updates currentSensorData.actualBrightnessEmergency)
            applyPwmDutyEmergencyLight(dutyCycle);

            // Delay for the calculated time per step
            if (delayPerStepMillis > 0) {
                 delay(delayPerStepMillis);
            } else if (numberOfSteps > 100) {
                 // If delay is 0 due to very fast ramp/many steps, yield briefly
                 delay(0);
            }
             // Optional: Check for timeout or interrupt condition here
        }
         unsigned long rampEndTime = millis();

        // Set the final target duty precisely after the loop finishes
        int finalDutyCycle = calculateLogarithmicDutyFromPercent(targetBrightnessPercent);
        applyPwmDutyEmergencyLight(finalDutyCycle);

        Serial.printf("  Ramp finished in %lu ms. Final Duty=%d (Target=%.1f%%)\n",
                       (rampEndTime - rampStartTime), finalDutyCycle, targetBrightnessPercent);

    } else {
        // Already within tolerance - ensure duty matches target precisely
        int currentDuty = calculateLogarithmicDutyFromPercent(startBrightnessPercent);
        int targetDuty = calculateLogarithmicDutyFromPercent(targetBrightnessPercent);
        if (currentDuty != targetDuty) {
             // Serial.println("  Within tolerance, but adjusting duty to precise target.");
             applyPwmDutyEmergencyLight(targetDuty);
        } else {
             // Serial.println("  Emergency light brightness already at target (within tolerance).");
        }
    }
}


/**
 * @brief Maps a float value from an input range to a uint8_t output range.
 * @param value The input float value to map.
 * @param inputMin The minimum value of the input range.
 * @param inputMax The maximum value of the input range.
 * @param outputMin The minimum value of the output uint8_t range.
 * @param outputMax The maximum value of the output uint8_t range.
 * @return The mapped uint8_t value, clamped to the output range.
 */
uint8_t mapFloatToUint8Range(float value, float inputMin, float inputMax, uint8_t outputMin, uint8_t outputMax) {
  // Handle potential division by zero if input range is zero width
  if (abs(inputMax - inputMin) < 1e-6) { // Use epsilon for float comparison
    return outputMin; // Or outputMax, depending on desired behavior
  }

  // Perform linear mapping
  float mappedFloat = (value - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;

  // Clamp the result strictly to the output uint8_t range
  if (mappedFloat < (float)outputMin) mappedFloat = (float)outputMin;
  if (mappedFloat > (float)outputMax) mappedFloat = (float)outputMax;

  // Convert to uint8_t, adding 0.5f for rounding
  return static_cast<uint8_t>(mappedFloat + 0.5f);
}


// ==========================================================================
// ==                    Communication Functions                           ==
// ==========================================================================

/**
 * @brief Sends the current sensor data (`currentSensorData`) to the Control Room via ESP-NOW.
 *        Simulates data if `IS_PHYSICAL_HARDWARE_PRESENT` is false.
 */
void sendSensorDataViaEspNow() {
  // --- Data Simulation (if hardware disabled) ---
  if (!IS_PHYSICAL_HARDWARE_PRESENT) {
    // Generate plausible random data for testing without hardware
    randomSeed(micros()); // Seed with microsecond timer for better randomness
    // Simulate main light potentially being on or off
    bool simMainOn = (random(0, 100) < lastReceivedControlData.targetBrightnessMain); // Higher chance if target is high
    if (simMainOn) {
        currentSensorData.actualBrightnessMain = random(MAIN_LIGHT_OFF_THRESHOLD + 1, 101); // Simulate ON state
        currentSensorData.currentMain = ((float)currentSensorData.actualBrightnessMain / 100.0f) * MAIN_LIGHT_100PCT_CURRENT_A * (random(90,111)/100.0f); // Current based on brightness +-10%
    } else {
        currentSensorData.actualBrightnessMain = random(0, MAIN_LIGHT_OFF_THRESHOLD + 1); // Simulate OFF state
        currentSensorData.currentMain = ((float)currentSensorData.actualBrightnessMain / 100.0f) * MAIN_LIGHT_100PCT_CURRENT_A * (random(50,151)/100.0f); // Low current, maybe noisy
    }

    // Simulate emergency light based on its target (assuming simulation doesn't ramp)
    currentSensorData.actualBrightnessEmergency = lastReceivedControlData.targetBrightnessEmergency; // Simple simulation
    // currentSensorData.currentEmergency = random(0, 201) / 100.0f; // 0.0 to 2.0 A
    // currentSensorData.voltageMain = random(1150, 1250) / 100.0f;    // 11.5 to 12.5 V
    // currentSensorData.voltageEmergency = random(1150, 1250) / 100.0f; // 11.5 to 12.5 V
    // Serial.println("Sending SIMULATED sensor data via ESP-NOW.");
  } else {
    // Serial.println("Sending ACTUAL sensor data via ESP-NOW."); // Optional message
  }

  // --- Send Data ---
  // Transmit the currentSensorData structure to the Control Room's MAC address
  esp_err_t sendResult = esp_now_send(CONTROL_ROOM_MAC, (uint8_t *)&currentSensorData, sizeof(currentSensorData));

  // Check initiation result (actual success/failure is confirmed in the callback)
  if (sendResult != ESP_OK) {
    Serial.print("Error initiating ESP-NOW send: ");
    Serial.println(esp_err_to_name(sendResult));
  }
}

// net_pen.ino END