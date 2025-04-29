// control_room.ino START

// ==========================================================================
// ==                            Includes                                  ==
// ==========================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>      // Required for esp_wifi_set_mac
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <LittleFS.h>      // Include LittleFS header
#include "config.h"        // Include shared configuration

// ==========================================================================
// ==                      Global Variables                                ==
// ==========================================================================

// --- State Variables ---
SensorData_t lastReceivedSensorData; // Stores the latest data from the net pen
ControlData_t currentControlData;  // Stores the current commands to send / last sent
bool hasControlDataChanged = true; // Flag to send control data initially or when changed

// --- Web Server & SSE Objects ---
AsyncWebServer webServer(80);       // Web server instance on port 80
AsyncEventSource sseEvents("/events"); // Server-Sent Events endpoint

// --- Timing Variables ---
unsigned long lastSseUpdateTimeMillis = 0;    // Timestamp for the last SSE update send
unsigned long lastSseHeartbeatTimeMillis = 0; // Timestamp for the last SSE heartbeat send

// ==========================================================================
// ==                      Function Prototypes                             ==
// ==========================================================================

// --- Initialization ---
void initializeDefaultValues();
void initializeLittleFS();
void initializeWiFiAndEspNow();
void initializeWebServer();

// --- ESP-NOW Callbacks ---
struct esp_now_recv_info; // Forward declaration for callback signature
void onEspNowDataReceived(const esp_now_recv_info * macInfo, const uint8_t *incomingData, int dataLength);
void onEspNowDataSent(const uint8_t *peerMacAddress, esp_now_send_status_t status);

// --- Web Server Handlers ---
void handleRootRequest(AsyncWebServerRequest *request);
void handleCssRequest(AsyncWebServerRequest *request);
void handleJsRequest(AsyncWebServerRequest *request);
void handleChartJsRequest(AsyncWebServerRequest *request);
void handleFaviconRequest(AsyncWebServerRequest *request);
void handleWebControlPost(AsyncWebServerRequest *request);
void handleSseConnect(AsyncEventSourceClient *client);
void handleNotFound(AsyncWebServerRequest *request);

// --- Core Logic ---
void sendControlDataViaEspNow();
void sendStatusUpdateToWebClients();

// ==========================================================================
// ==                         Setup Function                               ==
// ==========================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial); // Wait for serial connection (optional, good for debugging)
    Serial.println("\nControl Room ESP32 Starting...");

    Serial.println("Initializing Default Values...");
    initializeDefaultValues();

    Serial.println("Initializing LittleFS...");
    initializeLittleFS(); // Mount filesystem first

    Serial.println("Initializing WiFi and ESP-NOW...");
    initializeWiFiAndEspNow(); // Setup network connections

    Serial.println("Initializing Web Server...");
    initializeWebServer(); // Setup web interface

    Serial.println("Setup Complete.");
    hasControlDataChanged = true; // Ensure initial state is sent
}

// ==========================================================================
// ==                          Main Loop                                   ==
// ==========================================================================

void loop() {
    unsigned long currentMillis = millis();

    // Task 1: Periodically send status updates to connected web clients via SSE
    if (currentMillis - lastSseUpdateTimeMillis >= CONTROL_ROOM_EVENT_INTERVAL_MS) {
        lastSseUpdateTimeMillis = currentMillis;
        sendStatusUpdateToWebClients();
    }

    // Task 2: Periodically send heartbeat to keep SSE connection alive
    if (currentMillis - lastSseHeartbeatTimeMillis >= CONTROL_ROOM_HEARTBEAT_INTERVAL_MS) {
        lastSseHeartbeatTimeMillis = currentMillis;
        if (sseEvents.count() > 0) { // Only send if clients are connected
             sseEvents.send("ping", "heartbeat", millis()); // Send specific 'heartbeat' event
             // Serial.println("Sent SSE heartbeat"); // Uncomment for debugging
        }
    }

    // Task 3: Send control data via ESP-NOW if it has been changed (e.g., by web request)
    if (hasControlDataChanged) {
        sendControlDataViaEspNow();
        hasControlDataChanged = false; // Reset flag after sending
    }

    // Note: AsyncWebServer runs in the background, no explicit `server.handleClient()` needed in loop
}

// ==========================================================================
// ==                   Initialization Functions                           ==
// ==========================================================================

/**
 * @brief Initializes global variables to default values defined in config.h.
 */
void initializeDefaultValues() {
  // Initialize Control Data Defaults
  currentControlData.targetBrightnessMain = DEFAULT_TARGET_BRIGHTNESS_MAIN;
  currentControlData.targetBrightnessEmergency = DEFAULT_TARGET_BRIGHTNESS_EMERGENCY;
  // currentControlData.rampRatePercentPerSec = DEFAULT_RAMP_RATE_PERCENTAGE_PER_SEC; // Uncomment if used

  // Initialize Sensor Data Defaults (to indicate no data received yet)
  // lastReceivedSensorData.voltageMain = 0.0; // Uncomment if used
  lastReceivedSensorData.currentMain = 0.0;
  // lastReceivedSensorData.voltageEmergency = 0.0; // Uncomment if used
  // lastReceivedSensorData.currentEmergency = 0.0; // Uncomment if used
  lastReceivedSensorData.actualBrightnessMain = 0;
  lastReceivedSensorData.actualBrightnessEmergency = 0;
}

/**
 * @brief Mounts the LittleFS filesystem. Formats if mounting fails.
 */
void initializeLittleFS() {
    // The 'true' parameter formats the filesystem if it can't be mounted
    if (!LittleFS.begin(true)) {
        Serial.println("CRITICAL ERROR: Failed to mount LittleFS!");
        Serial.println("Ensure LittleFS is enabled in partitioning scheme and filesystem image is uploaded.");
        // Consider halting or indicating error permanently
        while (1) { delay(1000); }
    } else {
        Serial.println("LittleFS Mounted Successfully.");
        // Optional: Add code here to list files for debugging if needed
    }
}

/**
 * @brief Initializes WiFi connection, sets MAC address, starts mDNS, and initializes ESP-NOW.
 */
void initializeWiFiAndEspNow() {
  // --- WiFi Setup ---
  WiFi.mode(WIFI_STA); // Set WiFi mode to Station

  // Set the custom MAC Address for this ESP32
  if (esp_wifi_set_mac(WIFI_IF_STA, CONTROL_ROOM_MAC) != ESP_OK) {
      Serial.println("Error setting MAC address!");
  } else {
      Serial.println("MAC address set successfully via esp_wifi_set_mac().");
  }

  Serial.println("Attempting WiFi connection (DHCP)...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_AND_ESP_NOW_CHANNEL); // Connect using credentials and specific channel
  Serial.print("Connecting to SSID: "); Serial.println(WIFI_SSID);

  int wifiConnectTimeoutSeconds = 15; // Increased timeout slightly
  while (WiFi.status() != WL_CONNECTED && wifiConnectTimeoutSeconds > 0) {
    delay(500);
    Serial.print(".");
    wifiConnectTimeoutSeconds--;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFailed to connect to WiFi. Check credentials/network.");
    Serial.println("ESP-NOW might still work if channel is correct, but web server/mDNS will fail.");
    // Decide if execution should continue or halt
    // while(1) { delay(1000); } // Optional halt
  } else {
    Serial.println("\nWiFi Connected.");
    Serial.print("IP Address: "); Serial.println(WiFi.localIP());
    Serial.print("WiFi Channel: "); Serial.println(WiFi.channel());

    // Verify WiFi channel matches ESP-NOW channel
    if (WiFi.channel() != WIFI_AND_ESP_NOW_CHANNEL) {
      Serial.println("CRITICAL WARNING: WiFi connected on a DIFFERENT channel than configured for ESP-NOW!");
      Serial.printf("  WiFi Channel: %d, Expected ESP-NOW Channel: %d\n", WiFi.channel(), WIFI_AND_ESP_NOW_CHANNEL);
      Serial.println("  ESP-NOW communication WILL LIKELY FAIL!");
    } else {
      Serial.println("WiFi and ESP-NOW channels are synchronized.");
    }

    // Start mDNS responder
    if (!MDNS.begin(CONTROL_ROOM_HOSTNAME)) {
      Serial.println("Error setting up mDNS responder!");
    } else {
      Serial.println("mDNS responder started.");
      Serial.println("Access device at: http://" + String(CONTROL_ROOM_HOSTNAME) + ".local");
    }
  }

  // --- ESP-NOW Setup ---
  // Force ESP-NOW onto the configured channel (important if WiFi connects on different channel or fails)
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
  esp_wifi_set_channel(WIFI_AND_ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
  Serial.printf("ESP-NOW channel forced to: %d\n", WIFI_AND_ESP_NOW_CHANNEL);


  // Initialize ESP-NOW service
  if (esp_now_init() != ESP_OK) {
    Serial.println("CRITICAL ERROR: ESP-NOW Initialization Failed");
    ESP.restart(); // Restart if ESP-NOW fails
    return;
  }
  Serial.println("ESP-NOW Initialized.");

  // Register ESP-NOW callback functions
  esp_now_register_send_cb(onEspNowDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(onEspNowDataReceived)); // Cast needed for function signature match

  // Configure and add the Net Pen as an ESP-NOW peer
  esp_now_peer_info_t netPenPeerInfo;
  memset(&netPenPeerInfo, 0, sizeof(netPenPeerInfo)); // Zero initialize structure
  memcpy(netPenPeerInfo.peer_addr, NET_PEN_MAC, 6);
  netPenPeerInfo.channel = WIFI_AND_ESP_NOW_CHANNEL; // Use configured channel
  netPenPeerInfo.encrypt = false;                   // No encryption used
  netPenPeerInfo.ifidx = WIFI_IF_STA;               // Interface index

  if (esp_now_add_peer(&netPenPeerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer (Net Pen).");
    // Consider error handling - ESP-NOW sending will fail
  } else {
    Serial.printf("ESP-NOW Peer Added (Net Pen). MAC: %02X:%02X:%02X:%02X:%02X:%02X, Channel: %d\n",
      netPenPeerInfo.peer_addr[0], netPenPeerInfo.peer_addr[1], netPenPeerInfo.peer_addr[2],
      netPenPeerInfo.peer_addr[3], netPenPeerInfo.peer_addr[4], netPenPeerInfo.peer_addr[5],
      netPenPeerInfo.channel);
  }
}

/**
 * @brief Initializes the Asynchronous Web Server, sets up routes for serving files and handling API calls.
 */
void initializeWebServer() {
    // --- Static File Routes (served from LittleFS) ---
    webServer.on("/", HTTP_GET, handleRootRequest);
    webServer.on("/index.html", HTTP_GET, handleRootRequest); // Explicitly handle index.html too
    webServer.on("/style.css", HTTP_GET, handleCssRequest);
    webServer.on("/script.js", HTTP_GET, handleJsRequest);
    webServer.on("/chart.umd.js", HTTP_GET, handleChartJsRequest);
    webServer.on("/favicon.ico", HTTP_GET, handleFaviconRequest);

    // --- API Routes ---
    // Handle POST requests to update control parameters
    webServer.on("/control", HTTP_POST, handleWebControlPost);

    // --- Server-Sent Events (SSE) Route ---
    sseEvents.onConnect(handleSseConnect); // Attach connection handler
    webServer.addHandler(&sseEvents);      // Add SSE handler to the server

    // --- Not Found Handler ---
    // Optional: Serve index.html for GET requests likely intended for SPA routing
    // Or just send a 404 for any route not explicitly handled.
    webServer.onNotFound(handleNotFound);

    // --- Start Server ---
    webServer.begin();
    if (WiFi.status() == WL_CONNECTED) {
         Serial.println("Web Server Started. Access at http://" + WiFi.localIP().toString() + " or http://" + String(CONTROL_ROOM_HOSTNAME) + ".local");
    } else {
         Serial.println("Web Server configured, but WiFi not connected. Access via IP/mDNS unavailable.");
    }
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
void onEspNowDataReceived(const esp_now_recv_info * macInfo, const uint8_t *incomingData, int dataLength) {
    // Check if the sender MAC matches the expected Net Pen MAC
    if (memcmp(macInfo->src_addr, NET_PEN_MAC, 6) == 0) {
        // Check if the data length matches the expected SensorData structure size
        if (dataLength == sizeof(SensorData_t)) {
            // Copy received data into the global state variable
            memcpy(&lastReceivedSensorData, incomingData, sizeof(lastReceivedSensorData));

            // Print received data (optional, for debugging)
            // Serial.print("Received Sensor Data from Net Pen: vM="); // Uncomment/modify as needed
            // Serial.print(lastReceivedSensorData.voltageMain);
            // Serial.print("V, cM=");
            Serial.print("ESP-NOW RX: cM=");
            Serial.print(lastReceivedSensorData.currentMain);
            // Serial.print("A, vE=");
            // Serial.print(lastReceivedSensorData.voltageEmergency);
            // Serial.print("V, cE=");
            // Serial.print(lastReceivedSensorData.currentEmergency);
            // Serial.print("A, actM=");
            Serial.print("A, actM=");
            Serial.print(lastReceivedSensorData.actualBrightnessMain);
            Serial.print("%, actE=");
            Serial.print(lastReceivedSensorData.actualBrightnessEmergency);
            Serial.println("%");

            // Data will be sent to web clients in the next periodic `sendStatusUpdateToWebClients()` call.
        } else {
            Serial.printf("WARN: Received data from Net Pen MAC, but incorrect size: %d bytes (Expected: %d bytes)\n", dataLength, sizeof(SensorData_t));
        }
    } else {
        // Log if data is received from an unexpected MAC address
        Serial.print("Ignoring received ESP-NOW data from unexpected MAC: ");
        for(int i=0; i<6; i++) { Serial.print(macInfo->src_addr[i], HEX); if(i<5) Serial.print(":"); }
        Serial.printf(" (%d bytes)\n", dataLength);
    }
}

/**
 * @brief Callback function executed after attempting to send ESP-NOW data.
 * @param peerMacAddress MAC address of the recipient.
 * @param status Status of the send operation (ESP_NOW_SEND_SUCCESS or ESP_NOW_SEND_FAIL).
 */
void onEspNowDataSent(const uint8_t *peerMacAddress, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.print("!!! ESP-NOW Send Failure to MAC: ");
    for(int i=0; i<6; i++) { Serial.print(peerMacAddress[i], HEX); if(i<5) Serial.print(":"); }
    Serial.println(" !!!");
    // Consider setting hasControlDataChanged = true to retry sending later?
  } else {
    // Serial.println("ESP-NOW Control data sent successfully."); // Optional: uncomment for debugging successful sends
  }
}

// ==========================================================================
// ==                     Web Server Route Handlers                        ==
// ==========================================================================

/** @brief Serves the main HTML page. */
void handleRootRequest(AsyncWebServerRequest *request) {
    const char* filePath = "/index.html";
    if (LittleFS.exists(filePath)) {
        request->send(LittleFS, filePath, "text/html");
    } else {
        Serial.printf("ERROR: %s not found in LittleFS!\n", filePath);
        request->send(404, "text/plain", "Not Found: index.html");
    }
}

/** @brief Serves the CSS stylesheet. */
void handleCssRequest(AsyncWebServerRequest *request) {
    const char* filePath = "/style.css";
    if (LittleFS.exists(filePath)) {
        request->send(LittleFS, filePath, "text/css");
    } else {
        Serial.printf("ERROR: %s not found in LittleFS!\n", filePath);
        request->send(404, "text/plain", "Not Found: style.css");
    }
}

/** @brief Serves the main JavaScript file. */
void handleJsRequest(AsyncWebServerRequest *request) {
    const char* filePath = "/script.js";
    if (LittleFS.exists(filePath)) {
        request->send(LittleFS, filePath, "text/javascript; charset=utf-8");
    } else {
        Serial.printf("ERROR: %s not found in LittleFS!\n", filePath);
        request->send(404, "text/plain", "Not Found: script.js");
    }
}

/** @brief Serves the Chart.js library file. */
void handleChartJsRequest(AsyncWebServerRequest *request) {
    const char* filePath = "/chart.umd.js";
    if (LittleFS.exists(filePath)) {
        request->send(LittleFS, filePath, "text/javascript");
    } else {
        Serial.printf("WARN: %s requested but not found in LittleFS.\n", filePath);
        request->send(404, "text/plain", "Not Found: chart.umd.js");
    }
}

/** @brief Serves the favicon. */
void handleFaviconRequest(AsyncWebServerRequest *request) {
    const char* filePath = "/favicon.ico";
    if (LittleFS.exists(filePath)) {
        request->send(LittleFS, filePath, "image/x-icon");
    } else {
        // It's common for browsers to request this, log as warning or info if missing
        Serial.printf("INFO: %s requested but not found in LittleFS.\n", filePath);
        request->send(404, "text/plain", "Not Found: favicon.ico");
    }
}

/**
 * @brief Handles POST requests to the /control endpoint to update target brightness.
 * @param request Pointer to the incoming web server request object.
 */
void handleWebControlPost(AsyncWebServerRequest *request) {
    bool wasDataUpdated = false;
    String paramValueStr;

    // Check for 'targetMain' parameter in the POST body
    if (request->hasParam("targetMain", true)) { // 'true' checks POST parameters
        paramValueStr = request->getParam("targetMain", true)->value();
        int value = paramValueStr.toInt(); // Convert parameter to integer
        currentControlData.targetBrightnessMain = constrain(value, 0, 100); // Clamp value between 0 and 100
        wasDataUpdated = true;
        Serial.printf("Web POST: targetMain set to %d\n", currentControlData.targetBrightnessMain);
    }

    // Check for 'targetEmergency' parameter in the POST body
    if (request->hasParam("targetEmergency", true)) {
        paramValueStr = request->getParam("targetEmergency", true)->value();
        int value = paramValueStr.toInt();
        currentControlData.targetBrightnessEmergency = constrain(value, 0, 100);
        wasDataUpdated = true;
        Serial.printf("Web POST: targetEmergency set to %d\n", currentControlData.targetBrightnessEmergency);
    }

    // Check for 'rampRate' parameter (example, currently commented out in config/structs)
    // if (request->hasParam("rampRate", true)) {
    //     paramValueStr = request->getParam("rampRate", true)->value();
    //     float requestedRate = paramValueStr.toFloat();
    //     if (requestedRate > 0) { // Ensure positive ramp rate
    //         currentControlData.rampRatePercentPerSec = requestedRate;
    //         wasDataUpdated = true;
    //         Serial.printf("Web POST: rampRate set to %.1f\n", currentControlData.rampRatePercentPerSec);
    //     } else {
    //         Serial.println("Web POST: Ignoring invalid rampRate <= 0.");
    //     }
    // }

    // Respond to the client and trigger updates if necessary
    if (wasDataUpdated) {
        Serial.println("Control data updated via web POST.");
        hasControlDataChanged = true; // Flag to send updated data via ESP-NOW
        request->send(200, "text/plain", "OK"); // Send success response
        sendStatusUpdateToWebClients(); // Immediately notify web clients of the new target values
    } else {
        Serial.println("WARN: POST /control received, but no valid/recognized parameters found.");
        request->send(400, "text/plain", "Bad Request - No valid/recognized parameters"); // Send error response
    }
}

/**
 * @brief Handles new client connections to the Server-Sent Events endpoint.
 * @param client Pointer to the connected client object.
 */
void handleSseConnect(AsyncEventSourceClient *client){
    bool isClientReconnect = client->lastId() != 0; // Check if client provided a last event ID
    if (isClientReconnect) {
        Serial.printf("SSE Client Reconnected! Client Last Event ID: %u\n", client->lastId());
    } else {
        Serial.println("New SSE Client Connected.");
    }

    // IMPORTANT: Sending an event immediately upon connection here seems to cause
    // the connection to drop for some reason in ESPAsyncWebServer/AsyncTCP.
    // The periodic sender in the main loop will send data shortly anyway.
    // Consider sending initial state via a separate REST request if needed immediately.
    // sendStatusUpdateToWebClients(); // <-- AVOID CALLING THIS HERE
}

/** @brief Handles requests for resources that are not found. */
void handleNotFound(AsyncWebServerRequest *request){
    Serial.printf("WARN: Not Found - HTTP %d on %s\n", request->method(), request->url().c_str());
    // Simple 404 response
    request->send(404, "text/plain", "Resource Not Found");

    // Optional: Advanced handling for Single Page Applications (SPAs)
    // if (request->method() == HTTP_GET && !request->url().startsWith("/api") && request->url().indexOf('.') == -1) {
    //     Serial.println("-> Likely SPA route, serving index.html");
    //      request->send(LittleFS, "/index.html", "text/html");
    // } else {
    //     request->send(404, "text/plain", "Not found");
    // }
}

// ==========================================================================
// ==                       Core Logic Functions                           ==
// ==========================================================================

/**
 * @brief Sends the current control data (target brightness) to the Net Pen via ESP-NOW.
 */
void sendControlDataViaEspNow() {
    // Send data structure to the specific MAC address of the Net Pen
    esp_err_t sendResult = esp_now_send(NET_PEN_MAC, (uint8_t *) &currentControlData, sizeof(currentControlData));

    // Error handling is managed by the onEspNowDataSent callback
    if (sendResult == ESP_OK) {
       // Serial.println("Initiated ESP-NOW send of control data."); // Optional debug msg
    } else {
       Serial.print("Error initiating ESP-NOW send: ");
       Serial.println(esp_err_to_name(sendResult));
       // Consider setting hasControlDataChanged = true to retry later?
    }
}

/**
 * @brief Sends the latest sensor and target data to all connected web clients via Server-Sent Events (SSE).
 */
void sendStatusUpdateToWebClients() {
    // Only proceed if there are active SSE clients
    if(sseEvents.count() == 0) {
        // Serial.println("No SSE clients connected, skipping status update send."); // Optional debug msg
        return;
    }

    // Create a JSON document to hold the data
    // DynamicJsonDocument is often safer than StaticJsonDocument if size varies slightly
    DynamicJsonDocument jsonDoc(512); // Adjust size as needed

    // Populate the JSON document with current sensor and control data
    // Format floats for cleaner representation in the web UI
    // jsonDoc["vM"] = float(int(lastReceivedSensorData.voltageMain * 100)) / 100.0; // Example: 2 decimal places
    jsonDoc["cM"] = float(int(lastReceivedSensorData.currentMain * 100)) / 100.0;
    // jsonDoc["vE"] = float(int(lastReceivedSensorData.voltageEmergency * 100)) / 100.0;
    // jsonDoc["cE"] = float(int(lastReceivedSensorData.currentEmergency * 100)) / 100.0;
    jsonDoc["actM"] = lastReceivedSensorData.actualBrightnessMain;
    jsonDoc["actE"] = lastReceivedSensorData.actualBrightnessEmergency;
    jsonDoc["tgtM"] = currentControlData.targetBrightnessMain;
    jsonDoc["tgtE"] = currentControlData.targetBrightnessEmergency;
    // jsonDoc["rate"] = float(int(currentControlData.rampRatePercentPerSec * 10)) / 10.0; // Example: 1 decimal place

    // Serialize JSON document to a string
    String jsonDataString;
    serializeJson(jsonDoc, jsonDataString);

    // Send the JSON string as an SSE event named "update"
    sseEvents.send(jsonDataString.c_str(), "update", millis());

    // Debugging output (optional)
    // Serial.print("Sent SSE 'update' to ");
    // Serial.print(sseEvents.count());
    // Serial.println(" client(s).");
    // Serial.print("  Data: "); Serial.println(jsonDataString);
}

// control_room.ino END