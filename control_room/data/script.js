// script.js START

console.info("AquaAlert Script Starting"); // Indicate script load

// ==========================================================================
// ==                     DOM Content Loaded Event Listener                ==
// ==========================================================================
// Ensure the DOM is fully loaded before executing script logic
document.addEventListener('DOMContentLoaded', () => {
    console.info("DOM fully loaded and parsed.");

    // ==========================================================================
    // ==                     DOM Element References                         ==
    // ==========================================================================
    console.log("Gathering DOM element references...");

    // --- General Status ---
    const statusElem = document.getElementById('status');
    if (!statusElem) console.error("Status element (#status) not found!");

    // --- Main Light Elements ---
    const mainSlider = document.getElementById('mainSlider');
    const mainDesiredValue = document.getElementById('mainDesiredValue');
    const mainActualValue = document.getElementById('mainActualValue');
    const mainCurrent = document.getElementById('mainCurrent');
    // const mainVoltage = document.getElementById('mainVoltage'); // Uncomment if used
    const mainNumericReadings = document.getElementById('mainNumericReadings');
    const mainGraphReadings = document.getElementById('mainGraphReadings');
    const mainToggleBtn = document.querySelector('.btn-toggle-view[data-target="main"]');
    const mainChartCanvas = document.getElementById('mainChart');
    const mainBrightnessIcon = document.getElementById('mainBrightnessIcon');
    if (!mainSlider) console.error("Main slider not found!");
    if (!mainBrightnessIcon) console.error("Main brightness icon element not found!");

    // --- Emergency Light Elements ---
    const emergencyCard = document.querySelector('.emergency-light-card');
    const emergencySlider = document.getElementById('emergencySlider');
    const emergencyDesiredValue = document.getElementById('emergencyDesiredValue');
    const emergencyActualValue = document.getElementById('emergencyActualValue');
    // const emergencyCurrent = document.getElementById('emergencyCurrent'); // Uncomment if used
    // const emergencyVoltage = document.getElementById('emergencyVoltage'); // Uncomment if used
    const emergencyNumericReadings = document.getElementById('emergencyNumericReadings');
    const emergencyGraphReadings = document.getElementById('emergencyGraphReadings');
    const emergencyToggleBtn = document.querySelector('.btn-toggle-view[data-target="emergency"]');
    const emergencyChartCanvas = document.getElementById('emergencyChart');
    const emergencyBrightnessIcon = document.getElementById('emergencyBrightnessIcon');
    if (!emergencyBrightnessIcon) console.error("Emergency brightness icon element not found!");

    // --- Settings Elements ---
    const emergencySyncToggle = document.getElementById('syncEmergencyLight');
    // const rampRateInput = document.getElementById('rampRateInput'); // Uncomment if used
    if (!emergencySyncToggle) console.warn("Emergency sync toggle checkbox not found!");

    console.log("DOM element references gathered.");

    // ==========================================================================
    // ==                         State Variables                              ==
    // ==========================================================================
    // Store last non-zero brightness for the 'ON' button functionality
    let lastMainBrightness = 100;      // Default to 100% if no previous value
    let lastEmergencyBrightness = 100; // Default to 100%

    // --- SSE Connection State ---
    let evtSource = null;                     // Holds the EventSource object
    let connectionCheckTimer = null;          // Timer to detect SSE inactivity
    let reconnectTimerId = null;              // Timer for scheduling reconnect attempts
    const CONNECTION_TIMEOUT_MS = 6000;       // How long to wait for an SSE message before assuming disconnection
    const INITIAL_RECONNECT_DELAY_MS = 5000;  // Initial delay before first reconnect attempt
    const MAX_RECONNECT_DELAY_MS = 60000;     // Maximum delay between reconnect attempts (exponential backoff)
    let currentReconnectDelay = INITIAL_RECONNECT_DELAY_MS; // Current delay, increases on repeated failures

    // --- Chart Objects ---
    let mainChart = null;
    let emergencyChart = null;
    const MAX_DATA_POINTS = 30; // Max history points shown on charts


    // ==========================================================================
    // ==                         Chart Configuration & Functions              ==
    // ==========================================================================
    console.log("Configuring chart defaults...");

    // Default configuration shared by both charts
    const chartConfigDefaults = {
        type: 'line',
        data: {
            labels: [], // Timestamps or sequence numbers will populate this
            datasets: [
                // Dataset for Current (Amps) - Plotted on 'yA' axis
                {
                    label: 'Strøm (A)',
                    data: [], // Populated with sensor data
                    borderColor: 'rgb(255, 99, 132)', // Reddish color
                    backgroundColor: 'rgba(255, 99, 132, 0.1)', // Lighter fill
                    tension: 0.1, // Slight curve to the line
                    yAxisID: 'yA',
                    pointRadius: 1, // Smaller points
                    borderWidth: 2,
                },
                // Dataset for Actual Brightness (Percent) - Plotted on 'yP' axis
                {
                    label: 'Reell Lysstyrke (%)',
                    data: [], // Populated with sensor data
                    borderColor: 'rgb(75, 192, 192)', // Teal color
                    backgroundColor: 'rgba(75, 192, 192, 0.1)', // Lighter fill
                    tension: 0.1,
                    yAxisID: 'yP',
                    pointRadius: 1,
                    borderWidth: 2,
                },
                // Potential Dataset for Voltage (Volts) - Plotted on 'yV' axis (Example)
                // {
                //     label: 'Spenning (V)',
                //     data: [],
                //     borderColor: 'rgb(54, 162, 235)', // Blue color
                //     tension: 0.1,
                //     yAxisID: 'yV', // Voltage axis
                // }
            ]
        },
        options: {
            responsive: true,           // Chart adjusts to container size
            maintainAspectRatio: false, // Allows height to be controlled by CSS/container
            animation: false,           // Disable animations for real-time data feel
            scales: {
                // X-Axis (Time/Sequence)
                x: {
                    ticks: { display: false } // Hide labels on the x-axis for simplicity
                },
                // Y-Axis for Current (A) - Positioned on the Right
                yA: {
                    type: 'linear',
                    display: true,
                    position: 'right',
                    title: { display: true, text: 'Strøm (A)' },
                    grid: { drawOnChartArea: false }, // Only draw grid lines for the primary axis (yP)
                    suggestedMin: 0,
                    beginAtZero: true,
                },
                // Y-Axis for Percentage (%) - Positioned on the Left (Primary)
                yP: {
                   type: 'linear',
                   display: true,
                   position: 'left',
                   title: { display: true, text: 'Lysstyrke (%)' },
                   min: 0, // Hard min/max for percentage
                   max: 100,
                   // suggestedMin: 0, // Use hard min/max instead
                   // suggestedMax: 100,
                   beginAtZero: true,
                },
                 // Potential Y-Axis for Voltage (V) - Positioned on the Right (Example)
                // yV: {
                //     type: 'linear',
                //     display: true,
                //     position: 'right',
                //     title: { display: true, text: 'Spenning (V)' },
                //     grid: { drawOnChartArea: false },
                //     suggestedMin: 0
                // }
            },
            plugins: {
                legend: {
                    position: 'top', // Position legend above chart
                    labels: { boxWidth: 15, padding: 15 } // Adjust legend appearance
                },
                tooltip: { // Customize tooltips on hover
                    mode: 'index', // Show tooltips for all datasets at the hovered index
                    intersect: false, // Tooltips appear even if not directly hovering over a point
                }
            }
        }
    };

    /** Deep copies an object (used for chart configurations). */
    const deepCopy = (obj) => JSON.parse(JSON.stringify(obj));

    /** Initializes both main and emergency light charts. */
    function initCharts() {
         console.log("Initializing charts...");
         try {
            // --- Main Chart Initialization (uses default config directly) ---
            const mainConfig = deepCopy(chartConfigDefaults);
            // Ensure canvas element exists
            if (mainChartCanvas) {
                mainChart = new Chart(mainChartCanvas.getContext('2d'), mainConfig);
                console.log("Main chart initialized.");
                // console.log("Main chart datasets:", mainChart.config.data.datasets.map(ds => ds.label));
                // console.log("Main chart scales:", Object.keys(mainChart.config.options.scales));
            } else {
                console.error("Main chart canvas element not found!");
            }

            // --- Emergency Chart Initialization (modifies default config) ---
            const emergencyConfig = deepCopy(chartConfigDefaults);
            // Remove the 'Current (A)' dataset and its corresponding 'yA' axis
            emergencyConfig.data.datasets = emergencyConfig.data.datasets.filter(
                dataset => dataset.yAxisID !== 'yA' // Keep only datasets NOT using yA
            );
            delete emergencyConfig.options.scales.yA; // Remove the scale definition

            // Ensure canvas element exists
            if (emergencyChartCanvas) {
                emergencyChart = new Chart(emergencyChartCanvas.getContext('2d'), emergencyConfig);
                console.log("Emergency chart initialized (without Current axis).");
                // console.log("Emergency chart datasets:", emergencyChart.config.data.datasets.map(ds => ds.label));
                // console.log("Emergency chart scales:", Object.keys(emergencyChart.config.options.scales));
            } else {
                console.error("Emergency chart canvas element not found!");
            }

         } catch (e) {
             console.error("Error initializing charts:", e);
         }
    }

    /**
     * Updates a given chart instance with new data.
     * @param {Chart} chart - The Chart.js chart instance to update.
     * @param {string} label - The label for the new data point (e.g., timestamp).
     * @param {number} voltage - The voltage value (can be undefined).
     * @param {number} current - The current value (can be undefined).
     * @param {number} brightness - The brightness value (can be undefined).
     */
    function updateChart(chart, label, voltage, current, brightness) {
        if (!chart || !chart.config) { console.warn("Attempted to update non-existent or invalid chart."); return; }

        const config = chart.config;
        const data = config.data;
        const datasets = data.datasets;

        // Add new label
        data.labels.push(label);

        // Helper function to find dataset by label and add data
        const addDataToDataset = (datasetLabel, newData) => {
            const dataset = datasets.find(ds => ds.label === datasetLabel);
            if (dataset) {
                // Push the new data point, or NaN if undefined/null to create a gap in the line
                dataset.data.push(newData !== undefined && newData !== null ? newData : NaN);
            }
        };

        // Add data to respective datasets if they exist in this chart instance
        addDataToDataset('Strøm (A)', current);
        addDataToDataset('Reell Lysstyrke (%)', brightness);
        // addDataToDataset('Spenning (V)', voltage); // Uncomment if voltage dataset exists

        // Maintain history length: Remove oldest data point if exceeding max
        while (data.labels.length > MAX_DATA_POINTS) {
            data.labels.shift(); // Remove oldest label
            datasets.forEach(dataset => {
                if (dataset.data.length > 0) {
                    dataset.data.shift(); // Remove oldest data point from each dataset
                }
            });
        }

        // Update the chart to reflect changes
        try {
            chart.update();
        } catch (e) {
            console.error("Error updating chart:", e);
        }
    }

    // ==========================================================================
    // ==                         UI Update Functions                          ==
    // ==========================================================================

    /** Updates the connection status display. */
    function updateStatus(text, color = 'inherit', cssClass = '') {
        if (!statusElem) return;
        statusElem.textContent = text;
        statusElem.style.color = color; // Direct color for immediate feedback
        // Manage CSS classes for consistent styling (e.g., from style.css)
        statusElem.classList.remove('status-connecting', 'status-connected', 'status-error');
        if (cssClass) {
            statusElem.classList.add(`status-${cssClass}`);
        }
    }

    /** Returns the appropriate brightness icon based on the value. */
    function getBrightnessIcon(value) {
        const numValue = parseInt(value, 10);
        if (isNaN(numValue) || numValue <= 0) return '🌑'; // Off / 0%
        if (numValue < 50) return '🔅';                 // Dim (1-49%)
        if (numValue < 100) return '🔆';                // Bright (50-99%)
        return '☀️';                                    // Full (100%)
    }

    /** Updates the text content of a brightness icon element. */
    function updateIcon(iconElement, value) {
        if (iconElement) {
            iconElement.textContent = getBrightnessIcon(value);
        }
    }

    // ==========================================================================
    // ==                     SSE Connection Management                        ==
    // ==========================================================================

    /** Handles SSE connection timeout. */
    function handleConnectionTimeout() {
        console.warn(`SSE Connection Timeout: No message received for ${CONNECTION_TIMEOUT_MS / 1000} seconds.`);
        updateStatus("Frakoblet (Timeout)", "red", "error"); // Update UI
        connectionCheckTimer = null; // Clear the timer reference
        if (evtSource) {
            evtSource.close(); // Close the existing connection
            evtSource = null;
            console.log("Closed timed-out EventSource.");
        }
        scheduleReconnect(); // Attempt to reconnect
    }

    /** Resets the SSE connection timeout timer. Called on message receipt. */
    function resetConnectionTimer() {
        clearTimeout(connectionCheckTimer); // Clear any existing timeout
        // Set a new timeout
        connectionCheckTimer = setTimeout(handleConnectionTimeout, CONNECTION_TIMEOUT_MS);
    }

    /** Schedules an SSE reconnection attempt with exponential backoff. */
    function scheduleReconnect() {
        clearTimeout(reconnectTimerId); // Clear previous reconnect schedule
        clearTimeout(connectionCheckTimer); // Ensure no active connection timer
        console.log(`Scheduling SSE reconnect attempt in ${currentReconnectDelay / 1000} seconds...`);
        updateStatus(`Frakoblet (Prøver igjen om ${currentReconnectDelay / 1000}s)`, "orange", "connecting"); // Update UI

        reconnectTimerId = setTimeout(() => {
            console.log("Attempting SSE reconnection now...");
            setupSSE(); // Try to establish a new connection
            // Increase delay for the *next* potential failure
            currentReconnectDelay = Math.min(currentReconnectDelay * 2, MAX_RECONNECT_DELAY_MS);
        }, currentReconnectDelay);
    }

    /** Sets up the main Server-Sent Events (SSE) connection. */
    function setupSSE() {
        // Prerequisite check
        if (typeof (EventSource) === "undefined") {
            updateStatus("SSE Ikke Støttet", "red", "error");
            alert("Nettleseren din støtter ikke Server-Sent Events (SSE), som kreves for sanntidsoppdateringer.");
            return;
        }

        updateStatus("Kobler til...", "orange", "connecting"); // Initial status
        clearTimeout(connectionCheckTimer); // Clear any pending timers
        clearTimeout(reconnectTimerId);
        if (evtSource) { // Close any existing connection first
            evtSource.close();
            console.log("Closed previous EventSource before creating new one.");
            evtSource = null;
        }

        try {
            // Create the new EventSource connection
            evtSource = new EventSource('/events');
            console.log("New EventSource created, connecting to /events...");

            // --- EventSource Event Handlers ---

            // Called when the connection is successfully opened
            evtSource.onopen = function () {
                console.log("SSE Connection opened successfully.");
                updateStatus("Tilkoblet", "green", "connected");
                resetConnectionTimer(); // Start the inactivity timer
                currentReconnectDelay = INITIAL_RECONNECT_DELAY_MS; // Reset reconnect delay on success
                clearTimeout(reconnectTimerId); // Cancel any scheduled reconnect
            };

            // Called when an error occurs with the connection
            evtSource.onerror = function (e) {
                console.error("SSE Connection Error:", e);
                updateStatus("Frakoblet (Feil)", "red", "error");
                if (evtSource) {
                    console.log("SSE readyState on error:", evtSource.readyState);
                    // readyState: 0=CONNECTING, 1=OPEN, 2=CLOSED
                    evtSource.close(); // Ensure connection is closed
                    evtSource = null;
                }
                clearTimeout(connectionCheckTimer); // Stop inactivity timer
                console.log("SSE error detected, scheduling reconnect.");
                scheduleReconnect(); // Attempt to reconnect after delay
            };

            // --- Custom Event Listeners ---

            // Listener for 'update' events from the server
            evtSource.addEventListener("update", function (event) {
                // console.log("SSE 'update' received:", event.data); // Log raw data
                resetConnectionTimer(); // Reset inactivity timer on valid message
                try {
                    const data = JSON.parse(event.data); // Parse the JSON data payload

                    // Update UI elements with received data
                    if(mainActualValue) mainActualValue.textContent = data.actM ?? '--'; // Use nullish coalescing for defaults
                    if(mainCurrent) mainCurrent.textContent = data.cM !== undefined && data.cM !== null ? data.cM.toFixed(2) : '--';
                    // if(mainVoltage) mainVoltage.textContent = data.vM !== undefined && data.vM !== null ? data.vM.toFixed(2) : '--'; // Uncomment if used

                    if(emergencyActualValue) emergencyActualValue.textContent = data.actE ?? '--';
                    // if(emergencyCurrent) emergencyCurrent.textContent = data.cE !== undefined && data.cE !== null ? data.cE.toFixed(2) : '--'; // Uncomment if used
                    // if(emergencyVoltage) emergencyVoltage.textContent = data.vE !== undefined && data.vE !== null ? data.vE.toFixed(2) : '--'; // Uncomment if used


                    // Get current target values from data, default to 0 if missing
                    const currentTgtM = data.tgtM ?? 0;
                    const currentTgtE = data.tgtE ?? 0;
                    // const currentRate = data.rate !== undefined && data.rate !== null ? data.rate.toFixed(1) : '5.0'; // Uncomment if used

                    // Update control elements (sliders, number inputs) ONLY if they don't have focus
                    // to avoid disrupting user input. Also handles sync logic.
                    const mainHasFocus = document.activeElement === mainSlider || document.activeElement === mainDesiredValue;
                    if (!mainHasFocus) {
                        if (mainSlider) mainSlider.value = currentTgtM;
                        if (mainDesiredValue) mainDesiredValue.value = currentTgtM;
                        updateIcon(mainBrightnessIcon, currentTgtM); // Update icon based on target
                    }

                    const emergencyHasFocus = document.activeElement === emergencySlider || document.activeElement === emergencyDesiredValue;
                    // If emergency sync is OFF, update emergency controls if they don't have focus
                    if (emergencySyncToggle && !emergencySyncToggle.checked) {
                         if (!emergencyHasFocus) {
                            if (emergencySlider) emergencySlider.value = currentTgtE;
                            if (emergencyDesiredValue) emergencyDesiredValue.value = currentTgtE;
                            updateIcon(emergencyBrightnessIcon, currentTgtE); // Update icon based on its own target
                         }
                    }
                    // If emergency sync is ON, sync emergency controls to main controls *unless* user is interacting with either
                    else if (emergencySyncToggle && emergencySyncToggle.checked) {
                        if (!mainHasFocus && !emergencyHasFocus) {
                             syncEmergencyControls(); // Syncs emergency slider/value/icon to main's current value
                        }
                    }

                    // Update ramp rate input if it exists and doesn't have focus (Example)
                    // if (rampRateInput && document.activeElement !== rampRateInput) {
                    //     rampRateInput.value = currentRate;
                    // }

                    // Update Charts with new data point
                    const nowLabel = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit'}); // Simple time label
                    // Only update charts if their corresponding view is active
                    if (mainGraphReadings && mainGraphReadings.classList.contains('active')) {
                        updateChart(mainChart, nowLabel, data.vM, data.cM, data.actM);
                    }
                    if (emergencyGraphReadings && emergencyGraphReadings.classList.contains('active')) {
                        // Pass undefined for current/voltage as emergency chart doesn't use them
                        updateChart(emergencyChart, nowLabel, data.vE, undefined, data.actE);
                    }

                } catch (e) {
                    console.error("Failed to parse SSE 'update' data:", e, "\nRaw data:", event.data);
                }
            });

            // Listener for 'heartbeat' events from the server
            evtSource.addEventListener("heartbeat", function (event) {
                 // console.log("SSE 'heartbeat' received:", event.data); // Optional log
                 resetConnectionTimer(); // Reset inactivity timer
           });

        } catch (error) {
            console.error("Failed to create EventSource:", error);
            updateStatus("Feil ved Oppsett", "red", "error");
            scheduleReconnect(); // Attempt to reconnect if creation failed
        }
    }


    // ==========================================================================
    // ==                  Send Control Commands to Server                     ==
    // ==========================================================================

    /**
     * Sends control updates (target brightness, etc.) to the ESP32 server via POST request.
     * @param {object} payload - An object containing the parameters to send (e.g., { targetMain: 50 }).
     */
    async function sendControlUpdate(payload) {
        console.log("Sending control update to /control:", payload);
        try {
            const response = await fetch('/control', {
                method: 'POST',
                headers: {
                    // Standard header for form data
                    'Content-Type': 'application/x-www-form-urlencoded',
                },
                // Convert the payload object into a URL-encoded string
                body: new URLSearchParams(payload).toString()
            });

            // Check if the request was successful
            if (!response.ok) {
                const responseBody = await response.text(); // Get error details if available
                console.error("Failed to send control update. Status:", response.status, response.statusText, "\nResponse:", responseBody);
                // Provide temporary feedback to the user
                const originalStatus = statusElem ? statusElem.textContent : '';
                const originalClassList = statusElem ? Array.from(statusElem.classList) : [];
                updateStatus(`Sende feil (${response.status})`, "orange", "error"); // Indicate error visually
                // Revert status message after a short delay
                setTimeout(() => {
                    if (statusElem && statusElem.textContent === `Sende feil (${response.status})`) {
                        statusElem.textContent = originalStatus;
                        statusElem.className = ''; // Clear current classes
                        originalClassList.forEach(cls => statusElem.classList.add(cls)); // Reapply original classes
                    }
                 }, 2500); // Show error for 2.5 seconds
            } else {
                // console.log("Control update sent successfully."); // Optional success log
            }
        } catch (error) {
            // Handle network errors (e.g., server unreachable)
            console.error("Network error sending control update:", error);
            const originalStatus = statusElem ? statusElem.textContent : '';
            const originalClassList = statusElem ? Array.from(statusElem.classList) : [];
            updateStatus("Sende feil (Nettverk)", "orange", "error");
            setTimeout(() => {
                if (statusElem && statusElem.textContent === "Sende feil (Nettverk)") {
                    statusElem.textContent = originalStatus;
                    statusElem.className = ''; // Clear classes
                    originalClassList.forEach(cls => statusElem.classList.add(cls)); // Reapply original classes
                }
            }, 2500);
        }
    }

    // ==========================================================================
    // ==                     Emergency Light Sync Logic                       ==
    // ==========================================================================

    /** Sets the visual state and behavior based on the sync checkbox. */
    function setEmergencySyncState(isEnabled) {
        if (!emergencyCard || !emergencySyncToggle) {
             console.warn("Cannot set emergency sync state: Card or toggle element missing.");
             return;
        }
        emergencySyncToggle.checked = isEnabled; // Ensure checkbox reflects the state
        if (isEnabled) {
            emergencyCard.classList.add('emergency-sync-active'); // Apply CSS class for visual dimming/disabling
            console.log("Emergency sync enabled. Syncing controls...");
            syncEmergencyControls(); // Immediately update emergency controls to match main
        } else {
            emergencyCard.classList.remove('emergency-sync-active'); // Remove class
            console.log("Emergency sync disabled.");
        }
    }

    /** Copies the value and updates the icon of emergency controls to match main controls. */
    function syncEmergencyControls() {
        // Check if all necessary elements exist
        if (!emergencySlider || !emergencyDesiredValue || !mainSlider || !emergencyBrightnessIcon) {
             console.warn("Cannot sync emergency controls: One or more elements missing.");
             return;
        }
        const mainValue = mainSlider.value;
        // Update emergency slider and number input only if their values differ
        if (emergencySlider.value !== mainValue) emergencySlider.value = mainValue;
        if (emergencyDesiredValue.value !== mainValue) emergencyDesiredValue.value = mainValue;
        // Always update the icon
        updateIcon(emergencyBrightnessIcon, mainValue);
        // console.log(`Synced emergency controls to main value: ${mainValue}`); // Optional log
    }

    /** Called when user interacts with emergency controls; disables sync if active. */
    function handleEmergencyInteraction() {
        if (emergencySyncToggle && emergencySyncToggle.checked) {
            console.log("User interaction detected on emergency light controls while sync was active. Disabling sync.");
            setEmergencySyncState(false); // Turn off sync
        }
    }

    // ==========================================================================
    // ==                         Event Listeners                              ==
    // ==========================================================================
    console.log("Attaching event listeners...");

    // --- Sync Toggle Checkbox ---
    if (emergencySyncToggle) {
        emergencySyncToggle.addEventListener('change', () => {
            const isEnabled = emergencySyncToggle.checked;
            console.log(`Emergency light sync toggle changed. New state: ${isEnabled ? 'ENABLED' : 'DISABLED'}.`);
            setEmergencySyncState(isEnabled); // Apply visual state change

            // If sync is now enabled, send the current main value as the new emergency target
            if (isEnabled && mainSlider) {
                const currentMainValue = mainSlider.value;
                sendControlUpdate({ targetEmergency: currentMainValue });
                // Update lastEmergencyBrightness if sync is turned on while main light is on
                if (currentMainValue > 0) {
                    lastEmergencyBrightness = currentMainValue;
                }
            }
        });
    }

    // --- Main Light Controls ---
    // Slider 'input' event (fires continuously during drag)
    if (mainSlider) {
        mainSlider.addEventListener('input', () => {
            const value = mainSlider.value;
            if (mainDesiredValue) mainDesiredValue.value = value; // Update number input visually
            updateIcon(mainBrightnessIcon, value);           // Update icon visually
            // If sync is enabled, update emergency controls in real-time
            if (emergencySyncToggle && emergencySyncToggle.checked) {
                syncEmergencyControls();
            }
        });
    }
    // Slider 'change' event (fires when dragging stops)
    if (mainSlider) {
        mainSlider.addEventListener('change', () => {
            const value = mainSlider.value;
            if (mainDesiredValue) mainDesiredValue.value = value; // Ensure number input matches
            updateIcon(mainBrightnessIcon, value);           // Ensure icon matches
            if (value > 0) lastMainBrightness = value;       // Store last non-zero value for 'ON' button
            sendControlUpdate({ targetMain: value });         // Send update to server
            // If sync is enabled, also send the update for the emergency target
            if (emergencySyncToggle && emergencySyncToggle.checked) {
                syncEmergencyControls(); // Ensure visual sync first
                if (value > 0) lastEmergencyBrightness = value; // Update last emergency value too
                console.log("Main slider changed (sync ON), sending emergency update.");
                sendControlUpdate({ targetEmergency: value });
            }
        });
    }
    // Number Input 'input' event (fires on keyboard input)
    if (mainDesiredValue) {
        mainDesiredValue.addEventListener('input', () => {
            const valueStr = mainDesiredValue.value;
            updateIcon(mainBrightnessIcon, valueStr); // Update icon immediately based on input
            // Only update slider if input is a valid number within range (prevents slider jumping on invalid input)
            let value = parseInt(valueStr, 10);
            let updateSlider = false;
            if (!isNaN(value)) {
                value = Math.max(0, Math.min(100, value)); // Clamp the parsed number
                updateSlider = true;
            }
            // Update slider visually if the number input has focus and value is valid
            if (updateSlider && document.activeElement === mainDesiredValue) {
                 if(mainSlider) mainSlider.value = value;
                 // If sync is enabled, update emergency controls in real-time
                 if (emergencySyncToggle && emergencySyncToggle.checked) {
                     syncEmergencyControls();
                 }
            }
        });
    }
    // Number Input 'change' event (fires on blur or Enter key)
    if (mainDesiredValue) {
        mainDesiredValue.addEventListener('change', () => {
             let value = parseInt(mainDesiredValue.value, 10);
             // Validate and clamp the final value
             if (isNaN(value) || value < 0) value = 0;
             if (value > 100) value = 100;
             mainDesiredValue.value = value; // Update input field with clamped value
             if(mainSlider) mainSlider.value = value;    // Update slider to match
             updateIcon(mainBrightnessIcon, value); // Update icon
             if (value > 0) lastMainBrightness = value; // Store last non-zero value
             sendControlUpdate({ targetMain: value }); // Send update to server
             // If sync is enabled, also send the update for the emergency target
             if (emergencySyncToggle && emergencySyncToggle.checked) {
                syncEmergencyControls(); // Ensure visual sync first
                if (value > 0) lastEmergencyBrightness = value; // Update last emergency value too
                console.log("Main desired value changed (sync ON), sending emergency update.");
                sendControlUpdate({ targetEmergency: value });
            }
         });
    }

    // --- Emergency Light Controls ---
    // Slider 'input' event
    if (emergencySlider) {
        emergencySlider.addEventListener('input', () => {
            handleEmergencyInteraction(); // Disable sync if active
            // Only update if sync is OFF
            if (!emergencySyncToggle || !emergencySyncToggle.checked) {
                const value = emergencySlider.value;
                if (emergencyDesiredValue) emergencyDesiredValue.value = value;
                updateIcon(emergencyBrightnessIcon, value);
            } else {
                 // If sync somehow re-enabled during drag (unlikely), resync
                 syncEmergencyControls();
            }
        });
    }
    // Slider 'change' event
    if (emergencySlider) {
        emergencySlider.addEventListener('change', () => {
            handleEmergencyInteraction(); // Ensure sync is off
             if (!emergencySyncToggle || !emergencySyncToggle.checked) {
                 const value = emergencySlider.value;
                 if(emergencyDesiredValue) emergencyDesiredValue.value = value;
                 updateIcon(emergencyBrightnessIcon, value);
                 if (value > 0) lastEmergencyBrightness = value;
                 sendControlUpdate({ targetEmergency: value }); // Send update
             }
        });
    }
    // Number Input 'input' event
    if (emergencyDesiredValue) {
        emergencyDesiredValue.addEventListener('input', () => {
            handleEmergencyInteraction(); // Disable sync if active
             if (!emergencySyncToggle || !emergencySyncToggle.checked) {
                 const valueStr = emergencyDesiredValue.value;
                 updateIcon(emergencyBrightnessIcon, valueStr); // Update icon immediately
                 let value = parseInt(valueStr, 10);
                 let updateSlider = false;
                 if (!isNaN(value)) {
                     value = Math.max(0, Math.min(100, value));
                     updateSlider = true;
                 }
                 if (updateSlider && document.activeElement === emergencyDesiredValue) {
                    if(emergencySlider) emergencySlider.value = value; // Update slider visually
                 }
             } else {
                 // If sync somehow re-enabled during input (unlikely), resync
                 syncEmergencyControls();
             }
        });
    }
    // Number Input 'change' event
    if (emergencyDesiredValue) {
         emergencyDesiredValue.addEventListener('change', () => {
            handleEmergencyInteraction(); // Ensure sync is off
             if (!emergencySyncToggle || !emergencySyncToggle.checked) {
                 let value = parseInt(emergencyDesiredValue.value, 10);
                 if (isNaN(value) || value < 0) value = 0;
                 if (value > 100) value = 100;
                 emergencyDesiredValue.value = value; // Update input with clamped value
                 if(emergencySlider) emergencySlider.value = value;    // Update slider
                 updateIcon(emergencyBrightnessIcon, value); // Update icon
                 if (value > 0) lastEmergencyBrightness = value; // Store last non-zero value
                 sendControlUpdate({ targetEmergency: value }); // Send update
             }
         });
    }

    // --- Settings Controls ---
    // Ramp Rate Input (Example, currently commented out)
    // if (rampRateInput) {
    //     rampRateInput.addEventListener('change', () => {
    //         let rate = parseFloat(rampRateInput.value);
    //         // Validate: must be positive, non-zero
    //          if (isNaN(rate) || rate <= 0) {
    //              rate = 0.1; // Set to a minimum default if invalid
    //              rampRateInput.value = rate.toFixed(1); // Update input field
    //              alert("Lysstyrkehastighet (ramp rate) må være et positivt tall (min 0.1).");
    //          } else {
    //             rampRateInput.value = rate.toFixed(1); // Format to one decimal place
    //          }
    //         sendControlUpdate({ rampRate: rate }); // Send the valid rate
    //     });
    // }

    // --- ON/OFF Buttons ---
    document.querySelectorAll('.btn-on').forEach(button => {
        button.addEventListener('click', () => {
            const target = button.dataset.target; // 'main' or 'emergency'

            if (target === 'main') {
                // Set to last known brightness, or 100% if none
                const value = lastMainBrightness > 0 ? lastMainBrightness : 100;
                if(mainSlider) mainSlider.value = value;
                if(mainDesiredValue) mainDesiredValue.value = value;
                updateIcon(mainBrightnessIcon, value);
                sendControlUpdate({ targetMain: value });

                // Handle sync: If sync is on, turn emergency ON to the same value
                if (emergencySyncToggle && emergencySyncToggle.checked) {
                     setEmergencySyncState(true); // Ensure sync visual state is correct
                     if(emergencySlider) emergencySlider.value = value;
                     if(emergencyDesiredValue) emergencyDesiredValue.value = value;
                     updateIcon(emergencyBrightnessIcon, value);
                     lastEmergencyBrightness = value; // Update last emergency value as well
                     console.log("Main 'ON' clicked (sync ON), sending emergency update.");
                     sendControlUpdate({ targetEmergency: value });
                }
            } else if (target === 'emergency') {
                 handleEmergencyInteraction(); // Disable sync if active
                 // Only act if sync is OFF
                 if (!emergencySyncToggle || !emergencySyncToggle.checked) {
                     const value = lastEmergencyBrightness > 0 ? lastEmergencyBrightness : 100;
                     if(emergencySlider) emergencySlider.value = value;
                     if(emergencyDesiredValue) emergencyDesiredValue.value = value;
                     updateIcon(emergencyBrightnessIcon, value);
                     sendControlUpdate({ targetEmergency: value });
                 }
            }
        });
    });

    document.querySelectorAll('.btn-off').forEach(button => {
        button.addEventListener('click', () => {
            const target = button.dataset.target; // 'main' or 'emergency'

             if (target === 'main') {
                // Store the current value before setting to 0
                const currentVal = mainSlider ? parseInt(mainSlider.value, 10) : 0;
                if (currentVal > 0) lastMainBrightness = currentVal;
                // Set controls to 0
                if(mainSlider) mainSlider.value = 0;
                if(mainDesiredValue) mainDesiredValue.value = 0;
                updateIcon(mainBrightnessIcon, 0);
                sendControlUpdate({ targetMain: 0 });

                // Handle sync: If sync is on, turn emergency OFF as well
                if (emergencySyncToggle && emergencySyncToggle.checked) {
                    setEmergencySyncState(true); // Ensure sync visual state is correct
                    if(emergencySlider) emergencySlider.value = 0;
                    if(emergencyDesiredValue) emergencyDesiredValue.value = 0;
                    updateIcon(emergencyBrightnessIcon, 0);
                    // Don't update lastEmergencyBrightness when turning off via main sync
                    console.log("Main 'OFF' clicked (sync ON), sending emergency update.");
                    sendControlUpdate({ targetEmergency: 0 });
                }
            } else if (target === 'emergency') {
                 handleEmergencyInteraction(); // Disable sync if active
                 // Only act if sync is OFF
                 if (!emergencySyncToggle || !emergencySyncToggle.checked) {
                     const currentVal = emergencySlider ? parseInt(emergencySlider.value, 10) : 0;
                     if (currentVal > 0) lastEmergencyBrightness = currentVal;
                     if(emergencySlider) emergencySlider.value = 0;
                     if(emergencyDesiredValue) emergencyDesiredValue.value = 0;
                     updateIcon(emergencyBrightnessIcon, 0);
                     sendControlUpdate({ targetEmergency: 0 });
                 }
            }
        });
    });

    // --- View Toggle Buttons (Numeric/Graph) ---
    document.querySelectorAll('.btn-toggle-view').forEach(button => {
        button.addEventListener('click', () => {
            const target = button.dataset.target; // 'main' or 'emergency'
            const numericView = document.getElementById(`${target}NumericReadings`);
            const graphView = document.getElementById(`${target}GraphReadings`);
            const chart = (target === 'main') ? mainChart : emergencyChart;

            if (!numericView || !graphView) {
                console.error(`Could not find numeric or graph view elements for target: ${target}`);
                return;
            }

            // Toggle the 'active' class and button text
            if (graphView.classList.contains('active')) {
                // Switch to Numeric view
                graphView.classList.remove('active');
                numericView.classList.add('active');
                button.textContent = "Vis Graf";
            } else {
                // Switch to Graph view
                numericView.classList.remove('active');
                graphView.classList.add('active');
                button.textContent = "Vis Numerisk";
                 // IMPORTANT: Resize chart after its container becomes visible
                 // Use requestAnimationFrame to ensure DOM reflow has completed
                 requestAnimationFrame(() => {
                      if (chart && chart.ctx) { // Check if chart and its context exist
                          try {
                              chart.resize(); // Tell Chart.js to resize to its container
                          } catch (e) {
                              console.warn(`Error resizing chart '${target}':`, e);
                          }
                      } else {
                          console.warn(`Chart or context for '${target}' not ready for resize.`);
                      }
                 });
            }
        });
    });

    console.log("Event listeners attached.");

    // ==========================================================================
    // ==                         Initialization Calls                         ==
    // ==========================================================================
    console.log("Performing initial setup...");

    initCharts(); // Create chart instances

    // Set initial sync state based on checkbox default
    if (emergencySyncToggle && emergencyCard) {
       setEmergencySyncState(emergencySyncToggle.checked);
    } else {
        console.warn("Could not perform initial sync setup: Toggle or Card element missing.");
    }

    // Set initial brightness icons based on default slider values
    if (mainSlider && mainBrightnessIcon) {
        updateIcon(mainBrightnessIcon, mainSlider.value);
    }
    if (emergencySlider && emergencyBrightnessIcon) {
        // Use the value determined by the initial sync state check
        updateIcon(emergencyBrightnessIcon, emergencySlider.value);
    }

    setupSSE(); // Start the Server-Sent Events connection

    console.log("Initial setup complete. AquaAlert UI is active.");

}); // --- End DOMContentLoaded ---

// script.js END