#include <ESP8266WiFi.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <HDC2080.h>

// Configuration constants
#define MAX_WIFI_RETRY 200          // Maximum WiFi connection attempts
#define DONE_PIN 13                 // Status indicator pin
#define HDC2080_ADDR 0x40          // HDC2080 I2C address
#define CHECK_UPDATE_INTERVAL 12    // Update check interval
#define RTC_MEM_ADDR 16            // RTC memory start address
#define JSON_BUFFER_SIZE 1024      // MQTT JSON buffer size
#define MEASUREMENT_DELAY 300      // Delay before signaling completion

// Sensor configuration
#define HIGH_TEMP_THRESHOLD 28      // High temperature threshold (°C)
#define LOW_TEMP_THRESHOLD 22       // Low temperature threshold (°C)
#define HIGH_HUMIDITY_THRESHOLD 55  // High humidity threshold (%)
#define LOW_HUMIDITY_THRESHOLD 40   // Low humidity threshold (%)
#define MIN_VALID_TEMP -39.0       // Minimum valid temperature reading

// Version and OTA configuration
#define VERSION "v3.8"              // Firmware version
#define APP_NAME "SmartTS2"         // Application name
const char* OTA_UPDATE_URL = "http://updateserver";

// Network configuration - REPLACE WITH YOUR CREDENTIALS
const char* WIFI_SSID = "";         // Your WiFi network name
const char* WIFI_PASSWORD = "";     // Your WiFi password
const char* MQTT_SERVER = "";  // MQTT broker IP address
const int MQTT_PORT = 1883;         // MQTT broker port
const char* MQTT_USER = "";         // MQTT username (if required)
const char* MQTT_PASSWORD = "";     // MQTT password (if required)

// Global variables
HDC2080 sensor(HDC2080_ADDR);      // Temperature/humidity sensor
WiFiClient wifiClient;              // WiFi client
PubSubClient mqttClient(wifiClient); // MQTT client

// Device identification
String deviceName;                  // Unique device name
String clientId = "ESP8266Client-"; // MQTT client ID
String baseUniqueId;               // Base unique identifier from MAC
byte macAddress[6];                // Device MAC address

// Sensor data
float temperature = 0.0;           // Temperature reading
float humidity = 0.0;              // Humidity reading
int adcValue = 0;                  // ADC reading for battery voltage
float batteryVoltage = 0.0;        // Calculated battery voltage
int wifiRetryCount = 0;            // WiFi connection retry counter

/**
 * Check for firmware updates via HTTP
 */
void checkForFirmwareUpdate() {
    String updateUrl = String(OTA_UPDATE_URL);
    updateUrl += "?ver=" + String(VERSION);
    updateUrl += "&dev=" + String(APP_NAME);

    WiFiClient client;
    t_httpUpdate_return result = ESPhttpUpdate.update(client, updateUrl);

    switch (result) {
        case HTTP_UPDATE_FAILED:
            // Update failed - could log error if serial enabled
            break;
        case HTTP_UPDATE_NO_UPDATES:
            // No updates available
            break;
        case HTTP_UPDATE_OK:
            // Update successful - device will restart
            break;
    }
}

/**
 * Initialize WiFi connection
 * @return 0 on success, -1 on failure
 */
int initializeWiFi() {
    delay(10);

    // Return immediately if already connected
    if (WiFi.status() == WL_CONNECTED) {
        return 0;
    }

    // Begin WiFi connection
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    // Wait for connection with retry limit
    while (WiFi.status() != WL_CONNECTED) {
        delay(200);
        wifiRetryCount++;
        
        if (wifiRetryCount > MAX_WIFI_RETRY) {
            return -1; // Connection failed
        }
    }

    // Get MAC address and create unique identifiers
    WiFi.macAddress(macAddress);
    clientId += WiFi.macAddress();
    
    // Create base unique ID from MAC address
    baseUniqueId = "";
    for (int i = 0; i < 6; i++) {
        if (macAddress[i] < 16) baseUniqueId += "0";
        baseUniqueId += String(macAddress[i], HEX);
    }
    
    deviceName = "esp8266_sensor_" + baseUniqueId;

    return 0; // Success
}

/**
 * Connect to MQTT broker and publish sensor configurations for Home Assistant
 * @return 0 on success, -1 on failure
 */
int connectToMQTT() {
    mqttClient.setBufferSize(JSON_BUFFER_SIZE);

    // Attempt MQTT connection
    if (!mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
        return -1; // Connection failed
    }

    StaticJsonDocument<JSON_BUFFER_SIZE> doc;
    String jsonPayload;

    // Publish temperature sensor configuration for Home Assistant
    doc.clear();
    doc["device_class"] = "temperature";
    //doc["name"] = "ESP8266 Temperature " + baseUniqueId;
    doc["name"]= "esp8266_sensor_temperature_"+baseUniqueId;
    doc["state_topic"] = "homeassistant/sensor/" + deviceName + "/temperature/state";
    doc["unique_id"] = baseUniqueId + "_temp";
    doc["unit_of_measurement"] = "°C";
    doc["value_template"] = "{{ value_json.temperature | round(1) }}";

    serializeJson(doc, jsonPayload);
    if (!mqttClient.publish(("homeassistant/sensor/" + deviceName + "/temperature/config").c_str(), 
                           jsonPayload.c_str(), true)) {
        return -1; // Failed to publish
    }

    delay(100);

    // Publish humidity sensor configuration for Home Assistant
    doc.clear();
    doc["device_class"] = "humidity";
    //doc["name"] = "ESP8266 Humidity " + baseUniqueId;
    doc["name"]= "esp8266_sensor_humidity_"+baseUniqueId;
    doc["state_topic"] = "homeassistant/sensor/" + deviceName + "/humidity/state";
    doc["unique_id"] = baseUniqueId + "_hum";
    doc["unit_of_measurement"] = "%";
    doc["value_template"] = "{{ value_json.humidity | round(1) }}";

    serializeJson(doc, jsonPayload);
    if (!mqttClient.publish(("homeassistant/sensor/" + deviceName + "/humidity/config").c_str(), 
                           jsonPayload.c_str(), true)) {
        return -1; // Failed to publish
    }

    delay(100);

    // Publish device info sensor configuration
    doc.clear();
    doc["name"] = "ESP8266 Info " + baseUniqueId;
    doc["state_topic"] = "homeassistant/sensor/" + deviceName + "/info/state";
    doc["unique_id"] = baseUniqueId + "_info";

    serializeJson(doc, jsonPayload);
    mqttClient.publish(("homeassistant/sensor/" + deviceName + "/info/config").c_str(), 
                      jsonPayload.c_str(), true);

    return 0; // Success
}

/**
 * Round value to 1 decimal place
 */
double roundToOneDecimal(double value) {
    return round(value * 10.0) / 10.0;
}

/**
 * Round value to 2 decimal places
 */
double roundToTwoDecimals(double value) {
    return round(value * 100.0) / 100.0;
}

/**
 * Publish sensor data to MQTT
 * @return 0 on success, -1 on failure
 */
int publishSensorData() {
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    
    // Connect to MQTT if not connected
    if (!mqttClient.connected()) {
        if (connectToMQTT() != 0) {
            return -1; // Connection failed
        }
    }

    // Ensure device name is set
    if (deviceName.length() <= 15) { // "esp8266_sensor_" is 15 chars
        WiFi.macAddress(macAddress);
        baseUniqueId = "";
        for (int i = 0; i < 6; i++) {
            if (macAddress[i] < 16) baseUniqueId += "0";
            baseUniqueId += String(macAddress[i], HEX);
        }
        deviceName = "esp8266_sensor_" + baseUniqueId;
    }

    StaticJsonDocument<JSON_BUFFER_SIZE> doc;
    String jsonPayload;

    // Publish temperature data
    doc["temperature"] = roundToOneDecimal(temperature);
    serializeJson(doc, jsonPayload);
    mqttClient.publish(("homeassistant/sensor/" + deviceName + "/temperature/state").c_str(), 
                      jsonPayload.c_str());

    delay(100);

    // Publish humidity data
    doc.clear();
    doc["humidity"] = round(humidity);
    serializeJson(doc, jsonPayload);
    mqttClient.publish(("homeassistant/sensor/" + deviceName + "/humidity/state").c_str(), 
                      jsonPayload.c_str());

    delay(100);

    // Publish device info
    doc.clear();
    doc["version"] = VERSION;
    doc["app"] = APP_NAME;
    doc["adc_value"] = adcValue;
    doc["battery_voltage"] = roundToTwoDecimals(batteryVoltage);
    doc["wifi_rssi"] = WiFi.RSSI(); 
    doc["wifi_retries"] = wifiRetryCount;
    serializeJson(doc, jsonPayload);
    mqttClient.publish(("homeassistant/sensor/" + deviceName + "/info/state").c_str(), 
                      jsonPayload.c_str());

    mqttClient.loop();
    return 0; // Success
}

/**
 * Initialize HDC2080 sensor with optimal settings
 */
void initializeSensor() {
    sensor.begin();
    sensor.reset(); // Reset sensor to default state

    // Configure comfort zone thresholds
    sensor.setHighTemp(HIGH_TEMP_THRESHOLD);
    sensor.setLowTemp(LOW_TEMP_THRESHOLD);
    sensor.setHighHumidity(HIGH_HUMIDITY_THRESHOLD);
    sensor.setLowHumidity(LOW_HUMIDITY_THRESHOLD);

    // Configure measurement settings
    sensor.setMeasurementMode(TEMP_AND_HUMID); // Measure both temperature and humidity
    sensor.setRate(ONE_HZ);                    // 1Hz measurement rate
    sensor.setTempRes(FOURTEEN_BIT);           // 14-bit temperature resolution
    sensor.setHumidRes(FOURTEEN_BIT);          // 14-bit humidity resolution

    // Start measurement
    sensor.triggerMeasurement();
}

/**
 * Read sensor values and battery voltage
 */
void readSensorData() {
    // Read temperature and humidity
    temperature = sensor.readTemp();
    humidity = sensor.readHumidity();

    // Read battery voltage from ADC
    adcValue = analogRead(A0);
    batteryVoltage = adcValue * (4.2 / 1023.0); // Convert ADC to voltage (assuming 4.2V max)
}

/**
 * Signal completion by toggling DONE_PIN
 */
void signalCompletion() {
    digitalWrite(DONE_PIN, HIGH);
    delay(100);
    digitalWrite(DONE_PIN, LOW);
    
    // Continuous signaling loop
    while (true) {
        digitalWrite(DONE_PIN, HIGH);
        delay(1);
        digitalWrite(DONE_PIN, LOW);
        delay(1);
    }
}

/**
 * Main setup function - runs once
 */
void setup() {
    // Initialize pins
    pinMode(DONE_PIN, OUTPUT);
    digitalWrite(DONE_PIN, LOW);

    // Initialize sensor
    initializeSensor();

    // Initialize WiFi connection
    if (initializeWiFi() == 0) {
        // Check for firmware updates if connected
        checkForFirmwareUpdate();
    }

    // Read sensor data
    readSensorData();

    // Publish data if temperature reading is valid and WiFi is connected
    if (temperature >= MIN_VALID_TEMP && WiFi.status() == WL_CONNECTED) {
        publishSensorData();
    }

    // Signal completion and enter infinite loop
    delay(MEASUREMENT_DELAY);
    signalCompletion();
}

/**
 * Main loop function - not used in this deep sleep application
 */
void loop() {
    // Empty - all work done in setup() before deep sleep
}