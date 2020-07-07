#include <ArduinoJson.h>  // https://arduinojson.org/
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient
#include <WiFi.h>
#include <Wire.h>
#include "DHT.h" // Include DHT library
#include <HardwareSerial.h>
#include <TinyGPS++.h>

#define RX_PIN 16 // Pinout RX of ESP32
#define TX_PIN 17 // Pinout TX of ESP32

#define DHT_PIN 2     // Defines pin number to which the sensor is connected
#define DHT_TYPE DHT11 // Defines the sensor type. It can be DHT11 or DHT22

#define ECHO_PIN 12 // Analog input that receives the echo signal
#define TRIG_PIN 13 // Digital output that sends the trigger signal

DHT dhtSensor(DHT_PIN, DHT_TYPE); // Defines the sensor dht
HardwareSerial SerialGPS(1);
TinyGPSPlus gps;

// Replace the next variables with your Wi-Fi SSID/Password
const char *WIFI_SSID = "MIWIFI_2G_Hekd";
const char *WIFI_PASSWORD = "CEtTpYNd";
char macAddress[18];

const char *MQTT_BROKER_IP = "192.168.1.252";
const int MQTT_PORT = 1883;
const bool RETAINED = true;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  Serial.begin(9600); // Starts the serial communication
  Serial.println("\nBooting device...");
  pinMode(ECHO_PIN, INPUT);  // Sets the ECHO_PIN as an Input
  pinMode(TRIG_PIN, OUTPUT); // Sets the TRIG_PIN as an Output

  mqttClient.setServer(MQTT_BROKER_IP,
                       MQTT_PORT); // Connect the configured mqtt broker

  connectToWiFiNetwork(); // Connects to the configured network
  connectToMqttBroker();  // Connects to the configured mqtt broker
  dhtSensor.begin();

  SerialGPS.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Starts gps communication with UART


}

void loop() {
  checkConnections(); // We check the connection every time
  
  // Publish every 2 seconds
  static int nowTime = millis();
  static int startTime = 0;
  static int elapsedTime = 0;
  nowTime = millis();
  elapsedTime = nowTime - startTime;
  if (elapsedTime >= 2000) {
    publishBigJson("TemNav");
    publishBigJson("HumNav");
    publishBigJson("TempOfc");
    publishBigJson("HumOfc");
    publishBigJson("cuba1");    
    publishBigJson("cuba2");
    publishBigJson("cuba3");
    publishBigJson("cuba4");
    publishBigJson("cuba5");// Publishes a big json
    startTime = nowTime;
  static float temperature;
  temperature = dhtSensor.readTemperature(); // Reads the temperature, it takes
                                             // about 250 milliseconds
  Serial.println("Temperature: " + String(temperature) + "°C"); // Prints in a new line the result
  }
}

/* Additional functions */

float getDistance() {
  digitalWrite(TRIG_PIN, LOW); // Clear the TRIG_PIN by setting it LOW
  delayMicroseconds(5);

  // Trigger the sensor by setting the TRIG_PIN to HIGH for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH); // pulseIn() returns the duration (length of the pulse) in microseconds

  return duration * 0.034 / 2; // Returns the distance in cm
}

void publishBigJson(char *tpc) {
 
  static float distance;

  distance = getDistance();
  
  static float temperature;
  static float humidity;

  temperature = dhtSensor.readTemperature();
  humidity = dhtSensor.readHumidity();
  const String topicStr = createTopic(tpc);
  const char *topic = topicStr.c_str();
  if (SerialGPS.available()) {
    gps.encode(SerialGPS.read()); // Encodes all messages from GPS
  }

  DynamicJsonDocument doc(2048); // Create JSON document
  char buffer[2048]; // Create the buffer where we will print the JSON document
                     // to publish through MQTT

  doc["device"] = "ESP32"; // Add names and values to the JSON document
  doc["sensor"] = "DHT22";
  doc["sensor"] = "Ultras";
  JsonObject Ultras = doc.createNestedObject("Ultras"); // Add another Object
  Ultras["cm"] = distance;
  JsonObject sDHT11 = doc.createNestedObject("DHT11"); // Add another Object
  sDHT11["t"] = temperature;
  sDHT11["h"] = humidity;
  JsonObject sGPS = doc.createNestedObject("GPS"); // Add another Object
  sGPS["lat"] = gps.location.lat();
  sGPS["long"] = gps.location.lng();
  // Serialize the JSON document to a buffer in order to publish it
  size_t n = serializeJson(doc, buffer);
  mqttClient.publish_P(topic, buffer, n); // No RETAINED option
  Serial.println(" <= " + String(topic) + ": " + String(buffer));
}

String createTopic(char *topic) {
  String topicStr = topic;
  return topicStr;
}

void connectToWiFiNetwork() {
  Serial.print(
      "Connecting with Wi-Fi: " +
      String(WIFI_SSID)); // Print the network which you want to connect
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".."); // Connecting effect
  }
  Serial.print("..connected!  (ip: "); // After being connected to a network,
                                       // our ESP32 should have a IP
  Serial.print(WiFi.localIP());
  Serial.println(")");
  String macAddressStr = WiFi.macAddress().c_str();
  strcpy(macAddress, macAddressStr.c_str());
}

void connectToMqttBroker() {
  Serial.print(
      "Connecting with MQTT Broker:" +
      String(MQTT_BROKER_IP));    // Print the broker which you want to connect
  mqttClient.connect(macAddress);// Using unique mac address from ESP32
  while (!mqttClient.connected()) {
    delay(500);
    Serial.print("..");             // Connecting effect
    mqttClient.connect(macAddress); // Using unique mac address from ESP32
  }
  Serial.println("..connected! (ClientID: " + String(macAddress) + ")");
}

void checkConnections() {
  if (mqttClient.connected()) {
    mqttClient.loop();
  } else { // Try to reconnect
    Serial.println("Connection has been lost with MQTT Broker");
    if (WiFi.status() != WL_CONNECTED) { // Check wifi connection
      Serial.println("Connection has been lost with Wi-Fi");
      connectToWiFiNetwork(); // Reconnect Wifi
    }
    connectToMqttBroker(); // Reconnect Server MQTT Broker
  }
}
