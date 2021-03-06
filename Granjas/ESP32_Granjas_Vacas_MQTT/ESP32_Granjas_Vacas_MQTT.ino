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

DHT dhtSensor(DHT_PIN, DHT_TYPE); // Defines the sensor dht
HardwareSerial SerialGPS(1);
TinyGPSPlus gps;

// Replace the next variables with your Wi-Fi SSID/Password
const char *WIFI_SSID = "MIWIFI_2G_Hekd";
const char *WIFI_PASSWORD = "CEtTpYNd";
char macAddress[18];

const char *MQTT_BROKER_IP = "20.50.173.151";
const int MQTT_PORT = 1883;
const bool RETAINED = true;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  Serial.begin(9600); // Starts the serial communication
  Serial.println("\nBooting device...");

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
  if (elapsedTime >= 10000) {
    publishVaca("vaca1");
    publishVaca("vaca2");
    publishVaca("vaca3");
    publishVaca("vaca4");
    publishVaca("vaca5");
    publishGranja("mas_gener");
    publishGranja("can_siset");
    publishGranja("mas_almar");
    publishGranja("mas_badosa");
    publishGranja("mas_colom");
    startTime = nowTime;
  static float temperature;
  temperature = dhtSensor.readTemperature(); // Reads the temperature, it takes
                                             // about 250 milliseconds
  Serial.println("Temperature: " + String(temperature) + "°C"); // Prints in a new line the result
  }
}

/* Additional functions */

void publishVaca(char *tpc) {
  int randTemp = 3 - random(6);
  int randLeche = 20 - random(40);
  static float temperature;
  static float humidity;
  temperature = dhtSensor.readTemperature();
  humidity = dhtSensor.readHumidity();
  const String topicStr = createTopic(tpc);
  const char *topic = topicStr.c_str();
  if (SerialGPS.available()) {
    gps.encode(SerialGPS.read()); // Encodes all messages from GPS
  }
  
  StaticJsonDocument<128> doc; // Create JSON document of 128 bytes
  char buffer[128]; // Create the buffer where we will print the JSON document
                    // to publish through MQTT
  doc["temp"] = temperature + 8 + randTemp;
  doc["leche"] = 20 + randLeche;  
 
  doc["lat"] = gps.location.lat();
  doc["long"] = gps.location.lng();
  // Serialize the JSON document to a buffer in order to publish it
serializeJson(doc, buffer);
  mqttClient.publish(topic, buffer, RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(buffer));
}
void publishGranja(char *tpc) {
  int randTemp = 3 - random(6);
  int randHum = 5 - random(10);
  int randLeche = 25 - random(50);
  int randValla = random(2);
  static float temperature;
  static float humidity;
  temperature = dhtSensor.readTemperature();
  humidity = dhtSensor.readHumidity();
  const String topicStr = createTopic(tpc);
  const char *topic = topicStr.c_str();
  if (SerialGPS.available()) {
    gps.encode(SerialGPS.read()); // Encodes all messages from GPS
  }

  StaticJsonDocument<128> doc; // Create JSON document of 128 bytes
  char buffer[128]; // Create the buffer where we will print the JSON document
                    // to publish through MQTT

  doc["temp"] = temperature + randTemp;
  doc["hum"] = humidity + randHum;  
 
  doc["leche"] = 50 + randLeche;
  doc["Valla"] = randValla;
  // Serialize the JSON document to a buffer in order to publish it
serializeJson(doc, buffer);
  mqttClient.publish(topic, buffer, RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(buffer));
}
String createTopic(char *topic) {
  String topicStr = String(macAddress) + "/" + topic;
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
