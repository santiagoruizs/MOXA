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
const char *WIFI_SSID = "vodafone0D82";
const char *WIFI_PASSWORD = "32HYLHGMDBDY9R";
char macAddress[18];

const char *MQTT_BROKER_IP = "192.168.0.31";
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
  if (elapsedTime >= 2000) {
    DHT_temp();
    DHT_hum();
    GPS_lat();
    GPS_long();
    publishBigJson();     // Publishes a big json
    startTime = nowTime;
  static float temperature;
  
  }
}

/* Additional functions */

void DHT_temp() {
  static float temperature;
  temperature = dhtSensor.readTemperature();
  static const String topicStr = createTopic("temp");
  static const char *topic = topicStr.c_str();

  mqttClient.publish(topic, String(temperature).c_str(), RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(temperature));
}
void DHT_hum() {
  static float humidity;
  humidity = dhtSensor.readHumidity();
  static const String topicStr = createTopic("hum");
  static const char *topic = topicStr.c_str();

  mqttClient.publish(topic, String(humidity).c_str(), RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(humidity));
}
void GPS_lat() {
  if (SerialGPS.available()) {
    gps.encode(SerialGPS.read()); // Encodes all messages from GPS
  }
  static float lati = gps.location.lat();
  static const String topicStr = createTopic("lat");
  static const char *topic = topicStr.c_str();

  mqttClient.publish(topic, String(lati).c_str(), RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(lati));
}
void GPS_long() {
  if (SerialGPS.available()) {
    gps.encode(SerialGPS.read()); // Encodes all messages from GPS
  }
  static float longi = gps.location.lng();
  static const String topicStr = createTopic("long");
  static const char *topic = topicStr.c_str();

  mqttClient.publish(topic, String(longi).c_str(), RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(longi));
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
