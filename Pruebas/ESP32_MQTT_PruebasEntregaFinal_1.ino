#include <ArduinoJson.h>  // https://arduinojson.org/
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient
#include <WiFi.h>
#include <Wire.h>
#include "DHT.h" // Include DHT library
#include <HardwareSerial.h>
#include <TinyGPS++.h>

#define RX_PIN 16 // Pinout RX of ESP32
#define TX_PIN 17 // Pinout TX of ESP32
#define REFRESH_RATE 1000 // Defined in miliseconds
#define SLEEP_TIME 10 //Defined in seconds

#define DHT_PIN 14     // Defines pin number to which the sensor is connected
#define DHT_TYPE DHT11 // Defines the sensor type. It can be DHT11 or DHT22

DHT dhtSensor(DHT_PIN, DHT_TYPE); // Defines the sensor dht
HardwareSerial SerialGPS(1);
TinyGPSPlus gps;

//Global variables
static bool GPSread; //Controls the GPS data has been obtained
static bool DHTread; //Controls the DHT data has been obtained
static bool GPSready;
static float temperature;
static float humidity;
static float latGPS;
static float longGPS;

// Replace the next variables with your Wi-Fi SSID/Password
//const char *WIFI_SSID = "AulaAutomatica";
//const char *WIFI_PASSWORD = "ticsFcim";
const char *WIFI_SSID = "MiFibra-1843";
const char *WIFI_PASSWORD = "jSi9ewER";
char macAddress[18];

//const char *MQTT_BROKER_IP = "10.20.60.5";
const char *MQTT_BROKER_IP = "20.50.173.151";
const int MQTT_PORT = 1883;
const bool RETAINED = true;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  Serial.begin(9600); // Starts the serial communication
  delay(1000);
  Serial.println("\nBooting device...");

  mqttClient.setServer(MQTT_BROKER_IP,
                       MQTT_PORT); // Connect the configured mqtt broker

  //Enable comms
  connectToWiFiNetwork(); // Connects to the configured network
  connectToMqttBroker();  // Connects to the configured mqtt broker

  //Starting devices comms
  dhtSensor.begin();
  SerialGPS.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Starts gps communication with UART

  esp_sleep_enable_timer_wakeup(SLEEP_TIME * 1000000);  // Setup to wake up after SLEEP_TIME seconds, defined in microseconds
  
  TimerHandle_t xTimer = xTimerCreate("readGPSData", REFRESH_RATE, pdTRUE, (void *) 0, readGPSData);
  xTimerStart(xTimer, 0);
}

void loop() {
  //delay(1500);
  checkConnections(); // We check the connection every time

  //Methods to get data from devices
  GPSready = false;
  if (SerialGPS.available()) {
    gps.encode(SerialGPS.read()); // Encodes all messages from GPS
    GPSready = true;
  }
  //readGPSData();
  readDHTData();

  if(GPSread and DHTread){
    Serial.println("Data is available to be sent");
    publishCow();
  }
  else if(GPSread){
    Serial.println("Data from DHT not available");
  }
  else{
    Serial.println("Data from GPS not available");
  }
}

/* Additional functions */
void readGPSData(TimerHandle_t xTimer){
  if(GPSready){
    GPSread = false;
    latGPS = gps.location.lat();
    longGPS = gps.location.lng();
  
    if(latGPS and longGPS){
      GPSread = true; 
    }   
  }
}

void readDHTData(){
  DHTread = false;
  temperature = dhtSensor.readTemperature();
  humidity = dhtSensor.readHumidity();
  if(temperature and humidity){
    DHTread = true;
  }
}

void publishCow(){
  static const String topicStr = createTopic("cow");
  static const char *topic = topicStr.c_str();
  
  DynamicJsonDocument data2send(2048); // Create JSON
  char buffer[2048]; // Create the buffer where we will print the JSON document
                     // to publish through MQTT

  JsonObject sDHT11 = data2send.createNestedObject("DHT11"); // Add another Object
  sDHT11["t"] = temperature;
  sDHT11["h"] = humidity;
  JsonObject sGPS = data2send.createNestedObject("GPS"); // Add another Object
  sGPS["lat"] = latGPS;
  sGPS["long"] = longGPS;
  // Serialize the JSON document to a buffer in order to publish it
  size_t n = serializeJson(data2send, buffer);
  mqttClient.publish_P(topic, buffer, n); // No RETAINED option
  Serial.println(" <= " + String(topic) + ": " + String(buffer));

  Serial.println("Going to sleep for " + String(SLEEP_TIME) + " seconds");
  Serial.flush();  // Clears all unsent serial information
  esp_deep_sleep_start();  // Set the ESP32 to deep sleep mode
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
    //Serial.println("Connected MQTT Broker!");
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
