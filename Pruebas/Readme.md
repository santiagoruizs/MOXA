# Pruebas

En esta carpeta se añaden los siguientes archivos de prueba:
1. TTN_ESP32.ino: conexión LoRa vía OTAA (se descarta este código a favor de la comunicación ABP)
1. TTN_ESP32_ABP.ino: conexión LoRa vía ABP (se descarta este código a favor de ESP32_MQTT_PruebasEntregaFinal_1.ino)
1. GPS_bueno: pruebas de sleep + recuperar conexión con GPS. Se comprueba que habiendo encontrado señal de GPS, la lectura después de la reconexión toma unos pocos segundos
1. LoraWanABP.ino: prueba funcional de envío de mensaje HelloWorld a TTN 
1. ESP32_MQTT_PruebasEntregaFinal_1.ino: archivo de prueba para las granjas. Se recogen datos de GPS y de temperatura/humedad. Se trabaja con distintas comprobaciones para evitar enviar datos errónoes, además se separa la lectura del GPS en un loop distinto. (A tener en cuenta que es necesaria una modificación de la librería lmic para que funcione correctamente)

[[Go back]](/README.md)

Se añade el código de las dos pruebas finales más significativas
### [Code](ESP32_MQTT_PruebasEntregaFinal_1.ino)
```cpp
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
const char *WIFI_SSID = "AulaAutomatica";
const char *WIFI_PASSWORD = "ticsFcim";
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
    
    //Serial.print("LAT=");   Serial.println(gps.location.lat(), 6); // Latitude in degrees (double)
    //Serial.print("LONG=");  Serial.println(gps.location.lng(), 6); // Longitude in degrees (double)    
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
```
[[Go to top]](#Pruebas)

### [Code](LoraWanABP.ino)
Para el uso de este código se deja como referencia el tutorial http://akirasan.net/nodo-lorawan-con-esp32/
```cpp
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0xC4, 0x95, 0xDF, 0x11, 0x3E, 0x90, 0x77, 0xC9, 0x72, 0x8F, 0xF3, 0xDB, 0x1B, 0x36, 0x68, 0x25 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x53, 0x94, 0x1C, 0x23, 0x18, 0x5B, 0xB0, 0x5C, 0xAD, 0x8E, 0x63, 0xE3, 0x2B, 0xE1, 0x21, 0xB8 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26011BEB; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32}
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
```

[[Go to top]](#Pruebas)
