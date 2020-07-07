#include <TTN_esp32.h>
#include "TTN_CayenneLPP.h"
#include "WiFi.h"

const char *devAddr = "26011BEB";
const char *nwkSKey = "C495DF113E9077C9728FF3DB1B366825";
const char *appSKey = "53941C23185BB05CAD8E63E32BE121B8";

//const char* devAddr = "26011BEB"; // Change to TTN Device Address
//const char* nwkSKey = "C495DF113E9077C9728FF3DB1B366825"; // Change to TTN Network Session Key
//const char* appSKey = "53941C23185BB05CAD8E63E32BE121B8"; // Change to TTN Application Session Key

TTN_esp32 ttn ;
TTN_CayenneLPP lpp;

void message(const uint8_t* payload, size_t size, int rssi)
{
    Serial.println("-- MESSAGE");
    Serial.print("Received " + String(size) + " bytes RSSI= " + String(rssi) + "dB");

    for (int i = 0; i < size; i++)
    {
        Serial.print(" " + String(payload[i]));
        // Serial.write(payload[i]);
    }

    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    WiFi.mode(WIFI_OFF);
    delay(2000);
    Serial.println("Starting");
    ttn.begin();
    ttn.onMessage(message); // declare callback function when is downlink from server
    Serial.print("Starting personalize step");
    ttn.personalize(devAddr, nwkSKey, appSKey);
    Serial.print("Starting ShowStatus");
    ttn.showStatus();
}

void loop()
{
    static float nb = 18.2;
    nb += 0.1;
    lpp.reset();
    lpp.addTemperature(1, nb);
    if (ttn.sendBytes(lpp.getBuffer(), lpp.getSize()))
    {
        Serial.printf("Temp: %f TTN_CayenneLPP: %d %x %02X%02X\n", nb, lpp.getBuffer()[0], lpp.getBuffer()[1],
            lpp.getBuffer()[2], lpp.getBuffer()[3]);
    }
    delay(10000);
}
