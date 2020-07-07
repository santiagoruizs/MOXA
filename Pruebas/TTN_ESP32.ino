#include "TTN_esp32.h"
#include "TTN_CayenneLPP.h"

const char* devEui = "00B86926B917D3E8"; // Change to TTN Device EUI
const char* appEui = "70B3D57ED00317B1"; // Change to TTN Application EUI
const char* appKey = "C05FF177BE8D10602C9E5280A3CC2443"; // Chaneg to TTN Application Key

TTN_esp32 ttn ;
TTN_CayenneLPP lpp;

void message(const uint8_t* payload, size_t size, int rssi)
{
    Serial.println("-- MESSAGE");
    Serial.print("Received " + String(size) + " bytes RSSI=" + String(rssi) + "db");
    for (int i = 0; i < size; i++)
    {
        Serial.print(" " + String(payload[i]));
        // Serial.write(payload[i]);
    }

    Serial.println();
}

void setup()
{
    Serial.begin(9600);
    delay(2000);
    Serial.println("Starting");
    ttn.begin();
    ttn.onMessage(message); // Declare callback function for handling downlink
                            // messages from server
    
    Serial.println("Starting JOIN TTN ");
    //ttn.join(devEui, appEui, appKey);
    ttn.join(devEui, appEui, appKey, -1, 3000);
    Serial.print("Joining TTN ");
    while (!ttn.isJoined())
    {
        Serial.print(".");
        delay(500);
    }
    Serial.println("\njoined !");
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
