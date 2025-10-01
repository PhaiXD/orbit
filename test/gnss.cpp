#include <Arduino.h>
#include "orbit_pin_def.h"
#include <TinyGPS++.h>

HardwareSerial gnssSerial(PIN_RX, PIN_TX);
TinyGPSPlus lc86;

unsigned long delayTime;

void printValues();

void setup() {
    Serial.begin(115200);
    Serial.println(F("GNSS test"));
    gnssSerial.begin(115200);

    delayTime = 2000;

    Serial.println();
}


void loop() {    
    printValues();
    delay(delayTime);
}


void printValues() {
 
    while (gnssSerial.available())
    {
        lc86.encode(gnssSerial.read());
        Serial.print("Time: ");
        Serial.println(lc86.time.value());
    }

    if (lc86.location.isUpdated())
    {
        Serial.print("Latitude: ");
        Serial.println(lc86.location.lat(), 6);

        Serial.print("Longitude: ");
        Serial.println(lc86.location.lng(), 6);

        Serial.print("Altitude: ");
        Serial.println(lc86.altitude.meters(), 2);
    }

    
}