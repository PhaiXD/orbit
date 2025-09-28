#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
// #include "Click_GNSS_types.h"
// #include "Click_GNSS_config.h"
// #include "Click_GNSS_timer.h"

// Lora
#define NSS_PIN PB12
#define NRST_PIN PB10
#define DIO1_PIN PB9
#define BUSY_PIN PB8

// Bme280
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;

TinyGPSPlus gps;
HardwareSerial GPS_Serial(USART2);

TwoWire myWire(PB7, PB6);   // SDA = PB7, SCL = PB6

void displayInfo();

int i = 0;

void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600);
  myWire.begin();
  
  if (!bme.begin(0x76, &myWire)) {
    Serial.println("BME280 not found!");
    while (1) delay(1000);
  }
}

void loop(){
  i += 1;

  // ----- Bme280 ------

  Serial.print("#");
  Serial.println(i);

  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");

  Serial.print("Pressure = ");

  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();

  // -----------

  displayInfo();
  Serial.println("run");

  delay(1000);
}

void displayInfo() {
  Serial.print(F("Location: ")); 
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}