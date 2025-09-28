#include <Arduino.h>
#include <Wire.h>

// ----- LoRa -----
#include <SPI.h>
#include "RadioLib.h"
#define NSS_PIN PB12
#define NRST_PIN PB10
#define DIO1_PIN PB9
#define BUSY_PIN PB8

SPISettings spiSettings = SPISettings(4'000'000, MSBFIRST, SPI_MODE0);
SPIClass spi1(PB5, PB4, PA5);
SX1262 radio = new Module(NSS_PIN, DIO1_PIN, NRST_PIN, BUSY_PIN, spi1, spiSettings);
int transmissionState = RADIOLIB_ERR_NONE;
volatile bool transmittedFlag = false;

// ----- BME280 -----
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
TwoWire myWire(PB7, PB6);   // SDA = PB7, SCL = PB6

// // ----- GPS -----
// #include <TinyGPS++.h>
// #include <SoftwareSerial.h>
// TinyGPSPlus gps;
// struct GPSData {
//   double latitude = 0.0;
//   double longitude = 0.0;
//   float altitude = 0.0;
//   uint32_t timestamp = 0;
//   bool hasFix = false;
// } gpsData;
// HardwareSerial SerialGPS(2);


int i = 0;

constexpr struct
{
    float center_freq = 925.00'000f; // MHz
    float bandwidth = 125.f;         // kHz
    uint8_t spreading_factor = 9;    // SF: 6 to 12
    uint8_t coding_rate = 8;         // CR: 5 to 8
    uint8_t sync_word = 0x12;        // Private SX1262
    int8_t power = 22;               // up to 22 dBm for SX1262
    uint16_t preamble_length = 16;
} params;

void setFlag(void) {
  transmittedFlag = true; // LoRa
}

void setup() {
  Serial.begin(115200);
  // SerialGPS.begin(115200);
  myWire.begin();
  

  // ----- LoRa [Config] -----
  spi1.begin();
  pinMode(NSS_PIN, OUTPUT);
  digitalWrite(NSS_PIN, HIGH);
  int state = radio.begin();
  radio.setFrequency(915.0);
  radio.setSpreadingFactor(12);
  radio.setBandwidth(125.0);
  radio.setCodingRate(8);
  radio.setSyncWord(0x12);
  if (state != RADIOLIB_ERR_NONE) {
    while (true) {
      Serial.println("LoRa init failed, error code: " + String(state));
      delay(10);
    }
  }
  radio.setPacketSentAction(setFlag);
  
  // ----- BME280 [Check] -----
  if (!bme.begin(0x76, &myWire)) {
    while (1){
      Serial.println("BME280 not found!");
      delay(1000);
    }
  }
}

// int count = 0;

/*
void read_gnss() {
  // Feed TinyGPS++ with all available bytes from the GNSS serial
  while (SerialGPS.available()) {
    gps.encode((char)SerialGPS.read());
  }

  // When location data is updated, copy fields to gpsData
  if (gps.location.isUpdated()) {
    gpsData.latitude = gps.location.lat();
    gpsData.longitude = gps.location.lng();
    gpsData.hasFix = gps.location.isValid();
  }

  if (gps.altitude.isUpdated()) {
    gpsData.altitude = static_cast<float>(gps.altitude.meters());
  }

  // GPS time may update independently
  if (gps.time.isUpdated()) {
    // time.value() returns hhmmss as integer (e.g., 123045)
    gpsData.timestamp = gps.time.value();
  }
}

void print_gps_data() {
  Serial.println(F("----- GPS -----"));
  if (gpsData.hasFix) {
    Serial.print(F("Latitude : "));
    Serial.println(gpsData.latitude, 6);

    Serial.print(F("Longitude: "));
    Serial.println(gpsData.longitude, 6);

    Serial.print(F("Altitude : "));
    Serial.print(gpsData.altitude, 2);
    Serial.println(F(" m"));

    Serial.print(F("Timestamp: "));
    // Print hhmmss as HH:MM:SS for readability
    uint32_t t = gpsData.timestamp;
    uint8_t hh = (t / 10000) % 100;
    uint8_t mm = (t / 100) % 100;
    uint8_t ss = t % 100;
    if (t > 0) {
      if (hh < 10) Serial.print('0');
      Serial.print(hh);
      Serial.print(':');
      if (mm < 10) Serial.print('0');
      Serial.print(mm);
      Serial.print(':');
      if (ss < 10) Serial.print('0');
      Serial.println(ss);
    } else {
      Serial.println(F("N/A"));
    }
  } else {
    Serial.println(F("No fix yet"));
  }
  Serial.println(F("----------------\n"));
}

unsigned long lastPrint = 0;
*/

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

  // read_gnss();
  // // Print when location updated OR every 1 second as fallback
  // if (gps.location.isUpdated() || (millis() - lastPrint > 1000)) {
  //   print_gps_data();
  //   lastPrint = millis();
  // }

  // ----- LoRa -----
  // if(transmittedFlag) {
  //   transmittedFlag = false;

  //   if (transmissionState == RADIOLIB_ERR_NONE) {
  //   }

  //   radio.finishTransmit();
  //   delay(1000);

  //   String str = "Hello World! #" + String(count++);
  //   transmissionState = radio.startTransmit(str);
  //   Serial.println("Sending: " + str);
  // }

  delay(1000);
}