#include <Arduino.h>
#include <SPI.h>
#include "SdFat.h"
#include "orbit_pin_def.h"

// -------------------- SD Setup --------------------
SPIClass spi1(PIN_SPI_MOSI1, PIN_SPI_MISO1, PIN_SPI_SCK1); // MOSI, MISO, SCK

SdSpiConfig sd_config(PIN_NSS_SD, SHARED_SPI, SD_SCK_MHZ(2), &spi1);

SdFat  sd;
File  file;

// -------------------- Example Data --------------------
int sensorValue = 0;

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);

  spi1.begin();

  while(!Serial);
  delay(1000);
  Serial.println("Starting SD card...");

  // เริ่ม SD card
  if (!sd.begin(sd_config)) {
    Serial.println("SD init failed!");
  }
  Serial.println("SD initialized.");

  pinMode(PIN_NSS_SD, OUTPUT);
  digitalWrite(PIN_NSS_SD, HIGH);

  // สร้างไฟล์ CSV
  char filename[] = "DATA001.CSV";  // กำหนดชื่อเอง

  if (!file.open(filename, FILE_WRITE)) {
      Serial.println("Failed to open file!");
      // while(1);
  }

  Serial.print("Writing to file: ");
  Serial.println(filename);

  // เขียน header CSV
  file.println("Time(ms),Sensor");
  file.sync(); // เขียนลง SD


}

// -------------------- Loop --------------------
void loop() {
  sensorValue = analogRead(A0);

  unsigned long t = millis();
  file.print(t);
  file.print(",");
  file.println(sensorValue);

  file.sync(); // เขียนข้อมูลลง SD

  Serial.print("Logged: ");
  Serial.println(sensorValue);

  delay(1000);
}