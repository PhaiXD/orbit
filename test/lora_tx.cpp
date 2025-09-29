#include <Arduino.h>

#include <RadioLib.h>
#include <SPI.h>
#include "orbit_pin_def.h"

// SPI
SPIClass spi1(PIN_SPI_MOSI1, PIN_SPI_MISO1, PIN_SPI_SCK1);

SPISettings lora_spi_settings(4'000'000, MSBFIRST, SPI_MODE0);

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

SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

/* ================================================================================================== */

// Lora State
enum class LoRaState
{
  IDLE = 0,
  TRANSMITTING,
  RECEIVING
};

int transmissionState = RADIOLIB_ERR_NONE;

bool transmitFlag = false;

volatile bool operationDone = false;

int state;
uint8_t t;
uint8_t last_ack;
uint8_t last_nack;
uint32_t lora_tx_end_time;
volatile LoRaState lora_state = LoRaState::IDLE;
String line;
String stateR = "STARTUP";
float lora_rssi;
volatile bool rx_flag = false;
volatile bool tx_flag = true;
unsigned long last_time;
unsigned long last_time_line;

void setFlag(void) {
  tx_flag = true;
}

void setup() {
  delay(5000);

  spi1.begin();

  int lora_state = lora.begin(params.center_freq,
                              params.bandwidth,
                              params.spreading_factor,
                              params.coding_rate,
                              params.sync_word,
                              params.power,
                              params.preamble_length,
                              0,
                              false);
  state = state || lora.explicitHeader();
  state = state || lora.setCRC(true);
  state = state || lora.autoLDRO();

  
  if (lora_state == RADIOLIB_ERR_NONE)
  {
      Serial.println("SX1262 initialized successfully!");
      digitalWrite(ledPin2, 1);
  }
  else
  {
      Serial.print("Initialization failed! Error: ");
      Serial.println(lora_state);
      while (true){
        Serial.println(lora_state);
        delay(1000);
      }
  }

  last_ack = 0;
  last_nack = 0;

  Serial.begin(460800);
  spi1.begin(); // initialize SPI bus

  delay(1000);
    
  lora.setPacketSentAction(setFlag);

  last_time = millis();
  last_time_line = millis();
}

void loop() {

  if(millis() - last_time_line > 2000){
    line = String(t);

    line += "<3>";
    line += ",";
    line += String(t);
    line += ",";
    line += stateR;
    line += ",";
    line += String(random(50, 150)); // lat
    line += ",";
    line += String(random(50, 150)); // lon
    line += ",";
    line += String(random(50, 150)); // alt
    line += ",";
    line += String(random(50, 150)); // apogee
    line += ",";
    line += String(random(50, 150)); // volMon
    line += ",";
    line += String(last_ack);
    line += ",";
    line += String(last_nack);

    last_time_line = millis();
  }


  if(millis() - last_time > 2000 && tx_flag){
    tx_flag = false;
    lora_state = LoRaState::TRANSMITTING;
    state = lora.startTransmit(line);
    if(state == RADIOLIB_ERR_NONE){
      Serial.println("[TRANSMITTING...]"); 
    }
    else{
      while(1){
        Serial.println("Error code: "+ String(state));
        delay(1000);
      }
    }
    ++t;

    last_time = millis();
  }
}
