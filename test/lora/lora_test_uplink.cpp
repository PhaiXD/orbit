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
} lora_params;

SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

/* ================================================================================================== */

void loraSetup(){
  
  Serial.println("Initializing SX1262...");

  int lora_state = lora.begin(
    lora_params.center_freq,
    lora_params.bandwidth,
    lora_params.spreading_factor,
    lora_params.coding_rate,
    lora_params.sync_word,
    lora_params.power,
    lora_params.preamble_length,
    0,
    false
  );

  if(lora_state != RADIOLIB_ERR_NONE) {
    Serial.print("Begin failed, code: ");
    
    while(true){
        Serial.println(lora_state);
        delay(1000);
    };
  }

  Serial.print("Lora Begin: ");
  Serial.println(lora_state);

  // configure module safely
  lora_state = lora.explicitHeader();
  Serial.print("ExplicitHeader: ");
  Serial.println(lora_state);


  lora_state = lora.setCRC(true);
  Serial.print("SetCRC: ");
  Serial.println(lora_state);

  lora_state = lora.autoLDRO();
  Serial.print("AutoLDRO: ");
  Serial.println(lora_state);


  if (lora_state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(lora_state);
    while (true) { delay(10); }
  }
}


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

int state;
uint8_t t;
uint8_t last_ack;
uint8_t last_nack;
uint32_t lora_tx_end_time;
uint32_t printV;
volatile LoRaState lora_state = LoRaState::IDLE;
String line;
String stateR = "STARTUP";
float lora_rssi;
volatile bool operationDone = false;
volatile bool rx_flag = false;
volatile bool tx_flag = false;
unsigned long last_time;
unsigned long last_time_line;

void setFlag(void) {
  operationDone = true;
}

void setup() {
  delay(5000);

  last_ack = 0;
  last_nack = 0;

  Serial.begin(115200);
  spi1.begin(); // initialize SPI bus

  delay(1000);

  loraSetup();

  lora.setDio1Action(setFlag);

  t = 0;
  randomSeed(analogRead(A0));

  last_time = millis();
  last_time_line = millis();

  tx_flag = true;
  rx_flag = false;
  printV = millis()-1;
}

void loop() {
  if(millis() > printV){
    Serial.println(String(tx_flag) + " " + String(rx_flag));
    printV = millis() + 500;
  }

  if(millis() - last_time_line > 2000){
    line = "";

    // line += "<3>";
    // line += ",";
    // line += "920.4";  // freq
    // line += ",";
    // line += String(t);  // count
    // line += ",";
    // line += stateR; // ps
    // line += ",";
    // line += String(random(50, 150)); // lat
    // line += ",";
    // line += String(random(50, 150)); // lon
    // line += ",";
    // line += String(random(50, 150)); // alt
    // line += ",";
    // line += String(random(50, 150)); // apogee
    // line += ",";
    // line += String(random(50, 150)); // volMon
    // line += ",";
    // line += String(last_ack);
    // line += ",";
    line += String(last_nack);

    last_time_line = millis();

    tx_flag = true;
    lora_state = LoRaState::TRANSMITTING;
    lora.startTransmit(line);
    Serial.println("[TRANSMITTING...]"); 
    ++t;

    last_time = millis();
  }

  // if(millis() - last_time > 2000){

  // }

  if(operationDone){
    operationDone = false;

    if(tx_flag){
      // Set Tx Done
      tx_flag = false;
      lora_state = LoRaState::RECEIVING;
      lora.startReceive();
      Serial.println("[RECEIVING...]");
    }
    else{
      // On Receive
      if (lora.getPacketLength() > 0)
      {
          String rx_string;
          lora.readData(rx_string);
          lora_rssi = lora.getRSSI();
          Serial.print("[RECEIVED] :");
          Serial.println(rx_string);
          rx_flag = false;

          if(rx_string.substring(0,4) == "cmd "){
            last_ack++;
            rx_string = rx_string.substring(4);
            if(stateR == "STARTUP" && rx_string == "on"){
                stateR = "IDLESAFE";
            }
            else if (stateR == "IDLESAFE" && rx_string == "arm")
            {
                stateR = "ARM";
            }
            else if (stateR == "ARM" && rx_string == "reset"){
                stateR = "STARTUP";
            }
          }
          else{
            last_nack++;
          }

          lora_state = LoRaState::RECEIVING;
          lora.startReceive();
          Serial.println("[RECEIVING...]");
      }
    }
  }
}
