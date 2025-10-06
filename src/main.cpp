#include <Arduino.h>
#include "File_Utility.h"

#include <Wire.h>
#include <SPI.h>

#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include "LIS3DHTR.h"
#include <Adafruit_BME280.h>
// #include <Servo.h>

#include "SdFat.h"
#include "RadioLib.h"

#include "orbit_peripheral_def.h"
#include "orbit_pin_def.h"
#include "orbit_state_def.h"

// Device specific
#define THIS_FILE_PREFIX "ORBIT_LOGGER_"
#define THIS_FILE_EXTENSION "CSV"

// i2c
TwoWire i2c1(PIN_SDA, PIN_SCL);
Adafruit_BME280 bme1;
LIS3DHTR<TwoWire> imu;

// UARTS
HardwareSerial gnssSerial(PIN_RX, PIN_TX);
TinyGPSPlus lc86;

// SPI
SPIClass spi1(PIN_SPI_MOSI1, PIN_SPI_MISO1, PIN_SPI_SCK1);

// SD Card
SdSpiConfig sd_config(PIN_NSS_SD, SHARED_SPI, SD_SCK_MHZ(2), &spi1);
using sd_t = SdFat32;
using file_t = File32;
FsUtil<sd_t, file_t> sd_util;

// LoRa

// LoRa State
enum class LoRaState
{
    IDLE = 0,
    TRANSMITTING,
    RECEIVING
};

int status_lora;
volatile bool tx_flag = false;
volatile bool rx_flag = false;

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

// DATA
struct Data
{
    // 40 bits
    uint32_t timestamp;
    uint8_t counter;

    String state;

    // 160 bits
    double gps_latitude;
    double gps_longitude;
    float gps_altitude;

    // 96 bits
    float altitude;
    float temp;
    float humidity;
    float press;

    // 384 bits
    struct
    {
        float x = 0;
        float y = 0;
        float z = 0;
    } acc,vel;
} data;

// State
bool sdstate;

struct
{
    float altitude_offset{};
    float apogee;
} ground_truth;

// Communication data
String constructed_data;
String tx_data;

// time on function
struct plusTime{ // millis
    uint32_t led_control = 10; // LED
    uint32_t read_gnss = 10; // GPS
    uint32_t read_bme = 10; // BME
    uint32_t read_imu = 10; // IMU
    uint32_t construct_data = 10; // construct data
    uint32_t transmit_data = 10; // LORA
    uint32_t send_data = 2000; 
    uint32_t save_data = 100; // SD card
    uint32_t log_data = LOG_GROUND_INTERVAL;
    uint32_t save_data_to_sd_card = 1000;
    uint32_t print_data = 2000; // Serail monitor
    uint32_t check_state = 10; // Stage
}plusTime;

// delay time on function
struct nowTime{
    uint32_t led_control = millis(); // LED
    uint32_t read_gnss = millis(); // GPS
    uint32_t read_bme = millis(); // BME
    uint32_t read_imu = millis(); // IMU
    uint32_t construct_data = millis();
    uint32_t transmit_data = millis(); // LORA
    uint32_t send_data = millis();
    uint32_t save_data = millis(); // SD card
    uint32_t log_data = millis();
    uint32_t save_data_to_sd_card = millis();
    uint32_t print_data = millis(); // Serial monitor
    uint32_t check_state = millis(); // Stage
}nowTime;

// variables
volatile bool wake_flag = false;

extern void led_control();

extern void read_gnss();

extern void read_bme();

extern void read_imu();

extern void calculate_vel();

extern void construct_data();

extern void transmit_data();

extern void save_data();

extern void print_data();

extern void check_state();

extern void set_txflag();

template <typename SdType, typename FileType>
extern void init_storage(FsUtil<SdType, FileType> &sd_util_instance);

void setup()
{
    Serial.begin(460800);
    delay(2000);

    i2c1.begin();
    i2c1.setClock(300000u);

    spi1.begin();

    // GPIO
    pinMode(ledPin1, OUTPUT); // LED
    pinMode(ledPin2, OUTPUT); // LED
    pinMode(ledPin3, OUTPUT); // LED
    pinMode(ledPin4, OUTPUT); // LED

    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, HIGH);
    digitalWrite(ledPin3, HIGH);
    digitalWrite(ledPin4, HIGH);
    delay(200); // 200 ms ON

    // Turn all LEDs OFF
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
    digitalWrite(ledPin3, LOW);
    digitalWrite(ledPin4, LOW);
    // variable
    static bool state;

    pinMode(LORA_NSS, OUTPUT);
    digitalWrite(LORA_NSS, HIGH); // deselect LoRa

    // pinMode(PIN_NSS_SD, OUTPUT);
    // digitalWrite(PIN_NSS_SD, HIGH); // idle SD CS high

    /* ==================================== SD card ==================================== */

    sd_util.sd().begin(sd_config);

    init_storage(sd_util);
    Serial.println("SD SUCESS");
    digitalWrite(ledPin1, 1);
    

    /* ==================================== LoRa ==================================== */

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

    lora.setPacketSentAction(set_txflag);

    if (lora_state == RADIOLIB_ERR_NONE)
    {
        Serial.println("SX1262 initialized successfully!");
        digitalWrite(ledPin2, 1);
    }
    else
    {
        Serial.print("Initialization failed! Error: ");
        while (true){
            Serial.println(lora_state);
            delay(1000);
        }
    }

    /* ==================================== LIS3DHTR (IMU) ==================================== */

    imu.begin(i2c1,0x18);
    
    imu.setFullScaleRange(LIS3DHTR_RANGE_16G); // 6, 12, or 24 G
    imu.setOutputDataRate(LIS3DHTR_DATARATE_1_6KH);
    
    /* ==================================== lc86g UARTS ==================================== */

    gnssSerial.begin(115200);

    /* ==================================== bme280 ==================================== */

    float gnd = 0.f;

    if(!bme1.begin(0x76, &i2c1)){
        Serial.println("BME NOT FOUND");
    }
    else
    {
        bme1.SAMPLING_X16;
        for (size_t i = 0; i < 20; ++i)
        {
            read_bme();
        }
        gnd += data.altitude;
    }
    ground_truth.altitude_offset = gnd;

    /* ==================================== START ==================================== */
   
    Serial.println("START TASK");
}

void loop()
{
    
    if(millis() > nowTime.read_gnss + plusTime.read_gnss){
        read_gnss();
        nowTime.read_gnss = millis();
    }

    if(millis() > nowTime.read_bme + plusTime.read_bme){
        read_bme();
        nowTime.read_bme = millis(); // BME
    }

    if(millis() > nowTime.read_imu + plusTime.read_imu){
        read_imu();
        nowTime.read_imu = millis(); // IMU
    }

    if(millis() > nowTime.construct_data + plusTime.construct_data){
        construct_data();
        nowTime.construct_data = millis();
    }

    if(millis() > nowTime.transmit_data + plusTime.transmit_data){
        transmit_data();
        nowTime.transmit_data = millis(); // LORA
    }

    if(millis() > nowTime.save_data + plusTime.save_data){
        save_data();
        nowTime.save_data = millis(); // SD card
    }

    if(millis() > nowTime.print_data + plusTime.print_data){
        print_data();
        nowTime.print_data = millis();
    }

    if(millis() > nowTime.check_state + plusTime.check_state){
        check_state();
        nowTime.check_state = millis();
    }
}


/* ======================================= LOOP ======================================= */

void led_control()
{
    // digitalWrite(ledPin4, !digitalRead(ledPin4));
}

void read_gnss()
{

    while (gnssSerial.available())
    {
        lc86.encode(gnssSerial.read());
        data.timestamp = lc86.time.value();
    }

    if (lc86.location.isUpdated())
    {
        data.gps_latitude = lc86.location.lat();
        data.gps_longitude = lc86.location.lng();
        data.gps_altitude = lc86.altitude.meters();
    }
}

void read_bme()
{

    data.temp = bme1.readTemperature();
    data.press = bme1.readPressure() / 100.0F;
    data.humidity = bme1.readHumidity();
    data.altitude = bme1.readAltitude(SEALEVELPRESSURE_HPA);
    // bme1.readHumidity();
}

void read_imu()
{
    data.acc.x = imu.getAccelerationX();
    data.acc.y = imu.getAccelerationY();
    data.acc.z = imu.getAccelerationZ();
}

void construct_data()
{
    constructed_data = "";
    constructed_data +=
        String(data.counter) + ',' +
        String(data.timestamp) + ',' +
        data.state + ',' +
        String(data.gps_latitude, 6) + ',' +
        String(data.gps_longitude, 6) + ',' +
        String(data.altitude, 4) + ',' +
        String(ground_truth.apogee, 4) + ',' +
        String(data.temp) + ',' +
        String(data.press) + ',' +
        String(data.humidity) + ',' +
        String(data.acc.x) + ',' + 
        String(data.acc.y) + ',' + 
        String(data.acc.z) + '\n';

    tx_data = "";
    tx_data += 
        String(data.acc.x) + ',' +
        String(data.acc.y) + ',' +
        String(data.acc.z) + ',' +
        String(data.gps_latitude, 6) + ',' +
        String(data.gps_longitude, 6) + ',' +
        String(data.temp, 4) + ',' +
        String(data.press, 4) + ',' +
        String(data.altitude, 4) + ',' +
        String(data.humidity, 4) + ',' +
        String(max(0,(data.altitude - ground_truth.altitude_offset)), 4) + ',' +
        data.state;
}


template <typename SdType, typename FileType>
void init_storage(FsUtil<SdType, FileType> &sd_util_instance)
{
    sd_util_instance.find_file_name(THIS_FILE_PREFIX, THIS_FILE_EXTENSION);
    sd_util_instance.template open_one<FsMode::WRITE>();
}

void save_data()
{
    if(millis() > nowTime.log_data + plusTime.log_data){
        sd_util.file() << constructed_data;
        //  Serial.println("Data written and flushed.");
    }

    if(millis() > nowTime.save_data_to_sd_card + plusTime.save_data_to_sd_card){
        sd_util.flush_one();
        nowTime.save_data_to_sd_card = millis();
    }
}

void transmit_data()
{
    // Tx Loop
    if(millis() > nowTime.send_data + plusTime.send_data && tx_flag){
        lora.startTransmit(tx_data);
        Serial.println("[TRANSMITTING...]"); 
        ++data.counter;
        nowTime.send_data = millis();
    }
    // On Transmit
    if (!tx_flag)
    {
        Serial.print("[TRANSMITTED] ");
        Serial.println(tx_data);
        tx_flag = true;
    }
}

void check_state()
{
    const double alt_x = data.altitude - ground_truth.altitude_offset;
    //const double vel_x = data.vel.x;
    const double acc = data.acc.y;

    if(apogee - alt_x > 200){
        data.state = "MAIN DEPLOY";
    }

    if(data.state == "GROUND")
    {
        // Next: WAIT FOR POWER
        if(acc >= LAUNCH_ACC){
            plusTime.log_data = LOG_PAD_PREOP_INTERVAL;
            data.state = "RISING";
        }
    }
    else if(data.state == "RISING")
    {
        // Next: WAIT FOR APOGEE
        if(acc < LAUNCH_ACC){
            plusTime.log_data = LOG_APOGEE_INTERVAL;
            data.state = "COASTING";
        }
    }
    else if(data.state == "COASTING")
    {
        // Next: AT APOGEE WAIT FOR LANDING
        if(apogee - alt_x > 10){
            plusTime.log_data = LOG_LANDED_INTERVAL;
            data.state = "MAIN DEPLOY";
        }
    }
    else if(data.state == "MAIN DEPLOY"){
        if(alt_x <= 150){
            data.state = "SECOND DEPLOY";
        }
    }
    else if(data.state == "SECOND DEPLOY"){
        if(alt_x <= 40){
            data.state = "LANDED";
        }
    }
    else if(data.state == "LANDED")
    {
        // Next: WAIT FOR LANDED
    }

    if (alt_x > ground_truth.apogee)
    {
        ground_truth.apogee = alt_x;
    }
}

void print_data()
{
    Serial.println("====== DATA ======");

    Serial.print("Timestamp: ");
    Serial.println(data.timestamp);

    Serial.print("Counter: ");
    Serial.println(data.counter);

    Serial.println("---- STATES ----");
    Serial.print("STATE: ");
    Serial.println(data.state);

    Serial.println("---- GPS ----");
    Serial.print("Latitude: ");
    Serial.println(data.gps_latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(data.gps_longitude, 6);
    Serial.print("Altitude: ");
    Serial.println(data.gps_altitude, 4);

    Serial.println("---- ENV ----");
    Serial.print("Temperature: ");
    Serial.println(data.temp, 2);
    Serial.print("Altitude: ");
    Serial.println(data.altitude, 2);
    Serial.print("Pressure: ");
    Serial.println(data.press, 2);
    Serial.print("Humidity: ");
    Serial.println(data.humidity, 2);

    Serial.println("---- IMU ACC ----");
    Serial.print("X: ");
    Serial.print(data.acc.x, 3);
    Serial.print("  Y: ");
    Serial.print(data.acc.y, 3);
    Serial.print("  Z: ");
    Serial.println(data.acc.z, 3);

    /* Serial.println("---- VEL ----");
    Serial.print("X: ");
    Serial.print(data.vel.x, 3);
    Serial.print("  Y: ");
    Serial.print(data.vel.y, 3);
    Serial.print("  Z: ");
    Serial.println(data.vel.z, 3); */


    // Serial to ground web
    // Serial.print(String(data.acc.x, 3) + ',' 
    //             + String(data.acc.y, 3) + ',' 
    //             + String(data.acc.z, 3) + ',' 
    //             + String(data.gps_latitude, 6) + ',' 
    //             + String(data.gps_longitude, 6) + ',' 
    //             + String(data.temp, 2) + ',' 
    //             + String(data.press, 2) + ','
    //             + String(data.humidity, 2) + ','
    //             + String("100") + ','
    //             + String("RISING")
    //         );
}

void set_txflag()
{
    tx_flag = false;
}