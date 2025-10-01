#include <Arduino.h>
#include "orbit_pin_def.h"

#include "File_Utility.h"
#include "SdFat.h"

// SPI
SPIClass spi1(PIN_SPI_MOSI1, PIN_SPI_MISO1, PIN_SPI_SCK1);

SdSpiConfig sd_config(PIN_NSS_SD, SHARED_SPI, SD_SCK_MHZ(2), &spi1);

FsUtil<SdFat32, File32> sd_util;

template <typename SdFat32, typename File32>
extern void init_storage(FsUtil<SdFat32, File32> &sd_util_instance);

#define THIS_FILE_PREFIX "ORBIT_LOGGER_KENDOs"
#define THIS_FILE_EXTENSION "CSV"

String constructed_data;

struct{
  uint32_t flush = millis();
  uint32_t save = millis();
}timer;
struct peripherals_t
{
    union
    {
        struct
        {
            uint8_t imu;
            uint8_t bme;
            uint8_t sd;
        };
    };

    template <traits::has_ostream OStream>
    OStream &operator>>(OStream &ostream)
    {
        ostream << "IMU imu42688: " << imu << stream::crlf;
        ostream << "BME280: " << bme << stream::crlf;
        ostream << "SD: " << sd << stream::crlf;
        return ostream;
    }
};
peripherals_t pvalid;

void save_data();

void setup(){
  Serial.begin(115200);

  spi1.begin();

  delay(1000);
  
  pvalid.sd = sd_util.sd().begin(sd_config);
  if(pvalid.sd){
    init_storage(sd_util);
    Serial.println("SD SUCESS");
    digitalWrite(ledPin1, 1);
  }


}

void loop(){
  save_data();
  constructed_data = String(millis()) + '\n';
}

template <typename SdFat32, typename File32>
void init_storage(FsUtil<SdFat32, File32> &sd_util_instance)
{
    sd_util_instance.find_file_name(THIS_FILE_PREFIX, THIS_FILE_EXTENSION);
    sd_util_instance.template open_one<FsMode::WRITE>();
}

void save_data()
{

    if(millis() - timer.save > 400){
      sd_util.file() << constructed_data;
      Serial.println("Data written: " + String(constructed_data));
      timer.save = millis();
    }

    if(millis() - timer.flush > 2000)
    { 
      sd_util.flush_one(); 
      Serial.println("Data flushed.");
      timer.flush = millis();
    };
}