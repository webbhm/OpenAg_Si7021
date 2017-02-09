/**************************************************************************/
/*!
    @file     openag_Si7021.h
    @author Howard Webb
    @copied from   Limor Fried (Adafruit Industries)
    @license  BSD (see license.txt)

    This is a library for the Adafruit Si7021 breakout board
    ----> https://www.adafruit.com/products/3251

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0  - First release
    v2.0 - modifications for OpenAg 2.0
*/
/**************************************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include <Wire.h>
#include <openag_module.h>
#include <ros.h> //this must be included, and be put first
#include <std_msgs/Float32.h>	

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define SI7021_DEFAULT_ADDRESS         (0x40)  //64 decimal

#define SI7021_MEASRH_HOLD_CMD           0xE5
#define SI7021_MEASRH_NOHOLD_CMD         0xF5
#define SI7021_MEASTEMP_HOLD_CMD         0xE3
#define SI7021_MEASTEMP_NOHOLD_CMD       0xF3
#define SI7021_READPREVTEMP_CMD          0xE0
#define SI7021_RESET_CMD                 0xFE
#define SI7021_WRITERHT_REG_CMD          0xE6
#define SI7021_READRHT_REG_CMD           0xE7
#define SI7021_WRITEHEATER_REG_CMD       0x51
#define SI7021_READHEATER_REG_CMD        0x11
#define SI7021_ID1_CMD                   0xFA0F
#define SI7021_ID2_CMD                   0xFCC9
#define SI7021_FIRMVERS_CMD              0x84B8


/*=========================================================================*/

class OpenAg_Si7021 : public Module  //note: inheritance from OpenAg module
{
 public:
  OpenAg_Si7021(void);
  void begin();
  void update();
  bool get_Temperature(std_msgs::Float32 &msg);
  bool get_Humidity(std_msgs::Float32 &msg);

 private:
  float readTemperature(void);
  void reset(void);
  void readSerialNumber(void);
  float readHumidity(void);

  uint32_t sernum_a, sernum_b;

  uint8_t readRegister8(uint8_t reg);
  uint16_t readRegister16(uint8_t reg);
  void writeRegister8(uint8_t reg, uint8_t value);

  int8_t  _i2caddr;

  //Status Codes
  static const uint8_t CODE_COULDNT_FIND_ADDRESS = 1;
  static const uint8_t CODE_NO_RESPONSE = 2;

  bool _send_humidity;
  bool _send_temperature;
  uint32_t _time_of_last_reading;
  const static uint32_t _min_update_interval = 2000;
};

/**************************************************************************/


