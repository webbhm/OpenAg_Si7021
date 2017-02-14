/**************************************************************************/
/*!
    @file     OpenAg_Si7021.cpp
    @author   Limor Fried (Adafruit Industries)
    @license  BSD (see license.txt)

    This is a library for the Adafruit Si7021 breakout board
    ----> https://www.Adafruit.com/products/3251

    Adafruit invests time and resources providing this open source code,
    please support OpenAg and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include <OpenAg_Si7021.h>

/**************************************************************************/
//The i2c address cannot be changed, so no need to have it as an input parameter
OpenAg_Si7021::OpenAg_Si7021(void) {
  _i2caddr = SI7021_DEFAULT_ADDRESS;
  sernum_a = sernum_b = 0;
}

//change signature for OpenAg compatability
void OpenAg_Si7021::begin(){
  Wire.begin();
  reset();
  if (readRegister8(SI7021_READRHT_REG_CMD) != 0x3A){
     status_level = ERROR;
     status_code = 1;
     status_msg = "Unable to read register";
     return;
  }

  readSerialNumber();

  //Serial.println(sernum_a, HEX);
  //Serial.println(sernum_b, HEX);

  status_level = OK;
  status_code = CODE_OK;
  status_msg = "";
}

//throttle data flow
void OpenAg_Si7021::update(){
   if (millis() - _time_of_last_reading > _min_update_interval) {
      _time_of_last_reading = millis();
      _send_humidity = true;
      _send_temperature = true;
      status_level = OK;
      status_msg = "";
   }
}

bool OpenAg_Si7021::get_air_temperature(std_msgs::Float32 &msg){
   msg.data = readTemperature();
   bool res = _send_temperature;
   _send_temperature = false;
   return res;
}

bool OpenAg_Si7021::get_air_humidity(std_msgs::Float32 &msg){
   msg.data = readHumidity();
    bool res = _send_humidity;
   _send_humidity = false;
   return res;
}


//Adafruit code below


float OpenAg_Si7021::readHumidity(void) {
  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)SI7021_MEASRH_NOHOLD_CMD);
  Wire.endTransmission(false);
  delay(25);

  Wire.requestFrom(_i2caddr, 3);
  uint16_t hum = Wire.read();
  hum <<= 8;
  hum |= Wire.read();
  uint8_t chxsum = Wire.read();

  float humidity = hum;
  humidity *= 125;
  humidity /= 65536;
  humidity -= 6;

  return humidity;
}

float OpenAg_Si7021::readTemperature(void) {
  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)SI7021_MEASTEMP_NOHOLD_CMD);
  Wire.endTransmission(false);
  delay(25);

  Wire.requestFrom(_i2caddr, 3);
  uint16_t temp = Wire.read();
  temp <<= 8;
  temp |= Wire.read();
  uint8_t chxsum = Wire.read();

  float temperature = temp;
  temperature *= 175.72;
  temperature /= 65536;
  temperature -= 46.85;

  return temperature;
}

void OpenAg_Si7021::reset(void) {
  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)SI7021_RESET_CMD);
  Wire.endTransmission();
  delay(50);
}

void OpenAg_Si7021::readSerialNumber(void) {
  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)SI7021_ID1_CMD>>8);
  Wire.write((uint8_t)SI7021_ID1_CMD&0xFF);
  Wire.endTransmission();

  Wire.requestFrom(_i2caddr, 8);
  sernum_a = Wire.read();
  Wire.read();
  sernum_a <<= 8;
  sernum_a |= Wire.read();
  Wire.read();
  sernum_a <<= 8;
  sernum_a |= Wire.read();
  Wire.read();
  sernum_a <<= 8;
  sernum_a |= Wire.read();
  Wire.read();

  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)SI7021_ID2_CMD>>8);
  Wire.write((uint8_t)SI7021_ID2_CMD&0xFF);
  Wire.endTransmission();

  Wire.requestFrom(_i2caddr, 8);
  sernum_b = Wire.read();
  Wire.read();
  sernum_b <<= 8;
  sernum_b |= Wire.read();
  Wire.read();
  sernum_b <<= 8;
  sernum_b |= Wire.read();
  Wire.read();
  sernum_b <<= 8;
  sernum_b |= Wire.read();
  Wire.read();
}

/*******************************************************************/

void OpenAg_Si7021::writeRegister8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();

  //Serial.print("Wrote $"); Serial.print(reg, HEX); Serial.print(": 0x"); Serial.println(value, HEX);
}

uint8_t OpenAg_Si7021::readRegister8(uint8_t reg) {
  uint8_t value;
  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission(false);

  Wire.requestFrom(_i2caddr, 1);
  value = Wire.read();

  //Serial.print("Read $"); Serial.print(reg, HEX); Serial.print(": 0x"); Serial.println(value, HEX);
  return value;
}

uint16_t OpenAg_Si7021::readRegister16(uint8_t reg) {
  uint16_t value;
  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();

  Wire.requestFrom(_i2caddr, 2);
  value = Wire.read();
  value <<= 8;
  value |= Wire.read();

  //Serial.print("Read $"); Serial.print(reg, HEX); Serial.print(": 0x"); Serial.println(value, HEX);
  return value;
}
