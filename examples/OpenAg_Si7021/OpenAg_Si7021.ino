#include <openag_module.h>
#include <ros.h> //this must be included, and be put first
#include <std_msgs/Float32.h>
#include <Wire.h>
#include "OpenAg_Si7021.h"



OpenAg_Si7021 sensor = OpenAg_Si7021();

float humidity;
float temp;
std_msgs::Float32 msg;

void setup() {
  Serial.begin(9600);
  Serial.println("Si7021 test");
  sensor.begin();
}

void loop() {
  sensor.get_Humidity(msg);
  humidity = msg.data;
  Serial.print("Humidity:    "); Serial.print(humidity, 2);
  sensor.get_Temperature(msg);
  temp = msg.data;
  Serial.print("\tTemperature: "); Serial.println(temp, 2);
  delay(100);
}
