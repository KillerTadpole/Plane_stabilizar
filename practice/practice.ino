#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define chip_select 10
#define servo_read 0
#define servo_write 3

Servo servo;
File log_file;
Adafruit_MPU6050 mpu;
int cnt = 0;
int hi_low = 0;

void setup() {
  // put your setup code here, to run once:
  servo.attach(servo_write);
  SD.begin(chip_select);
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
}

void loop() {
  // put your main code here, to run repeatedly:
  // read input
  int data = analogRead(servo_read);
  
  // read mpu
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // log data
  String data_string = String(data)+",";
  data_string += String(hi_low)+",";
  data_string += String(a.acceleration.x)+",";
  data_string += String(a.acceleration.y)+",";
  data_string += String(a.acceleration.z)+",";
  data_string += String(temp.temperature)+",";
  log_file = SD.open("Log.csv", FILE_WRITE);
  log_file.println(data_string);
  log_file.close();

  // write servo
  servo.write(hi_low);

  // house keeping
  cnt +=1;
  if(cnt > 180)
  {
    cnt = 0;
  }
  if(cnt > 90)
  {
    hi_low = 170;
  }
  else
  {
    hi_low = 10;
  }
  delay(50);
}
