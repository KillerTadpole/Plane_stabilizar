#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

#define chip_select 10
#define servo_read 0
#define servo_write 3
#define cal_sig 1

Servo servo;
File log_file;
Adafruit_MPU6050 mpu;
int cal_val = 0;


void makeCal(void);
void logData(int data, sensors_event_t a);


void setup() {
  servo.attach(servo_write);
  SD.begin(chip_select);
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
}

void loop() {
  // read input
  int data = analogRead(servo_read);
  int calibrate = digitalRead(cal_sig);

  if (calibrate)
  {
    makeCal();
  }
  else
  {
    // read mpu
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    double angle = atan2(a.acceleration.x, a.acceleration.y);

    // Control block
    int roll_ctrl = 0;
  
    // write servo
    servo.write(roll_ctrl);
  
    // house keeping
    logData(data, a);
    delay(10);
  }
}


//**************************SUBROUTINES********************************

void makeCal(void)
{
  double temp = 0;
  int itter = 50;
  int delta = 20; // miliseconds
  sensors_event_t a, g, temper;
  for(int i = 0; i < itter; i++)
  {
      mpu.getEvent(&a, &g, &temper);
      temp += atan2(a.acceleration.x, a.acceleration.y);
      delay(delta);
  }
  cal_val = temp / itter;
}

void logData(int data, sensors_event_t a)
{
    String data_string = String(data)+",";
    data_string += String(a.acceleration.x)+",";
    data_string += String(a.acceleration.y)+",";
    data_string += String(a.acceleration.z)+",";
    log_file = SD.open("Log.csv", FILE_WRITE);
    log_file.println(data_string);
    log_file.close();
}
