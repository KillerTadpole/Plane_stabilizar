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
#define cal_sig 2
#define setpoint2angle 1.0 / 180

Servo servo;
File log_file;
Adafruit_MPU6050 mpu;
int cal_val = 0;


void makeCal(void);
void logData(int setpoint, int roll_ctrl, double plane_angle);
double getPlaneAngle(void);
String file = "Log.csv";


void setup() {
  //servo.attach(servo_write);
  SD.begin(chip_select);
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  pinMode(cal_sig, INPUT);
  digitalWrite(cal_sig, HIGH);
  int cnt = 0;
  file = "Log_" + String(cnt) + ".csv";
  while(SD.exists(file))
  {
    cnt++;
    file = "Log_" + String(cnt) + ".csv";
  }
}

void loop() {
  // read input
  int setpoint = analogRead(servo_read);
  int calibrate = digitalRead(cal_sig);

  if (!calibrate)
  {
    makeCal();
  }
  else
  {
    // Control block
    int roll_ctrl = 0;
    double plane_angle = getPlaneAngle();
    double desired_angle = setpoint * setpoint2angle;
    roll_ctrl = (setpoint-137)*(160.0/328.0) + 10;
  
    // write servo
    // servo.write(roll_ctrl);
  
    // house keeping
    logData(calibrate*180, cal_val, plane_angle);
    delay(10);
  }
}


//**************************SUBROUTINES********************************

void makeCal(void)
{
  cal_val = 0;
  double temp = 0;
  int itter = 50;
  int delta = 20; // miliseconds
  for(int i = 0; i < itter; i++)
  {
    temp += getPlaneAngle();
    delay(delta);
  }
  cal_val = temp / itter;
}

void logData(int setpoint, int roll_ctrl, double plane_angle)
{
  String data_string = String(setpoint)+",";
  data_string += String(roll_ctrl)+",";
  data_string += String(plane_angle)+",";
  log_file = SD.open(file, FILE_WRITE);
  log_file.println(data_string);
  log_file.close();
}

double getPlaneAngle(void)
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  double angle = atan2(a.acceleration.x, a.acceleration.y);
  return angle * 180.0 / PI - cal_val;
}
