#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

// Pin numbers
#define chip_select 10
#define servo_read 0
#define servo_write 3
#define cal_sig 2

// filter weights and other constants
#define setpoint2angle 1.0 / 180
#define LPF_a 0.85
#define LPF_b 1-LPF_a
#define ROLL_CENT 85
#define MAX_ROLL 30

//************************ global variabls *********************************
Servo servo;
File log_file;
Adafruit_MPU6050 mpu;
int cal_val = 0;
String file = "Log.csv";

// variabls for filter memory
double prev_y = 0;

//************************* function prototypes *****************************
void makeCal(void);
void logData(double setpoint, int roll_ctrl, double plane_angle, double other);
double getPlaneAngle(void);
double LPF(int setpoint);
int saturation(int);


void setup() {
  Serial.begin(115200);
  servo.attach(servo_write);
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
  double setpoint = analogRead(servo_read);
  int calibrate = digitalRead(cal_sig);

  if (!calibrate)
  {
    makeCal();
  }
  else
  {
    // Control block
    int roll_ctrl = ROLL_CENT;
    double plane_angle = getPlaneAngle();
    double desired_angle = setpoint * setpoint2angle;
    int ufiltered = (setpoint-137)*(160.0/328.0) + 10;
    roll_ctrl = LPF(setpoint);
    int other = roll_ctrl;
    roll_ctrl = (roll_ctrl-137)*(160.0/328.0) + 10;
  String send = String(plane_angle);
  Serial.println(send);
  
    // write servo
    servo.write(saturation(roll_ctrl));
  
    // house keeping
    logData(ufiltered, roll_ctrl, plane_angle, other);
    
    delay(200);
  }
}


//**************************SUBROUTINES********************************

void makeCal(void)
{
  cal_val = 0;
  double temp = 0;
  int itter = 25;
  int delta = 20; // miliseconds
  for(int i = 0; i < itter; i++)
  {
    temp += getPlaneAngle();
    delay(delta);
  }
  cal_val = temp / itter;
}

void logData(double setpoint, int roll_ctrl, double plane_angle, double other)
{
  String data_string = String(setpoint)+",";
  data_string += String(roll_ctrl)+",";
  data_string += String(plane_angle)+",";
  data_string += String(other)+",";
  log_file = SD.open(file, FILE_WRITE);
  log_file.println(data_string);
  log_file.close();
  //Serial.println(data_string);
}

double getPlaneAngle(void)
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  double angle = atan2(a.acceleration.y, a.acceleration.z);
  return angle * 180.0 / PI - cal_val;
}


double LPF(int setpoint)
{
  double y = 0.1 * setpoint + 0.9 * prev_y;
  prev_y = y;
  return y;
}

int saturation(int in)
{
  int out = in;
  if(in < ROLL_CENT-MAX_ROLL)
  {
    out = ROLL_CENT-MAX_ROLL;
  }
  else if(in > ROLL_CENT+MAX_ROLL)
  {
    out = ROLL_CENT+MAX_ROLL;
  }
  return out;
}
