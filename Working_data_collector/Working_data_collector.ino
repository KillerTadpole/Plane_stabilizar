#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
//#include <Serial.h>

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

int numSamples=0;
int setpoint = 0;

// variabls for filter memory
int set_1 = 0;
//int set_2 = 0;

//************************* function prototypes *****************************
void makeCal(void);
void logData(int set, int roll_ctrl, double plane_angle, double other);
double getPlaneAngle(void);
int LPF(int setpoint);
int saturation(int);


void setup() {
  //Serial.begin(115200);
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
  
  ADCSRA = 0;             // clear ADCSRA register
  ADCSRB = 0;             // clear ADCSRB register
  ADMUX |= (0 & 0x07);    // set A0 analog input pin
  ADMUX |= (1 << REFS0);  // set reference voltage
  ADMUX |= (1 << ADLAR);  // left align ADC value to 8 bits from ADCH register

  // sampling rate is [ADC clock] / [prescaler] / [conversion clock cycles]
  // for Arduino Uno ADC clock is 16 MHz and a conversion takes 13 clock cycles
  //ADCSRA |= (1 << ADPS2) | (1 << ADPS0);    // 32 prescaler for 38.5 KHz
  ADCSRA |= (1 << ADPS2);                     // 16 prescaler for 76.9 KHz
  //ADCSRA |= (1 << ADPS1) | (1 << ADPS0);    // 8 prescaler for 153.8 KHz

  ADCSRA |= (1 << ADATE); // enable auto trigger
  ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);  // enable ADC
  ADCSRA |= (1 << ADSC);  // start ADC measurements
}

ISR(ADC_vect)
{
  byte x = ADCH;  // read 8 bit value from ADC
  setpoint = (setpoint + x) / 2;
  numSamples++;
}

void loop() {
  // read input
  int calibrate = digitalRead(cal_sig);

  if (!calibrate)
  {
    makeCal();
  }
  else
  {
    // Control block
    int set = setpoint;
    int Nd = numSamples;
    numSamples = 0;
    int roll_ctrl = ROLL_CENT;
    double plane_angle = getPlaneAngle();
    double desired_angle = setpoint * setpoint2angle;
    //int ufiltered = (setpoint-137)*(160.0/328.0) + 10;
    int other = LPF(set);
    roll_ctrl = other-76 + ROLL_CENT;
  //String send = String(roll_ctrl);
  //Serial.println(send);
  
    // write servo
    servo.write(saturation(roll_ctrl));
  
    // house keeping
    logData(set, roll_ctrl, plane_angle, N);
    
    //delay(10);
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

void logData(int set, int roll_ctrl, double plane_angle, double other)
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


int LPF(int set)
{
  int y = set;
  if (set-set_1 > 2)
  {
    y = set_1;
  }
  set_1 = set;
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
