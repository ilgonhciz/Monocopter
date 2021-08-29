#include <Servo.h>
#include <Wire.h>
#include <L3G.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <math.h> 
#include <ServoInput.h>

#include "PinChangeInterrupt.h"
 
#define MY_PIN A0 // we could choose any pin

/* Servo */
Servo myservo_pitch;
Servo myservo_roll;

/* IMU */
L3G gyro;

Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);



/* Pin definitions */
byte channel_1 = A3; //roll pin
byte channel_2 = A2; //pitch pin
byte channel_5 = A1; //mode pin
/* Input value definitions */
float channel_1_value = 1500; //roll value
float channel_2_value = 1500; //pitch value
//float channel_5_value = 1500; //pitch value

int mode = 0; //fsm: 0: manual mode 1: level flight 2: tbd  


double kp = 0.2;
double ki = 0.000;
double kd = 0.02;
 

float servo_pitch_offset = 104;
float servo_roll_offset = 100; 

float att_pitch_should = 0;
float att_roll_should = 0;
float att_yaw_should = 0;  

sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_vec_t   orientation;

float gyro_offset_x, gyro_offset_y, gyro_offset_z, yaw_offset;

float acc_x, acc_y, acc_z;
float gyro_x_rate, gyro_y_rate, gyro_z_rate;
float gyro_x, gyro_y, gyro_z;
float pitch, roll, yaw; 

float yaw_rate; 
float vir_pitch, vir_roll; 


long time_elapsed_IMU, time_elapsed;

float full_rotation_count=0;
float alpha = 0.02;

double error_p , error_r;
double lastError_p, lastError_r;
double setPoint_pitch, setPoint_roll;
double cumError_r, rateError_r;

double cumError_p, rateError_p;


const int PitchPin = 2;  // MUST be interrupt-capable!
const int RollPin = 3;  // MUST be interrupt-capable!
const int PulseMin = 1000;  // microseconds (us)
const int PulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" example
ServoInputPin<PitchPin> pitch_channel_value(PulseMin, PulseMax);
ServoInputPin<RollPin> roll_channel_value(PulseMin, PulseMax);
 
volatile int channel_5_value = 1000;
volatile int prev_time = 0;
uint8_t latest_interrupted_pin;



void setup() {
  Serial.begin(115200);
  delay(500);
  //Serial.print("initialize!");
  Wire.begin();
  init_IMU(); 

  myservo_pitch.attach(5);
  myservo_roll.attach(6); 
  
  // initial position
  myservo_pitch.write(90);
  myservo_roll.write(90);
  attachPCINT(digitalPinToPCINT(MY_PIN), rising, RISING);
}

void loop() {
  long pre_time = millis();
  if (channel_5_value < 1250){
      mode = 0;
    }
  else{
    if (channel_5_value < 1750){
      mode = 1;
    }
    else{
      mode = 2;
    }
  }
  
  if(mode == 0){
      mode_0();
    }
  if (mode == 1){
      mode_1();
    }
  if (mode == 2){
      mode_2();
    }
  time_elapsed = time_elapsed_IMU = millis()- pre_time;
  print_all();
}
