// This program 
//   
#include <Wire.h>                     
#include <Balboa32U4.h>
#include <LSM6.h>
#include <LIS3MDL.h>


LSM6 imu;
LIS3MDL mag; 
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;

//Variables for calibration
int32_t sumAx=0;
int32_t sumAz=0; 
int32_t cAx=0;
int32_t gYZ=0;

//PID Gain constants
double Kp = 0.95; //0.6
double Kd = 0.15; //0.10
double Kpd = 0; //0.00225
double Kdd = 0; //0.2

//Variables for error calculations
double setpoint = 78.8; //78.8
double Tilting_angle;
double angle;
double angleRate;
double angleGyro;

//Time variables
uint16_t current_time;
uint16_t elapsed_time;
static uint16_t previous_time;

//Error variables
double error;
double cumulative_error;
double rateOfError;
double Pa;
double Da;
double Pd;
double Dd;
int16_t error_dis;
int16_t Start_Pos = error_dis;
int16_t displacement;
double prev_error_dis;

//Motor speed variables
double output_Ang;
double output_dis;
double motorspeed;
int16_t rightcounter;
int16_t leftcounter;
uint16_t countdiff;
double speed_left ;
double speed_right;
double speedwheel;
int16_t prev_leftcounter;
int16_t prev_rightcounter;
double displ_left;
double displ_right;

void setup()
{
  // join I2C bus
Wire.begin();
  if (!imu.init())   // Initialise the imu sensor
  {
    // Verify connection ‐ Failed to detect the LSM6.
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to detect the LSM6."));

      delay(100);
    }
  }
  imu.enableDefault();
  // Set the gyro full scale to 1000 dps because the default
  // value is too low, and leave the other settings the same.
  imu.writeReg(LSM6::CTRL2_G, 0b01011000);
  // Set the accelerometer full scale to 16 g because the default
  // value is too low, and leave the other settings the same.
  imu.writeReg(LSM6::CTRL1_XL, 0b01010100);

//Calculating average error on accelerometer.      
  for (int i=0; i<50 ; i++)
    {
    imu.read();
    sumAx += imu.a.x;
    delay(1);
    }
  cAx =  sumAx/50;
  
//Calculating average error on the gyro.
 int32_t total = 0;
 for (int i = 0; i < 50; i++)
  {
    imu.read();
    total += imu.g.y;
    delay(1);
  }

 gYZ = total / 50;

  delay(1000);
}
void loop()
{
  
  current_time = millis();

// Perform the balance updates at 100 Hz.
if ((uint16_t)(current_time - previous_time) < 10) { return; }
    
  imu.read();
  //Send values to display for monitoring.
  //Serial.print(leftcounter); Serial.print("\t");
  //Serial.print(rightcounter); Serial.print("\t");
  Serial.print(Tilting_angle); Serial.print("\t");
  Serial.print(Pa); Serial.print("\t");
  Serial.print(Da); Serial.print("\t");
  Serial.print(output_Ang); Serial.print("\t");
  Serial.print(displacement); Serial.print("\t");
  Serial.print(setpoint); Serial.print("\t");
  //Serial.print(Pd); Serial.print("\t");
  //Serial.print(Dd); Serial.print("\t");
  //Serial.print(output_dis); Serial.print("\t");
  Serial.print(motorspeed); Serial.print("\t");
  Serial.print("\n");
  //Serial.print("angle/error/output:\t");
  //Serial.print((imu.a.x)*0.488/1000); Serial.print("\t");
  //Serial.print((imu.a.y)*0.488/1000); Serial.print("\t");
  //Serial.print((imu.a.z)*0.488/1000); Serial.print("\t");
  //Serial.print("cAx/cAz/angle/error:\t");
  //Serial.print(cAx);Serial.print("\t");
  //Serial.print(cAy);Serial.print("\t");
  //Serial.print(cAz);Serial.print("\t");
  //Serial.print("AngleAcc/AngleGyro/Angle/Error/Output:\t");
  //Serial.print(Tilting_angle); Serial.print("\t");
  //Serial.print(angleGyro); Serial.print("\t");
  //Serial.print(angle); Serial.print("\t");
  //Serial.print(error); Serial.print("\t");
  //Serial.print(output); Serial.print("\t");
  //Serial.print((imu.g.x)/29); Serial.print("\t");
  //Serial.print((imu.g.y)/29); Serial.print("\t");
  //Serial.print((imu.g.z)/29); Serial.print("\t");  
  //delay(1000);Serial.print("\n");
  
    
// Motor Counters
  leftcounter = encoders.getCountsLeft();
  rightcounter = encoders.getCountsRight();
  countdiff = leftcounter - rightcounter; 
  displ_left += leftcounter - prev_leftcounter;
  displ_right += rightcounter - prev_rightcounter;
  displacement = (displ_left + displ_right)/2;
  speed_left = (leftcounter - prev_leftcounter);
  speed_right = (rightcounter - prev_rightcounter);
  speedwheel = (speed_left + speed_right)/2;
  prev_leftcounter = leftcounter;
  prev_rightcounter = rightcounter;
  
//integrateGyro();
  angleRate = (imu.g.y - gYZ) / 29;
  angleGyro += angleRate * 10;

// reading angle
  double  ax = ((imu.a.x - cAx)*0.488/1000);
  double  az = ((imu.a.z)*0.488/1000);
  Tilting_angle = (atan2(ax,az)) * 180/PI;    // in degree
      
//PID Angle Control.
  error = setpoint - Tilting_angle;
  elapsed_time = current_time - previous_time;
  rateOfError = angleRate;
  Pa = Kp*error;
  Da = Kd*rateOfError;
  output_Ang = Pa + Da;
  
//PI Controller - Displacement.
  Pd = Kpd*displacement;
  Dd = Kdd*speedwheel; 
  output_dis = Pd+Dd;
 
//Motor Control
  motorspeed += output_Ang + output_dis; 
  motorspeed = constrain(motorspeed,-300,300);

  //For Angle:
  //Safety motor if 60>Tilting_angle>135 = motor on else off 
  if (60 < Tilting_angle && Tilting_angle < 135)
    {
      ledYellow(1);
      ledRed(0);
      motors.setSpeeds(motorspeed+0.4*countdiff , -motorspeed-0.65*countdiff);// -motorspeed_left-(0.6*countdiff)
      delay(1);
    }
  else 
    {
      ledRed(1);
      ledYellow(0);
      motors.setSpeeds(0,0);
      delay(1);
      
      output_dis = 0;
    }
    previous_time = current_time;
  }


  
