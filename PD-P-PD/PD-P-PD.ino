// This program is a Multi Controller controlled self balancing inverted pendulum robot.
//
//Libraries:    
#include <Wire.h>                     
#include <Balboa32U4.h>
#include <LSM6.h>

//Classes renaming for ease of use.
LSM6 imu; //Variable for the Sensors. 
Balboa32U4Motors motors; //Variable for motors.
Balboa32U4Encoders encoders; //Variable for wheel encoders.

// -------------------Variable Declaration----------------------------------------//
//Variables for calibration
int32_t sumAx=0; //Summing variable for average error accelerometer x-axis.
int32_t cAx=0; //variable storing value for calibration of accelerometer x-axis. 
int32_t sumGz=0; //Summing variable for average error gyroscope y-axis.
int32_t gYZ=0; //variable storing value for calibration of gyroscope y-axis.

//Controllers Gain constants - Controller Variables.
double Kpa = 0.95; //Angle Proportional Gain (Full charge - 0.6). 
double Kda = 0.15; // Angle Derivative Gain (Full charge - 0.10).
double Pa; //Angle Proportional Controller Variable.
double Da; //Angle Derivative Controller Variable.
double output_Ang; //Angle controller summed output.
double Kpd = 0.00225; //Displacement Proportional Gain (Full charge - 0.00225).
double Kdd = 0.2; //Displacement derivative Gain (Full charge - 0.2).
double Pd; //Displacement Proportional Controller Variable.
double Dd; //Displacement Derivative Controller Variable.
double output_dis; //Displacement controller summed output.

//Variables for angle error calculations.
double setpoint = 78.8; //78.8 degrees.  
double Tilting_angle; //variable to store read/calculated calibrated angle.
double angleRate; //variable to store read/calculated calibrated change in angle.
uint16_t countdiff; //variable to calculate the encoder reading difference error.

//Time variables
uint16_t current_time; //variable to store time reading.
static uint16_t previous_time; //variable to store time reading.

//Error variables
double error; //Variable for proportional error.
double rateOfError; //Variable for derivative error.
int16_t error_dis;
int16_t Start_Pos = error_dis;
int16_t displacement;
double prev_error_dis;

//Motor related variables
double motorspeed; //Variable to store the motor speed value.
int16_t rightcounter; //Variable to store the right wheel encoder reading.
int16_t leftcounter; //Variable to store the left wheel encoder reading.
double speed_left ; //Variable to store left speed.
double speed_right; //Variable to store right speed.
double speedwheel; //Variable to store average speed.
int16_t prev_leftcounter; //Variable to store left encoder 2nd reading.
int16_t prev_rightcounter; //Variable to store right encoder 2nd reading.
double displ_left; //Variable to store left wheel displacement error.
double displ_right; //Variable to store right wheel displacement error.
// -------------------End of Variable Declaration----------------------------------------//
// -------------------System Setup Start-------------------------------------------------//
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
      Serial.println(F("Failed to detect the LSM6.")); //error message if connection failed.

      delay(100);
    }
  }
//------------------Sensor Configuration Start------------------------------------------//
  imu.enableDefault();
  // Set the gyro full scale to 1000 dps because the default
  // value is too low, and leave the other settings the same.
  imu.writeReg(LSM6::CTRL2_G, 0b01011000);
  // Set the accelerometer full scale to 16 g because the default
  // value is too low, and leave the other settings the same.
  imu.writeReg(LSM6::CTRL1_XL, 0b01010100);
//------------------Sensor Configuration End--------------------------------------------//
//------------------Calibration Calculation Start---------------------------------------//
//Calculating average error on accelerometer.      
  for (int i=0; i<50 ; i++) //loop over 50 readings.
    {
    imu.read(); //Reading from the sensor.
    sumAx += imu.a.x; //Sum the readings.
    delay(1);
    }
  cAx =  sumAx/50; //Accelerometer value for calibration.
  
//Calculating average error on the gyro.
  for (int i = 0; i < 50; i++)//loop over 50 readings.
  {
    imu.read(); //Reading from the sensor.
    sumGz += imu.g.y; //Sum the readings.
    delay(1);
  }

 gYZ = sumGz / 50; //Gyroscope value for calibration.

  delay(1000); //Delay to give time to the system to settle.
}
//------------------Calibration Calculation End---------------------------------------//
// -----------------System Setup End--------------------------------------------------//
// -----------------Main Programme Loop Start-----------------------------------------//
void loop()
{
  
  current_time = millis(); //Reading start time.

// Perform the balance updates at 100 Hz.
if ((uint16_t)(current_time - previous_time) < 10) { return; } // Constrain the main loop to a 100Hz or 10ms cycle.
    
  imu.read();// reading values from the imu
  //Send values to display for monitoring.
  //Serial.print(Tilting_angle); Serial.print("\t");
  //Serial.print(Pa); Serial.print("\t");
  //Serial.print(Da); Serial.print("\t");
  //Serial.print(output_Ang); Serial.print("\t");
  //Serial.print(displacement); Serial.print("\t");
  //Serial.print(setpoint); Serial.print("\t");
  //Serial.print(Pd); Serial.print("\t");
  //Serial.print(Dd); Serial.print("\t");
  //Serial.print(output_dis); Serial.print("\t");
  //Serial.print(motorspeed); Serial.print("\t");
  //Serial.print("\n");
    
// Motor Counters
  leftcounter = encoders.getCountsLeft(); //Reading left encoder.
  rightcounter = encoders.getCountsRight(); //Reading right encoder.
  countdiff = leftcounter - rightcounter;  //Calculating encoder error. 
  displ_left += leftcounter - prev_leftcounter; //left wheel displacement error.
  displ_right += rightcounter - prev_rightcounter; //right wheel displacement error.
  displacement = (displ_left + displ_right)/2; //Displacement average error.
  speed_left = (leftcounter - prev_leftcounter); //left wheel speed error.
  speed_right = (rightcounter - prev_rightcounter);//right wheel speed error.
  speedwheel = (speed_left + speed_right)/2; //Speed average error.
  prev_leftcounter = leftcounter; //Reading left encoder.
  prev_rightcounter = rightcounter;//Reading right encoder.
  
//Angle rate from gyroscope;
  angleRate = (imu.g.y - gYZ) / 29; //Angle rate calibrated value in degree/s.

//Angle value from accelerometer. 
  double  ax = ((imu.a.x - cAx)*0.488/1000); //calibrated x-axis reading from accelerometer
  double  az = ((imu.a.z)*0.488/1000); //y-axis reading from accelerometer
  Tilting_angle = (atan2(ax,az)) * 180/PI;    //Angle in degree
      
//PID Angle Control.
  error = setpoint - Tilting_angle; //Angle Error calculation.
  rateOfError = angleRate; //Angle rate error.
  Pa = Kp*error; //Angle Proportional controller.
  Da = Kd*rateOfError; //Angle Derivative controller.
  output_Ang = Pa + Da; //Angle Controller summed output.
  
//PD Controller - Displacement.
  Pd = Kpd*displacement; //Displacement Proportional controller.
  Dd = Kdd*speedwheel; //Displacement Derivative controller.
  output_dis = Pd+Dd; //Displacement Controller summed output.
 
//Motor Control
  motorspeed += output_Ang + output_dis; //motorspeed set as sum of controller values.
  motorspeed = constrain(motorspeed,-300,300);//constrain/scale the motor speed to avoid saturation.

  //For Angle:
  //Safety motor: if 60<Tilting_angle<135 then motor on else  motor off 
  if (60 < Tilting_angle && Tilting_angle < 135)
    {
      ledYellow(1); //Indicator robot is in operating range.
      ledRed(0);
      motors.setSpeeds(motorspeed+0.4*countdiff , -motorspeed-0.65*countdiff); //Motors are on receiving motorspeed with added correction to keep the robot straight. 
      delay(1);
    }
  else 
    {
      ledRed(1); //Indicator robot is beyond operating range.
      ledYellow(0);
      motors.setSpeeds(0,0); //Out of range then motor off.
      delay(1);
           
    }
    previous_time = current_time; //Reading end of loop time.
  }
// -----------------Main Programme Loop End-------------------------------------------//
// -----------------End of Multi-Controller Control Programme-------------------------//


  
