#include "Capteur_MPU6050.h"
#include <MPU6050.h>
#include <Sonar.h>
#include <MotorControl.h>
#include <PID.h>


// MAIN CODE
Capteur_MPU6050 imu = Capteur_MPU6050();
double angleX;
double angleY;

PID pidAngle = PID(20.0,10.0,1.0);
PID pidDist = PID(5.5,5.0,5.0);
double angleGoal = 90.0;
double distanceGoal = 150.0; //mm

MotorControl motor = MotorControl(13, 12, 11, 10);
Sonar rangeSensor = Sonar(9,8);


double measurement_angle(){
  // Get imu x and y angles
  // Options: 0 angles from gyro only
  //          1 using complementary filter
  //          2 using Kalman filter
  imu.getAngles(&angleX,&angleY, 1); 
  return (double)angleX;
  
}

double measurement_distance(){
  double range;
  range = rangeSensor.getRange();
  return (double)range;
}

void command(double cmd){
  motor.setPWM(cmd);
}

void setup() {
  Serial.begin(115200);
  
  // initialize imu
  imu.init();
  
  // Power Sonar
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);

  // Setup Angle PID
  pidAngle.setCommandFunc(command);
  pidAngle.setMeasurementFunc(measurement_angle);
  pidAngle.setGoal(angleGoal);
  pidAngle.setPeriod(10);
  pidAngle.enable();

  // Setup Position PID
  pidDist.setCommandFunc(command);
  pidDist.setMeasurementFunc(measurement_distance);
  pidDist.setGoal(distanceGoal);
  pidDist.setPeriod(10);
  pidDist.enable();
}

void loop() {

  // Uncomment for angle PID only
  pidAngle.run();

  // Uncomment for double PID
//  double angleCommand;
//  double distCommand;  
//  angleCommand = pidAngle.returnCommand();
//  distCommand = pidDist.returnCommand();
//  command(distCommand - angleCommand);
  
}
