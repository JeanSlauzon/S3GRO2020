#include "Capteur_MPU6050.h"
#include <MPU6050.h>
#include <Sonar.h>
#include <MotorControl.h>
#include <PID.h>


// MAIN CODE
Capteur_MPU6050 imu = Capteur_MPU6050();
double angleX;
double angleY;
PID pid = PID(1.0,0.0,0.1);
MotorControl motor = MotorControl(13, 12, 11, 10);
Sonar rangeSensor = Sonar(9,8);


double measurement(){
  // Get imu x and y angles
  // Options: 0 angles from gyro only
  //          1 using complementary filter
  //          2 using Kalman filter
  imu.getAngles(&angleX,&angleY, 1); 
  Serial.println(angleX);
  return (double)angleX;
  
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
  pid.setCommandFunc(command);
  pid.setMeasurementFunc(measurement);
  pid.setGoal(90);
  pid.setPeriod(10);
  pid.enable();
  
  
}

void loop() {
  pid.run();
}
