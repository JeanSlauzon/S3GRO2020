#include <MPU6050.h>
#include <Sonar.h>
#include <MotorControl.h>
#include <PID.h>


// MAIN CODE
MPU6050 imu;
PID pid = PID(0.00005,0.0001,0);
MotorControl motor = MotorControl(13, 12, 11, 10);
Sonar rangeSensor = Sonar(9,8);


double measurement(){
  int16_t gx, gy, gz;
  gy = imu.getRotationY();
  Serial.println(gy);
  return (double)gy;
  
}
void command(double cmd){
  motor.setPWM(cmd);
}

void setup() {
  Serial.begin(115200);
  
  // initialize imu
  imu.initialize();
  
  // Power Sonar
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  pid.setCommandFunc(command);
  pid.setMeasurementFunc(measurement);
  pid.setGoal(0);
  pid.setPeriod(10);
  pid.enable();
  
  
}

void loop() {

  pid.run();
}
