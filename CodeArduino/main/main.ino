#include <MPU6050.h>
#include <Sonar.h>
#include <MotorControl.h>
#include <PID.h>


// MAIN CODE
MPU6050 imu;
MotorControl motor = MotorControl(13, 12, 11, 10);
Sonar rangeSensor = Sonar(9,8);
void setup() {
  Serial.begin(115200);
  
  // initialize imu
  imu.initialize();
  
  // Power Sonar
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);

  motor.setPWM(.5);
  delay(1000);
  motor.disable();
  
  
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  Serial.println(rangeSensor.getRange());
  delay(500);
}
