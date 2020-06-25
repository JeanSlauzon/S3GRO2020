/* 
 * GRO 302 - Conception d'un robot mobile
 * Code de démarrage
 * Auteurs: Jean-Samuel Lauzon    
 *          Simon Chamorro
 * date: 15 juin 2020
*/

/*------------------------------ Librairies ---------------------------------*/
#include <ArduinoJson.h>
#include <Capteur_MPU6050.h>
#include <MotorControl.h>

/*------------------------------ Constantes ---------------------------------*/

#define BAUD            115200      // Frequence de transmission serielle

/*---------------------------- variables globales ---------------------------*/

Capteur_MPU6050 imu_;                 // Objet IMU
double angleX;                        // Variable pour angle X
double angleY;                        // Variable pour angle Y
double last_err = 0.0;
double last_t = millis();
double integral_err = 0.0;

MotorControl motor_(13, 12, 11, 10);  // Objet moteur          

/*------------------------- Prototypes de fonctions -------------------------*/

void command(double cmd);

double Kp = 0.1;
double Ki = 0.0;
double Kd = 0.0;

double angleGoal = 0.0;

/*---------------------------- fonctions "Main" -----------------------------*/


void setup() {
  Serial.begin(BAUD);
  
  // Initialisation imu
  imu_.init();
  
}

void loop() {

  // Angle PID only
  double cmd = 0.0;
  double error;
  double t = millis();
  double dt = t - last_t;
  error = angleGoal -  measurement_angle();
  integral_err = integral_err + error * dt;
  
  if (fabs(error) < 40.0) {
    double p_command = Kp * error;
    double d_command = Kd * (error - last_err)/dt;
    double i_command = Ki * integral_err;
    cmd = p_command + d_command + i_command;
  }
  motor_.setPWM(cmd);

  last_err = error;
  last_t = t;
  delay(10);
}


/*---------------------------Definition de fonctions ------------------------*/


double measurement_angle() {
  // Get imu x and y angles
  // Options: 0 angles from gyro only
  //          1 using complementary filter
  //          2 using Kalman filter
  imu_.getAngles(&angleX, &angleY, 2);
  return (double)angleX;
}
