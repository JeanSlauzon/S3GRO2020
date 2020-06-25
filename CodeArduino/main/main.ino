/* 
 * GRO 302 - Conception d'un robot mobile
 * Code de démarrage
 * Auteurs: Jean-Samuel Lauzon     
 * date: 15 mai 2020
*/

/*------------------------------ Librairies ---------------------------------*/
#include <ArduinoJson.h>
#include <Capteur_MPU6050.h>
#include <MPU6050.h>
#include <Sonar.h>
#include <MotorControl.h>

#include <SoftTimer.h>
#include <ArduinoJson.h>
#include <PID.h>
/*------------------------------ Constantes ---------------------------------*/

#define BAUD            115200      // Frequence de transmission serielle
#define UPDATE_PERIODE  100         // Periode (ms) d'envoie d'etat general

/*---------------------------- variables globales ---------------------------*/
Capteur_MPU6050 imu_;                 // Objet IMU
double angleX;                        // Variable pour angle X
double angleY;                        // Variable pour angle Y

MotorControl motor_(13, 12, 11, 10);  // Objet moteur
Sonar rangeSensor_(9,8);              // Objet sonar

volatile bool shouldSend_ = false;    // drapeau prêt à envoyer un message
volatile bool shouldRead_ = false;    // drapeau prêt à lire un message
volatile bool shouldPulse_ = false;   // drapeau pour effectuer un pulse
volatile bool isInPulse_ = false;     // drapeau pour effectuer un pulse
uint16_t pulseTime_ = 0;              // temps dun pulse en ms
float pulsePWM_ = 0;                  // amplitude de la tension au moteur [-1,1]

String error_;                        // message d'erreur pour le deverminage
SoftTimer timerSendMsg_;              // chronometre d'envoie de messages
SoftTimer timerPulse_;                // chronometre pour la duree d'un pulse
/*------------------------- Prototypes de fonctions -------------------------*/

void serialEvent();
void sendSerial();
void readSerial();
void endPulse();
void startPulse();
void command(double cmd);

double Kp = 0.4;
double Ki = 0.1;
double Kd = 0.0;

PID pidAngle = PID(Kp, Ki, Kd);
double angleGoal = 0;
double distanceGoal = 150.0; //mm

/*---------------------------- fonctions "Main" -----------------------------*/
void setup() {
  Serial.begin(BAUD);
  
  // Initialisation imu
  imu_.init();
  
  // Power Sonar
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);

  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(sendSerial);
  timerSendMsg_.enable();

  // Chronometre duration pulse
  timerPulse_.setCallback(endPulse);

  // Setup Angle PID
  pidAngle.setCommandFunc(command);
  pidAngle.setMeasurementFunc(measurement_angle);
  pidAngle.setGoal(angleGoal);
  pidAngle.setPeriod(10);
  pidAngle.setEpsilon(0.0);
  pidAngle.enable();
}

void loop() {
  if(shouldRead_){
    readSerial();
  }
  if(shouldPulse_){
    startPulse();
  }
  timerSendMsg_.update();
  timerPulse_.update();

  // Angle PID only
  double error;
  error = fabs(measurement_angle() - angleGoal);
  if (error < 40.0) {
    pidAngle.run();
  }
  else {
    command(0.0);
  }
}

/*---------------------------Definition de fonctions ------------------------*/
void serialEvent(){
  shouldRead_=true;
}

//Lire message du RPI
void readSerial(){
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, Serial);
    if (err) {
      error_ = "erreur deserialisation.";
      return;
    }else{
      error_ = "";
    }
    // Analyse du message
    bool pulse = doc["pulse"];
    if (pulse) {
      shouldPulse_ = true;
    }

    uint16_t pulseTime = doc["pulseTime"];
    if (pulseTime) {
      pulseTime_ = pulseTime;
    }
    
    float pulsePWM = doc["pulsePWM"];
    if (pulsePWM) {
      pulsePWM_ = pulsePWM;
    }

    float Kp = doc["setGoal"][0];
    float Ki = doc["setGoal"][1];
    float Kd = doc["setGoal"][2];
    if (Kp || Ki || Kd){
      pidAngle.setGains(Kp, Ki, Kd);
    }
    
  
    shouldRead_=false;
}

//Envoi d'un message au RPI
void sendSerial(){
  StaticJsonDocument<512> doc;
  // Construction du message a envoyer
  doc["time"] = millis();
  doc["angleX"] = angleX;
  doc["angleY"] = angleY;
  doc["sonar"] = rangeSensor_.getRange();
  doc["error"] = error_;
  doc["inPulse"] = isInPulse_;
  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
}

void endPulse(){
  /* Rappel du chronometre */
  motor_.setPWM(0);
  timerPulse_.disable();
  isInPulse_ = false;
}

void startPulse(){
  /* Demarrage d'un pulse */
  timerPulse_.setDelay(pulseTime_);
  timerPulse_.enable();
  timerPulse_.setRepetition(1);
  motor_.setPWM(pulsePWM_);
  shouldPulse_ = false;
  isInPulse_ = true;
}

void command(double cmd) {
  motor_.setPWM(cmd);
}

double measurement_angle() {
  // Get imu x and y angles
  // Options: 0 angles from gyro only
  //          1 using complementary filter
  //          2 using Kalman filter
  imu_.getAngles(&angleX, &angleY, 2);
  return (double)angleX;
}
