
#include <Wire.h>
#include "Kalman.h" 
#include "I2C.h"

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

#define BUZZER 12 //13
#define LED 12  //13
//motor  define

#define PWM_R 13  //5   //M2
#define STB 10
#define DIR_R1 11 //3 // INB_M2
#define DIR_R2 12  //4 // INA_M2
//encoder define
#define SPD_INT_R 3 //11   //interrupt INT2 = E2A
#define SPD_PUL_R 5 //12   // CNT2 = E2B
#define SPD_INT_L 2 //10   //interrupt INT1 = E1A
#define SPD_PUL_L 4 //9   // CNT1 = E1B

int pwm,pwm_l,pwm_r;
double Angle_Car;
double Gyro_Car;

double KA_P,KA_D;
double K_Base;

uint32_t LEDTimer;
bool blinkState = false;

void setup() {
  Serial.begin(115200);
  Init();  
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g

  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(200); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
 
  timer = micros();
  LEDTimer = millis();
  
  digitalWrite(BUZZER, HIGH);
  delay(50);  
  digitalWrite(BUZZER, LOW);
  delay(50);  
  digitalWrite(BUZZER, HIGH);
  delay(50); 
  digitalWrite(BUZZER, LOW);
  delay(50); 
  digitalWrite(BUZZER, HIGH);
  delay(50); 
  digitalWrite(BUZZER, LOW);
  delay(50); 
  
}

void loop() {

  double DataAvg[3];
  double AngleAvg = 0;

  DataAvg[0]=0; DataAvg[1]=0; DataAvg[2]=0;
       
    while(1)
    { //Serial.println("here");
      if(UpdateAttitude())
      { 
                                    
        DataAvg[2] = DataAvg[1];
        DataAvg[1] = DataAvg[0];
        DataAvg[0] = Angle_Car;
        AngleAvg = (DataAvg[0]+DataAvg[1]+DataAvg[2])/3;
        if(AngleAvg < 40 || AngleAvg > -40){  
          PWM_Calculate();
          Car_Control();
        }
      }      
  
      MusicPlay();   
    }
  

}


void PWM_Calculate()
{  
 
  pwm =  (Angle_Car-5 + K_Base)* KA_P   //P
      + Gyro_Car * KA_D; 

     pwm_r = pwm; //
     pwm_l = pwm;
}

void Car_Control()
{  
  
  if (pwm_r<0)
  {
    digitalWrite(DIR_R1, LOW);
    digitalWrite(DIR_R2, HIGH);
    pwm_r = -pwm_r;
  }
  else
  {
    digitalWrite(DIR_R1, HIGH);
    digitalWrite(DIR_R2, LOW);
  }
  if( Angle_Car > 45 || Angle_Car < -45 )
  {
    
    pwm_r = 0;
  }
 // pwm_l = pwm_l;  //adjust Motor different
//  pwm_r = pwm_r;
  // Serial.print(pwm_l);  Serial.print("\t"); Serial.println(pwm_r);
   
   analogWrite(PWM_R, pwm_r>127? 127:pwm_r); // Limite le InRush current à 50% //analogWrite(PWM_R, pwm_r>255? 255:pwm_r);
  
  /*
  Serial.println("pwm:"); Serial.print(pwm);
  Serial.print("\t");
  Serial.print("pwm_L:"); Serial.print(pwm_l);
  Serial.print("pwm_R:"); Serial.print(pwm_r);
 */ 
}

void Init()
{
  pinMode(BUZZER, OUTPUT);
  pinMode(SPD_PUL_L, INPUT);//
  pinMode(SPD_PUL_R, INPUT);//
  
  pinMode(PWM_R, OUTPUT);//
 
  pinMode(DIR_R1, OUTPUT);//
  pinMode(DIR_R2, OUTPUT); 
  pinMode(STB, OUTPUT);//
   digitalWrite(STB, HIGH);
   
  //init variables
 
  pwm = 0;pwm_l = 0;pwm_r = 0;
   
  KA_P = 50.0;
  KA_D = 1.5;  // 3.5;
  K_Base = 6.7;
  
}


int UpdateAttitude()
{
 if((micros() - timer) >= 10000) 
 {    //10ms 
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

 // gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
 // gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

 // compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  //compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

#if 0
  Serial.print(roll); Serial.print("\t");
 // Serial.print(gyroXangle); Serial.print("\t");
 // Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  Serial.print(pitch); Serial.print("\t");
 // Serial.print(gyroYangle); Serial.print("\t");
 // Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.println("\t");
#endif
#if 0 // Set to 1 to print the temperature
  Serial.print("\t");

  double temperature = (double)tempRaw / 340.0 + 36.53;
  Serial.print(temperature); Serial.print("\t");
#endif

  //Serial.print("\r\n");
  Serial.print(kalAngleX); // Blue Kalman
  Serial.print(" "); Serial.println(roll);// RED No Kalman
  // Serial.print("; ");Serial.print(roll);Serial.print("; ");
 //  Serial.println(timer);
  Angle_Car = kalAngleX;   //negative backward  positive forward
  Gyro_Car = gyroXrate;
 //Serial.print(Angle_Car);Serial.print("\t");Serial.println(Gyro_Car);

   return 1;
 }
  return 0;
}

bool isInBuzzer = false;

void MusicPlay()
{

  if( Angle_Car > 45 || Angle_Car < -45 ) //if car fall down ,let's alarm
  {
     if((millis() - LEDTimer) > 200)
    {
          LEDTimer = millis();
         blinkState = !blinkState;
         digitalWrite(LED, blinkState);
         isInBuzzer = true;
    }
  }
  else if(isInBuzzer == true)
  {
     digitalWrite(LED, 0);
     isInBuzzer = false;
  }
}
