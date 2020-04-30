#include <Wire.h>


class SonicRangeFinder
{
  public:
    SonicRangeFinder(uint8_t trigPin, uint8_t echoPin){
      trigPin_ = trigPin;
      echoPin_ = echoPin;
      pinMode(trigPin_, OUTPUT);
      pinMode(echoPin_, INPUT);
  
    };
    float getRange(){
      digitalWrite(trigPin_, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin_, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin_, LOW);
      long duration = pulseIn(echoPin_, HIGH);
      // Calculating the distance
      return duration*0.34/2.0;
    }
  private:
    uint8_t trigPin_;
    uint8_t echoPin_;
};


class MotorControl
{
  public:
    MotorControl(uint8_t PWMPin, uint8_t Input1Pin, uint8_t Input2Pin, uint8_t StandbyPin){
    PWMPin_ = PWMPin;
    Input1Pin_ = Input1Pin;
    Input2Pin_ = Input2Pin;
    StandbyPin_ = StandbyPin;
    pinMode(PWMPin_, OUTPUT);
    pinMode(Input1Pin_, OUTPUT);
    pinMode(Input2Pin_, OUTPUT);
    pinMode(StandbyPin_, OUTPUT);
    };

    void setPWM(float speed){
      digitalWrite(StandbyPin_, HIGH);
      // Verify speed between [-1, 1]
      if(speed>1){
        speed = 1;
      }
      if(speed<-1){
        speed = -1;
      }
      // Motor direction
      if(speed>0){
        digitalWrite(Input1Pin_, HIGH);
        digitalWrite(Input2Pin_, LOW);
      }else{
        digitalWrite(Input1Pin_, LOW);
        digitalWrite(Input2Pin_, HIGH);
      }
      // Set PWM
      speed = fabs(speed);
      analogWrite(PWMPin_, 255*speed);
    };

    void disable(){
      analogWrite(PWMPin_, 0);
      digitalWrite(StandbyPin_, LOW);
    };


  private:
    uint8_t PWMPin_;
    uint8_t Input1Pin_;
    uint8_t Input2Pin_;
    uint8_t StandbyPin_; 
};





MotorControl motor = MotorControl(13, 12, 11, 10);
SonicRangeFinder rangeSensor = SonicRangeFinder(9,8);
void setup() {
  Serial.begin(115200);
  pinMode(53,OUTPUT);
  digitalWrite(53, HIGH);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(rangeSensor.getRange());
  delay(1000);
}
