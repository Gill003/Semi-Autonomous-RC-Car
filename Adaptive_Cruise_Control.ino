#include <util/atomic.h>
#include <NewPing.h>

// Pins for motor 
#define PWM 5
#define IN2 3
#define IN1 4

//ultrasonic sensor
#define TRIGGER_PIN  7
#define ECHO_PIN     6
#define MAX_DISTANCE 200
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// globals
long prevT = 0;
int posPrev = 0;

//Distance measured
float Dm = 0;

//used to compute control signal 
float eintegral = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

}

void loop() {

  //measure distance of object in front
  Dm =sonar.ping_cm();

  //Get current time and calculate change in time
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  prevT = currT;

  // Set a target distance
  float Dt = 20;

  // Compute the control signal using PID control
  float kp = 20;
  float ki = 0.25;
  float e = Dm-Dt;
  eintegral = eintegral + e*deltaT;
  float u = kp*e + ki*eintegral;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  setMotor(dir,pwr,PWM,IN1,IN2);

  delay(1);
}

//Function which sets the motors speed by taking in the direction, speed and pins 
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}


