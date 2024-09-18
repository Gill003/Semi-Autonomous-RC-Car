#include <util/atomic.h>

// Pins for motor 
#define ENCA 15 // YELLOW
#define ENCB 16// WHITE
#define PWM 5
#define IN2 3
#define IN1 4

// globals
long prevT = 0;
int posPrev = 0;

// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;


//Filtered velocity
float v1Filt = 0;
float v1Prev = 0;

//used to compute control signal
float eintegral = 0;

void setup() {

  Serial.begin(115200);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  //Triggers when change is detected in the 2 signals of the encoder
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoderA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB),readEncoderB,CHANGE);
}

void loop() {

  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    
  }

  // Compute velocity 
  // Get current time and subtract the prevtimes to get change in time(deltaT)
  //Take change in position(pos-posPrev) over change in time to get velocity
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to Cm/hour
  // float v1 = (velocity/600.0*60.0);
  // Convert count/s to Km/hour
  //one revolution of the wheel is 680 positions on encoder
  float v1 = (((velocity/680.0)*3600.00)*21.67)/100000;
 

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  

  // Set a target (km/h)
  float vt = 2;

  // Compute the control signal u
  float kp = 5;
  float ki = 10;
  float e = vt-v1Filt;
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


//If encoder A has a state change, then readEncoderA is triggered which checks if Signal B is the same as signal A.
//If they are not the same then we increment position variable but if they are the same then we decrement it
void readEncoderA(){
  if(digitalRead(ENCB) != digitalRead(ENCA)) {
    pos_i ++;
  } else {
    pos_i --;
  }
}
//If encoder B has a state change, then readEncoderB is triggered which checks if Signal A is the same as signal B.
//If they are the same then we increment position variable but if they are not the same then we decrement it

void readEncoderB(){
    if (digitalRead(ENCA) == digitalRead(ENCB)) {
    pos_i ++;
  } else {
    pos_i --;
  }
}