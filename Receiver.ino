#include <Servo.h>
#include "SPI.h"
#include "NRFLite.h"
#include <NewPing.h>
#include <util/atomic.h> 

// Define constants for encoder pins
#define ENCA 15 
#define ENCB 16 

// Define constants for Ultrasonic sesnor and set maximum distance to 200cm
#define TRIGGER_PIN  7
#define ECHO_PIN     6
#define MAX_DISTANCE 200

// Define radio parameters
const static uint8_t RADIO_ID = 0;      
const static uint8_t PIN_RADIO_CE = 9;
const static uint8_t PIN_RADIO_CSN = 10;

// Define motor pins
const int forwardPin = 3;
const int backPin = 4;
const int speedPin = 5;

// Define buzzer pin which is used for emergency braking
const int buzzer = A0;

// Initialize sonar
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Define variables for servo
int servoVal = 80;
Servo Servo1;
int servoPin = 2;

// Variables for motor speed and control
int speed = 0;
bool moving = false;

// Define variables for radio packet
struct RadioPacket {
    uint8_t FromRadioId;
    uint32_t OnTimeMillis;
    uint32_t FailedTxCount;
    float yval;
    float xval;
    float time;
};

NRFLite radio;
RadioPacket radioData;

void setup() {
    Serial.begin(115200);
    // Set pin modes
    pinMode(buzzer, OUTPUT);
    pinMode(forwardPin, OUTPUT);
    pinMode(backPin, OUTPUT);
    Servo1.attach(servoPin);

    // Error message if radio initialization fails.
    if (!radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN)) {
        Serial.println("Cannot communicate with radio");
        while (1); // Wait here forever.
    }
}
//
void convertYVal(int yval) {
    // Set y limit of 1000 to keep conversions simpler.
    if (yval > 1000) {
        yval = 1000;
    }
    // Scale down yval to adjust for servos 180° 
    servoVal = (yval / 10) + 30;

    // 80° is set as center of servo.
    // Reverse servo values so the left is right and right is left.
    // This is because the servo movement is opposite of the joystick movement so it needs to be reversed.
    if (servoVal > 80) {
        servoVal = (130 - servoVal) + 30;
    } else if (servoVal < 80) {
        servoVal = (30 - servoVal) + 130;
    }
}

void convertXVal(int xval) {
  // The joystick values we receive are between 480 and 540 when still and go from 480-0 when pushed down and go from 540-1023 when pushed up
  // The input value which controls the motor speed is 0-255.
  // To scale the x values to control the speed of the motors, I set 480-0 to go from 0-255 so when pushed all the way down the speed is set to 255.
  // Similarly, the y values from 540-1023 are converted to 0-255, so when the joystick is pushed all the way up the speed is set to 255.
  // When the joystick is not moved and the x values are in the range of 480-540, the speed is set to 0.
    if (xval <= 480) {
        speed = ((480 - xval) * 2) / 3.7;
    } else if (xval >= 540) {
        speed = ((xval - 540) * 2) / 3.7;
    } else {
        speed = 0;
    }
}

void setDirection(int xval) {
  // When the joystick is pushed up (xval >=540), the RC car direction is set to forward.
  // When the joystick is pushed down (xval <+480), the RC car direction is set to reverse.
    if (xval <= 480) {
        digitalWrite(forwardPin, HIGH);
        digitalWrite(backPin, LOW);
    } else if (xval >= 540) {
        digitalWrite(forwardPin, LOW);
        digitalWrite(backPin, HIGH);
    } else {
        digitalWrite(forwardPin, LOW);
        digitalWrite(backPin, LOW);
    }
}


void checkEmergencyStop(int xval) {
  // This function checks if there is an object in front of the car using the ultrasonic sensor and stops the car.
  // The ultrasonic sensor gets the distance of the object in front of the car using the ping_cm() function and then checks if the distance is less than or equal to 25cm.
  // It also checks if the car is on track to crash into the object by checking if the xval>=540.
    float distance = sonar.ping_cm();
    Serial.println(distance);
    if (distance <= 25 && xval >= 540 && distance != 0) {
      // Buzzer alerts driver that there is an object in front of the car.
      // If the car is already moving towards the object, a reverse movement is triggered to stop the moment and simulate braking.
        tone(buzzer, 100);
        if (moving) {
            digitalWrite(forwardPin, LOW);
            digitalWrite(backPin, HIGH);
            analogWrite(speedPin, 1000);
            delay(50);
        }
        // The speed of the car is set to 0 and it is unable to move forward toward the object and moving is set to false.
        speed = 0;
        analogWrite(speedPin, speed);
        moving = false;
    } else 
    {
        // If there is no object in front of the car, the buzzer is turned off and if the speed is greater than 100, moving is set to true.
        noTone(buzzer);
        if (speed > 100) {
            moving = true;
        }
    }
}

void loop() 
{   //Enters while loop when the radio receives data from the transmitter
    while (radio.hasData()) {
        radio.readData(&radioData); // Read data from radio
        //Set the yval and xval from the received values
        int yval = radioData.yval;
        int xval = radioData.xval;

        // Run all the functions
        convertYVal(yval);
        convertXVal(xval);
        setDirection(xval);
        Servo1.write(servoVal);
        checkEmergencyStop(xval);
        analogWrite(speedPin, speed);
    }
}
