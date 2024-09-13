# Semi-Autonomous-RC-Car

## Description

I developed and designed an RC car from scratch, incorporating semi-autonomous features such as adaptive cruise control and low-speed emergency braking. Driven by a passion for embedded software development and automotive engineering, I integrated these interests into a project that aims to advance the capabilities of remote-controlled vehicles.

The RC car is built around an Arduino Nano microcontroller. For steering, I implemented a servo motor at the front wheel shaft and for the movement, I used 2 DC-encoded motors controlled using a L293D quadruple half-H driver. I designed and tested different prototypes on AutoCad and printed them using a 3D printer.

I used an SPI communication protocol on nRF24L01 modules, allowing seamless communication with the controller and the car. The controller has two joysticks for manual control of the car’s movement. For the Advanced Driver Assistance System (ADAS) features, I used an  HC-SR04 ultrasonic sensor at the front of the car to detect the distance of objects in front of the car which allowed me to implement low-speed braking and adaptive cruise control.

<img src="images/RCPic1.jpg" alt="Screenshot" width="50%">




## Table of Contents 

- [SPI Communication](#SPI-Communication)
- [Motor Driver](#Motor_driver)
- [Adaptive cruise control](#Adaptive_Cruise_Control)
- [Low-speed braking](#Low-speed_braking)

## SPI-Communication
I employed an nRF24L01 Transceiver Module, which uses SPI (Serial Peripheral Interface) communication. On the controller side, one nRF24L01 module acts as the master, while another nRF24L01 on the RC car serves as the receiver. I utilized an adapter to facilitate the connection of the nRF24L01 to the PCB, allowing for straightforward plug-in and soldering.

For power, the GND pin of the nRF24L01 is connected to the Arduino’s ground, and the VCC pin is connected to the Arduino’s 3.3V output. The CE (Chip Enable) pin, which is active-high, is connected to pin 9 on the Arduino. When enabled, this pin allows the nRF24L01 to either transmit or receive data. The CSN (Chip Select Not) pin, an active-low signal, is linked to pin 10 on the Arduino. This pin must be set low for the nRF24L01 to listen for data on the SPI bus.

The SCK (Serial Clock) pin, responsible for receiving clock pulses from the SPI bus master, is connected to pin 13 on the Arduino. The MOSI (Master Out Slave In) and MISO (Master In Slave Out) pins handle data transmission: MOSI (SPI input) is connected to pin 11, while MISO (SPI output) is connected to pin 12 on the Arduino.

The Arduino receives input from the joysticks, which it then sends to the master nRF24L01 module. This master module transmits the data to the nRF24L01 module on the RC car. The receiver module on the car then sends this data to the Arduino, where it is processed and used to control the RC car’s movements.




## Motor_driver



## Adaptive_Cruise_Control



## Low-speed_braking



