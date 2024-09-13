#include "SPI.h"
#include "NRFLite.h"


//Setup NRFL01 pins for SPI communication
const static uint8_t RADIO_ID = 1;             
const static uint8_t DESTINATION_RADIO_ID = 0; 
const static uint8_t PIN_RADIO_CE = 9;        
const static uint8_t PIN_RADIO_CSN = 10;

//Set up x and y values from joysticks to analog pins
const int y=A4;
const int x=A1;



unsigned long myTime;
int yval;
int xval;

// Create a packet to send to recvier
// Any packet up to 32 bytes can be sent.
struct RadioPacket 
{
    uint8_t FromRadioId;
    uint32_t OnTimeMillis;
    uint32_t FailedTxCount;
    float yval;
    float xval;
    float time;
};
NRFLite _radio;
RadioPacket _radioData;

void setup()
{
    Serial.begin(115200);
    

    // Initialize the radio module with specified parameters
    if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN))
    {
        Serial.println("Cannot communicate with radio");
        while (1); // Wait here forever.
    }
    _radioData.FromRadioId = RADIO_ID;
}

void loop()
{ 
    //Get current time, xval, and yval to send to reciver 
    myTime = millis();
    yval=analogRead(y);
    xval=analogRead(x);
    _radioData.yval = yval;
    _radioData.xval = xval;
    _radioData.time=myTime;

    //Print the y and x value that we are sending over for the user
    Serial.print("Sending ");
    Serial.print(" y:");
    Serial.print(_radioData.yval);
    Serial.print(" x:");
    Serial.print(_radioData.xval);
    Serial.print(" time:");
    Serial.print(_radioData.time);
    

    // Send the data packet to the destination radio module
    // The NO_ACK option means the sender will not wait for an acknowledgment from the receiver
    if (_radio.send(DESTINATION_RADIO_ID, &_radioData, sizeof(_radioData), NRFLite::NO_ACK)) // Note how '&' must be placed in front of the variable name.
    {
        // Print a success message if the data was sent successfully
        Serial.println("...Success");
    }
    else
    {
        // Print a failure message if sending the data failed
        Serial.println("...Failed");
        _radioData.FailedTxCount++;
    }

    delay(100);
}
