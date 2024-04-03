//This is not a completed program. Please be sure to include these things in the arduino program though


#include <Wire.h>
#define MY_ADDR 8

// Global variables to be used for I2C communication
volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
volatile uint8_t start = 0;
volatile uint8_t angle_received = 0;
volatile float angle = 0;

volatile union FloatUnion {
    uint8_t bytes[4];
    float floatValue;
} angle_convert;

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts
  Wire.onRequest(requestFromPi);
  Wire.onReceive(receiveFromPi);
}
//source: https://forum.arduino.cc/t/convert-ieee-754-32-to-float/323109/2


void receiveFromPi() // this receives the information from the pi about starting program and later angle
{ 
    offset = Wire.read(); //read reads in the first byte of the wire, with the register to write to (after address and write bit since master is writing)
    //until the master decides to stop sending data

    //this will need to be updated as well to fit. 
    while (Wire.available()) 
    {
      instruction[msgLength] = Wire.read(); //read it byte by byte
      msgLength++; // increments the value in the array to move to the next index that is sent from the pi
    }

    //offset = 0 here, not that it matters
    if (msgLength == 1 && msg[0] == 0){
        start=1; //this is a flag to raise to indicate to the arduino that we're ready to start spinning
    }
    //offset is 1 here, again, doesn't matter
    else if (msgLength == 4){
        angle_convert.bytes[0] = instruction[3];
        angle_convert.bytes[1] = instruction[2];
        angle_convert.bytes[2] = instruction[1];
        angle_convert.bytes[3] = instruction[0];
        angle = angle_convert.floatValue;
        angle_received = 1; //this is the flag that is raised to tell the controls team to change its state to rotating to a specific angle

    }
    msgLength = 0; //reset it here, since we use it already?
    //not sure how to implement, but if offset = 0, start spinning
    //if offset = 1, then the next 2 bytes are the angle. Figure out how to decode it or whatever
}


void requestFromPi() // this sends the information that we need to the pi i dont think this is used until later. Could be useful for acks, if we need those
{
    offset = Wire.read(); //read reads in the first byte of the wire, with the register to write to (after address and write bit since master is writing)
    //until the master decides to stop sending data
    while (Wire.available()) 
    {
      instruction[0] = Wire.read(); //read it byte by byte
      msgLength++;
    }
}