//This is not a completed program. Please be sure to include these things in the arduino program though


#include <Wire.h>
#define MY_ADDR 8
// Global variables to be used for I2C communication
volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t reply = 0;
volatile uint8_t msgLength = 0;

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts
  Wire.onRequest(requestFromPi);
  Wire.onReceive(receiveFromPi);
}

void receiveFromPi() // this receives the information from the pi about starting program and later angle
{ 
    offset = Wire.read(); //read reads in the first byte of the wire, with the register to write to (after address and write bit since master is writing)
    //until the master decides to stop sending data

    //this will need to be updated as well to fit. 
    while (Wire.available()) 
    {
      instruction[msgLength] = Wire.read(); //read it byte by byte
      quadrant = instruction[msgLength]; //theoretically here we are just taking the first byte
      msgLength++; // increments the value in the array to move to the next index that is sent from the pi
      Serial.print(quadrant); // debug staement to prove we are receiving the correct quadrant 
    }

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
    quadrant = instruction[0]; //theoretically here we are just taking the first byte
}