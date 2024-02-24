// Author : Jack Marley and Jack Martin
// Goal : to create a positional controller for the robot that keeps track of an Arocu Marker and move with it

// Libraries
#include "DualMC33926MotorShield.h" // motor 1 from stopped to full speed forward, ramps down to full speed reverse, and back to stopped
#include <Encoder.h> // Counts quadrature pulses from rotary & linear position encoders
#include <Wire.h> // allows for communication with an I2C Device
#include <math.h> // allows for math to be calculated

// Define global variables for Ardiuno Specific Functions
// New defines 
#define ADDRESS 8
DualMC33926MotorShield motorDrive;
// Old defines
#define PI 3.1416
#define CW 0 // clockwise
#define CCW 1 // counterclockwise

#define M1DIR 7 // motor 1 direction pin
#define M2DIR 8 // motor 2 direction pin
#define M1PWM 9 // motor 1 pwm pin
#define M2PWM 10 // motor 2 pwm pin
#define M1FB A0 // motor 1 feeback pin
#define M2FB A1 // motor 2 feedback pin
#define MED 4 // motor enable/disable pin

#define CLK1 2 // encoder 1 clock pin
#define CLK2 3 // encoder 2 clock pin
#define DT1 5 // encoder 1 data pin
#define DT2 6 // encoder 2 data pin

#define BATTVOLT 7.8 // battery voltage


// Global Variables that are I2C specific
// critical to use volatile due to communication with I2C
// ensures that the compiler doesn't optimize or make assumptions about these variables

// I2C communication protocal 
volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t reply = 0;
volatile uint8_t msgLength = 0;
int8_t quadrant;

// Global Variables for Motors
  
  // velocity globals
    // actual velocity of wheels
float realVelocity1 = 0.0; // velocity of right wheel
float realVelocity2 = 0.0; // veolcity of left wheel
      // the velocity we wants of the wheels
float desiredVelocity1 = 0.0; // desired velocity of right wheel
float desiredVelocity2 = 0.0; // desired veolcity of left wheel
  // the desiredPos
      // positional
float positionalError1 = 0.0; // error of right wheel
float positionalError2 = 0.0; // error of left wheel
      // velocity
float velocityError1 = 0.0; // error of right wheel
float velocityError2 = 0.0; // error of left wheel
    // integrator 
float integratorError1 = 0.0; // right wheel
float integratorError2 = 0.0; // left wheel

  // Positional globals
  // right wheel
float newPos1 = 0.0;
float oldPos1 = 0.0;
    // desired position of wheel
float desiredPos1 = 1.5;
  // left wheel
float newPos2 = 0.0;
float oldPos2 = 0.0;
    // desired position of wheel
float desiredPos2 = 0.0;

  // global variables of voltage
float volt1;
float volt2;

  // global variables PWM
float pwm1, pwm2; 

  // make the encoders for the pins
Encoder Encoder1(CLK1,DT1); // this is clk1 = pin 2, and dt1 = pin 5 // right wheel
Encoder Encoder2(CLK2,DT2); // this is clk1 = pin 3, and dt1 = pin 6 // left wheel

  /// delay
unsigned long desired_Ts_ms = 500;
unsigned long last_time_ms;

  // feedback global variables
    //Separate gains for motor 1 and 2, motor 2 tends to
    //rotate faster with the same input voltage compared to motor 1
const float velocityKp1 = 3.25;
const float velocityKp2 = 3.15;
  
  //Kp and Ki values for the position control system
const float posKp1 = 3.22; // right wheel
const float posKp2 = 2.99; // left wheel
const float posKi1 = 0.33; // right wheel
const float posKi2 = 0.33; // left wheel

// end of global variables

void setup() // runs once 
{
  motorDrive.init(); // allows for use of lib DualMC33926MotorShield.h
  Serial.begin(115200); // set UART bits per second

  // Start I2C communication relay
  Wire.onRequest(requestFromPi);
  Wire.onReceive(receiveFromPi);
  Wire.begin(ADDRESS);
}

void loop() 
/// this will contain the information sent by the pi so that way it is continuouly being read by the Ardiuno to update position accordingly 
{
  // allows to read what is sent by the PI to allow for wheels to rotate into
  // the following quadrants work in the following way:
  //             1 | 0
  //             2 | 3  
  if (quadrant != Wire.peek()) // reads from pi when position is being changed
  {
      if (quadrant == 0) // move to quadrant 0
      {
        desiredPos1 = 0;
        desiredPos2 = 0;
      } 
      else if (quadrant == 1) // move to quadrant 1 
      {
        desiredPos1 = 0;
        desiredPos2 = PI;
      } 
      else if (quadrant == 2) // move to quadrant 2
      {
        desiredPos1 = PI;
        desiredPos2 = PI;
      } 
      else if (quadrant == 3) // move to quadrant 3
      {
        desiredPos1 = PI;
        desiredPos2 = 0;
      }
  }
  Serial.println(quadrant); // debug statement to prove we are in the right quadrant

  /// how many radians per encoder count are there
  /// radians per encoder count = 2pi / 1600 * 2 = 0.007854 
  /// so approximately 0.008 radians per encoder count
  // this is not displaying anything except 0
  
  newPos1 = -1 * Encoder1.read() * .25 * 0.007854; // right wheel
  newPos2 = Encoder2.read() * .25 * 0.007854; // left wheel
  //  Serial.print("newPos1: ");
  //Serial.println(newPos1);
  //Serial.print("newPos2: ");
  //Serial.println(newPos2);

  // All of the following code below is determined using the 
  // the handouts in class in order to find and understand the PID
  // and PI algothrims 
  
  // velocity calculations
  realVelocity1 = (newPos1 - oldPos1) / ((float)(10/1000.0));
  realVelocity2 = (newPos2 - oldPos2) / ((float)(10/1000.0));
  //debug
  //Serial.print("realVelocity1: ");
  //Serial.println(realVelocity1);
  //Serial.print("realVelocity2: ");
  //Serial.println(realVelocity2);

  //Calculate errors from desired speed for each motor
  positionalError1 = desiredPos1 - newPos1;
  positionalError2 = desiredPos2 - newPos2;

  // debug
  //Serial.print("positionalError1: ");
  //Serial.println(positionalError1);
  //Serial.print("positionalError2: ");
  //Serial.println(positionalError2);

  //Calculate integral error
  integratorError1 += (positionalError1) * ((float)(delaySystem/1000.0));
  integratorError2 += (positionalError2) * ((float)(delaySystem/1000.0));

  // debug statement
  //Serial.print("integratorError1: ");
  //Serial.println(integratorError1);
  //Serial.print("integratorError2: ");
  //Serial.println(integratorError2);

  // find the velocity needed in order to shift our motors into the right quadrant
  desiredVelocity1 = posKp1 * positionalError1 + posKi1 * integratorError1;
  desiredVelocity2 = posKp2 * positionalError2 + posKi2 * integratorError2; 

  // debug statement
  //Serial.print("desiredVelocity1: ");
  //Serial.println(desiredVelocity1);
  //Serial.print("desiredVelocity2: ");
  //Serial.println(desiredVelocity2);

  //determine the error of our velocity based off of position and the velocity 
  velocityError1 = desiredVelocity1 - realVelocity1;
  velocityError2 = desiredVelocity2 - realVelocity2;

  volt1 = velocityKp1*velocityError1; // apply the necessary voltage to get the proper velocity needed 
  volt2 = velocityKp2*velocityError2; // apply the necessary voltage to get the proper velocity needed 

  // The following code below is old code from Jack Martin's Velocity Design
  // check the voltage values
  if (volt1<0) 
  {
    digitalWrite(M1DIR,HIGH); // M1DIR = pin 7
  } 
  else 
  {
    digitalWrite(M1DIR,LOW); // M1DIR = pin 7
  }


  if (volt2>0) {
    digitalWrite(M2DIR,HIGH); // M2DIR = pin 8
  } 
  else 
  {
    digitalWrite(M2DIR,LOW); // M2DIR = pin 8
  }


  // apply the needed voltage to allow the motor to run to that position
  pwm1 = 255*abs(volt1)/BATTVOLT; 
  pwm2 = 255*abs(volt2)/BATTVOLT;
 
  //Apply the PWM to set the voltage for the motors
  analogWrite(M1PWM,min(pwm1,255)); //M1PWM feedback pin 9
  analogWrite(M2PWM,min(pwm2,255)); // M2PWM feedback pin 10
 
  delay(10); // allow a delay so that way the motors can run before being set 

  oldPos1 = newPos1; // set old pos = to new that way we can keep track of pos
  oldPos2 = newPos2; // set old pos = to new that way we can keep track of pos

}

void receiveFromPi() // this receives the information from the pi in order to make the wheel move
{ 
    offset = Wire.read(); //read reads in the first byte of the wire, with the register to write to (after address and write bit since master is writing)
    //until the master decides to stop sending data
    while (Wire.available()) 
    {
      instruction[msgLength] = Wire.read(); //read it byte by byte
      quadrant = instruction[msgLength]; //theoretically here we are just taking the first byte
      msgLength++; // increments the value in the array to move to the next index that is sent from the pi
      Serial.print(quadrant); // debug staement to prove we are receiving the correct quadrant 

    }
}


void requestFromPi() // this sends the information that we need to the pi i dont think this is used until later
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
