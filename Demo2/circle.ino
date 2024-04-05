// name: move
// purpose: recive an angle, spin, move one foot

//////// includes ////////
#include <math.h>
#include <Wire.h>

//////// defines ////////
#define DEBUG 0
#define PI 3.1416
#define CW 0 // clockwise
#define CCW 1 // counterclockwise
// pins:
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
#define TPR 1600 // encoder ticks per rotation
#define DTIME 80 // encoder debounce time us (time high or low before encoder state change)
// properties:
#define WHEELDIAMETER 150 // wheel diameter (mm)
#define WHEELCIRCUM WHEELDIAMETER * PI // wheel circumfrance (mm)
#define AXLELENGTH 355.6 // length between wheels (mm)
#define ROTCIRCUM AXLELENGTH * PI // circumfrance of rotation (mm)
#define BATTVOLT 7.8 // battery voltage
// convertions:
#define TICK2FT 0.0009813 // converts ticks to feet, TICK2FT = WHEELCIRCUMFT / 1600
#define FT2TICK 1019 // converts feet to ticks, FT2TICK = 1600 / WHEELCIRCUMFT
#define TICK2ROT 0.095612787 // converts ticks to robot rotations, TICK2ROT = (WHEELCIRCUM / 1600) * (360 / ROTCIRCUM)
#define ROT2TICK 10.458852 // converts robot rotations to ticks, ROT2TICK = (1600 / WHEELCIRCUM) * (ROTCIRCUM / 360)
// control:
#define DIFFINC 1.28 // desired position difference increment (ticks)
#define BASEINC 1.28 // desired position base increment (ticks)
// i2c:
#define MY_ADDR 8

//////// variables ////////
typedef enum {LISTEN, STEPS, CIRCLE, DONE} state;
short ctrlBusy = 0; // control is completeing step
short ctrlStep = 0; // active controller step
long startTime; // loop start time (ms)
// control:
float setAngle = 0; // set angle (degrees)
float setPosition = 0; // set position (feet)
float setPositionDiff; // set position base (ticks)
float setPositionBase; // set position differece (ticks)
float angKp = 0; // position proportional gain
float angKi = 0; // position integral gain
float posKp = 0; // position proportional gain
float posKi = 0; // position integral gain
float veldKp = 0; // velocity proportional gain
float velbKp = 0; // velocity proportional gain
float maxVolt = 0;// maximum voltage
float desiredPositionDiff = 0; // desired position difference (ticks)
float desiredPositionBase = 0; // desired position base (ticks)
float desiredVelocityDiff; // desired velocity difference (ticks/s)
float desiredVelocityBase; // desired velocity base (ticks/s)
float voltageDiff; // voltage difference (V)
float voltageBase; // voltage base (V)
float posDiffError = 0; // position difference error (ticks)
float prevPosDiffError = 0; // previous position difference error (ticks)
float posDiffIntError = 0; // integral position difference error (ticks)
float maxPosDiffIntError = 80; // maximum integral position difference error (ticks)
float posBaseError = 0; // position base error (ticks)
float prevPosBaseError = 0; // previous position base error (ticks)
float posBaseIntError = 0; // integral position base error (ticks)
float maxPosBaseIntError = 80; // maximum integral position difference error (ticks)
float velDiffError; // velocity difference error (ticks/s)
float velBaseError; // velocity base error (ticks/s)
long ctrltime; // time of feedback control calculation (us)
long prevCtrltime; // previous time of feedback control calculation (us)
// measured:
volatile short tick = 0; // encoder has ticked
volatile short direction[2]; // encoder direction
volatile long ticks[2] = {0, 0}; // encoder ticks
volatile unsigned long ticktime[2]; // time of most recent encoder tick (us)
volatile unsigned long tickperiod[2]; // period between most recent two encoder ticks (us)
// computed:
float tps[2]; // encoder ticks per second
float tpssqr[2]; // encoder ticks per second^2
static float actualAngle; // actual angle (degrees)
static float actualPosition; // actual position (feet)
static float actualPositionDiff; // actual position difference (ticks)
static float actualPositionBase; // actual position base (ticks)
static float actualVelocity[2]; // actual velocity (ticks/s)
static float actualVelocityDiff; // actual velocity difference (ticks/s)
static float actualVelocityBase; // actual velocity base (ticks/s)
static float voltage[2]; // motor voltage
float amperage[2]; // motor amps
// thresholds:
static float angleThreshold = 0.4;
static float positionThreshold = 0.1;
// i2c:
volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
volatile uint8_t start = 0;
volatile uint8_t angle_received = 0;
volatile float angle = 0;

volatile union FloatUnion 
{
  uint8_t bytes[4];
  float floatValue;
} angle_convert;

//////// headers ////////
int ticks2milim();

//////// settup ////////
void setup() 
{  
  //// pins ////
  // motor:
  pinMode(M1DIR, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  pinMode(M1FB, INPUT);
  pinMode(M2FB, INPUT);
  pinMode(MED, OUTPUT);
  // encoder:
  pinMode(CLK1, INPUT_PULLUP);
  pinMode(CLK2, INPUT_PULLUP);
  pinMode(DT1, INPUT_PULLUP);
  pinMode(DT2, INPUT_PULLUP);
  //// motor ////
  digitalWrite(MED, 1); // enable motor voltage
  digitalWrite(M1DIR, 0); // set motor voltage direction
  digitalWrite(M2DIR, 1); // set motor voltage direction
  analogWrite(M1PWM, 0); // set motor voltage magnitude
  analogWrite(M2PWM, 0); // set motor voltage magnitude
  //// uart ////
  Serial.begin(2000000); // set UART bits per second
  //// i2c ////
  Wire.begin(MY_ADDR); // initialise
  Wire.onRequest(requestFromPi);
  Wire.onReceive(receiveFromPi);
  //// interrupts ////
  attachInterrupt(digitalPinToInterrupt(CLK1), clk1Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CLK2), clk2Change, CHANGE);
  //// stablise ////
  delay(1000);
  startTime = millis();
}

//////// loop ////////
void loop() 
{
  //// variables //// 
  static state setState = LISTEN; 
  static int fullCircleTicks = 0;
  // display:
  long time = millis();
  static long printtime = 0; // previous print time (ms)

  //// set ////
  switch(setState)
  {
    case LISTEN:
      if(start)
      {
        if(ctrlBusy == 0) // if not controling
        {
          resetCtrl();
          maxVolt = 1.8;
          setK(18, 8, 180, 3, 0.0008, 0.002);
          setAngle = 3600;
          setPositionDiff = rotdeg2ticks(setAngle);
          setPositionBase = feet2ticks(setPosition);
          ctrlBusy = 1;
        }
        if(angle_received)
        {
          setK(0, 0, 0, 0, 0, 0); // disable control
          setAngle = actualAngle;
          setPositionDiff = actualPositionDiff;
          desiredPositionDiff = actualPositionDiff;
          setPosition = actualPosition;
          setPositionBase = actualPositionBase;
          desiredPositionBase = actualPositionBase;
          ctrlBusy = 0;
          setState = STEPS;
        }
      }
      break;
    case STEPS:
      if(ctrlBusy == 0) // if not controling
      {
        switch(ctrlStep) // control steps
        {
          case 0: // step 1, rotate to requested angle
            resetCtrl();
            maxVolt = 4;
            setThresh(1, 0.4);
            setK(34, 8, 100, 3, 0.001, 0.002);
            setAngle = setAngle - angle;
            break;
          case 1: // step 2, move 1 foot
            resetCtrl();
            maxVolt = 7;
            setThresh(1, 0.4);
            setK(60, 1.4, 48, 0.2, 0.002, 0.001);
            setPosition = setPosition + 6.2;
            break;
          case 2: // step 3, rotate 90 degrees
            resetCtrl();
            maxVolt = 4;
            setThresh(1, 0.1);
            setK(34, 8, 70, 3, 0.001, 0.002);
            setAngle = setAngle - 90;
            break;
          default:
            setState = CIRCLE;
            break;
        }
        setPositionDiff = rotdeg2ticks(setAngle);
        setPositionBase = feet2ticks(setPosition);
        if(setState == STEPS) ctrlBusy = 1;
      }
      else if(ctrlBusy == 1) // if controling
      {
        if(reached(setAngle, actualAngle, angleThreshold) && reached(setPosition, actualPosition, positionThreshold) && steady(voltage, 1.1)) // if control has stopped
        {
          ctrlStep++;
          ctrlBusy = 0;
        }
      }
      break;
    case CIRCLE:
      if(ctrlBusy == 0)
      {
        maxVolt = 7;
        setK(18, 2, 18, 2, 0.002, 0.002);
        desiredVelocityDiff = 2000;
        desiredVelocityBase = 4000;
        fullCircleTicks = ticks[1] + 11200;
        ctrlBusy = 1;
      }
      if(reached(ticks[1], fullCircleTicks, 10)) 
      {
        ctrlBusy = 0;
        setState = DONE;
      }
      break;
    case DONE:
      setVolt(0, M1PWM, M1DIR);
      setVolt(0, M2PWM, M2DIR);
      break;
  }
  // ramp:
  if(abs(setPositionDiff - desiredPositionDiff) > DIFFINC) // if difference between set and desired position difference is larger than difference increment
    if((setPositionDiff - desiredPositionDiff) > 0) desiredPositionDiff += DIFFINC; // increment desired position difference
    else desiredPositionDiff -= DIFFINC; // decrement desired position difference
  else if(abs(setPositionDiff - desiredPositionDiff) > 0) desiredPositionDiff += (setPositionDiff - desiredPositionDiff); // increment/decrement desired position base by remainder
  if(abs(setPositionBase - desiredPositionBase) > BASEINC) // if difference between set and desired position base is larger than base increment
    if((setPositionBase - desiredPositionBase) > 0) desiredPositionBase += BASEINC; // increment desired position base
    else desiredPositionBase -= BASEINC; // decrement desired position base
  else if(abs(setPositionBase - desiredPositionBase) > 0) desiredPositionBase += (setPositionBase - desiredPositionBase); // increment/decrement desired position base by remainder

  //// track ////
  long tick0DEBUG = ticks[0];
  long tick1DEBUG = ticks[1];
  amperage[0] = analogRead(M1FB) / 1024 / 0.56; // compute motor 1 amps
  amperage[1] = analogRead(M2FB) / 1024 / 0.56; // compute motor 2 amps
  actualPositionDiff = (ticks[1] - ticks[0]) / 2;
  actualPositionBase = ticks[1] - actualPositionDiff;
  actualVelocity[0] = calctps(tickperiod[0], ticktime[0], direction[0]);
  actualVelocity[1] = calctps(tickperiod[1], ticktime[1], direction[1]);
  actualVelocityDiff = (actualVelocity[1] - actualVelocity[0]) / 2;
  actualVelocityBase = actualVelocity[1] - actualVelocityDiff;

  //// control ////
  prevCtrltime = ctrltime;
  ctrltime = micros();
  if(setState == STEPS || setState == LISTEN) apCtrl(desiredPositionDiff, desiredPositionBase);
  else if(setState == CIRCLE) vCtrl(desiredVelocityDiff, desiredVelocityBase);

  //// display ////
  actualAngle = ticks2rotdeg(actualPositionDiff);
  actualPosition = ticks2feet(actualPositionBase);
  if(DEBUG && ((time - printtime) > 1600)) // if it has been 1600ms since last print
  {
    Serial.print("\n\n");
    Serial.println("ROBOT");
    Serial.print("state: "); Serial.println(setState);
    Serial.print("reached: "); Serial.print(reached(setAngle, actualAngle, 1)); Serial.print(" "); Serial.print(reached(setPosition, actualPosition, 0.05)); Serial.print(" "); Serial.println(steady(voltage, 0.7));
    Serial.print("angle: "); Serial.print(setAngle); Serial.print(" degrees (set), "); Serial.print(ticks2rotdeg(desiredPositionDiff)); Serial.print(" degrees (desired), "); Serial.print(actualAngle); Serial.print(" degrees (actual), "); Serial.print(angle); Serial.println(" degrees (requested)");
    Serial.print("position: "); Serial.print(setPosition); Serial.print(" feet (set), "); Serial.print(ticks2feet(desiredPositionBase)); Serial.print(" feet (desired), "); Serial.print(actualPosition); Serial.println(" feet (actual)");
    Serial.print("difference error: "); Serial.print(posDiffError); Serial.print("p, "); Serial.print(posDiffIntError); Serial.println("i");
    Serial.print("base error: "); Serial.print(posBaseError); Serial.print("p, "); Serial.print(posBaseIntError); Serial.println("i");
    Serial.print("position difference: "); Serial.print(desiredPositionDiff); Serial.print(" ticks (desired), "); Serial.print(actualPositionDiff); Serial.println(" ticks (actual)");
    Serial.print("position base: "); Serial.print(desiredPositionBase); Serial.print(" ticks (desired), "); Serial.print(actualPositionBase); Serial.println(" ticks (actual)");
    Serial.print("velocity difference: "); Serial.print(desiredVelocityDiff); Serial.print(" ticks/s (desired), "); Serial.print(actualVelocityDiff); Serial.println(" ticks/s (actual)");
    Serial.print("velocity base: "); Serial.print(desiredVelocityBase); Serial.print(" ticks/s (desired), "); Serial.print(actualVelocityBase); Serial.println(" ticks/s (actual)");
    Serial.print("voltage difference: "); Serial.print(voltageDiff); Serial.println("V");
    Serial.print("voltage base: "); Serial.print(voltageBase); Serial.println("V");
    Serial.print("motor voltage: "); Serial.print(voltage[0]); Serial.print("V, "); Serial.print(voltage[1]); Serial.println("V");
    // motor 1:
    Serial.print("\n");
    Serial.print("MOTOR 1 at "); Serial.print(voltage[0]); Serial.print("V "); Serial.print(amperage[0]); Serial.println("A");
    Serial.print("actual position: "); Serial.print(tick0DEBUG); Serial.println(" ticks");
    // Serial.print("actual velocity: "); Serial.print(actualVelocity[0]); Serial.println(" ticks");
    // Serial.print("time: "); Serial.println((float) millis() / 1000);
    // motor 2:
    Serial.print("\n");
    Serial.print("MOTOR 2 at "); Serial.print(voltage[1]); Serial.print("V "); Serial.print(amperage[1]); Serial.println("A");
    Serial.print("actual position: "); Serial.print(tick1DEBUG); Serial.println(" ticks");
    // Serial.print("actual velocity: "); Serial.print(actualVelocity[1]); Serial.println(" ticks");
    // Serial.print("time: "); Serial.println((float) millis() / 1000);
    printtime = time;
  }
}

//////// interupts ////////
void clk1Change()
{
  //// variables ////
  static short encoder = digitalRead(CLK1); // state of encoder
  short clock; // encoder clock signal
  short data; // encoder data signal
  unsigned long time; // time of current interrupt
  static unsigned long rtime = 0; // time of last rising edge
  static unsigned long ftime = 0; // time of last falling edge
  short update = 0;

  //// update signals ////
  time = micros();
  clock = digitalRead(CLK1);
  data = digitalRead(DT1);

  //// debounce ////
  if((encoder == LOW) && (clock == 1)) // if looking for rising edge
  {
    if((time - ftime) < DTIME) return; // if time since falling edge is less than DTIMEus return
    else // rising edge found
    {
      rtime = time;
      encoder = HIGH;
      update = 1;
    }
  }
  else if((encoder == HIGH) && (clock == 0)) // if looking for falling edge
  {
    if((time - rtime) < DTIME) return; // if time since rising edge is less than DTIMEus return
    else // falling edge found
    {
      ftime = time;
      encoder = LOW;
      update = 1;
    }
  }

  //// track ticks and time ////
  if(update == 1)
  {
    // time:
    tickperiod[0] = time - ticktime[0];
    ticktime[0] = time;
    //Serial.print("tp: "); Serial.println(tickperiod1); // DEBUG
    //Serial.print("tt: "); Serial.println(ticktime1); // DEBUG
    // position:
    if(clock != data) // clock leading
    {
      direction[0] = CW;
      ticks[0]++;
    }
    else if(clock == data) // data leading
    {
      direction[0] = CCW;
      ticks[0]--;
    }
    tick = 1; // indicate a tick has occurred
  }
}
void clk2Change()
{
  //// variables ////
  static short encoder = digitalRead(CLK2); // state of encoder
  short clock; // encoder clock signal
  short data; // encoder data signal
  unsigned long time; // time of current interrupt
  static unsigned long rtime = 0; // time of last rising edge
  static unsigned long ftime = 0; // time of last falling edge
  short update = 0;

  //// update signals ////
  time = micros();
  clock = digitalRead(CLK2);
  data = digitalRead(DT2);

  //// debounce ////
  if((encoder == LOW) && (clock == 1)) // if looking for rising edge
  {
    if((time - ftime) < DTIME) return; // if time since falling edge is less than DTIMEus return
    else // rising edge found
    {
      rtime = time;
      encoder = HIGH;
      update = 1;
    }
  }
  else if((encoder == HIGH) && (clock == 0)) // if looking for falling edge
  {
    if((time - rtime) < DTIME) return; // if time since rising edge is less than DTIMEus return
    else // falling edge found
    {
      ftime = time;
      encoder = LOW;
      update = 1;
    }
  }

  //// track ticks and time ////
  if(update == 1)
  {
    // time:
    tickperiod[1] = time - ticktime[1];
    ticktime[1] = time;
    // position:
    if(clock != data) // clock leading
    {
      direction[1] = CCW;
      ticks[1]--;
    }
    else if(clock == data) // data leading
    {
      direction[1] = CW;
      ticks[1]++;
    }
    tick = 1; // indicate a tick has occurred
  }
}

//////// control ////////
void setK(float aKp, float aKi, float pKp, float pKi, float vdKp, float vbKp) // set control k values
{
  angKp = aKp;
  angKi = aKi;
  posKp = pKp;
  posKi = pKi;
  veldKp = vdKp;
  velbKp = vbKp;
}
void setThresh(float angThresh, float posThresh)
{
  angleThreshold = angThresh;
  positionThreshold = posThresh;
}
void setVolt(int voltage, short mpin, short dpin)
{
  // variables:
  short pwm;
  // set direction:
  if(voltage >= 0) digitalWrite(dpin, CW);
  else if(voltage < 0) digitalWrite(dpin, CCW);
  // set magnitude:
  pwm = 255 * (abs(voltage) / BATTVOLT);
  analogWrite(mpin, pwm);
}
void vCtrl(float desiredVelocityDiff, float desiredVelocityBase)
{
  velDiffError = desiredVelocityDiff - actualVelocityDiff;
  velBaseError = desiredVelocityBase - actualVelocityBase;
  //
  voltageDiff = veldKp * velDiffError;
  voltageBase = velbKp * velBaseError;
  voltage[0] = rail(voltageBase - voltageDiff, maxVolt);
  voltage[1] = rail(voltageBase + voltageDiff, maxVolt);
  setVolt(voltage[0], M1PWM, M1DIR);
  setVolt(voltage[1], M2PWM, M2DIR);
}
void apCtrl(float desiredPositionDiff, float desiredPositionBase)
{
  // control:
  prevPosDiffError = posDiffError;
  posDiffError = desiredPositionDiff - actualPositionDiff;
  if(abs(posDiffError) < rotdeg2ticks(10)) posDiffIntError = posDiffIntError + ((prevPosDiffError + posDiffError) / 2) * ((float)(ctrltime - prevCtrltime) / 1000000), angKp;
  else posDiffIntError = posDiffIntError / 2;
  prevPosBaseError = posBaseError;
  posBaseError = desiredPositionBase - actualPositionBase;
  if(abs(posDiffError) < feet2ticks(0.5)) posBaseIntError = posBaseIntError + ((prevPosBaseError + posBaseError) / 2) * ((float)(ctrltime - prevCtrltime) / 1000000), posKp;
  else posBaseIntError = posBaseIntError / 2;
  //
  desiredVelocityDiff = angKp * posDiffError + angKi * posDiffIntError;
  desiredVelocityBase = posKp * posBaseError + posKi * posBaseIntError;
  vCtrl(desiredVelocityDiff, desiredVelocityBase);
}
void resetCtrl()
{
  prevPosDiffError = 0; // previous position difference error (ticks)
  posDiffIntError = 0; // integral position difference error (ticks)
  prevPosBaseError = 0; // previous position base error (ticks)
  posBaseIntError = 0; // integral position base error (ticks)
  ctrltime = 0; // time of feedback control calculation (us)
  prevCtrltime = 0; // previous time of feedback control calculation (us)
}
short reached(float set, float actual, float maxDifference) // returns 1 if actual has reached set
{
  float difference = abs(set - actual);
  if(difference < maxDifference) return(1);
  else return(0);
}
short steady(float voltage[2], float threshold) // returns 1 if voltage is steady
{
  if((abs(voltage[0]) < threshold) && (abs(voltage[1]) < threshold)) return(1);
  else return(0);
}
float rail(float value, float maxValue)
{
  if(abs(value) > maxValue) // if exceed max
    if(value > 0) value = maxValue; // if positive rail positive
    else value = -maxValue; // if negative rail negative
  return(value);
}

//////// communications ////////
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
    if (msgLength == 1 && instruction[0] == 0){
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

//////// conversions ////////
float calctps(unsigned long tickperiod, unsigned long ticktime, short direction) // calculates ticks per second from tick period, tick time, and time
{
  //// variables ////
  long time = micros();
  float tps;

  //// calculate ////
  if((time - ticktime) > tickperiod) // if current tick period (incomplete) has exceeded previous tick period
    tickperiod = time - ticktime; // update tick period
  tps = 1 / ((float)tickperiod / 1000000); // 1e-6 seconds per 1 micro second
  if(direction == CCW) tps = -tps;

  //// return ////
  return(tps);
}
float ticks2rotdeg(float ticks) {return(ticks * TICK2ROT);} // converts ticks to robot rotation degrees
float rotdeg2ticks(float deg) {return(deg * ROT2TICK);} // converts robot rotation degrees to ticks
float ticks2feet(float ticks) {return(ticks * TICK2FT);} // converts ticks to feet
float feet2ticks(float feet) {return(feet * FT2TICK);} // converts feet to ticks
float ticks2rad(float ticks) {return(ticks * (PI / TPR));} // converts ticks to radians
float rad2ticks(float rad) {return(rad * (TPR / PI));} // converts radians to ticks
float rad2milim(float rad) {return(rad * (WHEELDIAMETER * PI));} // converts radians to milimeters
float milim2rad(float mm) {return(mm / (WHEELDIAMETER * PI));} // converts milimeters to radians

// authors: jack martin, jack marley, walter behaylo, holden drew
// date: 3/29/2024