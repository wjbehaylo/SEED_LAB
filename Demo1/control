// name: position control
// purpose: control the voltage for 2 motors to reach a desired position, while tracking position and velocity

//////// includes ////////
#include <math.h>

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

//////// variables ////////
short ctrlBusy = 0; // control is completeing step
short ctrlStep = 0; // active controller step
short ctrlDone = 0; // control steps are complete
long starttime; // loop start time (ms)
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
static float maxVolt = 0;// maximum voltage
// measured:
volatile short tick = 0; // encoder has ticked
volatile short direction[2]; // encoder direction
volatile long ticks[2] = {0, 0}; // encoder ticks
volatile unsigned long ticktime[2]; // time of most recent encoder tick (us)
volatile unsigned long tickperiod[2]; // period between most recent two encoder ticks (us)
// computed:
float tps[2]; // encoder ticks per second
float tpssqr[2]; // encoder ticks per second^2

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
  //// communication ////
  Serial.begin(2000000); // set UART bits per second
  //// interrupts ////
  attachInterrupt(digitalPinToInterrupt(CLK1), clk1Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CLK2), clk2Change, CHANGE);
  //// stablise ////
  delay(1000);
  starttime = millis();
}

//////// loop ////////
void loop() 
{
  //// variables //// 
  static float voltage[2]; // motor voltage
  float amperage[2]; // motor amps
  amperage[0] = analogRead(M1FB) / 1024 / 0.56; // compute motor 1 amps
  amperage[1] = analogRead(M2FB) / 1024 / 0.56; // compute motor 2 amps

  static float desiredPositionDiff = 0; // desired position difference (ticks)
  static float desiredPositionBase = 0; // desired position base (ticks)
  static float desiredVelocityDiff; // desired velocity difference (ticks/s)
  static float desiredVelocityBase; // desired velocity base (ticks/s)
  static float voltageDiff; // voltage difference (V)
  static float voltageBase; // voltage base (V)
  //
  static float actualAngle; // actual angle (degrees)
  static float actualPosition; // actual position (feet)
  static float actualPositionDiff; // actual position difference (ticks)
  static float actualPositionBase; // actual position base (ticks)
  static float actualVelocity[2]; // actual velocity (ticks/s)
  static float actualVelocityDiff; // actual velocity difference (ticks/s)
  static float actualVelocityBase; // actual velocity base (ticks/s)

  static float posDiffError = 0; // position difference error (ticks)
  static float prevPosDiffError = 0; // previous position difference error (ticks)
  static float posDiffIntError = 0; // integral position difference error (ticks)
  static float maxPosDiffIntError = 80; // maximum integral position difference error (ticks)
  static float posBaseError = 0; // position base error (ticks)
  static float prevPosBaseError = 0; // previous position base error (ticks)
  static float posBaseIntError = 0; // integral position base error (ticks)
  static float maxPosBaseIntError = 80; // maximum integral position difference error (ticks)
  static float velDiffError; // velocity difference error (ticks/s)
  static float velBaseError; // velocity base error (ticks/s)

  static long ctrltime; // time of feedback control calculation (us)
  static long prevctrltime; // previous time of feedback control calculation (us)

  unsigned short pwm;

  long time = millis();
  static long printtime = 0; // previous print time (ms)

  //// set ////
  if(ctrlDone == 0)
  {
    if(ctrlBusy == 0) // if not controling
    {
      switch(ctrlStep) // control steps
      {
        case 0: // step 1
          maxVolt = 4;
          setk(18, 8, 180, 3, 0.0008, 0.002);
          setAngle = -135;
          break;
        case 1: // step 2
          maxVolt = 4;
          setk(180, 1.4, 5, 0.2, 0.002, 0.0008);
          setPosition = 2.86;
          break;
        default:
          ctrlDone = 1;
          break;
      }
      setPositionDiff = rotdeg2ticks(setAngle);
      setPositionBase = feet2ticks(setPosition);
      ctrlBusy = 1;
    }
    else if(ctrlBusy == 1) // if controling
    {
      //Serial.println(ctrltime - prevctrltime); // DEBUG
      //Serial.println(reached(setAngle, actualAngle, 1)); // DEBUG
      //Serial.println(reached(setPosition, actualPosition, 0.05)); // DEBUG
      //Serial.println(voltage[0]); // DEBUG
      if(reached(setAngle, actualAngle, 0.1) && reached(setPosition, actualPosition, 0.05) && steady(voltage, 0.7)) // if control has stopped
      {
        ctrlStep++;
        ctrlBusy = 0;
      }
    }
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

  //// motor feeback ////
  prevctrltime = ctrltime;
  ctrltime = micros();
  // track:
  int tick0DEBUG = ticks[0];
  int tick1DEBUG = ticks[1];
  actualPositionDiff = (ticks[1] - ticks[0]) / 2;
  actualPositionBase = ticks[1] - actualPositionDiff;
  actualVelocity[0] = calctps(tickperiod[0], ticktime[0], direction[0]);
  actualVelocity[1] = calctps(tickperiod[1], ticktime[1], direction[1]);
  actualVelocityDiff = (actualVelocity[1] - actualVelocity[0]) / 2;
  actualVelocityBase = actualVelocity[1] - actualVelocityDiff;
  // control:
  prevPosDiffError = posDiffError;
  posDiffError = desiredPositionDiff - actualPositionDiff;
  /*if(abs(posDiffError) < rotdeg2ticks(10))*/ posDiffIntError = posDiffIntError + ((prevPosDiffError + posDiffError) / 2) * ((float)(ctrltime - prevctrltime) / 1000000), angKp;
  //else posDiffIntError = posDiffIntError / 2;
  prevPosBaseError = posBaseError;
  posBaseError = desiredPositionBase - actualPositionBase;
  /*if(abs(posDiffError) < feet2ticks(0.5))*/ posBaseIntError = posBaseIntError + ((prevPosBaseError + posBaseError) / 2) * ((float)(ctrltime - prevctrltime) / 1000000), posKp;
  //else posBaseIntError = posBaseIntError / 2;
  //
  desiredVelocityDiff = angKp * posDiffError + angKi * posDiffIntError;
  velDiffError = desiredVelocityDiff - actualVelocityDiff;
  desiredVelocityBase = posKp * posBaseError + posKi * posBaseIntError;
  velBaseError = desiredVelocityBase - actualVelocityBase;
  //
  voltageDiff = veldKp * velDiffError;
  voltageBase = velbKp * velBaseError;
  voltage[0] = rail(voltageBase - voltageDiff, maxVolt);
  voltage[1] = rail(voltageBase + voltageDiff, maxVolt);
  // set direction:
  if(voltage[0] >= 0) digitalWrite(M1DIR, CW);
  else if(voltage[0] < 0) digitalWrite(M1DIR, CCW);
  if(voltage[1] >= 0) digitalWrite(M2DIR, CW);
  else if(voltage[1] < 0) digitalWrite(M2DIR, CCW);
  // set magnitude:
  pwm = 255 * (abs(voltage[0]) / BATTVOLT);
  analogWrite(M1PWM, pwm);
  pwm = 255 * (abs(voltage[1]) / BATTVOLT);
  analogWrite(M2PWM, pwm);

  //// output ////
  actualAngle = ticks2rotdeg(actualPositionDiff);
  actualPosition = ticks2feet(actualPositionBase);

  //// display ////
  if(DEBUG && ((time - printtime) > 1600)) // if it has been 1600ms since last print
  {
    Serial.print("\n\n");
    Serial.println("ROBOT");
    Serial.print("control step: "); Serial.print(ctrlStep); Serial.print(" "); Serial.print(reached(setAngle, actualAngle, 1)); Serial.print(" "); Serial.print(reached(setPosition, actualPosition, 0.05)); Serial.print(" "); Serial.println(steady(voltage, 0.7));
    Serial.print("angle: "); Serial.print(setAngle); Serial.print(" degrees (set), "); Serial.print(ticks2rotdeg(desiredPositionDiff)); Serial.print(" degrees (desired), "); Serial.print(actualAngle); Serial.println(" degrees (actual)");
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
void setk(float aKp, float aKi, float pKp, float pKi, float vdKp, float vbKp) // set control k values
{
  angKp = aKp;
  angKi = aKi;
  posKp = pKp;
  posKi = pKi;
  veldKp = vdKp;
  velbKp = vbKp;
}
short reached(float set, float actual, float maxDifference) // if actual has reached set
{
  float difference = abs(set - actual);
  if(difference < maxDifference) return(1);
  else return(0);
}
short steady(float voltage[2], float threshold)
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

// author: jack martin, jack marley
// date: 1/30/2024
