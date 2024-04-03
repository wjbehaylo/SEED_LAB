//this is just me (walter) testing my angle calculation
//this file is just for CV to test code ideas before they are implemented on the arduino
volatile uint8_t instruction[4] = {66, 234, 0, 0};
// these values should ideally convert to being 117.0

union FloatUnion {
    uint8_t bytes[4];
    float floatValue;
} angle;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  angle.bytes[0] = instruction[3];
  angle.bytes[1] = instruction[2];
  angle.bytes[2] = instruction[1];
  angle.bytes[3] = instruction[0];
  Serial.println(angle.floatValue);
  return;
}
