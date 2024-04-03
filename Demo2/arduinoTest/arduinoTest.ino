//this is just me (walter) testing my angle calculation
//this file is just for CV to test code ideas before they are implemented on the arduino
volatile uint8_t instruction[4] = {66, 234, 0, 0};
// these values should ideally convert to being 117.0

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  float angle = (instruction[0]<<8 | instruction[1]);
  Serial.println(angle);
  end
}
