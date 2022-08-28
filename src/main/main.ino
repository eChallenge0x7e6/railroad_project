#include <Wire.h>

#define m0_addr (0x40)
#define m1_addr (0x41)
#define m2_addr (0x42)

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(9600);
  Wire.begin();
}

void loop() {
  Wire.requestFrom(m0_addr, 1);
  while (Wire.available()){
    char c = Wire.read();
    Serial.print(c);
  }
  static unsigned char test = 0;
  if(test<0xff){
    test++;
  } else {
    test = 0;
  }
  Wire.beginTransmission(m0_addr);
  Wire.write(test);
  Wire.endTransmission();
  delay(1000);
}
