#include <Wire.h>

#define address (0x40)
#define relay 8
#define on HIGH
#define off LOW
#define sen_a A3 //10bit
#define sen_b A2 //10bit

//Status Register bit map
#define EXIST_ON_RAIL (0)

//Command Register bit map
#define SAFE_SIG (0)

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(relay, OUTPUT);
  //pinMode(sen_a,INPUT_PULLUP);
  //pinMode(sen_b,INPUT_PULLUP);
  Serial.begin(9600);
  Wire.begin(address);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
}

unsigned char cmd_regist = 0;
unsigned char sts_regist = 0;
unsigned int val_sen_a = 0;
unsigned int val_sen_b = 0;
void loop() {
  static int on_rail_num = 0;
  val_sen_a = analogRead(sen_a);
  val_sen_b = analogRead(sen_b);
  if(val_sen_a<100){
    on_rail_num++;
  }else if(val_sen_a>300){
    on_rail_num--;
  }
  if(on_rail_num==0){
    sts_regist &= ~(1<<EXIST_ON_RAIL);
  } else {
    sts_regist |= 1<<EXIST_ON_RAIL;
  }
}

void requestEvent(){
  Wire.write(sts_regist);
  Serial.print("Get Request\n");
}

void receiveEvent(int num){
  while (0 < Wire.available()){
    cmd_regist = Wire.read();
    Serial.print(cmd_regist);
  }
  if(cmd_regist&(1<<SAFE_SIG)){
    digitalWrite(relay, on);
  } else {
    digitalWrite(relay, off);
  }
}
