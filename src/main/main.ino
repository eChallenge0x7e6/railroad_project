//ライブラリの宣言
#include <Wire.h>

//マクロの定義
#define m0_addr (0x40)
#define m1_addr (0x41)
#define m2_addr (0x42)
#define m3_addr (0x43)

//グローバル変数の宣言
union{
  uint8_t data;
  struct{
    uint8_t reserved : 4;
    uint8_t on_rail : 3;
    uint8_t output : 1;
  }rail_sts;
}com;//I2C受信用データ変数
union{
  uint8_t data;
  struct{
    char reserved : 6;
    char directry : 1;
    char output : 1;
  }cmd;
}tx;//I2C送信用データ変数

//プロトタイプ宣言
bool check(void);
void i2c_m0_tx(uint8_t data); //モジュールm0指令用関数
void i2c_m1_tx(uint8_t data); //モジュールm1指令用関数
void i2c_m2_tx(uint8_t data); //モジュールm2指令用関数
void i2c_m3_tx(uint8_t data); //モジュールm3指令用関数

/**
 * 初期化関数
 * 起動後、最初に一度だけ処理される。
 */
void setup() {
  Serial.begin(9600); //デバッグ用シリアル出力設定

  Wire.begin();       //I2C初期設定

  //各変数の初期化
  com.data = 0;
  tx.data = 0;

  //路線状態の確認
  check();

  //運行開始
  tx.cmd.output = 1;  //発車指令
  i2c_m0_tx(tx.data); //指令送信
}

/**
 * ループ関数
 * 初期化関数の後、常に繰り返し動作する。
 */
void loop() {
  //区間0の状態確認
  Wire.requestFrom(m0_addr, 1);   //区間状態を受け取る
  while (Wire.available()){       //受信データ数の確認
    com.data = Wire.read();       //データ読み取り
    Serial.print(com.data);
  }
  //区間0へ指令
  if(com.rail_sts.on_rail!=0){
    tx.cmd.output = 0;
  } else {
    tx.cmd.output = 1;
  }
  i2c_m0_tx(tx.data);   //指令送信
  
  delay(100);           //待機時間(msec)
}

bool check(void){
  //線路内の列車の有無を確認
  //残列車の誘導
  return true;
}

void i2c_m0_tx(uint8_t data){
  Wire.beginTransmission(m0_addr);  //送信アドレス設定
  Wire.write(data);                 //送信データ
  Wire.endTransmission();           //送信終了
}

void i2c_m1_tx(uint8_t data){
  Wire.beginTransmission(m1_addr);  //送信アドレス設定
  Wire.write(data);                 //送信データ
  Wire.endTransmission();           //送信終了
}

void i2c_m2_tx(uint8_t data){
  Wire.beginTransmission(m2_addr);  //送信アドレス設定
  Wire.write(data);                 //送信データ
  Wire.endTransmission();           //送信終了
}

void i2c_m3_tx(uint8_t data){
  Wire.beginTransmission(m3_addr);  //送信アドレス設定
  Wire.write(data);                 //送信データ
  Wire.endTransmission();           //送信終了
}