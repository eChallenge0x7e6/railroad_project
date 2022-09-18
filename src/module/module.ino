//ライブラリの宣言
#include <Wire.h>       //I2Cライブラリ
#include <MsTimer2.h>   //タイマーライブラリ

//マクロの定義
#define ADDRESS (0x41)      //I2C用モジュールアドレス
#define RELAY   (8)         //リレー出力ピン番号
#define ON      (HIGH)      //リレー用トランジスタのアクティブ極性
#define OFF     (LOW)       //リレー用トランジスタのネガティブ極性
#define SENS_A  (A3)        //センサA入力ピン番号
#define SENS_B  (A2)        //センサB入力ピン番号
#define ACT_THR (100)       //センサの閾値（アクティブ）
#define NEG_THR (500)       //センサの閾値（ネガティブ）
#define SENS_PERIOD   (10)  //列車検出の閾時間(msec)

//グローバル変数の宣言
union{
  uint8_t data;
  struct{
    uint8_t reserved : 4;
    uint8_t on_rail : 3;
    uint8_t output : 1;
  }rail_sts;
}com;                     //I2C送信用データ変数
union{
  uint8_t data;
  struct{
    char reserved : 6;
    char directry : 1;
    char output : 1;
  }cmd;
}rcv;                     //I2C受信用データ変数
int dir_flow = 1;         //列車進行方向変数　右回り：1, 左回り：-1
unsigned int r_sens_a = 0;//センサAの列車検出値
unsigned int r_sens_b = 0;//センサBの列車検出値
bool detect_a = false;    //列車検出フラグA
bool detect_b = false;    //列車検出フラグB

/**
 * 初期化関数
 * 起動後、最初に一度だけ処理される。
 */
void setup() {
  pinMode(RELAY, OUTPUT);       //リレー出力ピン設定

  Serial.begin(9600);           //デバッグ用シリアル出力設定

  Wire.begin(ADDRESS);          //I2C設定
  Wire.onRequest(RequestEvent); //I2Cリクエスト関数設定
  Wire.onReceive(ReceiveEvent); //I2C受信関数設定

  //各変数の初期化
  com.data = 0;
  rcv.data = 0;
  com.rail_sts.on_rail = 0;
  com.rail_sts.output = ON;
}

/**
 * ループ関数
 * 初期化関数の後、常に繰り返し動作する。
 */
void loop() {
  //loop関数内のローカル変数の宣言
  static long on_sens_a = 0;    //センサAの検出タイミング
  static long on_sens_b = 0;    //センサBの検出タイミング
  static int on_rail_num = 0;   //区間内の列車数

  r_sens_a = analogRead(SENS_A);//センサ読み取り
  r_sens_b = analogRead(SENS_B);//センサ読み取り

  //センサAの処理
  if(!detect_a && r_sens_a<ACT_THR){      //列車の検出
    detect_a = true;                      //検出フラグA ON
    on_sens_a = millis();                 //検出時間記録
  }else if(detect_a && r_sens_a>NEG_THR){ //列車の通過
    detect_a = false;                     //検出フラグA OFF
    if(millis()-on_sens_a > SENS_PERIOD){ //列車の通過時間確認
      on_rail_num += dir_flow;            //区間内の列車数の算出
    }//else Nothing to do
  }//else Nothing to do

  //センサBの処理
  if(!detect_b && r_sens_b<ACT_THR){      //列車の検出
    detect_b = true;                      //検出フラグB ON
    on_sens_b = millis();                 //検出時間記録
  }else if(detect_b && r_sens_b>NEG_THR){ //列車の通過
    detect_b = false;                     //検出フラグB OFF
    if(millis()-on_sens_b > SENS_PERIOD){ //列車の通過時間確認
      on_rail_num -= dir_flow;            //区間内の列車数の算出
    }//else Nothing to do
  }//else Nothing to do

  //区間状態の更新
  com.rail_sts.on_rail = abs(on_rail_num);
}

/**
 * I2C送信関数
 * リクエストに応え、区間状態を送信する。
 */
void RequestEvent(){
  Wire.write(com.data);//データ送信
  Serial.print("Sent data : 0x%02x\n", com.data);
}

/**
 * I2C受信関数
 * 受け取った命令を実行する。
 */
void ReceiveEvent(int num){
  while(Wire.available() > 0){//受信データ数の確認
    rcv.data = Wire.read();//データ受信
    Serial.print("Get data : 0x%02x", rcv.data);
  }
  if(rcv.cmd.output == 1){//受信データの読み取り
    com.rail_sts.output = ON;
  } else{
    com.rail_sts.output = OFF;
  }
  digitalWrite(RELAY, rcv.cmd.output);//レールに出力
  if(rcv.cmd.directry == 0){//進行方向の設定
    dir_flow = 1;
  } else{
    dir_flow = -1;
  }
}