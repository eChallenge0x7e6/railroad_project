//ライブラリの宣言
#include <Wire.h>       //I2Cライブラリ

#define DEBUG (1)

//マクロの定義
#ifdef DEBUG
  #define Debug(s) Serial.print(s)
#else
  #define Debug(s) 
#endif
#define ADDRESS (0x60)      //I2C用アドレス
#define RELAY   (8)         //リレー出力ピン番号
#define ON      (HIGH)      //リレー用トランジスタのアクティブ極性
#define OFF     (LOW)       //リレー用トランジスタのネガティブ極性
#define SENS_A  (A3)        //センサA入力ピン番号
#define SENS_B  (A2)        //センサB入力ピン番号
#define ACT_THR (100)       //センサの閾値（アクティブ）
#define NEG_THR (500)       //センサの閾値（ネガティブ）
#define SENS_PERIOD   (10)  //列車検出の閾時間(msec)

//グローバル変数の宣言
union{  //I2C送信用データ変数
  uint8_t data;
  struct{
    uint8_t reserved : 5;
    uint8_t sens_b : 1;
    uint8_t sens_a : 1;
    uint8_t work : 1;
  }sts;
}tx;
union{  //I2C受信用データ変数
  uint8_t data;
  struct{
    char reserved : 3;
    char code : 4;
    char reserved_0 : 1;
  }cmd;
}rx;

int cmd_step = 0;     //コマンド処理の進捗カウンタ

/**
 * ポイント切替関数
 */
void point_switch(void){
  static bool switch_on = OFF;    //ポイント出力
  switch_on ^= switch_on;         //出力を反転
  digitalWrite(RELAY, switch_on); //レールに出力
  Debug("ポイントを切替えました。");
}

/**
 * メインCPUからの指令処理関数
 */
void cmd_process(void){
  if(tx.sts.sens_a && tx.sts.sens_b && cmd_step==2){
    cmd_step++;
  } else if(tx.sts.sens_a && cmd_step==1){
    point_switch();
    cmd_step++;
  }//else Nothing to do
}

/**
 * センサ読み取り処理
 * @return レール上の列車の検出
 */
void senser_process(void){
  static bool detect_a = false;    //列車検出フラグA
  static bool detect_b = false;    //列車検出フラグB
  static long timestamp_a = 0;    //センサAの検出タイミング
  static long timestamp_b = 0;    //センサBの検出タイミング

  int r_sens_a = analogRead(SENS_A);//センサ読み取り
  int r_sens_b = analogRead(SENS_B);//センサ読み取り

  //センサAの処理
  if(!detect_a && r_sens_a<ACT_THR){        //列車の検出
    detect_a = true;                        //検出フラグA ON
    timestamp_a = millis();                 //検出時間記録
  }else if(detect_a && r_sens_a>NEG_THR){   //列車の通過
    detect_a = false;                       //検出フラグA OFF
  }//else Nothing to do

  //センサBの処理
  if(!detect_b && r_sens_b<ACT_THR){        //列車の検出
    detect_b = true;                        //検出フラグB ON
    timestamp_b = millis();                 //検出時間記録
  }else if(detect_b && r_sens_b>NEG_THR){   //列車の通過
    detect_b = false;                       //検出フラグB OFF
  }//else Nothing to do

  tx.sts.sens_a |= detect_a;
  tx.sts.sens_b |= detect_b;
}

/**
 * リセット関数：プログラムを最初から始める。
 */
void (*reset_func) (void) = 0;  //アドレスを0にしてプログラムを最初から始める。

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
  tx.data = 0;
  rx.data = 0;
}

/**
 * ループ関数
 * 初期化関数の後、常に繰り返し動作する。
 */
void loop() {
  senser_process();   //区間状態の更新
  if(cmd_step!=0){
    cmd_process();      //指令処理
  }//else Nothing to do
}

/**
 * I2C送信関数
 * リクエストに応え、区間状態を送信する。
 */
void RequestEvent(){
  switch(cmd_step){
    case 0:
      tx.sts.work = 0;
      break;
    case 1:
      tx.sts.work = 1;
      break;
    case 2:
      tx.sts.work = 0;
      break;
    default:
      break;
  }
  Wire.write(tx.data);//データ送信
  Debug("\"要求を確認しました。 : 0x%02xn\", tx.data");
}

/**
 * I2C受信関数
 * 受け取った命令を実行する。
 */
void ReceiveEvent(int num){
  while(Wire.available() > 0){//受信データ数の確認
    rx.data = Wire.read();//データ受信
    Debug("\"指令を受信しました。 : 0x%02x\", rx.data");
  }
  switch(rx.cmd.code){
    case 0x0://Wait
      break;
    case 0x1://Switch
      point_switch();
      break;
    case 0x2://Switch when detected
      cmd_step = 1;
      break;
    case 0xf:
      reset_func();
      break;
    default:
      break;
  }
}
