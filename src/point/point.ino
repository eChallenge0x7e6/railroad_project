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
enum mode_t{                //モード一覧
  NORMAL = 0,
  DEBUG_1,
}mode;
union{                      //I2C送信用データ変数
  uint8_t data;
  struct{
    uint8_t reserved : 7;
    uint8_t work : 1;       //ポイント動作状態 0:主線, 1:副線
  }sts;
  struct{
    uint8_t reserved : 2;
    uint8_t sens_b : 1;     //センサA検出フラグ 0:未検出, 1:検出
    uint8_t sens_a : 1;     //センサB検出フラグ 0:未検出, 1:検出
    uint8_t reserved_0 : 3;
    uint8_t work : 1;       //ポイント動作状態 0:主線, 1:副線
  }dbg;
}tx;
union{                      //I2C受信用データ変数
  uint8_t data;
  struct{
    char reserved : 3;
    char code : 4;          //指令コード
    char reserved_0 : 1;
  }cmd;
}rx;

bool detect_a = false;      //列車検出フラグA
bool detect_b = false;      //列車検出フラグB
int cmd_step = 0;           //コマンド処理の進捗カウンタ

/**
 * ポイント切替関数
 */
void point_switch(void){
  static bool switch_on = OFF;              //ポイント切替状態
  switch_on = !switch_on;                   //ポイントを切替
  digitalWrite(RELAY, switch_on);           //出力
  if(switch_on==OFF)  tx.sts.work = 0;
  else                tx.sts.work = 1;
  Debug("ポイントを切替えました。");
}

/**
 * メインCPUからの指令処理関数
 */
void cmd_process(void){
  if(detect_a && detect_b && cmd_step==2){  //副線通過確認
    cmd_step=0;                             //指令処理終了
  } else if(detect_a && cmd_step==1){       //主線列車検出
    point_switch();                         //ポイント切替
    cmd_step++;                             //指令処理シーケンスを進める
  }//else Nothing to do
}

/**
 * センサ読み取り処理
 * @return レール上の列車の検出
 */
void senser_process(void){
  static long timestamp_a = 0;              //センサAの検出タイミング
  static long timestamp_b = 0;              //センサBの検出タイミング

  int r_sens_a = analogRead(SENS_A);        //センサ読み取り
  int r_sens_b = analogRead(SENS_B);        //センサ読み取り

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
}

/**
 * リセット関数：プログラムを最初から始める。
 */
void (*reset_func) (void) = 0;              //アドレスを0にしてプログラムを最初から始める。

/**
 * 初期化関数
 * 起動後、最初に一度だけ処理される。
 */
void setup() {
  pinMode(RELAY, OUTPUT);                   //リレー出力ピン設定

  Serial.begin(9600);                       //デバッグ用シリアル出力設定

  Wire.begin(ADDRESS);                      //I2C設定
  Wire.onRequest(RequestEvent);             //I2Cリクエスト関数設定
  Wire.onReceive(ReceiveEvent);             //I2C受信関数設定

  //各変数の初期化
  tx.data = 0;
  rx.data = 0;
}

/**
 * ループ関数
 * 初期化関数の後、常に繰り返し動作する。
 */
void loop() {
  senser_process();                         //区間状態の更新
  if(cmd_step!=0){
    cmd_process();                          //指令処理
  }//else Nothing to do
}

/**
 * I2C送信関数
 * リクエストに応え、区間状態を送信する。
 */
void RequestEvent(){
  if(mode==DEBUG_1){
    tx.dbg.sens_a = detect_a;
    tx.dbg.sens_b = detect_b;
  }
  Wire.write(tx.data);                      //データ送信
  Debug("\"要求を確認しました。 : 0x%02xn\", tx.data");
}

/**
 * I2C受信関数
 * 受け取った命令を実行する。
 */
void ReceiveEvent(int num){
  while(Wire.available() > 0){              //受信データ数の確認
    rx.data = Wire.read();                  //データ受信
    Debug("\"指令を受信しました。 : 0x%02x\", rx.data");
  }
  switch(rx.cmd.code){
    case 0x0:                               //待機
      break;
    case 0x1:                               //切替
      point_switch();
      break;
    case 0x2:                               //センサA検出時、切替
      cmd_step = 1;
      break;
    case 0x7:                               //指令解除
      cmd_step = 0;
      mode = NORMAL;
      break;
    case 0xd:                               //デバッグモード
      mode = DEBUG_1;
      break;
    case 0xf:                               //リセット
      reset_func();
      break;
    default:
      break;
  }
}
