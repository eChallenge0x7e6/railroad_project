//ライブラリの宣言
#include <Wire.h>       //I2Cライブラリ
#include <MsTimer2.h>   //タイマーライブラリ

#define DEBUG (1)

//マクロの定義
#ifdef DEBUG
  #define Debug(s) Serial.print(s)
#else
  #define Debug(s) 
#endif
#define CW      (1)         //時計回り
#define CCW     (-1)        //反時計回り
#define ADDRESS (0x43)      //I2C用モジュールアドレス
#define RELAY   (8)         //リレー出力ピン番号
#define ON      (HIGH)      //リレー用トランジスタのアクティブ極性
#define OFF     (LOW)       //リレー用トランジスタのネガティブ極性
#define SENS_A  (A3)        //センサA入力ピン番号
#define SENS_B  (A2)        //センサB入力ピン番号
#define ACT_THR (100)       //センサの閾値（アクティブ）
#define NEG_THR (500)       //センサの閾値（ネガティブ）
#define SENS_PERIOD   (10)  //列車検出の閾時間(msec)
#define TMR_PERIOD    (3000)//コマンドの停車ディレイタイム

//グローバル変数の宣言
union{  //I2C送信用データ変数
  uint8_t data;
  struct{
    uint8_t reserved : 4;
    uint8_t on_rail : 3;
    uint8_t output : 1;
  }sts;
}tx;
union{  //I2C受信用データ変数
  uint8_t data;
  struct{
    char reserved : 3;
    char code : 4;
    char directry : 1;
  }cmd;
}rx;
bool cmd_flg = false;     //コマンド処理の要求フラグ
int dir_rail = CW;        //列車進行方向変数

/**
 * 発車関数
 */
void rail_on(void){
  tx.sts.output = ON;
  Debug("発車しました。");
}

/**
 * 停車関数
 */
void rail_off(void){
  tx.sts.output = OFF;
  Debug("停車しました。");
}

/**
 * メインCPUからの指令処理関数
 */
void cmd_process(int on_rail){
  static int reg_num = 0;
  static bool init = true;
  if(cmd_flg && init){
    reg_num = on_rail;
    init = false;
  } else if(cmd_flg && on_rail!=reg_num){
    switch(rx.cmd.code){
      case 0x3:
        if(on_rail>reg_num){//進入時
          rail_off();
          cmd_flg = false;
        }//else Nothing to do
        break;
      case 0x4:
        if(on_rail>reg_num){//進入時
          rail_off();
          MsTimer2::set(TMR_PERIOD, rail_on);
          MsTimer2::start();
          cmd_flg = false;
        }//else Nothing to do
        break;
      case 0x5:
        if(on_rail<reg_num){//通過時
          rail_off();
          cmd_flg = false;
        }//else Nothing to do
        break;
      case 0x6:
        if(on_rail<reg_num){//通過時
          rail_off();
          MsTimer2::set(TMR_PERIOD, rail_on);
          MsTimer2::start();
          cmd_flg = false;
        }//else Nothing to do
        break;
      default://エラー
        break;
    }
  } else if(!cmd_flg && !init){
    init = true;
  }//else Nothing to do
}

/**
 * センサ読み取り処理
 * @return レール上の列車の台数
 */
uint8_t senser_process(void){
  static bool detect_a = false;    //列車検出フラグA
  static bool detect_b = false;    //列車検出フラグB
  static long timestamp_a = 0;    //センサAの検出タイミング
  static long timestamp_b = 0;    //センサBの検出タイミング
  static int on_rail_num = 0;   //区間内の列車数

  int r_sens_a = analogRead(SENS_A);//センサ読み取り
  int r_sens_b = analogRead(SENS_B);//センサ読み取り

  //センサAの処理
  if(!detect_a && r_sens_a<ACT_THR){      //列車の検出
    detect_a = true;                      //検出フラグA ON
    timestamp_a = millis();                 //検出時間記録
  }else if(detect_a && r_sens_a>NEG_THR){ //列車の通過
    detect_a = false;                     //検出フラグA OFF
    if(millis()-timestamp_a > SENS_PERIOD){ //列車の通過時間確認
      on_rail_num += dir_rail;            //区間内の列車数の算出
    }//else Nothing to do
  }//else Nothing to do

  //センサBの処理
  if(!detect_b && r_sens_b<ACT_THR){      //列車の検出
    detect_b = true;                      //検出フラグB ON
    timestamp_b = millis();                 //検出時間記録
  }else if(detect_b && r_sens_b>NEG_THR){ //列車の通過
    detect_b = false;                     //検出フラグB OFF
    if(millis()-timestamp_b > SENS_PERIOD){ //列車の通過時間確認
      on_rail_num -= dir_rail;            //区間内の列車数の算出
    }//else Nothing to do
  }//else Nothing to do

  return abs(on_rail_num);
}

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
  //区間状態の更新
  tx.sts.on_rail = senser_process();

  //指令処理
  cmd_process(tx.sts.on_rail);

  //レールに出力
  digitalWrite(RELAY, tx.sts.output);
}

/**
 * I2C送信関数
 * リクエストに応え、区間状態を送信する。
 */
void RequestEvent(){
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
  if(rx.cmd.directry==0)  dir_rail = 1;//進行方向の設定
  else                    dir_rail = -1;
  switch(rx.cmd.code){
    case 0x0://Wait
      break;
    case 0x1://Go
      rail_on();
      break;
    case 0x2://Stop
      rail_off();
      break;
    case 0x3://Stop when increase
    case 0x4://Stop 3s when increase
    case 0x5://Stop when decrease
    case 0x6://Stop 3s when decrease
      cmd_flg = true;
      break;
    case 0x7://Clear code
      cmd_flg = false;
      break;
    default:
      break;
  }
}
