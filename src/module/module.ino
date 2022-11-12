//ライブラリの宣言
#include <Wire.h>       //I2Cライブラリ
#include <MsTimer2.h>   //タイマーライブラリ
#include <EEPROM.h>

#define DEBUG (1)

//マクロの定義
#ifdef DEBUG
  #define Debug(s) Serial.print(s)
#else
  #define Debug(s) 
#endif
#define CW      (1)         //時計回り
#define CCW     (-1)        //反時計回り
#define ADDRESS (0x42)      //I2C用モジュールアドレス
#define RELAY   (8)         //リレー出力ピン番号
#define ON      (HIGH)      //リレー用トランジスタのアクティブ極性
#define OFF     (LOW)       //リレー用トランジスタのネガティブ極性
#define SENS_A  (A3)        //センサA入力ピン番号
#define SENS_B  (A2)        //センサB入力ピン番号

//グローバル変数の宣言
union{
  uint8_t data[6] = {0};
  struct{
    byte en;
    byte act;
    byte neg;
    byte sens;
    byte mask;
    byte det;
    byte wdt;
  }prm;
}eeprom;
enum mode_t{                //モード一覧
  NORMAL = 0,
  DEBUG_1,
}mode;
union{                      //I2C送信用データ変数
  uint8_t data;
  struct{
    uint8_t reserved : 4;
    uint8_t on_rail : 3;    //列車の台数
    uint8_t output : 1;     //レール出力
  }sts;
  struct{
    uint8_t reserved : 2;
    uint8_t detect_a : 1;   //センサA検出フラグ
    uint8_t detect_b : 1;   //センサB検出フラグ
    uint8_t on_rail : 3;    //列車の台数
    uint8_t output : 1;     //レール出力
  }dbg;
}tx;
union{                      //I2C受信用データ変数
  uint8_t data;
  struct{
    uint8_t mlt_flg : 1;    //連続受信フラグ
    uint8_t reserved : 2;   //未使用
    uint8_t code : 4;       //指令コード
    uint8_t directry : 1;   //進行方向 0:時計回り, 1:反時計回り
  }cmd;
}rx;
int ACT_THR = 100;
int NEG_THR = 400;
int SENS_PERIOD = 3;
int MASK_PERIOD = 500;
int TMR_PERIOD  = 3000;
unsigned long WDT_PERIOD = 15000; //区間通過最大時間(ミリ秒)
bool cmd_flg = false;       //指令処理の要求フラグ
int dir_rail = CW;          //列車進行方向変数
bool power = false;         //レール出力

/**
 * 発車関数
 */
void rail_on(void){
  power = ON;
  //Debug("発車しました。");
}

/**
 * 停車関数
 */
void rail_off(void){
  power = OFF;
  //Debug("停車しました。");
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
        if(on_rail>reg_num){                //進入時
          rail_off();
          cmd_flg = false;
        }//else Nothing to do
        break;
      case 0x4:
        if(on_rail>reg_num){                //進入時
          rail_off();
          MsTimer2::set(TMR_PERIOD, rail_on);
          MsTimer2::start();
          cmd_flg = false;
        }//else Nothing to do
        break;
      case 0x5:
        if(on_rail<reg_num){                //通過時
          rail_off();
          cmd_flg = false;
        }//else Nothing to do
        break;
      case 0x6:
        if(on_rail<reg_num){                //通過時
          rail_off();
          MsTimer2::set(TMR_PERIOD, rail_on);
          MsTimer2::start();
          cmd_flg = false;
        }//else Nothing to do
        break;
      case 0x8:
        if(on_rail>reg_num){                //進入時
          MsTimer2::set(TMR_PERIOD, rail_off);
          MsTimer2::start();
          cmd_flg = false;
        }//else Nothing to do
        break;
      case 0x9:
        if(on_rail<reg_num){                //通過時
          MsTimer2::set(TMR_PERIOD, rail_off);
          MsTimer2::start();
          cmd_flg = false;
        }//else Nothing to do
        break;
      default:                              //エラー
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
  static bool detect_a = false;             //列車検出フラグA
  static bool detect_b = false;             //列車検出フラグB
  static unsigned long timestamp_a = 0;     //センサAの検出タイミング
  static unsigned long timestamp_b = 0;     //センサBの検出タイミング
  static int on_rail_num = 0;               //区間内の列車数
  static bool mask_a = false;
  static bool mask_b = false;
  static unsigned long wdt = 0;
  int r_sens_a = analogRead(SENS_A);
  int r_sens_b = analogRead(SENS_B);

  /*
    static int old_sens_a = analogRead(SENS_A);
    static int old_sens_b = analogRead(SENS_B);
    int gap_a = analogRead(SENS_A)-old_sens_a;        //センサ読み取り
    int gap_b = analogRead(SENS_B)-old_sens_b;        //センサ読み取り
    old_sens_a += gap_a;
    old_sens_b += gap_b;
    Debug("ギャップ:");Debug(gap_a);
    Debug(", レベル:");Debug(old_sens_a);Debug("\n");
    //センサAの処理
    if(gap_a>NEG_THR && detect_a){            //列車の通過
      detect_a = false;                       //検出フラグA OFF
      if(SENS_PERIOD < millis()-timestamp_a){ //列車の通過時間確認
        on_rail_num += dir_rail;              //区間内の列車数の算出
        mask_a = true;                        //不感フラグ ON
        Debug("A通過時間:");Debug(millis()-timestamp_a);Debug("ミリ秒\n");
      }//else Nothing to do
    } else if(gap_a<ACT_THR && !detect_a){    //列車の検出
      detect_a = true;                        //検出フラグA ON
      timestamp_a = millis();                 //検出時間記録
    } else if((detect_a|mask_a) && MASK_PERIOD < millis()-timestamp_a){  //タイムオーバー
      detect_a = false;                       //検出フラグA OFF
      mask_a = false;                         //不感フラグ OFF
    }//else Nothing to do
    //センサBの処理
    if(gap_b>NEG_THR && detect_b){            //列車の通過
      detect_b = false;                       //検出フラグB OFF
      if(SENS_PERIOD < millis()-timestamp_b){ //列車の通過時間確認
        on_rail_num -= dir_rail;              //区間内の列車数の算出
        mask_b = true;                        //不感フラグ ON
        Debug("B通過時間:");Debug(millis()-timestamp_b);Debug("ミリ秒\n");
      }
    } else if(gap_b<ACT_THR && !detect_b){    //列車の検出
      detect_b = true;                        //検出フラグB ON
      timestamp_b = millis();                 //検出時間記録
    } else if((detect_b|mask_b) && MASK_PERIOD < millis()-timestamp_b){  //タイムオーバー
      detect_b = false;                       //検出フラグB OFF
      mask_b = false;                         //不感フラグ OFF
    }//else Nothing to do
  */
  
  if(mask_a){                               //不感処理
    if(millis()-timestamp_a > MASK_PERIOD) mask_a = false;  //解除
    //else Nothing to do
  } else if(!detect_a && r_sens_a<ACT_THR){ //列車の検出
    detect_a = true;                        //検出フラグA ON
    timestamp_a = millis();                 //検出時間記録
    Debug("A:");Debug(r_sens_a);Debug("\n");
  } else if(detect_a && r_sens_a>NEG_THR){  //列車の通過
    detect_a = false;                       //検出フラグA OFF
    long pass_time = millis()-timestamp_a;
    if(pass_time > SENS_PERIOD && pass_time < MASK_PERIOD){            //列車の通過時間確認
      on_rail_num += dir_rail;              //区間内の列車数の算出
      mask_a = true;
      Debug("A:");Debug(r_sens_a);Debug("\n");
      Debug("A通過時間:");Debug(pass_time);Debug("ミリ秒\n");
    }//else Nothing to do
  }//else Nothing to do

  //センサBの処理
  if(mask_b){                               //不感処理
    if(millis()-timestamp_b > MASK_PERIOD) mask_b = false;  //解除
    //else Nothing to do
  } else if(!detect_b && r_sens_b<ACT_THR){ //列車の検出
    detect_b = true;                        //検出フラグB ON
    timestamp_b = millis();                 //検出時間記録
    Debug("B:");Debug(r_sens_b);Debug("\n");
  } else if(detect_b && r_sens_b>NEG_THR){  //列車の通過
    detect_b = false;                       //検出フラグB OFF
    long pass_time = millis()-timestamp_b;
    if(pass_time > SENS_PERIOD && pass_time < MASK_PERIOD){            //列車の通過時間確認
      on_rail_num -= dir_rail;              //区間内の列車数の算出
      mask_b = true;
      Debug("B:");Debug(r_sens_b);Debug("\n");
      Debug("B通過時間:");Debug(pass_time);Debug("ミリ秒\n");
    }//else Nothing to do
  }//else Nothing to do
  
  if(mode==DEBUG_1){
    tx.dbg.detect_a |= detect_a;
    tx.dbg.detect_b |= detect_b;
  }//else Nothing to do

  //センサエラーの確認
  int chk_num = on_rail_num*dir_rail;
  if(chk_num>=0){
    tx.sts.on_rail = chk_num;               //送信用列車台数データ
    if(chk_num==0){
      wdt = millis();
    } else if(chk_num>3 || WDT_PERIOD<millis()-wdt){  //列車台数が異常または規定時間以上列車が居座る場合
      on_rail_num-=dir_rail;                //1台減算
      Debug("通過センサエラーを検出、修正しました。\n");
    }//else Nothing to do
  } else {                                  //エラー処理
    tx.sts.on_rail = 0;
    on_rail_num = 0;
    chk_num = 0;
    Debug("進入センサエラーを検出、修正しました。\n");
  }
  return chk_num;
}

/**
 * リセット関数：プログラムを最初から始める。
 */
void (*resetFunc)(void) = 0;  //アドレスを0にしてプログラムを最初から始める。

/**
 * 初期化関数
 * 起動後、最初に一度だけ処理される。
 */
void setup() {
  Serial.begin(115200);           //デバッグ用シリアル出力設定
  for(int i=0; i<sizeof(eeprom); i++) eeprom.data[i] = EEPROM.read(i);
  if(eeprom.prm.en==0x01){
    ACT_THR = eeprom.prm.act<<2;
    NEG_THR = eeprom.prm.neg<<2;
    SENS_PERIOD = eeprom.prm.sens;
    MASK_PERIOD = eeprom.prm.mask<<2;
    TMR_PERIOD  = eeprom.prm.det<<8;
    WDT_PERIOD  = eeprom.prm.wdt<<8;
    Debug("EEPROMからパラメータを読み取ります。");
  }//else Nothing to do

  //初期設定
  Wire.begin(ADDRESS);          //I2C設定
  Wire.onRequest(RequestEvent); //I2Cリクエスト関数設定
  Wire.onReceive(ReceiveEvent); //I2C受信関数設定
  
  pinMode(RELAY, OUTPUT);       //リレー出力ピン設定

  //各変数の初期化
  mode = NORMAL;
  tx.data = 0;
  rx.data = 0;
}

/**
 * ループ関数
 * 初期化関数の後、常に繰り返し動作する。
 */
void loop() {
  #ifdef DEBUG
    static unsigned long time = millis();
    unsigned long period = millis()-time;
    if(period>10) {Debug(period);Debug("ms\n");}
    //else Nothing to do
    time = time + period;
  #endif
  
  //区間状態の更新
  int on_rail_num = senser_process();

  //指令処理
  cmd_process(on_rail_num);

  //レールに出力
  digitalWrite(RELAY, power);
}

/**
 * I2C送信関数
 * リクエストに応え、区間状態を送信する。
 */
void RequestEvent(){
  tx.sts.output = power;
  Wire.write(tx.data);        //データ送信
  //Debug("状態を送信しました。");Debug(tx.data);Debug("\n");
  tx.data = 0x00;
}

/**
 * I2C受信関数
 * 受け取った命令を実行する。
 */
void ReceiveEvent(int num){
  char data[8];
  rx.data = Wire.read();  //データ受信
  if(rx.cmd.mlt_flg){
    int i=0;
    while(Wire.available() > 0) data[i++] = Wire.read();
  }//else Nothing to do
  Debug("指令を受信しました。:");Debug(rx.data);Debug("\n");
  
  //進行方向の設定
  if(rx.cmd.directry==0 && dir_rail==CCW){
    dir_rail = CW;
    Debug("時計回り進行\n");
  } else if(rx.cmd.directry==1 && dir_rail==CW) {
    dir_rail = CCW;
    Debug("反時計回り進行\n");
  }//else Nothing to do

  switch(rx.cmd.code){
    case 0x0:             //待機
      break;
    case 0x1:             //発車
      cmd_flg = false;
      rail_on();
      break;
    case 0x2:             //停止
      cmd_flg = false;
      rail_off();
      break;
    case 0x3:             //列車進入時、停止
    case 0x4:             //列車進入時、一時停止
    case 0x5:             //列車通過時、停止
    case 0x6:             //列車通過時、一時停止
    case 0x8:             //列車進入時、遅延停止
    case 0x9:             //列車通過時、遅延停止
      cmd_flg = true;
      rail_on();
      break;
    case 0x7:             //指令解除
      cmd_flg = false;
      mode = NORMAL;
      Debug("指令が解除されました。\n");
      break;
    case 0xd:             //デバッグモード
      cmd_flg = false;
      mode = DEBUG_1;
      Debug("デバッグモード\n");
      break;
    case 0xe:
      eeprom.prm.en   = 0x01;
      eeprom.prm.act  = data[0];
      eeprom.prm.neg  = data[1];
      eeprom.prm.sens = data[2];
      eeprom.prm.mask = data[3];
      eeprom.prm.det  = data[4];
      eeprom.prm.wdt  = data[5];
      ACT_THR = data[0]<<2;
      NEG_THR = data[1]<<2;
      SENS_PERIOD = data[2];
      MASK_PERIOD = data[3]<<2;
      TMR_PERIOD  = data[4]<<8;
      WDT_PERIOD  = data[5]<<8;
      Debug("パラメータが設定されました。\n");
      break;
    case 0xf:             //リセット
      Debug("リセットされます。\n");
      resetFunc();
      break;
    default:
      break;
  }
}
