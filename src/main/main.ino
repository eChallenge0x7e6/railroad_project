//ライブラリの宣言
#include <Wire.h>

//マクロの定義
#define DEBUG (1)
#ifdef DEBUG
  #define Debug(s) Serial.print(s)
#else
  #define Debug(s) 
#endif
#define M_NUM         (4)   //モジュール総数
#define P_NUM         (1)   //ポインタ総数
#define CW            (1)   //時計回り
#define CCW           (-1)  //反時計回り
#define CW_SENS       (9)   //進行方向検出用ピン番号
#define CCW_SENS      (8)   //進行方向検出用ピン番号
#define LIMIT_PERIOD  (15000) //区間通過最大時間(ミリ秒)

//変数型の定義
enum mode_t{                //メイン用モード一覧
  NORMAL = 0,
  OVERTAKE,
  OVERTAKE_1,
  DEBUG_1,
};
union rx_t{                 //I2C受信用データ型
  uint8_t data = 0x00;
  struct{
    uint8_t reserved : 3;   //未使用
    uint8_t passed : 1;     //追越しフラグ
    uint8_t  on_rail : 3;    // Number of train
    uint8_t output : 1;     // 0:Stop, 1:Go
  }sts;
  struct{
    uint8_t reserved : 2;   //未使用
    uint8_t sens_b : 1;     // センサB 0:未検出, 1:検出
    uint8_t sens_a : 1;     // センサA 0:未検出, 1:検出
    uint8_t  on_rail : 3;    // Number of train
    uint8_t output : 1;     // 0:Stop, 1:Go
  }dbg;
};
union tx_t{                 //I2C送信用データ型
  uint8_t data = 0x00;
  struct{
    uint8_t mlt_flg : 1;    //連続送信フラグ
    uint8_t reserved : 2;   //未使用
    uint8_t code : 4;       //指令コード
    uint8_t directry : 1;   //0:時計回り, 1:反時計回り
  }cmd;
};
enum rail_command_t{        //モジュール用コマンド一覧
  RC0 = 0,  //待機
  RC1,      //発車
  RC2,      //停車
  RC3,      //列車進入後、停車
  RC4,      //列車進入後、一時停止
  RC5,      //列車通過後、停車
  RC6,      //列車通過後、一時停止
  RC7,      //指令解除
  RC8,      //列車進入後、遅延停止
  RC9,      //列車通過後、遅延停止
  RCd,      //デバッグモード切替
  RCe,      //パラメータ送信
  RCf,      //リセット
};
enum point_command_t{       //ポイント用コマンド一覧
  PXC0 = 0, //待機
  PXC1,     //切替
  PXC2,     //列車検出直後、切替
  PXC7,     //指令解除
  PXCd,     //デバッグモード切替
  PXCf,     //リセット
};
struct rail_info_t{         //モジュール情報構造体
  int addr = 0x40;          //I2Cアドレス
  union rx_t rx;            //受信データ
  union tx_t tx;            //送信データ
  int critical = 0;         //危険検出変数　0:安全、1:対処中
  bool point_exist = false; //ポイントレーンの有無
  int point_num = 0;        //ポイントレーン番号
};
struct point_info_t{        //ポイント情報構造体
  int addr = 0x60;          //I2Cアドレス
  union rx_t rx;            //受信データ
  union tx_t tx;            //送信データ
  bool passed = false;      //追越し済みフラグ
  bool work = false;        //動作状態　true:追越線, false:主線
};
union{                      //モジュール用パラメータ構造体
  uint8_t data[6] = {0};
  struct{
    byte code;
    byte act;
    byte neg;
    byte sens;
    byte mask;
    byte det;
    byte wdt;
  }prm;
}rail;
//グローバル変数の宣言
mode_t mode = NORMAL;
struct rail_info_t rail_info[M_NUM];    //モジュール情報
struct point_info_t point_info[P_NUM];  //ポイント情報
int dir = CW;                           //進行方向 1:時計回り, -1:反時計回り
int post_dir = CW;                      //進行方向切替わり確認用変数

enum error_type{    //エラータイプ一覧
  NO_ERR = 0,       //エラーなし
  //注意レベル
  DIRECTRY_ERR,     //進行方向検出エラー
  I2C_DATA_ERR,     //I2C送受信データ異常
  //一時停止レベル
  TRAFFIC_ERR,      //列車渋滞エラー
  //緊急停止レベル
  UNEXPECTED_ERR,   //想定外コードエラー
};

/**
 * エラー関数：エラー発生時に呼び出す
 * @param err:エラータイプ
 */
void error(error_type err){
  Debug("Occur Error Case:");Debug(err);Debug("\n");
  switch(err){
    case DIRECTRY_ERR:
    Debug("Error of DirectionSensor\n");
      break;
    case I2C_DATA_ERR:
      break;
    case TRAFFIC_ERR:
      break;
    case UNEXPECTED_ERR:
      break;
    case NO_ERR:
    default:
      break;
  }
}

/**
 * 路線状況から危険状態を確認、指令を発令する。(*常時発令状態)
 * @return true:発令, false:安全
 */
bool analyze(void){
  bool ret = false;         //発令フラグ

  for(int i=0; i<M_NUM; i++){
    int fm = i+dir;   //前方区間番号
    int bm = i-dir;   //後方区間番号
    if(fm>=M_NUM)       fm = 0;
    else if(fm<0)       fm = M_NUM-1;
    //else Nothing to do
    if(bm>=M_NUM)       bm = 0;
    else if(bm<0)       bm = M_NUM-1;

    if(mode==NORMAL){                     /*列車2台用制御コード*/
      if(rail_info[i].rx.sts.on_rail>=1){ //列車間距離接近
        //Debug("Detect Critical");
        rail_info[bm].tx.cmd.code = RC2;
        //Debug("Back rail");Debug(bm);Debug("was stopped.\n");
        ret = true;
      } else if(rail_info[i].rx.sts.on_rail==0){ //安全確認
        //Debug("Escape the critical");
        rail_info[bm].tx.cmd.code = RC1;
        //Debug("Back rail");Debug(bm);Debug("was started\n");
        ret = true;
      }//else Nothing to do
    } else if(mode==OVERTAKE){           /*追越し用制御コード*/
      if(rail_info[i].rx.sts.on_rail>1){  //列車間距離接近
        if(rail_info[i].critical==0){
          rail_info[i].critical = 1;      //危険状態フラグ
          rail_info[i].tx.cmd.code = RC6; //通過後一時停止
          Debug("***Detect Critical***");Debug("Rail");Debug(i);Debug("was stopped a temporarily\n");
          if(rail_info[fm].rx.sts.on_rail==0){
            if(rail_info[fm].point_exist && point_info[rail_info[fm].point_num].work==0){ //前方副線に空きあり
              rail_info[fm].critical = 1;
              switch_point(rail_info[fm].point_num, PXC2);                                //列車検出後、ポイント切替
              Debug("At Point");Debug(fm);Debug(" pass a slow one\n");
            }//else Nothing to do
          }//else Nothing to do
          ret = true;
        }//else Nothing to do
      } else {                            //列車安全運航
        if(rail_info[i].critical==1){
          if(rail_info[i].point_exist){   //ポイントがある場合
            check_point();                //ポイント情報取得
            if(point_info[rail_info[i].point_num].work && point_info[rail_info[i].point_num].passed){ //追越し確認後
              switch_point(rail_info[i].point_num, PXC1);   //ポイント切替
              rail_info[i].critical = 0;  //危険回避
              Debug("Passed\n");
            }//else Nothing to do
          } else {                        //ポイントがない場合
            rail_info[i].critical==0;     //危険回避
            rail_info[i].tx.cmd.code = RC1;
          }//else Nothing to do
        } else {
          rail_info[i].tx.cmd.code = RC0; //通常運行
        }
      }
      ret = true;   //常時送信
    } else if(mode==OVERTAKE_1){          /*列車2台用制御コード*/
      if(rail_info[i].rx.sts.on_rail>=1){ //列車間距離接近
        //Debug("Escape the critical");
        if(rail_info[fm].point_exist && point_info[rail_info[fm].point_num].work==0){ //前方副線に空きあり
          switch_point(rail_info[fm].point_num, PXC2);                                //列車検出後、ポイント切替
          Debug("At point");Debug(fm);Debug(" pass a slow one\n");
        }//else Nothing to do
        rail_info[i].tx.cmd.code = RC2;
        //Debug("Back rail");Debug(bm);Debug(" was stopped\n");
        ret = true;
      } else { //安全確認
        //Debug("Escape the critical");
        check_point();                //ポイント情報取得
        if(point_info[rail_info[i].point_num].work && point_info[rail_info[i].point_num].passed){ //追越し確認後
          switch_point(rail_info[i].point_num, PXC1);   //ポイント切替
          Debug("追越しが完了しました。\n");
        }//else Nothing to do
        rail_info[bm].tx.cmd.code = RC1;  //通常運行
        //Debug("Back rail");Debug(bm);Debug(" was started\n");
        ret = true;
      }
    }//else Nothing to do
  }
  return true;
}

/**
 * 進行方向を確認
*/
void check_dir(void){
  //進行方向確認
  int check = 2*digitalRead(CW_SENS) + digitalRead(CCW_SENS);
  switch(check){
    case 0:
      error(DIRECTRY_ERR);
      break;
    case 1:
      dir = CW;
      Debug("Direction:CW\n");
      break;
    case 2:
      dir = CCW;
      Debug("Direction:CCW\n");
      break;
    default:
      break;
  }
}

/**
 * 全モジュールにリクエスト送信、路線状態を受信
 */
void check_module(void){
  int num = 0;
  //各区間の状態確認
  for(int i=0; i<M_NUM; i++){
    Wire.requestFrom(rail_info[i].addr, 1); //区間状態を受信
    while (Wire.available()){               //受信データ数の確認
      rail_info[i].rx.data = Wire.read();   //データ読み取り
      Debug("|");
      if(rail_info[i].critical) Debug("C");
      else                      Debug(" ");
      Debug(" Rail");Debug(i);Debug(":");
      Debug(rail_info[i].rx.sts.on_rail);Debug(",");   //デバッグ用出力
      Debug(rail_info[i].rx.sts.output);
    }
    num += rail_info[i].rx.sts.on_rail;
  }
  Debug(" SUM:");Debug(num);Debug("\n");
}

/**
 * 指定のポイントの状態を受信する。
 */
void check_point(void){
  for(int i=0; i<P_NUM; i++){
    Wire.requestFrom(point_info[i].addr, 1);            //ポイントの状態受信
    while(Wire.available()){
      point_info[i].rx.data = Wire.read();
    }
    point_info[i].passed = point_info[i].rx.sts.passed; //追越しフラグ
    point_info[i].work = point_info[i].rx.sts.output;   //ポイント切替状態の取得
  }
}

/**
 * 指定アドレスに進行方向を加えて指令を送信する。
 */
void send(char addr, char code){
  tx_t tmp;
  tmp.cmd.code = code;
  if(dir==CW) tmp.cmd.directry = 0;                        //進行方向追加
  else        tmp.cmd.directry = 1;
  i2c_tx(addr, tmp.data);                                  //指令送信
}

/**
 * 全モジュールに指令を送信する。
 */
void send_all(void){
  for(int i=0; i<M_NUM; i++){
    if(dir==CW) rail_info[i].tx.cmd.directry = 0;           //進行方向追加
    else        rail_info[i].tx.cmd.directry = 1;
    i2c_tx(rail_info[i].addr, rail_info[i].tx.data);        //指令送信
    rail_info[i].tx.data = 0x00;                            //データクリア
  }
  for(int i=0; i<P_NUM; i++){
    i2c_tx(point_info[i].addr, point_info[i].tx.data);      //指令送信
    point_info[i].tx.data = 0x00;                           //データクリア
  }
}

/**
 * 全区間に指令を指定して送信する。
 * @param code: モジュール用コマンド
 */
void send_all_cmd(const rail_command_t* code){
  for(int i=0; i<M_NUM; i++) rail_info[i].tx.cmd.code = *(code+i);
  send_all();
}

/**
 * ポイント切替関数
 * @param num : ポイント番号
 * @param code: ポイント用コマンド
 */
void switch_point(int num, char code){
  point_info[num].tx.cmd.code = code;
  i2c_tx(point_info[num].addr, point_info[num].tx.data);
  Debug("Point");Debug(num);Debug(" was switched\n");
}

/**
 * 線路内初期化関数
 * @return true:成功, false:エラー
 */
bool init_t(void){
  check_module();//線路内の列車の有無を確認
  
  //残列車の誘導
  return true;
}

/**
 * 初期化関数：各区間のモジュールをリセット、線路内の列車を整理する。
 * 動作中は時計回りの一定速度での走行とする。
 * @param term : 区間通過最長時間（ミリ秒）
 */
bool init(int term){
  const rail_command_t code_0[M_NUM] = {RC1, RC1, RC1, RC1};
  send_all_cmd(code_0);   //全区間発車指令
  //間隔を空ける。
  const rail_command_t code_1[M_NUM] = {RC5, RC5, RC5, RC5};
  send_all_cmd(code_1);   //通過後停止
  delay(term);
  //区間0,2を空にする。
  const rail_command_t code_2[M_NUM] = {RC1, RC8, RC1, RC8};
  send_all_cmd(code_2);   //区間0,2を排車
  delay(term);
  const rail_command_t code_3[M_NUM] = {RCf, RC2, RCf, RC2};
  send_all_cmd(code_3);   //区間0,2をリセット
  delay(10);
  //区間1を空にする。
  const rail_command_t code_4[M_NUM] = {RC8, RC1, RC8, RC1};
  send_all_cmd(code_4);   //区間1,3を排車
  delay(term);
  const rail_command_t code_5[M_NUM] = {RC2, RCf, RC2, RC2};
  send_all_cmd(code_5);   //区間1をリセット
  delay(10);
  //区間3を空にする
  switch_point(0, PXC1);   //ポイント切替
  const rail_command_t code_6[M_NUM] = {RC2, RC2, RC2, RC5};
  send_all_cmd(code_6);   //区間3ポイント内を排車
  delay(term);
  const rail_command_t code_7[M_NUM] = {RC2, RC2, RC2, RCf};
  send_all_cmd(code_7);   //区間3をリセット
  switch_point(0, PXC1);  //ポイント切替
  Debug("finished initializing\n");
  return true;
}

/**
 * センサの動作確認用関数
 * @param module : 区間番号
 * @param sensor 0:センサA, 1:センサB
 * @return true:成功, false:失敗
 */
bool test_sensor(int module, int sensor){
  bool res = false;
  unsigned long tmr_limit = millis();
  while(mode==DEBUG_1){
    check_dir();
    check_module();
    for(int i=0; i<M_NUM; i++) {Debug("|out");Debug(i);Debug(":");Debug(rail_info[i].rx.dbg.output);Debug(",A:");Debug(rail_info[i].rx.dbg.sens_a);Debug(",B:");Debug(rail_info[i].rx.dbg.sens_b);Debug("\n");}
    if(sensor==0 && rail_info[module].rx.dbg.sens_a==1){               //センサBで列車が検出された。
      Debug("Rail");Debug(module);Debug(" SensorA was checked\n");
      res = true;
      break;
    } else if(sensor==1 && rail_info[module].rx.dbg.sens_b==1){        //センサBで列車が検出された。
      Debug("Rail");Debug(module);Debug(" SensorB was checked\n");
      res = true;
      break;
    } else if(LIMIT_PERIOD<(millis()-tmr_limit)){                          //15秒以内にセンサBに検出されない場合、デバッグモード#1を終了する。
      Debug("Missed checking for：Rail");Debug(module);Debug(" sensor cannot be checked\n");
      break;
    }//else Nothing to do
    delay(100);                                                   //100ミリ秒待機
  }
  return res;
}

/**
 * デバッグモード#1：レール出力とセンサ入力の確認
 * 列車を区間0に一台配置してください。出力を時計回りとし、シリアルにて状態を確認してください。
 */
void debug_mod_1(void){
  bool combined[M_NUM] = {true, false, false, true};  //センサ兼用フラグ

  for(int i=0; i<M_NUM; i++) rail_info[i].tx.cmd.code = RC1;
  send_all();                                         //全区間発車指令

  for(int i=0; i<M_NUM; i++){
    int num = i;
    if(!test_sensor(num, 1)) return;                  //センサBの確認

    //次の区間の確認に移る
    if(i<M_NUM-1) num++;
    else          num = 0;
    
    //センサを兼用の際は、スキップする。
    if(combined[num]) continue;
    //else Nothing to do

    send(rail_info[num].addr, RC1);                   //指令送信
    if(!test_sensor(num, 0)) return;                  //センサAの確認

    if(num!=0){
      send(rail_info[i-dir].addr, RC2);               //後方に停止指令送信
      Debug("Back rail");Debug(i-1);Debug(" was stopped\n");
    }//else Nothing to do
  }
  Debug("Completed to debug\n");
}

/**
 * I2C送信関数
 * @param n : モジュール番号
 * @param data : データ
 */
void i2c_tx(uint8_t addr, uint8_t data){
  Wire.beginTransmission(addr);     //送信アドレス設定
  Wire.write(data);                 //送信データ
  Wire.endTransmission();           //送信終了
}

/**
 * 複数バイトI2C送信関数
*/
void i2c_tx_mlt(uint8_t addr, uint8_t num, uint8_t* data){
  Wire.beginTransmission(addr);     //送信アドレス設定
  for(int i=0; i<num; i++)  Wire.write(*(data+i));                 //送信データ
  Wire.endTransmission();           //送信終了
}

/**
 * リセット関数：プログラムを最初から始める。
 */
void (*resetFunc)(void) = 0;  //アドレスを0にしてプログラムを最初から始める。

/**
 * Serialに受信データがある時、loop()の最後に呼び出される。
 * @から始まるコマンドを受け付ける。
 */
void serialEvent(){
  uint8_t buff[16] = {0};
  int i = 0;
  delay(1);
  while(Serial.available()) buff[i++] = Serial.read();
  for(int j=0; j<i; j++){
    char c = (char)buff[j];
    if(c=='@'){
      c = (char)buff[++j];
      switch(c){
        case '0':         //緊急停止
          for(int k=0; k<M_NUM; k++) rail_info[k].tx.cmd.code = RC2;
          send_all();
          break;
        case '1':         //強制ポイント切替
          point_info[0].tx.cmd.code = PXC1;
          i2c_tx(point_info[0].addr, point_info[0].tx.data);
          Debug("Switch point\n");
          delay(100);
          Wire.requestFrom(point_info[0].addr, 1);  //区間状態を受信
          delay(100);
          while (Wire.available()) Debug(Wire.read());
          Debug("\n");
          break;
        case 'D':         //デバッグモード#1
          mode = DEBUG_1;
          for(int k=0; k<M_NUM; k++) rail_info[k].tx.cmd.code = RCd;  //デバッグモード指令
          for(int k=0; k<P_NUM; k++) point_info[k].tx.cmd.code = RCd; //デバッグモード指令
          send_all();
          Debug("Debug#1 was started\n");
          delay(100);
          debug_mod_1();
          break;
        case 'I':         //線路内初期化処理
          init();
          break;
        case 'M':         //モード変更
          mode = (mode_t)(buff[++j]-0x30);
          for(int k=0; k<M_NUM; k++) rail_info[k].tx.cmd.code = RC1;
          send_all();
          Debug("Switch mode:");Debug(mode);Debug("\n");
          break;
        case 'R':         //リセット
          const rail_command_t reset_code[M_NUM] = {RCf, RCf, RCf, RCf};
          send_all_cmd(reset_code);         //全区間リセット
          Debug("Reset\n");
          delay(100);
          resetFunc();
          break;
        case 'S':         //モジュールに転送
          uint8_t addr = buff[++j];
          int k=0;
          rail.data[k++] = 0xe;
          while(j<i) {
            if((char)buff[++j]==',')  k++;
            else                rail.data[k] = rail.data[k]*10 + (buff[j]-0x30);
          }
          i2c_tx_mlt(addr, sizeof(rail.data), rail.data);  //シリアルデータをI2Cに送信する
          break;
        default:
          //無効なコマンドエラー
          break;
      }
    }
  }
}

/**
 * 初期化関数:起動後、最初に一度だけ処理される。
 */
void setup() {
  #ifdef DEBUG
    Serial.begin(115200);       //デバッグ用シリアル出力設定
    Debug("In debug mode\n");
  #endif

  //初期設定
  Wire.begin();               //I2C初期設定
  pinMode(CW_SENS, INPUT);    //列車進行方向検出　時計回り　ローアクティブ
  pinMode(CCW_SENS, INPUT);   //列車進行方向検出　反時計回り　ローアクティブ

  //各変数の初期化
  for(int i=0; i<M_NUM; i++) rail_info[i].addr += i;
  for(int i=0; i<P_NUM; i++) point_info[i].addr += i;
  rail_info[3].point_exist = true;
  rail_info[3].point_num = 0;
  Debug("p1\n");
  //路線状態の確認と初期化
  while(!init_t()) error(TRAFFIC_ERR);
  Debug("p2\n");
  //運行開始
  const rail_command_t start_code[M_NUM] = {RC1, RC1, RC1, RC1};
  send_all_cmd(start_code);
  for(int i=0; i<M_NUM; i++) rail_info[i].tx.cmd.code = RC1; //発車指令
  send_all();
  Debug("Start Rail road control\n");
}

/**
 * ループ関数:初期化関数の後、常に繰り返し動作する。
 */
void loop() {
  #ifdef DISABLE
    int deb = millis();
    Debug("loop period:");Debug(deb);Debug("\n");
  #endif
  
  check_dir();                        //進行方向の確認
  check_module();                     //路線状態の確認
  if(mode==OVERTAKE){
    check_point();                    //ポイント状態の確認
  }//else Nothing to do

  if(analyze()){                      //状況判別と指令
    send_all();                       //全区間へ送信
  }//else Nothing to do
  delay(100);                         //待機時間(msec)
}
