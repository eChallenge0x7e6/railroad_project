//ライブラリの宣言
#include <Wire.h>

//マクロの定義
#define DEBUG (1)
#ifdef DEBUG
  #define Debug(s) Serial.print(s)
#else
  #define Debug(s) 
#endif
#define M_NUM   (4)         //モジュール総数
#define P_NUM   (1)         //ポインタ総数
#define CW      (1)         //時計回り
#define CCW     (-1)        //反時計回り
#define CW_SENS   (9)       //進行方向検出用ピン番号
#define CCW_SENS  (8)       //進行方向検出用ピン番号

//変数型の定義
enum mode_t{                //メイン用モード一覧
  NORMAL = 0,
  DEBUG_1,
};
union rx_t{                 //I2C受信用データ型
  uint8_t data = 0x00;
  struct{
    uint8_t reserved : 4;   //未使用
    uint8_t on_rail : 3;    // Number of train
    uint8_t output : 1;     // 0:Stop, 1:Go
  }sts;
  struct{
    uint8_t reserved : 2;   //未使用
    uint8_t sens_b : 1;     // センサB 0:未検出, 1:検出
    uint8_t sens_a : 1;     // センサA 0:未検出, 1:検出
    uint8_t on_rail : 3;    // Number of train
    uint8_t output : 1;     // 0:Stop, 1:Go
  }dbg;
};
union tx_t{                 //I2C送信用データ型
  uint8_t data = 0x00;
  struct{
    uint8_t reserved : 3;   //未使用
    uint8_t code : 4;       //指令コード
    uint8_t directry : 1;   //0:時計回り, 1:反時計回り
  }cmd;
};
enum rail_command_t{        //モジュール用コマンド一覧
  RC0 = 0,  //待機
  RC1,      //発車
  RC2,      //停車
  RC3,      //列車進入後、停車
  RC4,      //列車進入後、規定時間停車
  RC5,      //列車通過後、停車
  RC6,      //列車通過後、規定時間停車
  RC7,      //指令解除
  RCd,      //デバッグモード切替
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
  bool work = false;        //動作状態　true:追越線, false:主線
};

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
  Debug("エラーが発生しました。CASE:");Debug(err);Debug("\n");
  switch(err){
    case DIRECTRY_ERR:
      break;
    case I2C_DATA_ERR:
      break;
    case TRAFFIC_ERR:
      break;
    case UNEXPECTED_ERR:
      break;
  }
}

/**
 * 全モジュールにリクエスト送信、路線状態を受信
 * 進行方向の確認
 */
void check(void){
  int num = 0;
  //各区間の状態確認
  for(int i=0; i<M_NUM; i++){
    Wire.requestFrom(rail_info[i].addr, 1); //区間状態を受信
    while (Wire.available()){               //受信データ数の確認
      rail_info[i].rx.data = Wire.read();   //データ読み取り
      Debug("|");
      if(rail_info[i].critical) Debug("C");
      else                      Debug(" ");
      Debug(" 区間");Debug(i);Debug(":");
      Debug(rail_info[i].rx.sts.on_rail);Debug(",");   //デバッグ用出力
      Debug(rail_info[i].rx.sts.output);
    }
    num += rail_info[i].rx.sts.on_rail;
  }
  Debug(" SUM:");Debug(num);Debug("\n");

  //進行雄方向確認
  if(digitalRead(CW_SENS)==HIGH && digitalRead(CCW_SENS)==LOW){
    dir = CW;
  } else if(digitalRead(CW_SENS)==LOW && digitalRead(CCW_SENS)==HIGH){
    dir = CCW;
  } else if(digitalRead(CW_SENS)==LOW && digitalRead(CCW_SENS)==LOW){
    error(DIRECTRY_ERR);
  }//else Nothing to do
}

/**
 * 指定のポイントの状態を受信する。
 * @param num : ポイント番号
 */
void check_point(int num){
  Wire.requestFrom(point_info[num].addr, 1);            //ポイントの状態受信
  while(Wire.available()){
    point_info[num].rx.data = Wire.read();
  }
  point_info[num].work = point_info[num].rx.sts.output; //ポイント切替状態の取得
}

/**
 * 路線状況から危険状態を確認、指令を発令する。
 * @return true:発令, false:安全
 */
bool analyze(void){
  bool ret = false;         //発令フラグ

  if(dir!=post_dir){        //進行方向が切替わった場合
    ret = true;
    for(int i=0; i<M_NUM; i++) rail_info[i].tx.data = 0x00; //以前の送信データをクリア
  }//else Nothing to do

  for(int i=0; i<M_NUM; i++){
    int fm = i+dir;         //前方区間番号
    int bm = i-1*dir;       //後方区間番号
    if(fm>=M_NUM){
      fm = 0;
    } else if(bm>=M_NUM){
      bm = 0;
    }//else Nothing to do
    if(fm<0){
      fm = M_NUM-1;
    }else if(bm<0){
      bm = M_NUM-1;
    }//else Nothing to do
    /*列車二台用制御コード*/
    if(rail_info[i].rx.sts.on_rail>=1){  //列車間距離接近
      //Debug("危険状態を検出しました。");
      //rail_info[i].critical = 1;        //危険状態指定
      rail_info[bm].tx.cmd.code = RC2;
      //Debug("後方区間");Debug(bm);Debug("に停止指令を発令しました。\n");
      ret = true;
    }else if(rail_info[i].rx.sts.on_rail==0){ //安全確認
      //Debug("危険状態を回避しました。");
      //rail_info[i].critical = 0;        //危険状態クリア
      rail_info[bm].tx.cmd.code = RC1;
      //Debug("後方区間");Debug(bm);Debug("に発車指令を発令しました。\n");
      ret = true;
    /*列車三台制御用コード
    if(rail_info[i].critical==0 && rail_info[i].rx.sts.on_rail>1){  //列車間距離接近
      rail_info[i].critical = 1;        //危険状態指定
      Debug("危険状態を検出しました。");
      rail_info[i].tx.cmd.code = RC6;   //車間距離を空ける
      if(rail_info[fm].point_exist){      //前方に追い越しレーンあり
        int num = rail_info[fm].point_num;
        rail_info[fm].critical = 1;
        point_info[num].work = true;      //ポイント動作フラグ
        point_info[num].tx.cmd.code = PC1;//ポイント切替指示
        Debug("ポイント);Debug(num);Debug("にて追越し指令を発令しました。\n");
      }//else Nothing to do
      if(rail_info[bm].rx.sts.on_rail>0){ //後方列車あり
        rail_info[bm].tx.cmd.code = RC2;
        Debug("後方車に停止指令を発令しました。\n");
      } else {                            //後方列車なし
        rail_info[bm].tx.cmd.code = RC4;
        Debug("後方車に停止3s指令を発令しました。\n");
      }
      ret = true;
    } else if(rail_info[i].critical==1 && rail_info[i].rx.sts.on_rail<=1){  //列車は接近していないが、安全確認されていない。
      if(rail_info[i].point_exist){       //追越しレーンがある場合
        int num = rail_info[i].point_num;
        check_point(num);                 //ポインタの状態受信
        if(!point_info[num].work){        //主線復帰
          rail_info[i].critical = 0;      //安全確認
          Debug("危険状態は回避しました。\n");
        }//else Nothing to do
      } else {                            //追越しレーンがない場合
        rail_info[i].critical = 0;        //安全確認
        Debug("危険状態は回避しました。\n");
      }
    */
    }//else Nothing to do
  }
  return ret;
}

/**
 * 危険状態の区間に指令を送信する。
 */
void send(void){
  for(int i=0; i<M_NUM; i++){
    if(rail_info[i].critical==1){
      if(dir==CW) rail_info[i].tx.cmd.directry = 0;             //進行方向追加
      else        rail_info[i].tx.cmd.directry = 1;
      i2c_tx(rail_info[i].addr, rail_info[i].tx.data);          //指令送信
      if(rail_info[i].point_exist){
        int num = rail_info[i].point_num;
        i2c_tx(point_info[num].addr, point_info[num].tx.data);  //指令送信
      }//else Nothing to do
    }//else Nothing to do
  }
}

/**
 * 全モジュールに指令を送信する。
 */
void send_all(void){
  for(int i=0; i<M_NUM; i++){
    if(dir==CW) rail_info[i].tx.cmd.directry = 0;     //進行方向追加
    else        rail_info[i].tx.cmd.directry = 1;
    i2c_tx(rail_info[i].addr, rail_info[i].tx.data);  //指令送信
  }
}

/**
 * 線路内初期化関数
 * @return true:成功, false:エラー
 */
bool init_t(void){
  check();//線路内の列車の有無を確認
  
  //残列車の誘導

  for(int i=0; i<M_NUM; i++){
    rail_info[i].tx.cmd.code = RC1; //発車指令
  }
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
  long tmr_limit = millis();
  while(mode==DEBUG_1){
    for(int i=0; i<M_NUM; i++){
      Wire.requestFrom(rail_info[i].addr, 1);                       //区間状態を受信
      while (Wire.available()) rail_info[i].rx.data = Wire.read();  //データ読み取り
      Debug("|out");Debug(i);Debug(":");Debug(rail_info[i].rx.dbg.output);Debug(",A:");Debug(rail_info[i].rx.dbg.sens_a);Debug(",B:");Debug(rail_info[i].rx.dbg.sens_b);Debug("\n");
    }
    if(sensor==0 && rail_info[module].rx.dbg.sens_a==1){               //センサBで列車が検出された。
      Debug("区間");Debug(module);Debug("のセンサAを確認しました。\n");
      res = true;
      break;
    } else if(sensor==1 && rail_info[module].rx.dbg.sens_b==1){        //センサBで列車が検出された。
      Debug("区間");Debug(module);Debug("のセンサBを確認しました。\n");
      res = true;
      break;
    } else if(15000<(millis()-tmr_limit)){                          //15秒以内にセンサBに検出されない場合、デバッグモード#1を終了する。
      Debug("デバッグ失敗：区間");Debug(module);Debug("のセンサを検出できませんでした。\n");
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
  bool combined[M_NUM] = {true, true, false, false};  //センサ兼用フラグ
  for(int i=2; i!=1;){
    i2c_tx(rail_info[i].addr, RC1<<1);                //発車指令送信
    if(!test_sensor(i, 1)) return;                    //センサBの確認

    //次の区間の確認に移る
    if(i<M_NUM-1) i++;
    else          i = 0;
    
    //センサを兼用の際は、スキップする。
    if(combined[i]) continue;
    //else Nothing to do

    i2c_tx(rail_info[i].addr, RC1<<1);                //指令送信
    if(!test_sensor(i, 0)) return;                    //センサAの確認

    if(i!=0){
      i2c_tx(rail_info[i-1].addr, RC2<<1);            //後方に停止指令送信
      Debug("後方区間");Debug(i-1);Debug("に停止指令を発令しました。\n");
    }//else Nothing to do
  }
  Debug("デバッグモード#1を完遂しました。\n");
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
 * リセット関数：プログラムを最初から始める。
 */
void (*reset_func) (void) = 0;  //アドレスを0にしてプログラムを最初から始める。

/**
 * Serialに受信データがある時、loop()の最後に呼び出される。
 * @から始まるコマンドを受け付ける。
 */
void serialEvent(){
  char buff[16] = {0};
  int i = 0;
  delay(1);
  while(Serial.available()) buff[i++] = Serial.read();
  for(int j=0; j<i; j++){
    if(buff[j]=='@'){
      char n = buff[j+1];
      switch(n){
        case '0':         //緊急停止
          for(int k=0; k<M_NUM; k++) rail_info[k].tx.cmd.code = RC2;
          send_all();
          break;
        case '1':         //デバッグモード#1
          Debug("デバッグモード#1を開始します。\n");
          mode = DEBUG_1;
          for(int k=0; k<M_NUM; k++) rail_info[k].tx.cmd.code = RCd;  //デバッグモード指令
          send_all();
          delay(100);
          debug_mod_1();
          break;
        case '2':
          point_info[0].tx.cmd.code = PXC1;
          i2c_tx(point_info[0].addr, point_info[0].tx.data);
          Debug("ポイント切替\n");
          delay(100);
          Wire.requestFrom(point_info[0].addr, 1);                       //区間状態を受信
          delay(100);
          while (Wire.available()) Debug(Wire.read());
          Debug("\n");
          break;
        case 'N':         //通常モード
          mode = NORMAL;
          break;
        case 'R':         //リセット
          for(int k=0; k<M_NUM ; k++) rail_info[k].tx.cmd.code = RCf; //リセット指令
          send_all();
          delay(100);
          reset_func();
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
    Serial.begin(9600);       //デバッグ用シリアル出力設定
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
  
  //路線状態の確認と初期化
  while(!init_t()) error(TRAFFIC_ERR);

  //運行開始
  send_all();
}

/**
 * ループ関数:初期化関数の後、常に繰り返し動作する。
 */
void loop() {
  //int deb = millis();
  //Debug("loop period:");Debug(deb);Debug("\n");
  check();                //路線状態の確認
  if(analyze()){          //状況判別と指令
    send_all();         //全区間へ送信
  }//else Nothing to do

  delay(100);             //待機時間(msec)
}
