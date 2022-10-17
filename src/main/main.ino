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
union rx_t{                 //I2C受信用データ型
  uint8_t data = 0x00;
  struct{
    uint8_t reserved : 4;   //未使用
    uint8_t on_rail : 3;    // Number of train
    uint8_t output : 1;     // 0:Stop, 1:Go
  }sts;
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
};
enum point_command_t{       //ポインタ用コマンド一覧
  PXC0 = 0,  //待機
  PXC1,      //列車検出直後、切替
  PXC2,      //
};
struct rail_info_t{         //モジュール情報構造体
  int addr = 0x40;          //I2Cアドレス
  union rx_t rx;        //受信データ
  union tx_t tx;        //送信データ
  int critical = 0;         //危険検出変数　0:安全、1:対処中
  bool point_exist = false; //ポイントレーンの有無
  int point_num = 0;        //ポイントレーン番号
};
struct point_info_t{        //ポインタ情報構造体
  int addr = 0x60;          //I2Cアドレス
  union rx_t rx;        //受信データ
  union tx_t tx;        //送信データ
  bool work = false;        //動作状態　true:追越線, false:主線
};

//グローバル変数の宣言
struct rail_info_t rail_info[M_NUM];    //モジュール情報
struct point_info_t point_info[P_NUM];  //ポインタ情報
int dir = CW;                           //進行方向 1:時計回り, -1:反時計回り

void error(int err){
  Debug("例外が発生しました。");
}

/**
 * 全モジュールにリクエスト送信、路線状態を受信
 * 進行方向の確認
 */
void check(void){
  //各区間の状態確認
  for(int i=0; i<M_NUM; i++){
    Wire.requestFrom(rail_info[i].addr, 1); //区間状態を受信
    while (Wire.available()){               //受信データ数の確認
      rail_info[i].rx.data = Wire.read();   //データ読み取り
      Debug(" 区間%d:, i");
      Debug(rail_info[i].rx.sts.on_rail);             //デバッグ用出力
    }
  }
  Debug("\n");

  //進行雄方向確認
  if(digitalRead(CW_SENS)==LOW && digitalRead(CCW_SENS)==HIGH){
    dir = CW;
  } else if(digitalRead(CW_SENS)==HIGH && digitalRead(CCW_SENS)==LOW){
    dir = CCW;
  } else if(digitalRead(CW_SENS)==LOW && digitalRead(CCW_SENS)==LOW){
    //Error
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
  bool ret = false;   //発令フラグ
  for(int i=0; i<M_NUM; i++){
    if(rail_info[i].critical==0 && rail_info[i].rx.sts.on_rail>1){  //列車間距離接近
      rail_info[i].critical = 1;        //危険状態指定
      rail_info[i].tx.cmd.code = RC6;   //車間距離を空ける
      int fm = i+dir;                   //前方区間番号
      int bm = i-1*dir;                 //後方区間番号
      if(fm>=M_NUM){
          fm = 0;
      } else if(bm>=M_NUM){
          bm = 0;
      }//else Nothing to do
      if(rail_info[fm].point_exist){      //前方に追い越しレーンあり
        int num = rail_info[fm].point_num;
        rail_info[fm].critical = 1;
        point_info[num].work = true;      //ポイント動作中
        point_info[num].tx.cmd.code = PC1;//ポイント切替指示
      }//else Nothing to do
      if(rail_info[bm].rx.sts.on_rail>0){     //後方列車あり
        rail_info[bm].tx.cmd.code = RC2;
      } else {                            //後方列車なし
        rail_info[bm].tx.cmd.code = RC4;
      }
      ret = true;     //発令フラグ
    } else if(rail_info[i].critical==1 && rail_info[i].rx.sts.on_rail<=1){  //列車は接近していないが、安全確認されていない。
      int num = rail_info[i].point_num;
      if(rail_info[i].point_exist){       //追越しレーンがある場合
        check_point(num);                 //ポインタの状態受信
        if(!point_info[num].work){        //主線復帰
          rail_info[i].critical = 0;      //安全確認
        }//else Nothing to do
      } else {                            //追越しレーンがない場合
        rail_info[i].critical = 0;        //安全確認
      }
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
      if(dir==1)rail_info[i].tx.cmd.directry = 0;               //進行方向追加
      else      rail_info[i].tx.cmd.directry = 1;
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
    if(dir==1)rail_info[i].tx.cmd.directry = 0;       //進行方向追加
    else      rail_info[i].tx.cmd.directry = 1;
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
 * 初期化関数
 * 起動後、最初に一度だけ処理される。
 */
void setup() {
  #ifdef DEBUG
    Serial.begin(9600);         //デバッグ用シリアル出力設定
  #endif

  //初期設定
  Wire.begin();               //I2C初期設定
  pinMode(CW_SENS, INPUT);    //列車進行方向検出　時計回り　ローアクティブ
  pinMode(CCW_SENS, INPUT);   //列車進行方向検出　反時計回り　ローアクティブ

  //各変数の初期化
  for(int i=0; i<M_NUM; i++) rail_info[i].addr += i;
  for(int i=0; i<P_NUM; i++) point_info[i].addr += i;
  rail_info[0].point_exist = true;
  rail_info[0].point_num = 0;
  
  //路線状態の確認と初期化
  while(!init_t()) error(0);

  //運行開始
  send_all();
}

/**
 * ループ関数
 * 初期化関数の後、常に繰り返し動作する。
 */
void loop() {
  check();                //路線状態の確認
  if(analyze()){          //状況判別と指令
    send();          //各区間へ送信
  }//else Nothing to do

  delay(100);             //待機時間(msec)
}
