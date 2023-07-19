/* 警告音を鳴らすコード
 tone(buzzerPin, 1500); //ブザーを440Hzで鳴らす
 delay(300); //100ms 鳴らす
 noTone(buzzerPin); //ブザーを止める
 */

#include <ZumoMotors.h> //モータライブラリ
#include <Pushbutton.h> //ボタンライブラリ
#include <LSM303.h>
LSM303 compass;

#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_60X);

float red, green, blue;
unsigned int r_min, g_min, b_min; //r,g,bの最小値
unsigned int r_max, g_max, b_max; //r,g,bの最大値
unsigned long timeStarted; //開始時間を格納
unsigned long timeNow, timePrev; //現在時間を格納
int mode;

//通信用
unsigned long timeInitSend, timeNowSend, timePrevSend; //  通信用の時間管理変数
float mx=0, my=0, mz=0;
float ax=0, ay=0, az=0;

float red_G, green_G, blue_G; // カラーセンサで読み取ったRGB値（0-255）


int phaseFlag = 0; //フェーズをフラグで管理、初期値0

ZumoMotors motors; //ZumoMotorsクラスのインスタンス作成

//超音波センサー設定
const int trig = 2; //Trigピンをデジタル7番に接続
const int echo = 4; //Echoピンをデジタル8番に接続

unsigned long interval; //Echoのパルス幅
const int buzzerPin = 3; //ブザーは3番ピン

//ロボット・チーム設定
int robotSerial = 1; //自陣から見た時、左から1〜3まで
int teamNumber = 1; //1なら赤、2なら青が自陣

void setup() {
	Serial.begin(9600);
	tcs.begin(); //RGBセンサー起動
	pinMode(trig, OUTPUT); //7番を出力ポートに設定
	pinMode(echo, INPUT); //8番を入力ポートに設定
	//追記
	Wire.begin();
	setupCompass();
	calibrationCompass();
	timeInitSend = millis();
	timePrevSend=0;
	
}

void loop() {
	
	switch(checkColor()) {
		case 0: //検出されなかった
			
			//moveFB(150,150);
			if(cupIsDetected()==1)moveFB(0,0);
			break;
		case 1: //黒色を検出した
			
			break;
		case 2: //青色を検出した
			//moveAfterDetected(2, teamNumber);
			break;
		case 3: //赤色を検出した
			moveAfterDetectedRed(1, teamNumber);
			break;
		default:
			break;
	}
	
	magnetSensor();
	
	/*if(cupIsDetected()) { //コップが検出された
	 if(cupIsStoped()) { //コップが停止している
	 phaseFlag = 1;
	 }
	 }*/
	
	switch(phaseFlag) {
		case 0://初期状態
			timePrev = millis();
			phaseFlag = 1;
			break;
		case 1://コップを検知・停止していることを確認
			//takeCupToGoal();
			turnTo(130.0);
			tone(buzzerPin, 1500); //ブザーを440Hzで鳴らす
			delay(300); //100ms 鳴らす
			noTone(buzzerPin); //ブザーを止める
			
			//if ( millis() - timePrev> 5000){
			phaseFlag = 2;
			//}
			
			break;
		case 2:
			moveFB(0,0);
			break;
		default:
			break;
	}
	
	
	//通信用
	//getRGB(red_G, green_G, blue_G); // カラーセンサでRGB値を取得(0-255)
	timeNowSend = millis() - timeInitSend; // 経過時間
	sendData(); // データ送信
	
	
}

void takeCupToGoal() {//コップをゴールまで持ち帰る
	
}

void moveAfterDetectedRed(int sceneCode, int teamNumber) {
	//自陣・1なら赤、2なら青
	
	unsigned long detectedTime;
	//detectedTime = millis();
	//timeNow = millis();
	
	if(sceneCode == teamNumber) { //自陣の色を検出した
		moveFB(150, 150);
		delay(300);
		moveFB(-150,-150);
		delay(150);
		moveFB(100,-100); //時計回りで回転
		delay(500);
		moveFB(150,150);
	} else { //敵陣の色を検出した
		
	}
	
}

int checkColor() { //色を確認する
	tcs.getRGB(&red, &green, &blue); //UInt(16bit)でカラーの値を読み込む
	
	if(95 < blue && 95 < green && 95 < red) { //もし色が白ならば
		return 0;
	} else if(blue < 95 && green < 95 && red < 95) { //もし色が黒ならば
		return 1;
	} else if(blue < 95 && green < 95 && 105 < red) { //もし色が赤ならば
		return 3;
	} else if(105 < blue && green < 95 && red < 95) { //もし色が青ならば
		return 2;
	}
	
	return 0;
	
}

int cupIsDetected() { //コップを確認する
	
	double distance = distanceForCup();
	
	if(distance != 0 && distance <= 20) { //コップまでの距離が20センチ以下
		return 1;
	}
	
	return 0;
}

double distanceForCup() {
	digitalWrite(trig, HIGH);
	delayMicroseconds(10); //10μs のパルスを Tring ピンに出力
	digitalWrite(trig, LOW);
	//Echo 信号が HIGH である時間(μs)を pulseIn 関数で計測
	interval = pulseIn(echo, HIGH, 23071);
	//23071μs 以上経過したら、超音波が反射して返ってこないとみなして 0 を返す
	double distance = interval / 10000.0 * 346.75 / 2.0; //cmに変換
	
	return distance;
}

int cupIsStoped() { //コップが停止していることを確認
	unsigned long cISStartedTime;
	unsigned long cISFirstStartedTime;
	unsigned long cISNowTime;
	int distCheckTimes;
	cISStartedTime = millis();
	int moveCheck = 1;
	double dist[10];
	
	double distance;
	double distMin, distMax; //補正のため
	
	while(moveCheck == 1) {//0.7秒間チェック
		distance = distanceForCup();
		cISNowTime = millis();
		dist[distCheckTimes] = distance;
		distCheckTimes++;
		if(distCheckTimes > 4) {
			distCheckTimes = 0;
			double sumed = 0;
			for(int i = 0;i <= 4;i++){
				sumed += dist[i];
			}
			sumed = sumed / 5.0;
			
			if(sumed < distMin) {
				distMin = sumed;
			}
			
			if(distMax < sumed) {
				distMax = sumed;
			}
			
			if(cISNowTime - cISStartedTime >= 500){
				if(distMax - distMin < 5.0) { //最大距離が最小距離と5センチ差以下ならば
					return 1;
				} else { //コップが動いている
					distMax = 0;
					distMin = 500;
					cISStartedTime = millis();
				}
			}
			
			if(cISNowTime - cISFirstStartedTime >= 2000) { //開始から2.0秒間コップの停止が感知できなかったら
				return 0;
			}
			
		}
		
		
	}
	
	return 0;
}

void moveFB(int left, int right) {
	motors.setLeftSpeed(left);
	motors.setRightSpeed(right);
}

//ここからコンパス
#define CALIBRATION_SAMPLES 70  // Number of compass readings to take when calibrating
#define CRB_REG_M_2_5GAUSS 0x60 // CRB_REG_M value for magnetometer +/-2.5 gauss full scale
#define CRA_REG_M_220HZ    0x1C // CRA_REG_M value for magnetometer 220 Hz update rate

int standard_G=180;//向かせる向き
float heading_G = 180;//ロボットが向いてる向き


void magnetSensor() { //ロボットが向いている方向を更新
	//引数angularの角度と同じ方角になった時に1を返す関数
	// 北 = 0, 東 = 90, 南 = 180, 西 = 270//青線が19,赤線が-50//heading_Gの値は129〜-129
	compass.read();
	compass.m_min.x = min(compass.m.x,compass.m_min.x);  compass.m_max.x = max(compass.m.x,compass.m_max.x);
	compass.m_min.y = min(compass.m.y,compass.m_min.y);  compass.m_max.y = max(compass.m.y,compass.m_max.y);
	compass.m_min.z = min(compass.m.z,compass.m_min.z);  compass.m_max.z = max(compass.m.z,compass.m_max.z);
	ax = compass.a.x/256; map(compass.a.x,-32768,32767,-128,127);
	ay = compass.a.y/256; map(compass.a.y,-32768,32767,-128,127);
	az = compass.a.z/256; map(compass.a.z,-32768,32767,-128,127);
	mx = map(compass.m.x,compass.m_min.x,compass.m_max.x,-128,127);
	my = map(compass.m.y,compass.m_min.y,compass.m_max.y,-128,127);
	mz = map(compass.m.z,compass.m_min.z,compass.m_max.z,-128,127);
	Serial.println(mx);
	heading_G = atan2(my,mx) * 180 / M_PI;
	/*if(abs(standard_G-heading_G)<=5){
	 return 1;
	 }*/
	//return 0;
}

float sum_e = 0;
void turnTo(float theta_r) {//任意の向きに向かせる関数
	
	float u;
	float KP = 4.0;
	float TIinv = 2/1000.0;
	while(1){
		magnetSensor();
		heading_G = atan2(my,mx) * 180 / M_PI;
		if (heading_G<0) heading_G += 360;
		float e = theta_r;//-heading_G;//相対方向回転に変更
		if (e<-180) {
			e+=360;
		}
		if (e>180)  {
			e-=360;
		}
		if (abs(e) > 45.0 ) { // |e|>45のときはP制御
			u = KP*e;           // P制御
		} else {              // |e|<=45 の時はPI制御
			sum_e += TIinv*e*(timeNow-timePrev);
			u = KP*(e+sum_e);   // PI 制御
		}
		if ( u> 180 ) {
			u = 180;  // 飽和
		}
		if ( u<-180 ) {
			u = -180; // 飽和
		}
		moveFB(u,-u);
		//if(u < 5)break;
		if(theta_r - heading_G < 15 && heading_G - theta_r < 15)break;
	}
	//return u;
}
//ここまでコンパス



//ここから通信
void write1byteint(int x) {
	Serial.write(x+128);
}

void write2byte(int x) {
	Serial.write(x>>8);
	Serial.write(x&255);
}
// 通信方式2
void sendData()
{
	static unsigned long timePrevSend = 0;
	static boolean flag_start = true; // 最初の通信かどうか
	int inByte;
	//Serial.println(mx);
	// if文の条件： 最初の通信である || 最後のデータ送信から500ms経過 || (データ送信要求が来ている && 最後のデータ送信から50ms経過)
	if ( flag_start == true || timeNowSend - timePrevSend > 500 ||
		(Serial.available() > 0 && timeNowSend - timePrevSend > 50) ) {
		flag_start = false;
		while(Serial.available() > 0){ // 送信要求が複数来ていた場合は全て読み込む
			inByte = Serial.read();
		}
		
		Serial.write('H');
		Serial.write(mode);
		Serial.write((int)red);
		Serial.write((int)green);
		Serial.write((int)blue);
		//Serial.write((int)ax);
		//Serial.write((int)ay);
		//Serial.write((int)az);
		
		Serial.write((int)mx);
		Serial.write((int)my);
		Serial.write((int)mz);   /*
								  write1byteint((int)ax);
								  write1byteint((int)ay);
								  write1byteint((int)az);
								  write1byteint((int)mx);
								  write1byteint((int)my);
								  write1byteint((int)mz);*/
		
		timePrevSend = timeNowSend;
	}
}
/*
 void sendData() {
 // put your main code here, to run repeatedly:Serial.write(1);
 static unsigned char i = 0;
 
 Serial.write('H');
 
 unsigned char rgb = i >> 4;
 
 unsigned int rBits = rgb;
 Serial.write(rBits);
 
 unsigned int gbBits = 0;
 gbBits = gbBits | (unsigned int)rgb;
 gbBits = gbBits | ((unsigned int)rgb << 4);
 
 Serial.write(gbBits);
 
 Serial.write("H");
 unsigned char rBits = red >> 4;
 unsigned char gBits = green >> 4;
 unsigned char bBits = blue >> 4;
 Serial.write(rBits);
 Serial.write((gBits << 4)| bBits);
 //Serial.write(i);
 //Serial.write(i);
 //Serial.write(i);
 Serial.write(0x0A);
 delay(80);
 i++;
 if(i > 255)i = 0;
 }*/
