/* 警告音を鳴らすコード
 tone(buzzerPin, 1500); //ブザーを440Hzで鳴らす
 delay(300); //100ms 鳴らす
 noTone(buzzerPin); //ブザーを止める
 */
#include <Wire.h>
#include <ZumoMotors.h> //モータライブラリ
#include <Pushbutton.h> //ボタンライブラリ
#include <LSM303.h>
LSM303 compass;
Pushbutton button(ZUMO_BUTTON);
#include "Adafruit_TCS34725.h"

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

unsigned long start_time; 

ZumoMotors motors; //ZumoMotorsクラスのインスタンス作成

//超音波センサー設定
const int trig = 2; //Trigピンをデジタル7番に接続
const int echo = 4; //Echoピンをデジタル8番に接続

unsigned long interval; //Echoのパルス幅
const int buzzerPin = 3; //ブザーは3番ピン

//ロボット・チーム設定
int robotSerial = 1; //自陣から見た時、左から1〜3まで
int teamNumber = 1; //1なら赤、2なら青が自陣

//kokomocompas
//ここからコンパス
#define CALIBRATION_SAMPLES 70  // Number of compass readings to take when calibrating
#define CRB_REG_M_2_5GAUSS 0x60 // CRB_REG_M value for magnetometer +/-2.5 gauss full scale
#define CRA_REG_M_220HZ    0x1C // CRA_REG_M value for magnetometer 220 Hz update rate

int standard_G=180;//向かせる向き
float heading_G = 180;//ロボットが向いてる向き
float goalDirection;

//int mx,my = 0;

void setupCompass() {
	compass.init();
	compass.enableDefault();
	compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
	compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate
	delay(1000); // 良く分からないが必要
}

void  calibrationCompass() {
	unsigned int index;
	int motorL, motorR;
	
	LSM303::vector<int16_t> running_min = {
		32767, 32767, 32767
	}
	, running_max = {
		-32767, -32767, -32767
	};
	
	motorL = 200;
	motorR = -200;
	motors.setLeftSpeed(motorL);
	motors.setRightSpeed(motorR);
	
	for (index = 0; index < CALIBRATION_SAMPLES; index ++) {
		// Take a reading of the magnetic vector and store it in compass.m
		compass.read();
		
		running_min.x = min(running_min.x, compass.m.x);
		running_min.y = min(running_min.y, compass.m.y);
		
		running_max.x = max(running_max.x, compass.m.x);
		running_max.y = max(running_max.y, compass.m.y);
		
		delay(50);
	}
	
	motorL = 0;
	motorR = 0;
	motors.setLeftSpeed(motorL);
	motors.setRightSpeed(motorR);
	
	// Set calibrated values to compass.m_max and compass.m_min
	compass.m_max.x = running_max.x;
	compass.m_max.y = running_max.y;
	compass.m_max.z = running_max.z;
	compass.m_min.x = running_min.x;
	compass.m_min.y = running_min.y;
	compass.m_min.z = running_min.z;
}

void CalibrationCompassManual() {
	compass.m_min.x = 0;
	compass.m_min.y = 0;
	compass.m_max.x = 0;
	compass.m_max.y = 0;
}

template <typename T> float heading(LSM303::vector<T> v) {
	float x_scaled =  2.0 * (float)(v.x - compass.m_min.x) / ( compass.m_max.x - compass.m_min.x) - 1.0;
	float y_scaled =  2.0 * (float)(v.y - compass.m_min.y) / (compass.m_max.y - compass.m_min.y) - 1.0;
	
	float angle = atan2(y_scaled, x_scaled) * 180 / M_PI;
	if (angle < 0)
		angle += 360;
	return angle;
}

// Yields the angle difference in degrees between two headings
float relativeHeading(float heading_from, float heading_to)
{
	float relative_heading = heading_to - heading_from;
	
	// constrain to -180 to 180 degree range
	if (relative_heading > 180)
		relative_heading -= 360;
	if (relative_heading < -180)
		relative_heading += 360;
	
	return relative_heading;
}

// Average 10 vectors to get a better measurement and help smooth out
// the motors' magnetic interference.
float averageHeading(){
	LSM303::vector<int32_t> avg = {
		0, 0, 0
	};
	
	for (int i = 0; i < 10; i ++)
	{
		compass.read();
		avg.x += compass.m.x;
		avg.y += compass.m.y;
	}
	avg.x /= 10.0;
	avg.y /= 10.0;
	
	// avg is the average measure of the magnetic vector.
	return heading(avg);
}

float averageHeadingLP()
{
	static LSM303::vector<int32_t> avg = {
		0, 0, 0
	};
	
	compass.read();
	avg.x = 0.2 * compass.m.x + 0.8 * avg.x;
	avg.y = 0.2 * compass.m.y + 0.8 * avg.y ;
	
	
	// avg is the average measure of the magnetic vector.
	return heading(avg);
}

void setup() {
	Serial.begin(9600);
	tcs.begin(); //RGBセンサー起動
	pinMode(trig, OUTPUT); //7番を出力ポートに設定
	pinMode(echo, INPUT); //8番を入力ポートに設定
	//追記
	Wire.begin();
	setupCompass();
	calibrationCompass();
	button.waitForButton(); // Zumo buttonが押されるまで待機
	standard_G = atan2(my,mx) * 180 / M_PI;
	timeInitSend = millis();
	timePrevSend=0;
	
	phaseFlag = -1;
	
}

void loop() {
	
	switch(checkColor()) {
		case 0: //検出されなかった
			moveFB(150,150);
			if(cupIsDetected()==1) {
				moveFB(0,0);
			}
			break;
		case 1: //黒色を検出した
			moveFB(-130,-130);
			delay(700);
			turnTo(90);
			
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
	
	tone(buzzerPin, 1500); //ブザーを440Hzで鳴らす
	delay(300); //100ms 鳴らす
	noTone(buzzerPin); //ブザーを止める
	
	if(cupIsDetected()) { //コップが検出された
		if(cupIsStoped()) { //コップが停止している
			phaseFlag = 1;
		}
	}
	
	switch(phaseFlag) {
		case -1:
			magnetSensor();
			goalDirection = heading_G;
			phaseFlag = 0;
			break;
		case 0://初期状態
			
			break;
		case 1://コップを検知・停止していることを確認
			takeCupToGoal();
			break;
		case 2:
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
	magnetSensor();
	
	//turnTo(180); //ゴール方向に回転
	if(!cupIsDetected()) {
		return 0;
	}
	
	turnTo(heading_G - goalDirection + 180);
	tone(buzzerPin, 500); //ブザーを440Hzで鳴らす
	delay(300); //100ms 鳴らす
	noTone(buzzerPin); //ブザーを止める
	moveFB(150,150);
	while(checkColor() != 2) {//青を検出するまで
		
	}
	//青を検出した
	
	delay(700); //0.7sだけもっと進む
	
	moveFB(-150,-150); //後進する
	delay(1000); //1.0sだけ進む
	
	turnTo(160); //180°まで回転
	
	moveFB(150,150);
	
	
}

void moveAfterDetectedRed(int sceneCode, int teamNumber) {
	//自陣・1なら赤、2なら青
	
	unsigned long detectedTime;
	detectedTime = millis();
	timeNow = millis();
	
	if(sceneCode == teamNumber) { //自陣の色を検出した
		/*moveFB(150, 150);
		 delay(300);
		 moveFB(-150,-150);
		 delay(150);
		 moveFB(100,-100); //時計回りで回転
		 delay(500);
		 moveFB(150,150);*/
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
	
	if(distance != 0 && distance <= 4) { //コップまでの距離が20センチ以下
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
	//cISStartedTime = millis();
	int moveCheck = 1;
	double dist[10];
	
	double distance;
	double distMin, distMax; //補正のため
	distMin = 7;
	distMax = 7;
	
	while(moveCheck == 1) {//0.7秒間チェック
		distance = distanceForCup();
		//cISNowTime = millis();
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
			
			//if(cISNowTime - cISStartedTime >= 500){
			if(distMax - distMin < 2.5) { //最大距離が最小距離と5センチ差以下ならば
				
				return 1;
			} else { //コップが動いている
				distMax = 0;
				distMin = 500;
				//cISStartedTime = millis();
			}
			//}
			
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

//int standard_G=180;//向かせる向き
//float heading_G = 180;//ロボットが向いてる向き


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
	heading_G = atan2(my,mx) * 180 / M_PI;
	if(abs(standard_G-heading_G)<=5){
		return 1;
	}
	
	
	/*
	 float u;
	 float KP = 4.0;
	 float TIinv = 2/1000.0;
	 //heading_G = atan2(my,mx) * 180 / M_PI;
	 //if (heading_G<0) heading_G += 360;
	 float e = angular;
	 if(e<-180) {
	 e+=360;
	 }
	 if(e>180) {
	 e-=360;
	 }
	 if (abs(e) > 45.0) { // |e|>45のときはP制御
	 u = KP*e;           // P制御
	 } else {              // |e|<=45 の時はPI制御
	 sum_e += TIinv*e*(timeNow_G-timePrev_G);
	 u = KP*(e+sum_e);   // PI 制御
	 }
	 if ( u> 180 ) {
	 u = 180;  // 飽和
	 }
	 if ( u<-180 ) u = -180; // 飽和
	 
	 if (abs(angular)<=5) {
	 
	 }
	 */
	
	return 0;
}

float sum_e = 0;
void turnTo(float theta_r) {//任意の向きに向かせる関数
	
	float u;
	float KP = 4.0;
	float TIinv = 2/1000.0;
	magnetSensor();
	standard_G = atan2(my,mx) * 180 / M_PI;
	while(1){
		magnetSensor();
		heading_G = atan2(my,mx) * 180 / M_PI;
		if (heading_G<0) heading_G += 360;
		float e = theta_r + standard_G - heading_G;//相対方向回転に変更
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
		if ( u>= 0 ) {
			u = 150;  // 飽和
		}
		if ( u<0 ) {
			u = -150; // 飽和
		}
		moveFB(u,-u);
		//if(u < 5)break;
		if(10 > e && e > -10){
			break;
		}
	}
	//return u;
}
//ここまでコンパス

/*
void turnToGoal(void) {//任意の向きに向かせる関数
	
	float u;
	float KP = 4.0;
	float TIinv = 2/1000.0;
	magnetSensor();
	standard_G = atan2(my,mx) * 180 / M_PI;
	while(1){
		magnetSensor();
		heading_G = atan2(my,mx) * 180 / M_PI;
		if (heading_G<0) heading_G += 360;
		float e = theta_r + standard_G - heading_G;//相対方向回転に変更
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
		if(10 > e && e > -10)break;
	}
	//return u;
}
*/


//ここから通信
void write1byteint(int x) {
	Serial.write(x+128);
}

void write2byte(int x) {
	Serial.write(x>>8);
	Serial.write(x&255);
}
// 通信方式2
void sendData() {
	static unsigned long timePrevSend = 0;
	static boolean flag_start = true; // 最初の通信かどうか
	int inByte;
	Serial.println(red);
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
		Serial.write((int)ax);
		Serial.write((int)ay);
		Serial.write((int)az);
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
