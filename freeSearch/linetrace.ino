/*void linetrace_bang_bang(){
  static float lightMin = 0; // 各自で設定
  static float lightMax = 255; // 各自で設定 （わざとエラーが出るようにしてある）
  static float speed = 100; // 回転・直進の基本速度
  static float Kp = 0.75; // 回転速度
  float lightNow;
  float speedDiff;

  lightNow = (red_G + green_G + blue_G ) / 3.0;
  if ( lightNow < (lightMin + lightMax) / 2.0 ) // 右回転
    speedDiff = -Kp * speed;//回転方向の速度を決定
  else // 左回転
    speedDiff = Kp * speed;//回転方向の速度を決定
  motorL_G = speed - speedDiff;//回転・直進を合わせた速度
  motorR_G = speed + speedDiff;//同上
}*/


void linetrace_P(){
  static float lightMin = 0; // 各自で設定
  static float lightMax = 255; // 各自で設定 （わざとエラーが出るようにしてある）
  static float lightAve = 125;
  static float speed = 100; // 回転・直進の基本速度
  static float Kp = 5; // 回転速度
  float lightNow;
  float speedDiff;

  lightNow = (red_G + green_G + blue_G ) / 3.0;
  if ( lightNow < (lightMin + lightMax) / 2.0 ){ // 右回転
    //Kp = map(lightAve - lightNow,0,0.8,0,128);
    //Kp = ((lightMin + lightMax) / 2.0 - lightNow )/125;
    speedDiff = map(lightMax - lightNow,0,100,0,50);//回転方向の速度を決定
  }else{ // 左回転
    //Kp = map(lightNow - lightAve,0,0.8,0,128);
    //Kp = (lightNow - (lightMin + lightMax) / 2.0)/(lightMax - lightMin);
    speedDiff = map(lightNow - lightMin,0,100,0,-50);//回転方向の速度を決定
  }  
  motorL_G = speed - speedDiff;//回転・直進を合わせた速度
  motorR_G = speed + speedDiff;//同上
}



void task_B()
{
  static unsigned long delayStandard = 0; // ロボットのみを止めるときの基準時刻
  static unsigned long startTime; // static変数，時間計測ははunsigned long
  static int point = 0;//赤線部分を通過した回数
  int rasen;
  int motorL;
  int motorR;
  // この変数は1秒停止の開始時間を覚えておくために使用

  switch ( mode_G ) {
    case 0:
      mode_G = 1;
      startTime = millis();
      break;  // break文を忘れない（忘れるとその下も実行される）

    case 1:
      motors.setSpeeds(150,-150);//その場回転
      if ( millis() - startTime > 2300) { 
        startTime = millis();
        mode_G = 2;
      }
      break;
    case 2:
      motors.setSpeeds(150,150);//直進
      if ( millis() - startTime > 1000) { 
        startTime = millis();
        mode_G = 3;
      }
      break;
    case 3:
      motors.setSpeeds(150,-150);//90度右に向く
      if ( millis() - startTime > 1000) { 
        startTime = millis();
        mode_G = 4;
      }
      break;
    case 4:
      motorL = 150;
      motorR = 60;
      motors.setSpeeds(motorL,motorR);//右旋回
      motorR ++;
      if ( millis() - startTime > 12000) { 
        startTime = millis();
        mode_G = 5;
      }
      break;

    case 5://左に向く
      motors.setSpeeds(0,150);
      if ( millis() - startTime > 1000) { 
        startTime = millis();
        mode_G = 6;
      }
      break;

      case 6://右に向き直す
      motors.setSpeeds(150,0);
      if ( millis() - startTime > 1000) { 
        startTime = millis();
        mode_G = 3;
      }
      break;
      

  }
}
      //if ( identify_color( 60, 172, 198 ) ) { // 青かどうか　（引数の値は各自で設定）
  //    if ( identify_color( 147, 172, 198 ) )  // 赤かどうか　（引数の値は各自で設定）

int identify_color( int red, int green, int blue )
{
  float d2;
  float d2_max = 60; // パラメーター（適宜調整）

  d2 = pow(red - red_G, 2) + pow(green - green_G, 2) + pow(blue - blue_G, 2);
  if ( d2 < d2_max * d2_max )
    return 1;
  else
    return 0;
}

int maintainState( unsigned long period )
{
  static int flagStart = 0; // 0:待ち状態，1:現在計測中
  static unsigned long startTime = 0;

  if ( flagStart == 0 ) {
    startTime = timeNow_G; // 計測を開始したtimeNow_Gの値を覚えておく
    flagStart = 1; // 現在計測中にしておく
  }

  if ( timeNow_G - startTime > period ) { // 計測開始からの経過時間が指定時間を越えた
    flagStart = 0; // 待ち状態に戻しておく
    startTime = 0; // なくても良いが，形式的に初期化
    return 1;
  }
  else
    return 0;
}
