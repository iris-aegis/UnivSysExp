import processing.serial.*;
Serial port;
int mode;
int green, red, blue, green_p, red_p, blue_p;
int count;
boolean sof_f = false; // SoF(Start of flame) を発見したかどうかのフラグ

void setup() {
  size(1200, 400);
  background(255);
  count = 0;
  //println(Serial.list());
  // String arduinoPort = Serial.list()[1];
  // port = new Serial(this, arduinoPort, 9600 );
  port = new Serial(this, "/dev/ttyUSB1", 9600 ); // シリアルポート名は各自の環境に合わせて適宜指定
  red_p = 0;
  green_p = 0;
  blue_p = 0;
}

void draw() {

  float y_p, y;

  stroke(200, 200, 200);
  line(0, height*0.1, width, height*0.1);
  line(0, height*0.9, width, height*0.9);

  y_p = map(red_p, 0, 255, height*0.9, height*0.1);
  y = map(red, 0, 255, height*0.9, height*0.1);
  stroke(255, 0, 0);
  line((count-1)*10, y_p, (count)*10, y );

  y_p = map(green_p, 0, 255, height*0.9, height*0.1);
  y = map(green, 0, 255, height*0.9, height*0.1);
  stroke(0, 255, 0);
  line((count-1)*10, y_p, (count)*10, y );

  y_p = map(blue_p, 0, 255, height*0.9, height*0.1);
  y = map(blue, 0, 255, height*0.9, height*0.1);
  stroke(0, 0, 255);
  line((count-1)*10, y_p, (count)*10, y );

  red_p = red;
  green_p = green;
  blue_p = blue;

  stroke(255);
  fill(255);
  rect(0, height-50, 240, 50);
  textSize(50);
  fill(0);
  text("mode=", 20, height-10);
  text((int)mode, 200, height-10);
  noFill();

  if ( count >= 120 ) {
    count = 0;
    background(255);
  }

  ++count;
}

void keyPressed() { // keyが押されると呼ばれる関数
  if (key == 's') {
    noLoop(); // draw()の繰返し処理を停止
  } else if (key == 'c') {
    background(255); // draw()の繰返し処理を再開
    loop();
  }
}


// 通信方式2
void serialEvent(Serial p) {

  int l = p.available(); // 受信バッファ内のデータ数
  boolean bod_f = false; // 1組のデータ(block of data)が得られたか？

  while (l>0) { // 受信バッファ内にデータがある場合
    if (sof_f == false) { // SoFを発見していない場合
      if (p.read() == 'H') { // SoF(Start of Frame)の検査
        sof_f = true; // SoFの発見
      }
      l--; // 受信バッファのデータ数の修正
    }
    if (sof_f == true) { // SoFを発見している場合
      if (l >= 4) { // 受信バッファのデータ数が4以上

        mode =  p.read();
        red =  p.read();
        green = p.read();
        blue =  p.read();

        print(" RGB = ");
        println(red, green, blue);

        bod_f = true; // 1組のデータを読み込んだ
        sof_f = false; // 1組のデータを読み取ったのでSoFの発見をクリア
        l -= 4; // 受信バッファのデータ数の修正
      } else { // 受信バッファのデータ数不足の場合
        break; // whileループを中断
      }
    }
  }
  if (bod_f == true) // 1組のデータを読み込んだので
    p.write("A"); // 次のデータ送信要求を送信
}
// このプログラムではHに続くデータが4つ（1バイトのデータが4つ）で一組を想定している．
// プログラム中の4の部分は適宜変更が必要
