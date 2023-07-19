import processing.serial.*;
Serial port;
Serial port1;
Serial port2;
Serial port3;
float scrx = 1500.0, scry = 1000.0;
//int scrx = 1500, scr_y = 1000;
PImage elR, elV, elB;
PImage cR, cV, cB;
PImage lu1,lu2, lub;

PFont abs;
PFont min;

float CX=250, CY=250;
float box,boy;

int robotid;
int[] ax = new int[4], ay = new int[4], az = new int[4];
int[] mx = new int[4], my = new int[4], mz = new int[4];
int[] phase = new int[4];
int[] r= new int[4],g= new int[4],b= new int[4];
float[] itix = new float[4],itiy = new float[4];

float itix_max = scrx*41/100,itiy_max = scry*43/100;
float itix_min = scrx*5/100,itiy_min = scry*1/100;

int temp = 0;

int img_x1,img_y1, img_x2,img_y2, img_x3,img_y3;

void setup() {
   size(1500,1000,P2D);//scrx,scryを更新する事
  //port = new Serial(this, "COM4",9600);  // COM3 は各自の環境に合わせて修正せよ
  port1 = new Serial(this, "/dev/ttyUSB0",9600);  // 
  port2 = new Serial(this, "/dev/ttyUSB1",9600);  // 
  port3 = new Serial(this, "/dev/ttyUSB2",9600);  // 
  port1.clear();
  port2.clear();
  port3.clear();
  elR = loadImage("edgelessR.PNG");
  elV = loadImage("edgelessV.PNG");
  elB = loadImage("edgelessB.PNG");
  cR = loadImage("circleR.PNG");
  cV = loadImage("circleV.PNG");
  cB = loadImage("circleB.PNG");
  lu1 = loadImage("leftup1.PNG");
  lu2 = loadImage("leftup2.PNG");
  lub = loadImage("leftupb.png");
  
  itix[1] = scrx*10/100;itiy[1] = scry*1/100;
  itix[2] = scrx*5/100;itiy[2] = scry*22/100;
  itix[3] = scrx*10/100;itiy[3] = scry*42/100;
  
  colorMode(RGB,256);
    
  abs = createFont("ABSTER.otf", scrx/10);
  min = createFont("Corporate-Mincho-ver3.otf",scrx/10);
  
}

void rotateViewPoint(float phi, float[] x) {
  float sphi = sin(phi);
  float cphi = cos(phi);
  float x0 = cphi*x[0]+sphi*x[1];
  float x1 = -sphi*x[0]+cphi*x[1];
  x[0] = x0;
  x[1] = x1;
}

float[] X0 = new float[3];
float[] X1 = new float[3];
void line3Dsub(float[] x, boolean isset) {
  final float[] vp = {0, -1/sqrt(2), 1/sqrt(2)};

  rotateViewPoint(2.0*PI/3.0,x);
  // Projection
  float c = 0.0;
  for (int i=0; i<3; i++) {
    c += vp[i]*x[i];
  }
  for (int i=0; i<3; i++) {
      x[i] -= c*vp[i];
  }
  c = sqrt(x[1]*x[1]+x[2]*x[2]);
  if (x[1]>=0) {
    x[1] = c;
  } else {
    x[1] = -c;
  }
  x[0] = CX+x[0];
  x[1] = CY-x[1];
  // x[2]=0;  
  if (isset) {
    for (int i=0; i<3; i++) {
      X0[i] = x[i];
    }
  } else {
    for (int i=0; i<3; i++) {
      X1[i] = x[i];
    }
    line(X0[0],X0[1],X1[0],X1[1]);
  }
}

void line3D(float x0, float y0,float z0,float x1,float y1,float z1) {
  float[] x = new float[3];
  x[0] = x0;  x[1] = y0;  x[2] = z0;
  line3Dsub(x,true);
  x[0] = x1;  x[1] = y1; x[2] = z1;
  line3Dsub(x,false);
}

float SCALE = 1.0;
void drawVec(float X, float Y, float Z) {
  stroke(87,103,129);
  line3D(0,0,0,scrx*9/100,0,0); line3D(scrx*9/100,0,0,scrx*8/100,5,0); line3D(scrx*9/100,0,0,scrx*8/100,-5,0);
  line3D(0,0,0,0,scrx*9/100,0); line3D(0,scrx*9/100,0,5,scrx*8/100,0); line3D(0,scrx*9/100,0,-5,scrx*8/100,0);
  line3D(0,0,0,0,0,scrx*9/100); line3D(0,0,scrx*9/100,5/sqrt(2),-5/sqrt(2),scrx*8/100); line3D(0,0,scrx*9/100,-5/sqrt(2),5/sqrt(2),scrx*8/100);
  stroke(164,164,180,100);  
  float x = SCALE * X;
  float y = SCALE * Y;
  float z = SCALE * Z;
  line3D(0,0,0,x,y,0);
  line3D(x,y,0,x,y,z);
  line3D(0,0,0,x,0,0);
  line3D(x,0,0,x,y,0);
  line3D(x,y,0,0,y,0);
  line3D(0,y,0,0,0,0);
  stroke(241,224,230);line3D(0,0,0,x,y,z);//メイン線
  fill(87,103,129);  //text(X,CX-80,490);  text(Y,CX,490);  text(Z,CX+80,490);
}

int x,y;
int ani=10;
int a=0;
int t=0;
void draw() {
  t++;
  background(0);//背景初期化 
    
  //以降画像処理
  //左上背景
  tint(255);
  image(lub, 0, 0, scrx/2, scry/2);//画像を表示//第３第４引数で大きさを決める
  tint(255,200-((ani-80)*10));
  image(lu1, 0, 0, scrx/2, scry/2);//画像を表示//第３第４引数で大きさを決める
  if(a == 0)ani++;
  if(a == 1)ani--;
  if(ani<0)a = 0;
  if(ani>160)a = 1;
  tint(255,(ani-80)*10);
  image(lu2, 0, 0, scrx/2, scry/2);//画像を表示//第３第４引数で大きさを決める
  
  //ロボットアイコン  
  if(t%100 == 1){
    for(int i=1;i<4;i++){
      itix[i]+=ax[i]*ax[i]/16.0;//ここの値で調整//現在は18秒
      itiy[i]+=ay[i]*ay[i]/16.0;//現在は14秒
      if(itix[i] > itix_max)itix[i] = itix_max;
      if(itiy[i] > itiy_max)itiy[i] = itiy_max;
      if(itix[i] < itix_min)itix[i] = itix_min;
      if(itiy[i] < itiy_min)itiy[i] = itiy_min;
    }
  }
  
  tint(255);
  image(cR, itix[1], itiy[1], scrx/25, scrx/25);//ロボット１
  image(cV, itix[2], itiy[2], scrx/25, scrx/25);//ロボット２
  image(cB, itix[3], itiy[3], scrx/25, scrx/25);//ロボット３  

  
  //UI背景
  tint(100);
  image(elR, scrx/2 + scrx/100, scry/100, scrx*48/100, scry*48/100);//画像を表示//第３第４引数で大きさを決める
  image(elV, scrx/100, scry/2 + scry/100, scrx*48/100, scry*48/100);//画像を表示//第３第４引数で大きさを決める
  image(elB, scrx/2 + scrx/100, scry/2 + scry/100, scrx*48/100, scry*48/100);//画像を表示//第３第４引数で大きさを決める
  
    
  for(int i=1;i<4;i++){//右上、左下、右下ボックス表示
    //セル背景
    switch(i){
      case 1:box = 0; boy = 0;
        break;
      case 2:box = -scrx/2; boy = scry/2;
        break;
      case 3:box = 0; boy = scry/2;
        break;
    }
    strokeWeight(scrx/100);
    strokeJoin(ROUND);
    stroke(50,50,72,200);
    noFill();
    rect(box+scrx*52/100, boy+scry*2/100+scrx/200, scrx*22/100, scry*22/100-scrx/100);        // 文字のセル
    rect(box+scrx*52/100, boy+scry*26/100+scrx/200, scrx*22/100,scry*22/100-scrx/100);        // RGBのセル
    rect(box+scrx*76/100, boy+scry*2/100+scrx/200, scrx*22/100, scry*46/100-scrx/100);        // ax,mxのセル
    noStroke();
    fill(50,50,72,200);
    rect(box+scrx*52/100+scrx/200, boy+scry*2/100+scrx/100, scrx*21/100, scry*22/100-scrx*2/100);        // 文字のセル
    rect(box+scrx*52/100+scrx/200, boy+scry*26/100+scrx/100, scrx*21/100, scry*22/100-scrx*2/100);        // RGBのセル
    rect(box+scrx*76/100+scrx/200, boy+scry*2/100+scrx/100, scrx*21/100, scry*46/100-scrx*2/100);        // ax,mxのセル
    
    //文字(robotid,phase)
    textFont(abs,scrx*3/100);
    fill(241,224,230);
    text("ROBOTID", box+scrx*54/100, boy+scry*11/100);
    textFont(min,scrx*3/100);
    text("："+i, box+scrx*68/100, boy+scry*11/100-3);
    textFont(abs,scrx*3/100);
    phase[i]=0;
    switch(phase[i]){
      case 0:fill(133,137,164);
        break;
      case 1:fill(241,224,230);
        break;
    }
    text("PHASE",box+scrx*54/100, boy+scry*19/100);
    textFont(min,scrx*3/100);
    text("："+phase[i], box+scrx*68/100, boy+scry*19/100-3);
    
    //RGB
    strokeWeight(scry*5/100);
    stroke(133,137,164);//BLACK
    line(box+scrx*55/100,boy+scry*31/100,box+scrx*70/100,boy+scry*31/100);
    line(box+scrx*55/100,boy+scry*37/100,box+scrx*70/100,boy+scry*37/100);
    line(box+scrx*55/100,boy+scry*43/100,box+scrx*70/100,boy+scry*43/100);
    stroke(237,64,64);//R
    if(r[i]>0)line(box+scrx*55/100,boy+scry*31/100,box+scrx*(55+r[i]*15/256)/100,boy+scry*31/100);else line(box+scrx*55/100,boy+scry*31/100,box+scrx*65/100,boy+scry*31/100);
    stroke(64,237,67);//G
    if(g[i]>0)line(box+scrx*55/100,boy+scry*37/100,box+scrx*(55+g[i]*15/256)/100,boy+scry*37/100);else line(box+scrx*55/100,boy+scry*37/100,box+scrx*65/100,boy+scry*37/100);
    stroke(64,67,237);//B  
    if(b[i]>0)line(box+scrx*55/100,boy+scry*43/100,box+scrx*(55+b[i]*15/256)/100,boy+scry*43/100);else line(box+scrx*55/100,boy+scry*43/100,box+scrx*65/100,boy+scry*43/100);//要検証
    
    //ax[1] = 20;ay[1] = 20;az[1] = 20;//デバッグ用
    //mx[1] = 20;my[1] = 20;mz[1] = 20;
    
    // Draw Acceleration vector
    textFont(abs,scrx*1/100);
    strokeWeight(5);
    CX = box+scrx*87/100;//250
    CY = boy+scry*15/100;
    SCALE = 2.0;//2.0
    drawVec(ax[i],ay[i],az[i]);
    //text("Acceleration",CX-40,40);
    text("AX",CX-96,CY+104); text("AY",CX+118,CY+69); text("AZ",CX-14,CY-104);  
    // Draw Magnetic flux vector
    CX = box+scrx*87/100;//750
    CY = boy+scry*37/100;
    SCALE = 1.0;//1.0
    drawVec(mx[i],my[i],mz[i]);
    //text("Magnetic flux",CX-40,40);
    text("MX",CX-96,CY+104); text("MY",CX+118,CY+69); text("MZ",CX-14,CY-104);  
    // Draw Heading direction
    /*CX = 600;
    float scale = 0.5;
    line(CX-scale*my[1],100+scale*mx[1],CX+scale*my[1],100-scale*mx[1]);
    line(CX+scale*my[1],100-scale*mx[1],CX+0.6*scale*my[1]+0.2*scale*mx[1],100-0.6*scale*mx[1]+0.2*scale*my[1]);
    line(CX+scale*my[1],100-scale*mx[1],CX+0.6*scale*my[1]-0.2*scale*mx[1],100-0.6*scale*mx[1]-0.2*scale*my[1]);
    */
  }  
}
int read1byteint(Serial p) {
  int x = p.read();
  x -= 128;
  return x;
}

int read2byte(Serial p) {
  int x = p.read();  x <<= 8;   x |=p.read();
  if (x>32767) x -= 65536;
  return x;
}

void serialEvent(Serial p) {
  temp=1;
  if(p == port1){
    if (p.available() >=12) {
      if (p.read() == 'H') {
        robotid =read1byteint(p);
        ax[robotid] = read1byteint(p);  ay[robotid] = read1byteint(p);  az[robotid] = read1byteint(p);
        mx[robotid] = read1byteint(p);  my[robotid] = read1byteint(p);  mz[robotid] = read1byteint(p);
        phase[robotid] = read1byteint(p);
        r[robotid] = read1byteint(p);
        g[robotid] = read1byteint(p);
        b[robotid] = read1byteint(p);
        
  //      p.clear();
      }
    }
  }
  if(p == port2){
    if (p.available() >=12) {
      if (p.read() == 'H') {
        robotid =read1byteint(p);
        ax[robotid] = read1byteint(p);  ay[robotid] = read1byteint(p);  az[robotid] = read1byteint(p);
        mx[robotid] = read1byteint(p);  my[robotid] = read1byteint(p);  mz[robotid] = read1byteint(p);
        phase[robotid] = read1byteint(p);
        r[robotid] = read1byteint(p);
        g[robotid] = read1byteint(p);
        b[robotid] = read1byteint(p);
        
  //      p.clear();
      }
    }
  }
  if(p == port3){
    if (p.available() >=12) {
      if (p.read() == 'H') {
        robotid =read1byteint(p);
        ax[robotid] = read1byteint(p);  ay[robotid] = read1byteint(p);  az[robotid] = read1byteint(p);
        mx[robotid] = read1byteint(p);  my[robotid] = read1byteint(p);  mz[robotid] = read1byteint(p);
        phase[robotid] = read1byteint(p);
        r[robotid] = read1byteint(p);
        g[robotid] = read1byteint(p);
        b[robotid] = read1byteint(p);
        
  //      p.clear();
      }
    }
  }
}
