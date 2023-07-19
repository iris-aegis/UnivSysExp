import processing.serial.*;
Serial port;

int ax = 0, ay = 0, az = 0;
int mx = 0, my = 0, mz = 0;
int CX=250, CY=250;
int mode_G;

void settings() {
  size(1000,500);
}

void setup() {
   port = new Serial(this, "/dev/ttyUSB1",9600);  // COM3 は各自の環境に合わせて修正せよ
   port.clear();
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
  stroke(128);
  line3D(0,0,0,250,0,0); line3D(250,0,0,240,5,0); line3D(250,0,0,240,-5,0);
  line3D(0,0,0,0,250,0); line3D(0,250,0,5,240,0); line3D(0,250,0,-5,240,0);
  line3D(0,0,0,0,0,250); line3D(0,0,250,5/sqrt(2),-5/sqrt(2),240); line3D(0,0,250,-5/sqrt(2),5/sqrt(2),240);
  stroke(128,64,0);
  float x = SCALE * X;
  float y = SCALE * Y;
  float z = SCALE * Z;
  line3D(0,0,0,x,y,0);
  line3D(x,y,0,x,y,z);
  line3D(0,0,0,x,0,0);
  line3D(x,0,0,x,y,0);
  line3D(x,y,0,0,y,0);
  line3D(0,y,0,0,0,0);
  stroke(255,0,0);  line3D(0,0,0,x,y,z);
  fill(0);  text(X,CX-80,490);  text(Y,CX,490);  text(Z,CX+80,490);
  text('X',CX-100,CY+160); text('Y',CX+200,CY+60); text('Z',CX-20,CY-180);
  
}


void draw() {
  noStroke();  fill(255);  rect(0,0,1000,500);
  // Draw Acceleration vector
  CX = 250;
  SCALE = 2.0;
  drawVec(ax,ay,az);
  text("Acceleration",CX-40,40);
  // Draw Magnetic flux vector
  CX = 750;
  SCALE = 1.0;
  drawVec(mx,my,mz);
  text("Magnetic flux",CX-40,40);
  // Draw Heading direction
  CX = 600;
  float scale = 0.5;
  line(CX-scale*my,100+scale*mx,CX+scale*my,100-scale*mx);
  line(CX+scale*my,100-scale*mx,CX+0.6*scale*my+0.2*scale*mx,100-0.6*scale*mx+0.2*scale*my);
  line(CX+scale*my,100-scale*mx,CX+0.6*scale*my-0.2*scale*mx,100-0.6*scale*mx-0.2*scale*my);
  text(mode_G,500,40);
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
  if (p.available() >=8 ) {
    if (p.read() == 'H') {
      ax = read1byteint(p);  ay = read1byteint(p);  az = read1byteint(p);
      mx = read1byteint(p);  my = read1byteint(p);  mz = read1byteint(p);
      mode_G = p.read();  //read1byteunsigned;
//      p.clear();
    }
  }
}
