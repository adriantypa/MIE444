#define m11 6
#define m12 7
#define m1e 5
#define m21 8
#define m22 9
#define m2e 10
#define m31 12
#define m32 13
#define m3e 11

//Calibration
#define dStop 250 //Delay for resting motors
#define d90cw 650  //Delay for 90 deg cw rotation
#define d180cw 1300 //1325>1300
#define dStep  360 //265>360
#define dCont 25  //Delay for controller angle
#define m2Mod 0.95 //1>0.95

int count=0;
bool dontMove = false;
bool resetError = false;
bool in4Way = false;
int d4Way = 600;

int code;
// a=turntest, b=cw, c=ccw, d=stop, e=forward;


int spd=255;

void ccw(){ //delay 360ms for 90 degrees
  digitalWrite(m11, HIGH);
  digitalWrite(m12, LOW);
  analogWrite(m1e, spd);
  digitalWrite(m21, HIGH);
  digitalWrite(m22, LOW);
  analogWrite(m2e, spd);
  digitalWrite(m31, HIGH);
  digitalWrite(m32, LOW);
  analogWrite(m3e, spd);
}

void cw(){//delay 360ms for 90 degrees
  digitalWrite(m11, LOW);
  digitalWrite(m12, HIGH);
  analogWrite(m1e, spd);
  digitalWrite(m21, LOW);
  digitalWrite(m22, HIGH);
  analogWrite(m2e, spd);
  digitalWrite(m31, LOW);
  digitalWrite(m32, HIGH);
  analogWrite(m3e, spd);
}

void stopp(){
  digitalWrite(m11, LOW);
  digitalWrite(m12, LOW);
  analogWrite(m1e, spd);
  digitalWrite(m21, LOW);
  digitalWrite(m22, LOW);
  analogWrite(m2e, spd);
  digitalWrite(m31, LOW);
  digitalWrite(m32, LOW);
  analogWrite(m3e, spd);
}

void forward(){
  digitalWrite(m11, HIGH);
  digitalWrite(m12, LOW);
  analogWrite(m1e, spd);
  digitalWrite(m21, LOW);
  digitalWrite(m22, HIGH);
  analogWrite(m2e, spd*m2Mod);
  digitalWrite(m31, LOW);
  digitalWrite(m32, LOW);
  analogWrite(m3e, spd);
}

void reverse(){
  digitalWrite(m11, LOW);
  digitalWrite(m12, HIGH);
  analogWrite(m1e, spd);
  digitalWrite(m21, HIGH);
  digitalWrite(m22, LOW);
  analogWrite(m2e, spd);
  digitalWrite(m31, LOW);
  digitalWrite(m32, LOW);
  analogWrite(m3e, spd);
}
void takeStep(){
  forward();
  delay(dStep);
  stopp();
  delay(dStop);
}

void setup() {
  Serial.begin(19200);
  
  pinMode(m11, OUTPUT);
  pinMode(m12, OUTPUT);
  pinMode(m1e, OUTPUT);
  pinMode(m21, OUTPUT);
  pinMode(m22, OUTPUT);
  pinMode(m2e, OUTPUT);
  pinMode(m31, OUTPUT);
  pinMode(m32, OUTPUT);
  pinMode(m3e, OUTPUT);

  digitalWrite(m11, LOW);
  digitalWrite(m12, LOW);
  analogWrite(m1e, spd);
  digitalWrite(m21, LOW);
  digitalWrite(m22, LOW);
  analogWrite(m2e, spd);
  digitalWrite(m31, LOW);
  digitalWrite(m32, LOW);
  analogWrite(m3e, spd);

  delay(3000); //Startup Delay
}
void turnTest(){
  cw();
  delay(d90cw);
  stopp();
  delay(1000);
  ccw();
  delay(d90cw);
  stopp();
  delay(1000);
  cw();
  delay(d180cw);
  stopp();
  delay(1000);
  cw();
  delay(d180cw);
  stopp();
  delay(1000);
}

void loop() {
  while (!Serial.available()){
  }
  code = Serial.read();
  switch(code){
    case 97: turnTest(); Serial.print('d'); code = 0; break;//a
    case 98: cw(); code = 0; break;//b
    case 99: ccw(); code = 0; break;//c
    case 100: stopp(); code = 0; break;//d
    case 101: forward(); code = 0; break;//e
    case 102: reverse(); code = 0; break;//f
    
  }
  //takeStep();
  //turnTest();
  
}
