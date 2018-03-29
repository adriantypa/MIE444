   #include <Stepper.h>
#include <AccelStepper.h>
#define HALFSTEP 8

// Gripper motor pin definitions
#define motorPin1  36     // IN1 on the ULN2003 driver 1
#define motorPin2  34     // IN2 on the ULN2003 driver 1
#define motorPin3  32    // IN3 on the ULN2003 driver 1
#define motorPin4  30    // IN4 on the ULN2003 driver 1

// Define Trig and Echo Pins of Lower Ultrasonic
#define trigL  12  
#define echoL  13

//TO BE CHANGED DURING COMPETITION
int caseModifier = 1; //0=A, 1=B, 2=C, 3=D


// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
AccelStepper stepper1(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4);

// initialize gripper motor variables
int spd = 1000;
int pst = 3000;

// initialize lower ulstrasonic variables
long durL;
float distL;
float distL2;
float distU;
bool gripFlag = false;
int gripCount = 0;

#define trigPin 23
#define echoPin 2
// for ultrasonic interupt

volatile long echo_start = 0;                         // Records start of echo pulse 
volatile long echo_end = 0;                           // Records end of echo pulse
volatile long echo_duration = 0;                      // Duration - difference between end and start
volatile int trigger_time_count = 0;                  // Count down counter to trigger pulse time
volatile int currentDir;
volatile bool valueGet = false;

//Calibration
#define dStop 250 //Delay for resting motors
#define d90cw 640  //Delay for 90 deg cw rotation
#define d180cw 1280 //
#define dStep  360 //
#define dCont 25  //Delay for controller angle
#define m2Mod 0.95 //1>0.95

int a = 41;  //For displaying segment "a"
int b = 43;  //For displaying segment "b"
int c = 45;  //For displaying segment "c"
int d = 47;  //For displaying segment "d"
int e = 49;  //For displaying segment "e"
int f = 51;  //For displaying segment "f"
int g = 53;  //For displaying segment "g"

long distance;

float cm[8];
bool poi = false;
int currDir = -1;
int currLoc = -1;
int guessDir = -1;
int guessLoc = -1;
bool newLoc = false;
bool localized = false;

float particle[5][4][5]= //[position][heading][measurement] 0=B,1=R,2=F,3=L, 4=probablitiy
{
{//Four way
  {45, 75, 75, 75, 0.05},//S
  {75, 75, 75, 45, 0.05},//E
  {75, 75, 45, 75, 0.05},//N
  {75, 45, 75, 75, 0.05}//W
},
{//Long wall
  {45, 135, 75, 15, 0.05},//S
  {135, 75, 15, 45, 0.05},//E
  {75, 15, 45, 135, 0.05},//N
  {15, 45, 135, 75, 0.05}//W
},
{//Cave
  {45, 75, 15, 105, 0.05},//S
  {75, 15, 105, 45, 0.05},//E
  {15, 105, 45, 75, 0.05},//N
  {105, 45, 75, 15, 0.05}//W
},
{//South Loading
  {45, 15, 75, 45, 0.05},//S
  {15, 75, 45, 45, 0.05},//E
  {75, 45, 45, 15, 0.05},//N
  {45, 45, 15, 75, 0.05}//W
},
{//North Loading
  {15, 45, 45, 75, 0.05},//S
  {45, 45, 75, 15, 0.05},//E
  {45, 75, 15, 45, 0.05},//N
  {75, 15, 45, 45, 0.05}//W
}
};

int decision[5][4][5]= //[position][heading][casepickup, casedelivery] 0=Go Forward,1=Turn CW,2=Turn CCW,3=Turn 180, 4=Block Detection
{
{//Four way
  {0, 0, 3, 2, 2},//S
  {1, 1, 2, 0, 0},//E
  {3, 3, 4, 1, 1},//N
  {2, 2, 1, 3, 3}//W
},
{//Long wall
  {1, 1, 1, 3, 4},//S
  {3, 3, 3, 2, 1},//E
  {2, 2, 2, 4, 3},//N
  {0, 0, 0, 1, 2}//W
},
{//Cave
  {1, 3, 2, 2, 2},//S
  {3, 2, 0, 0, 0},//E
  {2, 4, 1, 1, 1},//N
  {0, 1, 3, 3, 3}//W
},
{//South Loading
  {3, 0, 3, 3, 3},//S
  {2, 1, 1, 1, 1},//E
  {4, 3, 0, 0, 0},//N
  {1, 2, 2, 2, 2}//W
},
{//North Loading
  {1, 1, 1, 1, 1},//S
  {3, 3, 0, 0, 0},//E
  {2, 2, 2, 2, 2},//N
  {4, 0, 3, 3, 3}//W
}
};

const int stepsPerRevolution = 200;

float ir[8];

int count=0;
bool dontMove = false;
bool resetError = false;
//bool in4Way = false;
int d4Way = 600;
bool nextCW = true;
bool blockPickup = false;
bool haveBlock = false;
bool dropBlock = false;
bool blockDetected = false;
bool endd = false;
bool checkingSideLeft = false;


Stepper myStepper(stepsPerRevolution, 31, 33, 35, 37);

//int spd=255;

int outputR, outputL;
float Setpoint=15;
float errSumR, lastErrR, errSumL, lastErrL;
float kp=10, ki=0, kd=5;

void rightPID(){
   /*Compute all the working error variables*/
   if(cm[2]>30){
    Setpoint=particle[currLoc][currDir][1];
    resetError=true;
   }
   else{
    Setpoint=15;
   }
   if(cm[2]<(Setpoint+15)&&cm[2]>(Setpoint-15)){
     if(checkingSideLeft){
       resetError=true;
     }
     
     float error = Setpoint - cm[2];
     if(resetError){
      lastErrR=error;
      resetError=false;
      errSumR=0;
     }
     errSumR += (error);
     float dErr = (error - lastErrR);
    
     /*Compute PID Output*/
     
     outputR = kp * error + ki * errSumR + kd * dErr;
     
     if(abs(outputR)<150){
      //displayDigit(int(error));
     }
     else{
      outputR=outputR*150/abs(outputR);
     }
    
     /*Remember some variables for next time*/
     lastErrR = error;
     checkingSideLeft=false;
   }
}
void leftPID(){
   /*Compute all the working error variables*/
   if(cm[6]>30){
    Setpoint=particle[currLoc][currDir][3];
    resetError=true;
   }
   else{
    Setpoint=15;
   }
   if(cm[6]<(Setpoint+15)&&cm[6]>(Setpoint-15)){
     if(!checkingSideLeft){
       resetError=true;
     }
    
     float error = Setpoint - cm[6];
     if(resetError){
      lastErrL=error;
      resetError=false;
     }
     errSumL += (error);
     float dErr = (error - lastErrL);
    
     /*Compute PID Output*/
     
     outputL = kp * error + ki * errSumL + kd * dErr;
     if(abs(outputL)<150){
      //displayDigit(int(error)); 
     }
     else{
      outputL=outputL*150/abs(outputL);
     }
     /*Remember some variables for next time*/
     lastErrL = error;
     checkingSideLeft=true;
   }
}
void takeStep(){
  Serial1.println('e');
  delay(dStep);
  Serial1.println('d');
  delay(dStop);
}
void takeSmallStep(){
  Serial1.println('e');
  delay(dStep/2);
  Serial1.println('d');
  delay(dStop);
}
void takeSmallStepBack(){
  Serial1.println('f');
  delay(dStep/2);
  Serial1.println('d');
  delay(dStop);
}
void takeVerySmallStep(){
  Serial1.println('e');
  delay(dStep/4);
  Serial1.println('d');
  delay(dStop);
}
void takeVerySmallStepBack(){
  Serial1.println('f');
  delay(dStep/4);
  Serial1.println('d');
  delay(dStop);
}

void controller(){
  //Forward Checking
  if(cm[4]<20&&!blockPickup){ //Distance to front before turning
    //in4Way=false;
    
    if(cm[4]==0){
      dontMove=true;
    }
    else if(!dontMove){
      while(cm[4]>17||cm[4]<13){
        if(cm[4]>17){
          takeVerySmallStep();
          detect();
        }
        else{
          takeVerySmallStepBack();
          detect();
        }
      }
      if(cm[2]>20){
        currDir=(currDir+3)%4;
        Serial1.println('b');
        delay(d90cw);
        Serial1.println('d');
        delay(dStop);
      }
      else if(cm[6]>20){
        currDir=(currDir+1)%4;
        Serial1.println('c');
        delay(d90cw);
        Serial1.println('d');
        delay(dStop);
      }
      else{
        currDir=(currDir+2)%4;
        Serial1.println('b');
        delay(d180cw);
        Serial1.println('d');
        delay(dStop);
      }
      resetError = true;
    }    
  }
  
  //Left Checking
  if (cm[6]<30&&!dontMove){
    //in4Way=false;
    
    leftPID();
    if(cm[6]==0){
      dontMove=true;
    }
    else if(outputL>0){
      Serial1.println('b');
      delay(abs(outputL));//small degree turn
      Serial1.println('d');
      delay(dStop);//To allow for slow stop
    }
    else if(outputL<0){
      Serial1.println('c');
      delay(abs(outputL));//small degree turn
      Serial1.println('d');
      delay(dStop); //To allow for slow stop
    }
  }
  //Right Checking --------- Make it Proportional
  else if(cm[2]<30&&!dontMove){//else if(cm[2]<30&&!dontMove){
    //in4Way=false;
    rightPID();
    if(cm[2]==0){
      dontMove=true;
    }
    else if(outputR>0){
      Serial1.println('c');
      delay(abs(outputR));//small degree turn
      Serial1.println('d');
      delay(dStop);//To allow for slow stop
    }
    else if(outputR<0){
      Serial1.println('b');
      delay(abs(outputR));//small degree turn
      Serial1.println('d');
      delay(dStop); //To allow for slow stop
    }
  }
  else if(!dontMove){
    if(cm[2]<cm[6]){
      rightPID();
      if(cm[2]==0){
        dontMove=true;
      }
      else if(outputR>0){
        Serial1.println('c');
        delay(abs(outputR));//small degree turn
        Serial1.println('d');
        delay(dStop);//To allow for slow stop
      }
      else if(outputR<0){
        Serial1.println('b');
        delay(abs(outputR));//small degree turn
        Serial1.println('d');
        delay(dStop); //To allow for slow stop
      }
    }
    else{
      leftPID();
      if(cm[6]==0){
        dontMove=true;
      }
      else if(outputL>0){
        Serial1.println('b');
        delay(abs(outputL));//small degree turn
        Serial1.println('d');
        delay(dStop);//To allow for slow stop
      }
      else if(outputL<0){
        Serial1.println('c');
        delay(abs(outputL));//small degree turn
        Serial1.println('d');
        delay(dStop); //To allow for slow stop
      }
    }
  }
  else{
  }
}


void openRTurn(){
  if(cm[4]<=cm[0]){
    if(cm[4]>particle[currLoc][currDir][2]+2||cm[4]<particle[currLoc][currDir][2]-2){
      if(cm[4]>particle[currLoc][currDir][2]){
        takeVerySmallStep();
      }
      else{
        takeVerySmallStepBack();
      }
    }
    else{
      currDir=(currDir+3)%4;
      Serial1.println('b');
      delay(d90cw);
      Serial1.println('d');
      delay(dStop);
      newLoc=false;
    }
  }
  else{
    if(cm[0]>particle[currLoc][currDir][0]+2||cm[0]<particle[currLoc][currDir][0]-2){
      if(cm[0]>particle[currLoc][currDir][0]){
        takeVerySmallStepBack();
      }
      else{
        takeVerySmallStep();
      }
    }
    else{
      currDir=(currDir+3)%4;
      Serial1.println('b');
      delay(d90cw);
      Serial1.println('d');
      delay(dStop);
      newLoc=false;
    }
  }
}


void openLTurn(){
  if(cm[4]<=cm[0]){
    if(cm[4]>particle[currLoc][currDir][2]+2||cm[4]<particle[currLoc][currDir][2]-2){
      if(cm[4]>particle[currLoc][currDir][2]){
        takeVerySmallStep();
      }
      else{
        takeVerySmallStepBack();
      }
    }
    else{
      currDir=(currDir+1)%4;
      Serial1.println('c');
      delay(d90cw);
      Serial1.println('d');
      delay(dStop);
      newLoc=false;
    }
  }
  else{
    if(cm[0]>particle[currLoc][currDir][0]+2||cm[0]<particle[currLoc][currDir][0]-2){
      if(cm[0]>particle[currLoc][currDir][0]){
        takeVerySmallStepBack();
      }
      else{
        takeVerySmallStep();
      }
    }
    else{
      currDir=(currDir+1)%4;
      Serial1.println('c');
      delay(d90cw);
      Serial1.println('d');
      delay(dStop);
      newLoc=false; 
    }
  }
}

//void openRTurn(){
//  if(cm[4]>particle[currLoc][currDir][2]+5||cm[4]<particle[currLoc][currDir][2]-5){
//    if(cm[4]>particle[currLoc][currDir][2]){
//      takeSmallStep();
//    }
//    else{
//      takeSmallStepBack();
//    }
//  }
//  else{
//    currDir=(currDir+3)%4;
//    Serial1.println('b');
//    delay(d90cw);
//    Serial1.println('d');
//    delay(dStop);
//  }
//}
//
//
//void openLTurn(){
//  if(cm[4]>particle[currLoc][currDir][2]+2.5||cm[4]<particle[currLoc][currDir][2]-2.5){
//    if(cm[4]>particle[currLoc][currDir][2]){
//      takeSmallStep();
//    }
//    else{
//      takeSmallStepBack();
//    }
//  }
//  else{
//    currDir=(currDir+1)%4;
//    Serial1.println('c');
//    delay(d90cw);
//    Serial1.println('d');
//    delay(dStop);
//  }
//}


void navigate(){
  //7=SW, 6=W, 5=NW, 4=N, 3=NE, 2=E, 1=SE, 0=S
  detect();
  printVal();

  checkPOI();
  controller();
  if(!dontMove){
    takeStep();
  }
  if(localized) displayDigit(currDir+20);
  dontMove=false;
}


void detect(){
  if(nextCW){
    for(int i=7;i>=0;i--){
      currentDir=i;
      valueGet=false;
      myStepper.step(stepsPerRevolution/8);
      
      trigger_pulse();   
      unsigned long breakTime = millis();
      while(!valueGet){
        unsigned long now = millis();
        if(now-breakTime>500){
          cm[currentDir]=0;
          break;
        }
      }
    }
  nextCW=false;
  }
  else{
    for(int i=0;i<8;i++){
      trigger_pulse();        
      unsigned long breakTime = millis();
      while(!valueGet){
        unsigned long now = millis();
        if(now-breakTime>500){
          cm[currentDir]=0;
          break;
        }
      }

      currentDir=i;
      valueGet=false;
      myStepper.step(-stepsPerRevolution/8);
    }
  nextCW=true;
  }
}


void fDetect(){
  trigger_pulse();   
  unsigned long breakTime = millis();
  while(!valueGet){
    unsigned long now = millis();
    if(now-breakTime>500){
      cm[4]=0;
      break;
    }
  }
}


void findBlock(){
//  if(nextCW){
//    myStepper.step(stepsPerRevolution/2);
//  }
//  else{
//    myStepper.step(-stepsPerRevolution/2);
//  }
  while(!gripFlag){
    if(cm[4]>20){
      //read signal from Lower Ultrasonic
      digitalWrite(trigL, LOW); // Clears the trigPin
      delayMicroseconds(2); 
      digitalWrite(trigL, HIGH); // Sets the trigPin on HIGH state for 10 micro seconds
      delayMicroseconds(10);
      digitalWrite(trigL, LOW);
      durL = pulseIn(echoL, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
      // Calculating the distance
      distL= durL*0.034/2;
      distU= cm[4];
      // Prints the distance on the Serial Monitor
      Serial.print("Distance L:  ");
      Serial.println(distL);
      Serial.print("Distance U: ");
      Serial.println(distU);
      Serial.println("--------------");
      delay(300);
     
      if ( (distL<=9)&&(distL>=6)&&(distU>=16)){
        //Serial.println("success");
        gripFlag = true;
        blockDetected=true;
        displayDigit(11);
      }
      else if(distL<6){
        takeSmallStepBack();
        blockDetected=true;
        displayDigit(11);
      }
      else{
        detect();
        printVal();
      
        //checkPOI();
        if(!blockDetected){
          controller();
        }
        takeSmallStep();
        dontMove = false;
      }
    }
    else{
      takeSmallStepBack();
      gripFlag = true;
      blockDetected=true;
      displayDigit(11);

      detect();
      printVal();
      controller();
      takeSmallStep();
      dontMove = false;
    }
  }
  
//  if(nextCW){
//    myStepper.step(-stepsPerRevolution/2);
//  }
//  else{
//    myStepper.step(stepsPerRevolution/2);
//  }
}


void printVal(){
  Serial.print(cm[5]);//Serial.print("/");Serial.print(ir[5]);
  Serial.print(" ");
  Serial.print(cm[4]);//Serial.print("/");Serial.print(ir[4]);
  Serial.print(" ");
  Serial.print(cm[3]);//Serial.print("/");Serial.print(ir[3]);
  Serial.println();
  Serial.print(cm[6]);//Serial.print("/");Serial.print(ir[6]);
  Serial.print("   ");
  Serial.print(cm[2]);//Serial.print("/");Serial.print(ir[2]);
  Serial.println();
  Serial.print(cm[7]);//Serial.print("/");Serial.print(ir[7]);
  Serial.print(" ");
  Serial.print(cm[0]);//Serial.print("/");Serial.print(ir[0]);
  Serial.print(" ");
  Serial.print(cm[1]);//Serial.print("/");Serial.print(ir[1]);
  Serial.println();
  Serial.println();
}


void trigger_pulse()
{
    digitalWrite(trigPin, LOW);  // Added this line
    delayMicroseconds(2); // Added this line
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10); // Added this line
    digitalWrite(trigPin, LOW);
}


// --------------------------
// echo_interrupt() External interrupt from HC-SR04 echo signal. 
// Called every time the echo signal changes state.
//
// Note: this routine does not handle the case where the timer
//       counter overflows which will result in the occassional error.
// --------------------------
void echo_interrupt()
{
  switch (digitalRead(echoPin))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_end = 0;                                 // Clear the end time
      echo_start = micros();                        // Save the start time
      break;
      
    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_end = micros();                          // Save the end time
      echo_duration = echo_end - echo_start;        // Calculate the pulse duration
      //if(!blockPickup){
        if(echo_duration/58<1000){
          cm[currentDir]=echo_duration/58;
        }
        else{
          cm[currentDir]=0;
        }
      //}
//      else{
//        if(echo_duration/58<1000){
//          cm[4]=echo_duration/58;
//        }
//        else{
//          cm[4]=0;
//        }
//      }
      valueGet=true;
      break;
  }
}


void setup() {
  Serial.begin(9600);
  Serial1.begin(19200);

  myStepper.setSpeed(60);

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  delay(3000); //Startup Delay
  //forward();
  attachInterrupt(digitalPinToInterrupt(echoPin), echo_interrupt, CHANGE);  // Attach interrupt to the sensor echo input

  pinMode(a, OUTPUT);  //A
  pinMode(b, OUTPUT);  //B
  pinMode(c, OUTPUT);  //C
  pinMode(d, OUTPUT);  //D
  pinMode(e, OUTPUT);  //E
  pinMode(f, OUTPUT);  //F
  pinMode(g, OUTPUT);  //G

  // Setup variables for gripper mechanism
  stepper1.setMaxSpeed(1500.0);
  stepper1.moveTo(pst);
  stepper1.setSpeed(spd);
  pinMode(trigL, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoL, INPUT); // Sets the echoPin as an Input 

  poi = false;
  count=0;
  dontMove = false;
  resetError = false;
  //in4Way = false;
  d4Way = 600;
  nextCW = true;
  gripFlag=false;
  blockPickup = false;
  haveBlock = false;
  dropBlock = false;
  blockDetected=false;
  endd = false;
  newLoc = false;
  checkingSideLeft = false;
  localized = false;
}


void checkPOI(){
  poi=false;
  if(cm[4]>30){
    if((cm[2]>30&&cm[6]>30)||(cm[2]>30&&cm[0]>30)||(cm[6]>30&&cm[0]>30)){
        poi=true;
    }
  }
  else if(cm[2]>30&&cm[6]>30&&cm[0]>30){
    poi=true;
  }
  
  if(poi){
    //Find which one we're at
    for(int i=0;i<5;i++){
      for(int j=0;j<4;j++){
        bool match[4] = {false,false,false,false};
        for(int k=0;k<4;k++){
          if(cm[2*k]<particle[i][j][k]+15&&cm[2*k]>particle[i][j][k]-15){
            match[k]=true;
          }
          else if(i==1&&((j+k)%4==1)&&cm[2*k]>60){
            match[k]=true;
          }
        }
        if(match[0]&&match[1]&&match[2]&&match[3]){
          if(guessLoc==i||i==0){
            currDir=j;
            currLoc=i;
            displayDigit(currLoc);   
            newLoc = true;
            localized = true;
          }
          else{
            guessDir=j;
            guessLoc=i;
          }
        }
      }
    }
    //Make Decision
    if(!haveBlock&&newLoc){
      switch(decision[currLoc][currDir][0]){
        case 0: dontMove=false; newLoc=false; break;
        case 1: openRTurn(); displayDigit(14); dontMove=true; break;
        case 2: openLTurn(); displayDigit(15); dontMove=true; break;
        case 3: 
          currDir=(currDir+2)%4;
          Serial1.println('b');
          delay(d180cw);
          Serial1.println('d');
          delay(dStop);
          newLoc=false;
          break;
        case 4: blockPickup=true; dontMove=true; resetError=true; newLoc=false; center(); break;
      }
    }
    else if(newLoc){
      switch(decision[currLoc][currDir][1+caseModifier]){
        case 0: dontMove=false; newLoc=false; break;
        case 1: openRTurn(); displayDigit(14); dontMove=true; break;
        case 2: openLTurn(); displayDigit(15); dontMove=true; break;
        case 3: 
          currDir=(currDir+2)%4;
          Serial1.println('b');
          delay(d180cw);
          Serial1.println('d');
          delay(dStop);
          newLoc=false;
          break;
        case 4: dropBlock = true; dontMove=true; newLoc=false; break;
      }
    }
  }
}
void displayDigit(int digit)
{
  turnOff();
  //North on back/left/front/right
  if(digit<0){}
  else if(digit==20)
    digitalWrite(d,HIGH);
  else if(digit==21)
    digitalWrite(e,HIGH);
  else if(digit==22)
    digitalWrite(g,HIGH);
  else if(digit==23)
    digitalWrite(c,HIGH);
  //block detected or placed
  else if(digit==11){
    digitalWrite(a,LOW);
    digitalWrite(b,LOW);
    digitalWrite(c,HIGH);
    digitalWrite(d,HIGH);
    digitalWrite(e,HIGH);
    digitalWrite(f,LOW);
    digitalWrite(g,HIGH);
  }
  //block lifted
  else if(digit==12){
    digitalWrite(a,HIGH);
    digitalWrite(b,HIGH);
    digitalWrite(c,LOW);
    digitalWrite(d,LOW);
    digitalWrite(e,LOW);
    digitalWrite(f,HIGH);
    digitalWrite(g,HIGH);
  }
  //PID
  else if(digit==13){
    digitalWrite(a,LOW);
    digitalWrite(b,HIGH);
    digitalWrite(c,HIGH);
    digitalWrite(d,LOW);
    digitalWrite(e,HIGH);
    digitalWrite(f,HIGH);
    digitalWrite(g,LOW);
  }
  //OpenRTurn
    else if(digit==14){
    digitalWrite(a,LOW);
    digitalWrite(b,HIGH);
    digitalWrite(c,HIGH);
    digitalWrite(d,LOW);
    digitalWrite(e,LOW);
    digitalWrite(f,LOW);
    digitalWrite(g,LOW);
  }
   //OpenLTurn
    else if(digit==15){
    digitalWrite(a,LOW);
    digitalWrite(b,LOW);
    digitalWrite(c,LOW);
    digitalWrite(d,LOW);
    digitalWrite(e,HIGH);
    digitalWrite(f,HIGH);
    digitalWrite(g,LOW);
  }
  else{
    //Conditions for displaying segment a
    if(digit!=1 && digit != 4)
    digitalWrite(a,HIGH);
    
    //Conditions for displaying segment b
    if(digit != 5 && digit != 6)
    digitalWrite(b,HIGH);
    
    //Conditions for displaying segment c
    if(digit !=2)
    digitalWrite(c,HIGH);
    
    //Conditions for displaying segment d
    if(digit != 1 && digit !=4 && digit !=7)
    digitalWrite(d,HIGH);
    
    //Conditions for displaying segment e 
    if(digit == 2 || digit ==6 || digit == 8 || digit==0)
    digitalWrite(e,HIGH);
    
    //Conditions for displaying segment f
    if(digit != 1 && digit !=2 && digit!=3 && digit !=7)
    digitalWrite(f,HIGH);
    if (digit!=0 && digit!=1 && digit !=7)
    digitalWrite(g,HIGH);
  }
}
void turnOff()
{
  digitalWrite(a,LOW);
  digitalWrite(b,LOW);
  digitalWrite(c,LOW);
  digitalWrite(d,LOW);
  digitalWrite(e,LOW);
  digitalWrite(f,LOW);
  digitalWrite(g,LOW);
}
void gripperRun() 
{
    stepper1.runSpeed();
    if (stepper1.distanceToGo() == 0) {
      Serial.println("stop");
      if (stepper1.currentPosition() >= pst) {
        stepper1.moveTo(1);
        gripCount++;
      } else {
        stepper1.moveTo(pst);
        gripCount++;
      }
      //stepper1.moveTo(-stepper1.currentPosition());
      stepper1.setSpeed(-spd);
      spd = -spd;

      if (gripCount>=2) {
        if(dropBlock){
          endd=true;
        }
        gripCount = 0;
        gripFlag = false;
        
        digitalWrite(trigL, LOW); // Clears the trigPin
        delayMicroseconds(2); 
        digitalWrite(trigL, HIGH); // Sets the trigPin on HIGH state for 10 micro seconds
        delayMicroseconds(10);
        digitalWrite(trigL, LOW);
        durL = pulseIn(echoL, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
        // Calculating the distance
        distL2= durL*0.034/2;
        if(distL2>distL+5){ //*********************************************************************************************Needs new condition
          blockPickup=false;
          haveBlock=true;
          if(!dropBlock){
            displayDigit(12);
          }
          
          //TODO: Cases for after block picked up
        }
        else{
          takeSmallStepBack();
          takeSmallStepBack();
        }
      }
      
      digitalWrite(motorPin1, LOW); 
      digitalWrite(motorPin2, LOW);
      digitalWrite(motorPin3, LOW);
      digitalWrite(motorPin4, LOW);   
      delay(1000); 
      if(gripCount==1){
        if(dropBlock){
          Serial1.println('b');
          delay(200);
          Serial1.println('d');
          delay(dStop);
        }
        else{
           takeSmallStep();
           //takeSmallStep();
        }
      }
    }
}

void center(){
  if(currLoc==4){
    Serial1.println('b');
    delay(d90cw);
    Serial1.println('d');
    delay(dStop);
  }
  else if(currLoc==3){
    Serial1.println('c');
    delay(d90cw);
    Serial1.println('d');
    delay(dStop);
  }
  detect();
  while(cm[4]>16||cm[4]<14){
    if(cm[4]>16){
      takeVerySmallStep();
      detect();
    }
    else{
      takeVerySmallStepBack();
      detect();
    }
  }
  if(currLoc==3){
    Serial1.println('b');
    delay(d90cw);
    Serial1.println('d');
    delay(dStop);
  }
  else if(currLoc==4){
    Serial1.println('c');
    delay(d90cw);
    Serial1.println('d');
    delay(dStop);
  }
  detect();
}

void loop() {
  if(!endd){
    //blockPickup=true;
    //haveBlock=true;
  
    if(blockPickup){
      if(blockDetected){
        displayDigit(11); //For Testing
      }
      else{
        turnOff();
      }
      if(gripFlag){  
        gripperRun();
      }
      else{
        findBlock();
      }
    }
    else if(dropBlock){
        displayDigit(11); 
      //TODO: Block drop code
      if(!gripFlag){
        if(caseModifier==3){
//          switch(currDir){
//            case 0: break;
//            case 1: openRTurn(); dontMove=true; break;
//            case 2: 
//              currDir=(currDir+2)%4;
//              Serial1.println('b');
//              delay(d180cw);
//              Serial1.println('d');
//              delay(dStop);
//              break;
//            case 3: openLTurn(); dontMove=true; break;
//          }
          blockPickup=true;

          for(int i=0;i<6;i++){
            detect();
            controller();
            takeStep();
          }

          blockPickup=false;
          
        }
        else{
//          switch(currDir){
//            case 0: 
//              currDir=(currDir+2)%4;
//              Serial1.println('b');
//              delay(d180cw);
//              Serial1.println('d');
//              delay(dStop);
//              break;
//            case 1: openLTurn(); dontMove=true; break;
//            case 2: break;
//            case 3: openRTurn(); dontMove=true; break;
//          }
          blockPickup=true;
          
          detect();
          controller();
          takeStep();
  
          detect();
          //controller(); //For Testing Remove
          takeStep();
          blockPickup=false;
        }
        gripFlag=true;
      }
      else{
        gripperRun();
      }
    }
    else{
      navigate();
    }
  }
}

