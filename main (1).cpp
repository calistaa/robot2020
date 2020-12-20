#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <NewPing.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
#define MAX_DISTANCE 100

//motor
#define MOTOR_1A PA_7
#define MOTOR_1B  PB_0

#define MOTOR_2A PB_1
#define MOTOR_2B PA_6

//servo
#define GATE_SERVO PB9
#define CLAW_SERVO PA8
#define ARM_SERVO PB8
Servo armServo;
const int A=1;
int armUp=70,armHome=170,armDown=180;
int armCurr;

Servo clawServo;
const int C=2;
int clawClosed=150,clawOpen=0,clawHome=45;
int clawCurr;


Servo gateServo;
const int G=3;
int gateClosed=0,gateOpen=90;
int gateCurr;


//sonar
#define TRIGGER PB14
 #define ECHO PB15

 //tape
 #define REAR_SENSOR PA3
 #define TAPE_LEFT PA5
 #define TAPE_RIGHT PA4
#define TAPE_MIDDLE PA2

//misc
 #define CAN_SENSOR PB10
#define TURNPOT PA1
#define BUTTON PB3

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
NewPing sonar(TRIGGER, ECHO, MAX_DISTANCE);


int getDistance();
void drive(int one,int two);
void moveServo(int val,int pos);
bool onTape();
void tapeFollow();

int speed1=317;
 int speed2=277;
void setup(){
  pinMode(MOTOR_1A, OUTPUT);
  pinMode(MOTOR_1B, OUTPUT);
  pinMode(MOTOR_2A, OUTPUT);
  pinMode(MOTOR_2B, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TURNPOT, INPUT_ANALOG);

  // Tape sensors
  pinMode(TAPE_LEFT, INPUT_ANALOG);
  pinMode(TAPE_MIDDLE, INPUT_ANALOG);
  pinMode(TAPE_RIGHT, INPUT_ANALOG);

  armServo.attach(ARM_SERVO);
  gateServo.attach(GATE_SERVO);
  clawServo.attach(CLAW_SERVO);
    armCurr=armHome;
  gateCurr=gateClosed;
  clawCurr=clawOpen;
  armServo.write(armHome);
  gateServo.write(gateOpen);
  clawServo.write(clawHome);


  Serial.begin(9600);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

}


void loop(){

  float distance=999999;
  moveServo(C,clawHome);
if(digitalRead(BUTTON)){
  /**
 delay(1000);

 //drive out
 drive(speed1,speed2);
 delay(1000);
 drive(0,0);
 delay(200);**/

  bool edge=false;
  int count=0;
    while(!edge&&count<2){//loop on each can
    count++;

      //find can
     bool found=false;
     //spin for can
  
       int  cycles=0;//timeout sequence
          while(!found){//cycles<100&&
          cycles++;
           drive(0,0);
          delay(60);
          drive(0,speed2*1.4);//spin
       

            display.clearDisplay();
       display.setCursor(0, 0);
     display.print("Spinning: ");
      display.println(distance);
     display.display();
       distance=getDistance();
    
       if(distance<75){
         found=true;
         break;
       }
    }

//slow motor
    drive(0,0);
    delay(400);
    
    //drive to can
      drive(speed1,speed2);
    while(distance>10&&!edge){
       if(onTape()){
          
               edge=true;
              break;
            }
      display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Homing: ");
    display.println(distance);
  display.display();
      delay(1);
      distance=getDistance();
    }
    if(!edge){//grab can sequence
    drive(0,0);
    moveServo(A,armDown);
    moveServo(C,clawClosed);
    moveServo(G,gateClosed);
    delay(200);
    moveServo(A,armUp);
     delay(800);
     moveServo(C,clawOpen);
       delay(800);
        moveServo(A,armHome);
        delay(1000);
         }

         }
        moveServo(C,clawClosed);
         //drive to tape
      while(!edge){
        drive(speed1,speed2);
        if(onTape()){
               edge=true;
              break;
            }

      }


         //tape to bin
         while(!digitalRead(BUTTON)){
             tapeFollow();
         if(analogRead(REAR_SENSOR)<55)break;//docked

          }

        //dump can
        drive(0,0);
        delay(100);
        moveServo(G,gateOpen);
        delay(2000);
}
}
int left_vals[] = {0, 0, 0};
int right_vals[] = {0, 0, 0};
int mid_vals[] = {0, 0, 0};
int tape_index = 0;
int tape_speed = -250;
int tape_left = 0;
int tape_right = 0;
int tape_middle = 0;
int prev_tape[] = {0, 0, 0};
int prevSide = 0; // left: -1, right: 1
int error = 0;
int derivative = 0;
int integral = 0;
int prevError = 0;
int tape_thresholds[3]={80,30,80}; // Calibrated threshold values
int tape_max[3];
bool tape_detected[] = {0, 0, 0};
double tape_pid[] = {25, 200, 0}; // kp, kd, ki //250
int speedInc = 0;


bool onTape(){
/**
    left_vals[tape_index] = analogRead(TAPE_LEFT);
  mid_vals[tape_index] = analogRead(TAPE_MIDDLE);
  right_vals[tape_index] = analogRead(TAPE_RIGHT);
    tape_index = (tape_index + 1) % 3;

  int tape_thresholds[3]={80,35,80}; // Calibrated threshold values
  bool tape_detected[] = {0, 0, 0};
  tape_left = (left_vals[0] + left_vals[1] + left_vals[2]) / 3;
  tape_middle = (mid_vals[0] + mid_vals[1] + mid_vals[2]) / 3;
  tape_right = (right_vals[0] + right_vals[1] + right_vals[2]) / 3;

   tape_detected[0] = tape_thresholds[0] < tape_left;
  tape_detected[1] = tape_thresholds[1]>  tape_middle;
  tape_detected[2] = tape_thresholds[2] < tape_right;
  return (tape_detected[0]||tape_detected[1]||tape_detected[2]);
  **/
   tape_detected[0] = tape_thresholds[0] < analogRead(TAPE_LEFT);
  tape_detected[1] = tape_thresholds[1]>   analogRead(TAPE_MIDDLE);
  tape_detected[2] = tape_thresholds[2] <  analogRead(TAPE_RIGHT);
  return (tape_detected[0]||tape_detected[1]||tape_detected[2]);
}

void tapeFollow(){
  
  // Read tape values - running average of last 3 values
  left_vals[tape_index] = analogRead(TAPE_LEFT);
  mid_vals[tape_index] = analogRead(TAPE_MIDDLE);
  right_vals[tape_index] = analogRead(TAPE_RIGHT);
  tape_index = (tape_index + 1) % 3;
  tape_left = (left_vals[0] + left_vals[1] + left_vals[2]) / 3;
  tape_middle = (mid_vals[0] + mid_vals[1] + mid_vals[2]) / 3;
  tape_right = (right_vals[0] + right_vals[1] + right_vals[2]) / 3;

  tape_detected[0] = tape_left > tape_thresholds[0];
  tape_detected[1] = tape_middle < tape_thresholds[1];
  tape_detected[2] = tape_right > tape_thresholds[2];

 // To left -> negative, to right -> postive
  if ((!tape_detected[0] && tape_detected[1] && !tape_detected[2])){ // middle only
    error = 0;
  }else if(tape_detected[0] && tape_detected[2]){//both sides, AKA orthognal |drastic countermeasure and somewhat arbitrary to pick left
     if(prevSide==0)prevSide = 1;
       error=prevSide*10;

   }else if (tape_detected[0] || tape_detected[2]){ // something on
        error = tape_detected[0] * -3 + tape_detected[2] * 3;
      if (tape_detected[1])
          error /= 2;
     if (tape_detected[0])
      prevSide = -1;
    if (tape_detected[2])
      prevSide = 1;
  
  }else{//none
  error=prevSide*5;
}
 
//10>error>-10
  derivative = error - prevError;
  integral = error + integral;

  // Windup
  if (integral > 50){
    integral = 50;
  }else if (integral < -50){
    integral = -50;
  }

  prevError = error;

  //prev_tape[0] = tape_left;
  //prev_tape[1] = tape_middle;
  //prev_tape[2] = tape_right;

  speedInc = error * tape_pid[0] + derivative * tape_pid[1] + integral * tape_pid[2];
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("L: ");
  display.print(tape_detected[0]);
   display.print(" _ ");
    display.println(tape_left);
    display.print("M: ");
  display.print(tape_detected[1]);
   display.print(" _ ");
    display.println(tape_middle);
   display.print("R: ");
  display.print(tape_detected[2]);
   display.print(" _ ");
    display.println(tape_right);
        display.print("incr: ");
                display.println(speedInc);
  display.display();
  // Avoid switching directions!!
  /**
  if (speedInc > -tape_speed){
    speedInc = -tape_speed;
  }else if (speedInc < tape_speed){
    speedInc = tape_speed;
  }**/
   drive((tape_speed - speedInc),(tape_speed + speedInc));
}


void moveServo(int val,int pos){
  Servo servo;
  int curr=0;
  switch(val){
    case A:
    servo=armServo;
    curr=armCurr;
    armCurr=pos;
    break;
     case G:
    servo=gateServo;
    curr=gateCurr;
    gateCurr=pos;
    break;
     case C:
    servo=clawServo;
    curr=clawCurr;
     clawCurr=pos;
    break;
  }
  servo.write(pos);
  
}

int getDistance()
{
  delay(20);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  int dist =sonar.convert_cm(uS);
 if(dist==0){
   dist=999999;
 }
 return dist;
}

void drive(int one, int two){//- backwards, + forwards
  
    
      if(one>0){
     pwm_start(MOTOR_1A,512, one,RESOLUTION_10B_COMPARE_FORMAT);
     pwm_start(MOTOR_1B,512, 0,RESOLUTION_10B_COMPARE_FORMAT);
      }else if(one<0){
     pwm_start(MOTOR_1A,512, 0,RESOLUTION_10B_COMPARE_FORMAT);
     pwm_start(MOTOR_1B,512, -one,RESOLUTION_10B_COMPARE_FORMAT);
      }else{
     pwm_start(MOTOR_1A,512, 0,RESOLUTION_10B_COMPARE_FORMAT);
     pwm_start(MOTOR_1B,512, 0,RESOLUTION_10B_COMPARE_FORMAT);
      }

    if(two>0){
     pwm_start(MOTOR_2A,512, two,RESOLUTION_10B_COMPARE_FORMAT);
     pwm_start(MOTOR_2B,512, 0,RESOLUTION_10B_COMPARE_FORMAT);
      }else if(two<0){
     pwm_start(MOTOR_2A,512, 0,RESOLUTION_10B_COMPARE_FORMAT);
     pwm_start(MOTOR_2B,512, -two,RESOLUTION_10B_COMPARE_FORMAT);
      }else{
     pwm_start(MOTOR_2A,512, 0,RESOLUTION_10B_COMPARE_FORMAT);
     pwm_start(MOTOR_2B,512, 0,RESOLUTION_10B_COMPARE_FORMAT);
      }
}