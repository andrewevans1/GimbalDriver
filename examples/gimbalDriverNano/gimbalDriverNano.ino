#include <GimbalDriver.h>

//Declare pin assignments on Arduino Nano
#define STEP_1 8
#define DIR_1  7
#define MS1_1  9
#define MS2_1  10
#define EN_1   11

#define STEP_2 3
#define DIR_2  2
#define MS1_2  4
#define MS2_2  5
#define EN_2   6

//analog pins
#define ACC_X 0 
#define ACC_Y 1
#define ACC_Z 2

//constants
#define FULL_STEP 1
#define HALF_STEP 2
#define QUARTER_STEP 4
#define EIGHTH_STEP 8

#define MAX_ANGLE 30.0 //defined by structure
//observed necessary offsets for accelerometer
#define OFFSET_X 0 
#define OFFSET_Y 0
#define OFFSET_Z 0
//Declare variables for functions
double x, y, z, mag, norm_x, norm_y, norm_z;

//initialize motors
Motor inner(STEP_1, DIR_1, MS1_1, MS2_1, EN_1, MAX_ANGLE);
Motor outer(STEP_2, DIR_2, MS1_2, MS2_2, EN_2, MAX_ANGLE);
Motor * pointerToInner = &inner;
Motor * pointerToOuter = &outer;
MotorController motorController(pointerToInner, pointerToOuter);

void balance(int loops);

//program specific pin assignments for switch
int BAlANCE_MODE = 12;
int SPIN_DEMO = 13;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);
  analogReference(EXTERNAL); //set anolog reference to 3v3

  Serial.println("Begin motor control");
  Serial.println();

  //Print function list for user selection
  Serial.println("Enter command");
  Serial.println();
  pinMode(BAlANCE_MODE, INPUT);
  pinMode(SPIN_DEMO, INPUT);
}

void loop() {
  
  //take accelerometer data and react to keep gimbal level
  if (digitalRead(BAlANCE_MODE) == HIGH) {
    Serial.println("balancing");
    balance(1);
  }
  
  //follow demo path to show capabilities
  else if (digitalRead(SPIN_DEMO) == HIGH){
    Serial.println("spin demo");
    motorController.spin(25, 1);    
  }

  //full functionality -- direct commanding
  else {
    Serial.println("Please input a command");
    //parse command input
    String commandArray[3]; //[move motor angle] for individual movement
                            // or [point elevation azimuth] for pointing
    //hang until user input
    while (!Serial.available()){
      ;
    }

    //split commands on space
    while(Serial.available()){
      commandArray[0] = Serial.readStringUntil(' ');
      commandArray[1] = Serial.readStringUntil(' ');
      commandArray[2] = Serial.readStringUntil(' ');   
    }

    //repeat command to user
    Serial.print(commandArray[0]); Serial.print(' ');
    Serial.print(commandArray[1]); Serial.print(' ');
    Serial.println(commandArray[2]);

    //act based on filled command array
    if (commandArray[0] == "point"){
      //Serial.print("1: " + commandArray[1]); Serial.println("2: " + commandArray[2]);
      motorController.point(commandArray[1].toDouble(), commandArray[2].toDouble());
    }
    else if (commandArray[0] == "move"){
      
      if (commandArray[1] == "outer"){
        outer.moveMotor(commandArray[2].toDouble());
        //Serial.println("got it");
      }
      else if (commandArray[1] == "inner"){
        inner.moveMotor(commandArray[2].toDouble());
      }
      else {
        Serial.println("please enter valid motor selection");
      }
        
    }
    else if (commandArray[0] == "set"){
      if(commandArray[1] == "outer"){
        outer.setResolution(commandArray[2].toInt());
      }
      else if(commandArray[1] == "inner"){
        inner.setResolution(commandArray[2].toInt());
      }
      else {
        Serial.println("please enter valid motor selection");
      }
    }
    else if (commandArray[0] == "oscillate"){
      if(commandArray[1] == "outer"){
        outer.oscillate(MAX_ANGLE, commandArray[2].toDouble());  
      }
      else if(commandArray[1] == "inner"){
        inner.oscillate(MAX_ANGLE, commandArray[2].toDouble());
      }
      else {
        Serial.println("please enter valid motor selection");
      }
    }
    else if (commandArray[0] == "reset"){
      motorController.reset();
    }
    else if (commandArray[0] == "get"){
      if(commandArray[1] == "outer"){
        Serial.println(outer.getState());  
      }
      else if(commandArray[1] == "inner"){
        Serial.println(inner.getState());  
      }
      else {
        Serial.println("please enter valid motor selection");
      }
    }
    else if (commandArray[0] == "spin") {
      motorController.spin(commandArray[1].toDouble(), commandArray[2].toDouble());
    }
    else if (commandArray[0] == "osc") {
      motorController.oscillate(commandArray[1].toDouble(), commandArray[2].toDouble());
    }
    else if (commandArray[0] == "path"){
      motorController.point(20, 0); delay(500);
      motorController.point(20, 45); delay(500);
      motorController.point(20, 90); delay(500);
      motorController.point(20, 135); delay(500);
      motorController.point(20, 180); delay(500);
      motorController.point(20, -135); delay(500);
      motorController.point(20, -90); delay(500);
      motorController.point(20, -45); delay(500);
      motorController.point(20, 0); delay(500);
      motorController.reset();
    }
    else if (commandArray[0] == "balance"){
      int loops = commandArray[1].toInt();
      Serial.print("loops: "); Serial.println(loops);
      for(int i = 0; i < loops; i++){
        x = analogRead(0);       // read analog input pin 0
        y = analogRead(1);       // read analog input pin 1
        z = analogRead(2);       // read analog input pin 20
        x = x - OFFSET_X;
        y = y - OFFSET_Y;
        z = z - OFFSET_Z;
        mag = sqrt(x*x + y*y + z*z);
        norm_x = x/mag;
        norm_y = y/mag;
        norm_z = z/mag;
        Serial.print("Vector: ");
        Serial.print(norm_x);
        Serial.print(" ");
        Serial.print(norm_y);
        Serial.print(" ");
        Serial.println(norm_z);
        motorController.point(norm_x, norm_y, norm_z);
        delay(500);
      }
    }
    else {
      Serial.println("please enter a valid command");
      Serial.println("[set motor resolution] to set a motor's resolution");
      Serial.println("[move motor angle] for individual movement");
      Serial.println("or [point elevation azimuth] for pointing");
    }
    Serial.println();
  }
}

void balance(int loops) {
  Serial.print("loops: "); Serial.println(loops);
  for(int i = 0; i < loops; i++){
    x = analogRead(ACC_X);       // read analog input pin 0
    y = analogRead(ACC_Y);       // read analog input pin 1
    z = analogRead(ACC_Z);       // read analog input pin 20
    x = x - OFFSET_X;
    y = y - OFFSET_Y;
    z = z - OFFSET_Z;
    mag = sqrt(x*x + y*y + z*z);
    norm_x = x/mag;
    norm_y = y/mag;
    norm_z = z/mag;
    Serial.print("Vector: ");
    Serial.print(norm_x);
    Serial.print(" ");
    Serial.print(norm_y);
    Serial.print(" ");
    Serial.println(norm_z);
    motorController.point(norm_x, norm_y, norm_z);
    delay(100);
  }
}