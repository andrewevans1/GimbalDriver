#include <GimbalDriver.h>

//Declare pin functions on Arduino
#define STEP_1 10
#define DIR_1  9
#define MS1_1  12
#define MS2_1  11
#define EN_1   13
#define STEP_2 6
#define DIR_2  5
#define MS1_2  8
#define MS2_2  7
#define EN_2   4

#define ACC_X 0
#define ACC_Y 1
#define ACC_Z 2

#define FULL_STEP 1
#define HALF_STEP 2
#define QUARTER_STEP 4
#define EIGHTH_STEP 8

#define MAX_ANGLE 30.0
//Declare variables for functions
double accel_x, accel_y, accel_z;
//initialize motors
Motor inner(STEP_1, DIR_1, MS1_1, MS2_1, EN_1, MAX_ANGLE);
Motor outer(STEP_2, DIR_2, MS1_2, MS2_2, EN_2, MAX_ANGLE);
Motor * pointerToInner = &inner;
Motor * pointerToOuter = &outer;
MotorController motorController(pointerToInner, pointerToOuter);

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);
  Serial.println("Begin motor control");
  Serial.println();

  //Print function list for user selection
  Serial.println("Enter command");
  Serial.println();
}

void loop() {
  //parse command input
  String commandArray[3]; //[move motor angle] for individual movement
                          // or [point elevation azimuth] for pointing
  while (!Serial.available()){
    ;
  }

  while(Serial.available()){
    //String tmp = Serial.readStringUntil(' ');
    //commandArray[i] = tmp;
    //Serial.println(tmp);
    //i++;
    commandArray[0] = Serial.readStringUntil(' ');
    commandArray[1] = Serial.readStringUntil(' ');
    commandArray[2] = Serial.readStringUntil(' ');   
  }

  Serial.print(commandArray[0]); Serial.print(' ');
  Serial.print(commandArray[1]); Serial.print(' ');
  Serial.println(commandArray[2]);
  //act based on filled command array
  if (commandArray[0] == "point"){
    //Serial.print("1: " + commandArray[1]); Serial.println("2: " + commandArray[2]);
    motorController.point(commandArray[1].toDouble(), commandArray[1].toDouble());
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
  else {
    Serial.println("please enter a valid command");
    Serial.println("[set motor resolution] to set a motor's resolution");
    Serial.println("[move motor angle] for individual movement");
    Serial.println("or [point elevation azimuth] for pointing");
  }
  Serial.println();
}