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

//Declare variables for functions
double accel_x, accel_y, accel_z;
//initialize motors
Motor inner(STEP_1, DIR_1, MS1_1, MS2_1, EN_1);
Motor outer(STEP_2, DIR_2, MS1_2, MS2_2, EN_2);

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);
  Serial.println("Begin motor control");
  Serial.println();
  //Print function list for user selection
  Serial.println("Enter command in form:"); 
  Serial.println("[motor(inner/outer) direction(ccw/cw) angle(degrees) stepResolution(1/2/4/8)");
  Serial.println();
}

void loop() {
  //parse command input
  String commandArray[3]; //[move motor angle] for individual movement
                          // or [point elevation azimuth] for pointing
  int i = 0;
  while(Serial.available()){
    String tmp = Serial.readStringUntil(" ");
    commandArray[i] = tmp;
    i++;
  }

  //act based on filled command array
  if (commandArray[0] == "point") {
    point(commandArray[0].toDouble(), commandArray[1].toDouble());
  }
  else if (commandArray[0] == "move"){
    
    if (commandArray[1] == "outer"){
      outer.moveMotor(commandArray[2].toDouble());
    }
    else {
      inner.moveMotor(commandArray[2].toDouble());
    }
      
  }
  else if (commandArray[0] == "set"){
    if(commandArray[1] == "outer"){
      outer.setResolution(commandArray[2].toInt());
    }
    else if(commandArray[1] == "inner"){
      inner.setResolution(commandArray[2].toInt());
    }
  }
  else {
    Serial.println("please enter a valid command");
    Serial.println("[set motor resolution] to set a motor's resolution");
    Serial.println("[move motor angle] for individual movement");
    Serial.println(" or [point elevation azimuth] for pointing");
  }
  
}

bool point(double elevation, double azimuth){
  double coords[] = {sin(elevation) * cos(azimuth), sin(elevation)*sin(azimuth), cos(elevation) };
  double theta = asin(-coords[1]);
  double phi = asin(coords[0] / cos(theta));
  double outerRot = theta - outer.getPos();
  double innerRot = phi - inner.getPos();
  outer.setPos(theta);
  inner.setPos(phi);
  outer.moveMotor(outerRot);
  inner.moveMotor(innerRot);
}