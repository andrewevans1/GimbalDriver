//TODO: redo user commands
//TODO: move the functions to a header file, leave this purely as main function
//TODO: make header file/functions for angle stuff
//TODO: data structure for each motor to track pin states and clean up code

#include "Arduino.h"

#define OUTER  1
#define INNER  2
#define COUNTERCLOCKWISE 1
#define CLOCKWISE        2

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
char user_input;
int x_1, x_2, y_1, y_2;
int state;
int resolution_1;
int resolution_2;
double innerMotorPos = 0;
double outerMotorPos = 0;
double accel_x, accel_y, accel_z;

void setup() {
  pinMode(STEP_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);
  pinMode(MS1_1, OUTPUT);
  pinMode(MS2_1, OUTPUT);
  pinMode(EN_1,  OUTPUT);
  pinMode(STEP_2, OUTPUT);
  pinMode(DIR_2, OUTPUT);
  pinMode(MS1_2, OUTPUT);
  pinMode(MS2_2, OUTPUT);
  pinMode(EN_2,  OUTPUT);

  resetEDPins();
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
  String commandArray[4]; //[motor, direction, angle, step size] for individual movement
                          // or [elevation, azimuth] for pointing
  int i = 0;
  while(Serial.available()){
    String tmp = Serial.readStringUntil(" ");
    commandArray[i] = tmp;
    i++;
  }

  //act based on filled command array
  if (i == 2) {
    point(commandArray[0].toDouble(), commandArray[1].toDouble());
  }
  else if (i == 4){
    int motor;
    if (commandArray[0] == "outer"){
      motor = OUTER;
    }
    else {
      motor = INNER;
    }
    
    int dir;
    if(commandArray[1] == "ccw"){
      dir = COUNTERCLOCKWISE;
    }
    else {
      dir = CLOCKWISE;
    }
  
    double angle = commandArray[2].toDouble();
    int stepResolution = commandArray[3].toInt();
    
    moveMotor(motor, dir, angle, stepResolution);  
  }
  else {
    Serial.println("please enter a valid command");
    Serial.println("[motor direction angle step_size] for individual movement");
    Serial.println(" or [elevation azimuth] for pointing");
  }
  
  //resetEDPins();
}

void resetEDPins()
{
  digitalWrite(STEP_1, LOW);
  digitalWrite(DIR_1, LOW);
  digitalWrite(MS1_1, LOW);
  digitalWrite(MS2_1, LOW);
  digitalWrite(EN_1, HIGH);
  digitalWrite(STEP_2, LOW);
  digitalWrite(DIR_2, LOW);
  digitalWrite(MS1_2, LOW);
  digitalWrite(MS2_2, LOW);
  digitalWrite(EN_2, HIGH);
}

bool moveMotor(int motor, double angle) {
  int stepResolution;
  if (motor == OUTER){
    stepResolution = resolution_1;
  }
  else if (motor == INNER){
    stepResolution = resolution_2;
  }
  else {
    return false;
  }
  
  int steps = angleToSteps(angle, stepResolution);
  
  if (angle > 0){
    StepForward(motor, steps);
  }
  else if (angle < 0) {
    ReverseStep(motor, steps);
  }
  else{
    return false;
  }
  
  return true;
}

void moveMotor(int motor, int dir, double angle, int stepResolution) {
  SetStepMode(motor, stepResolution);
  int steps = angleToSteps(angle, stepResolution);
  if (dir == COUNTERCLOCKWISE){
    StepForward(motor, steps);
  }
  else if (dir == CLOCKWISE) {
    ReverseStep(motor, steps);
  }
  else{
    ;
  }
}

//Default microstep mode function
void StepForward(int motor, int steps)
{
  if (motor == 1){
    Serial.println("Motor 1 forward " + String(steps) + " steps");
    digitalWrite(DIR_1, LOW); //Pull direction pin low to move "forward"
    for(x_1= 0; x_1<steps; x_1++) { //loop number of steps to take 
      digitalWrite(STEP_1,HIGH); //Trigger one step forward
      delay(1);
      digitalWrite(STEP_1,LOW); //Pull step pin low so it can be triggered again
      //delay(1);
    }
  }
  else if (motor == 2) {
    Serial.println("Motor 2 forward " + String(steps) + " steps");
    digitalWrite(DIR_2, LOW); //Pull direction pin low to move "forward"
    for(x_2= 0; x_2<steps; x_2++) { //loop number of steps to take 
      digitalWrite(STEP_2,HIGH); //Trigger one step forward
      delay(1);
      digitalWrite(STEP_2,LOW); //Pull step pin low so it can be triggered again
      //delay(1);
    } 
  }
  else {
    Serial.println("invalid motor selection");
  }
}

void ReverseStep(int motor, int steps)
{
  if (motor == 1){
    Serial.println("Motor 1 back " + String(steps) + " steps");
    digitalWrite(DIR_1, HIGH); //Pull direction pin low to move "forward"
    for(x_1= 0; x_1<steps; x_1++) { //loop number of steps to take 
      digitalWrite(STEP_1,HIGH); //Trigger one step forward
      delay(1);
      digitalWrite(STEP_1,LOW); //Pull step pin low so it can be triggered again
      delay(1);
    }
  }
  else if (motor == 2) {
    Serial.println("Motor 2 back " + String(steps) + " steps");
    digitalWrite(DIR_2, HIGH); //Pull direction pin low to move "forward"
    for(x_2= 0; x_2<steps; x_2++) { //loop number of steps to take 
      digitalWrite(STEP_2,HIGH); //Trigger one step forward
      delay(1);
      digitalWrite(STEP_2,LOW); //Pull step pin low so it can be triggered again
      delay(1);
    } 
  }
  else {
    Serial.println("invalid motor selection");
  }
}

// 1/8th microstep foward mode function
void SetStepMode(int motor, int resolution)
{
  if (motor == 1){
    if (resolution == 1){
      Serial.println("Set motor 1 to full step resolution");
      digitalWrite(MS1_1, LOW);
      digitalWrite(MS2_1, LOW); 
    }
    else if (resolution == 2){
      Serial.println("Set motor 1 to half step resolution");
      digitalWrite(MS1_1, HIGH);
      digitalWrite(MS2_1, LOW);
    }
    else if (resolution == 4){
      Serial.println("Set motor 1 to quarter step resolution");
      digitalWrite(MS1_1, LOW);
      digitalWrite(MS2_1, HIGH);
    }
    else if (resolution == 8){
      Serial.println("Set motor 1 to eighth step resolution");
      digitalWrite(MS1_1, HIGH);
      digitalWrite(MS2_1, HIGH);
    }
    else {
      Serial.println("invalid resolution");
    }
  }
  else if (motor == 2){
    if (resolution == 1){
      Serial.println("Set motor 2 to full step resolution");
      digitalWrite(MS1_2, LOW);
      digitalWrite(MS2_2, LOW); 
    }
    else if (resolution == 2){
      Serial.println("Set motor 2 to half step resolution");
      digitalWrite(MS1_2, HIGH);
      digitalWrite(MS2_2, LOW);
    }
    else if (resolution == 4){
      Serial.println("Set motor 2 to quarter step resolution");
      digitalWrite(MS1_2, LOW);
      digitalWrite(MS2_2, HIGH);
    }
    else if (resolution == 8){
      Serial.println("Set motor 2 to eighth step resolution");
      digitalWrite(MS1_2, HIGH);
      digitalWrite(MS2_2, HIGH);
    }
    else {
      Serial.println("invalid resolution");
    }
  }
  else {
    Serial.println("invalid motor selection");
  }
}

int angleToSteps(double angle, int resolution){
  return angle*(200/360)*resolution; //motor has 200 finite steps per revolution
}




//Forward/reverse stepping function
void Oscillate(int motor)
{
  if (motor == 1){
    Serial.println("Alternate between stepping forward and reverse.");
    for(x_1= 1; x_1<5; x_1++)  //Loop the forward stepping enough times for motion to be visible
    {
      //Read direction pin state and change it
      state=digitalRead(DIR_1);
      if(state == HIGH)
      {
        digitalWrite(DIR_1, LOW);
      }
      else if(state ==LOW)
      {
        digitalWrite(DIR_1,HIGH);
      }
      
      for(y_1=1; y_1<1000; y_1++)
      {
        digitalWrite(STEP_1,HIGH); //Trigger one step
        delay(1);
        digitalWrite(STEP_1,LOW); //Pull step pin low so it can be triggered again
        delay(1);
      }
    }
  }
  else if (motor == 2){
    Serial.println("Alternate between stepping forward and reverse.");
    for(x_2= 1; x_2<5; x_2++)  //Loop the forward stepping enough times for motion to be visible
    {
      //Read direction pin state and change it
      state=digitalRead(DIR_2);
      if(state == HIGH)
      {
        digitalWrite(DIR_2, LOW);
      }
      else if(state ==LOW)
      {
        digitalWrite(DIR_2,HIGH);
      }
      
      for(y_2=1; y_2<1000; y_2++)
      {
        digitalWrite(STEP_1,HIGH); //Trigger one step
        delay(1);
        digitalWrite(STEP_1,LOW); //Pull step pin low so it can be triggered again
        delay(1);
      }
    }
  }
  else {
    Serial.println("invalid motor selection");
  }
}


bool point(double elevation, double azimuth){
  double coords[] = {sin(elevation) * cos(azimuth), sin(elevation)*sin(azimuth), cos(elevation) };
  double theta = asin(-coords[1]);
  double phi = asin(coords[0] / cos(theta));
  double outerRot = theta - outerMotorPos;
  double innerRot = phi - innerMotorPos;
  outerMotorPos = theta;
  innerMotorPos = phi;
  moveMotor(OUTER, outerRot);
  moveMotor(INNER, innerRot);
}

