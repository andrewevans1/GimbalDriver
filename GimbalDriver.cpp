#include "Arduino.h"
#include "GimbalDriver.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

//TODO: take steps per revolution as constructor argument
//Motor object for controlling Sparkfun Easy Driver motor controller
Motor::Motor(int stepPin, int dirPin, int MS1Pin, int MS2Pin, int ENPin, double maxAngle)
{
    _stepPin = stepPin;
    _dirPin = dirPin;
    _MS1Pin = MS1Pin;
    _MS2Pin = MS2Pin;
    _ENPin = ENPin;
    pinMode(_stepPin, OUTPUT);
    digitalWrite(_stepPin, LOW);
    pinMode(_dirPin, OUTPUT);
    digitalWrite(_dirPin, LOW);
    pinMode(_MS1Pin, OUTPUT);
    digitalWrite(_MS1Pin, LOW);
    pinMode(_MS2Pin, OUTPUT);
    digitalWrite(_MS2Pin, LOW);
    pinMode(_ENPin,  OUTPUT);
    digitalWrite(_ENPin, HIGH);
    _resolution = 1;
    _angle = 0.0;
    _maxAngle = maxAngle;
}

//for checking position of motor
//used by MotorController when pointing
double Motor::getPos(){
    Serial.print("current pos "); Serial.println(_angle);
    return _angle;
}
//internal method for tracking changes in angular position
void Motor::setPos(double angle){
    _angle = angle;
}

// If valid, return angle,
// else, return max angle it can rotate
double Motor::checkValidMove(double angle){
    if (abs(_angle + angle) <= _maxAngle) {
        Serial.println("valid");
        return angle;
    }
    else if ((_angle + angle) > 0){
        Serial.println("exceeds +max angle, stopping at limit");
        return (_maxAngle - _angle);
    }
    else {
        Serial.println("exceeds -max angle, stopping at limit");
        return (-_maxAngle - _angle);
    }
    return angle;
}
//Basic method for rotating motor forward
void Motor::stepForward(int steps){
    digitalWrite(_dirPin, LOW); //Pull direction pin low to move "forward"
    for(int i= 0; i<steps; i++) { //loop number of steps to take 
      digitalWrite(_stepPin,HIGH); //Trigger one step forward
      delay(1);
      digitalWrite(_stepPin,LOW); //Pull step pin low so it can be triggered again
      delay(1);
      //Serial.println(String(i));
    }
    //_angle += steps/_resolution;
    //Serial.println("finished stepping forward");
}
//Basic method for rotating motor in reverse
void Motor::reverseStep(int steps){
    digitalWrite(_dirPin, HIGH); //Pull direction pin low to move "backward"
    for(int i= 0; i<steps; i++) { //loop number of steps to take 
      digitalWrite(_stepPin,HIGH); //Trigger one step forward
      delay(1);
      digitalWrite(_stepPin,LOW); //Pull step pin low so it can be triggered again
      delay(1);
    }
    //_angle -= steps/_resolution;
}

//Helper method for calculating the steps to take based on micro-stepping resolution
int Motor::angleToSteps(double angle){
    int steps = angle*(2048.0/360.0)*_resolution; //motor has 2080 finite steps per revolution according to data sheet
    //Serial.println(String(steps));
    return steps;
}
//Method for setting the microstepping resolution
void Motor::setResolution(int resolution){
    switch(resolution){
        case 1: //full step
            digitalWrite(_MS1Pin, LOW);
            digitalWrite(_MS2Pin, LOW);
            break;
        case 2: //half step
            digitalWrite(_MS1Pin, HIGH);
            digitalWrite(_MS2Pin, LOW);
            break;
        case 4: //quarter step
            digitalWrite(_MS1Pin, LOW);
            digitalWrite(_MS2Pin, HIGH);
            break;
        case 8: //eighth step
            digitalWrite(_MS1Pin, HIGH);
            digitalWrite(_MS2Pin, HIGH);
            break;
        default:
            Serial.println("invalid resolution, " + String(resolution));
            return;
    }
    _resolution = resolution;
}
//Higher lever function to be used in main program or by MotorController
void Motor::moveMotor(double angle){
    enableMotor();
    angle = checkValidMove(angle);
    int steps = angleToSteps(abs(angle));
    if (angle > 0){
        Serial.println("steps +" + String(steps));
        stepForward(steps);
    }
    else if (angle < 0) {
        Serial.println("steps -" + String(steps));
        reverseStep(steps);
    }
    else{
        disableMotor();
        return;
    }
    _angle += angle;
    disableMotor();
}

//Miscellaneous method for demonstration of motor movement
void Motor::oscillate(double maxAngle, int loops){
    //TODO: check that motor begins at start pos
    enableMotor();
    int maxSteps = angleToSteps(maxAngle);
    Serial.println("steps " + String(maxSteps));
    //initial displacement
    int state = digitalRead(_dirPin);
    if(state == HIGH)
        digitalWrite(_dirPin, LOW);
    
    else if(state ==LOW)
        digitalWrite(_dirPin,HIGH);
    
    for(int j = 1; j < maxSteps; j++)
    {
        digitalWrite(_stepPin,HIGH); //Trigger one step
        delay(1);
        digitalWrite(_stepPin,LOW); //Pull step pin low so it can be triggered again
        delay(1);
    }
    delay(250);

    for(int i = 0; i<loops; i++)  //Loop the forward stepping enough times for motion to be visible
    {
        //Read direction pin state and change it
        state = digitalRead(_dirPin);
        if(state == HIGH)
            digitalWrite(_dirPin, LOW);
        
        else if(state ==LOW)
            digitalWrite(_dirPin,HIGH);
        
        for(int j = 1; j < maxSteps*2; j++)
        {
            digitalWrite(_stepPin,HIGH); //Trigger one step
            delay(1);
            digitalWrite(_stepPin,LOW); //Pull step pin low so it can be triggered again
            delay(1);
        }
        delay(250);
        
    }
    //return to start pos
    state = digitalRead(_dirPin);
    if(state == HIGH)
        digitalWrite(_dirPin, LOW);
    
    else if(state ==LOW)
        digitalWrite(_dirPin,HIGH);
    
    for(int j = 1; j < maxSteps; j++)
    {
        digitalWrite(_stepPin,HIGH); //Trigger one step
        delay(1);
        digitalWrite(_stepPin,LOW); //Pull step pin low so it can be triggered again
        delay(1);
    }
    
    disableMotor();
}
//private function to switch enable pin
void Motor::enableMotor(){
    Serial.println("enabling");
    digitalWrite(_ENPin, LOW);
}
//private function for disabling motor
//to be called immediately after moving motor
void Motor::disableMotor(){
    Serial.println("disabling");
    digitalWrite(_ENPin, HIGH);
}
//reset pins
void Motor::reset(){
    digitalWrite(_stepPin, LOW);
    digitalWrite(_dirPin, LOW);
    digitalWrite(_MS1Pin, LOW);
    digitalWrite(_MS2Pin, LOW);
    digitalWrite(_ENPin, HIGH);
}

//test function for checking pin states and motor status
String Motor::getState(){
    String tmp_str = "StepPin: ";
    tmp_str += String(digitalRead(_stepPin));
    tmp_str += "\nDirPin: ";
    tmp_str += String(digitalRead(_dirPin));
    tmp_str += "\nMS1Pin: ";
    tmp_str += String(digitalRead(_MS1Pin));
    tmp_str += "\nMS2Pin: ";    
    tmp_str += String(digitalRead(_MS2Pin));
    tmp_str += "\nENPin: ";
    tmp_str += String(digitalRead(_ENPin));
    tmp_str += "\nPos: ";
    tmp_str += String(_angle);
    tmp_str += "\nResolution: ";
    tmp_str += String(_resolution);
    return(tmp_str);
}


/*
* Class for operating two stepper motors in a 2 axis configuration
* Allows for pointing of 2-axis gimbal 
*/
MotorController::MotorController(Motor* inner_motor, Motor* outer_motor) {
    _inner_motor = inner_motor;
    _outer_motor = outer_motor;
}

//main function of MotorController class
//Takes in a direction as spherical coordinates, then transforms the coordinates into motor rotations
//Transformations assume gimbal begins pointing in the (0,0,1) direction
//Knowing the current motor positions, we can figure out the necessary rotions to get to that point 
void MotorController::point(double elevation_, double azimuth_){
    double elevation = elevation_/RAD_TO_DEG;
    double azimuth = azimuth_/RAD_TO_DEG;
    Serial.print("elevation: "); Serial.println(String(elevation));
    Serial.print("azimuth: "); Serial.println(String(azimuth));
    double coords[] = {sin(elevation) * cos(azimuth), sin(elevation)*sin(azimuth), cos(elevation) };
    Serial.print("x: "); Serial.println(String(coords[0]));
    Serial.print("y: "); Serial.println(String(coords[1]));
    Serial.print("z: "); Serial.println(String(coords[2]));
    double theta = asin(-coords[1]);
    double phi = asin(coords[0] / cos(theta));
    Serial.print("theta: "); Serial.println(String(theta));
    Serial.print("phi: "); Serial.println(String(phi));
    theta = theta * RAD_TO_DEG;
    phi = phi * RAD_TO_DEG;
    
    double currentOuterAngle = _outer_motor->getPos();
    double currentInnerAngle = _inner_motor->getPos();
    double outerRot = theta - currentOuterAngle;
    double innerRot = phi - currentInnerAngle;
    Serial.print("outerRot "); Serial.print(String(outerRot));
    Serial.print(" innerRot "); Serial.println(String(innerRot));
    _outer_motor->moveMotor(outerRot);
    _inner_motor->moveMotor(innerRot);
}
//Not sure if this does what I want it to
void MotorController::oscillate(double maxAngle, int loops){
    _inner_motor->oscillate(maxAngle, loops);
    _outer_motor->oscillate(maxAngle, loops);
}
//Miscillaneous function similar to the motor's oscillate
//must alternate between controlling one motor and the other based on moveMotor() limitations
//Might consider controlling the actual pins here...
void MotorController::spin(double elevation, int loops){
    //move to starting position
    //point(elevation, 0);
    //begin oscillations
    double max_angle = elevation;
    Serial.print("max_steps: "); Serial.println(String(max_angle));
    double step_size = 1; //set smaller for smoother movement
    Serial.print("step_size: "); Serial.println(String(step_size));
    Serial.print("iterations: "); Serial.println(String(max_angle/step_size));
    //delay(1000);
    for (int i = 0; i < loops; i++){
        //TODO: might get truncation errors here
        Serial.println("1");
        for (int j = 0; j < max_angle/step_size; j++){
            _inner_motor->moveMotor(step_size);
            delay(1);
            _outer_motor->moveMotor(-step_size);
            delay(1);
        }
        Serial.println("2");
        for (int j = 0; j < max_angle/step_size; j++){
            _inner_motor->moveMotor(-step_size);
            delay(1);
            _outer_motor->moveMotor(-step_size);
            delay(1);
        }
        Serial.println("3");
        for (int j = 0; j < max_angle/step_size; j++){
            _inner_motor->moveMotor(-step_size);
            delay(1);
            _outer_motor->moveMotor(step_size); 
            delay(1);
        }
        Serial.println("4");
        for (int j = 0; j < max_angle/step_size; j++){
            _inner_motor->moveMotor(step_size);
            delay(1);
            _outer_motor->moveMotor(step_size);
            delay(1);
        }
    }
}
//return motors to origin
void MotorController::reset(){
    _inner_motor->moveMotor(-_inner_motor->getPos());
    _outer_motor->moveMotor(-_outer_motor->getPos());
}