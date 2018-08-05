#include "Arduino.h"
#include "GimbalDriver.h"

Motor::Motor(int stepPin, int dirPin, int MS1Pin, int MS2Pin, int ENPin)
{
    _stepPin = stepPin;
    _dirPin = dirPin;
    _MS1Pin = MS1Pin;
    _MS2Pin = MS2Pin;
    _ENPin = ENPin;
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(_MS1Pin, OUTPUT);
    pinMode(_MS2Pin, OUTPUT);
    pinMode(_ENPin,  OUTPUT);
    _resolution = 1;
    _pos = 0;
}

double Motor::getPos(){
    return _pos;
}
void Motor::setPos(double pos){
    _pos = pos;
}
void Motor::stepForward(int steps){
    digitalWrite(_dirPin, LOW); //Pull direction pin low to move "forward"
    for(int i= 0; i<steps; i++) { //loop number of steps to take 
      digitalWrite(_stepPin,HIGH); //Trigger one step forward
      delay(1);
      digitalWrite(_stepPin,LOW); //Pull step pin low so it can be triggered again
      delay(1);
    }
}
void Motor::reverseStep(int steps){
    digitalWrite(_dirPin, HIGH); //Pull direction pin low to move "backward"
    for(int i= 0; i<steps; i++) { //loop number of steps to take 
      digitalWrite(_stepPin,HIGH); //Trigger one step forward
      delay(1);
      digitalWrite(_stepPin,LOW); //Pull step pin low so it can be triggered again
      delay(1);
    }
}
int Motor::angleToSteps(double angle){
    return angle*(200/360)*_resolution; //motor has 200 finite steps per revolution
}

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
void Motor::moveMotor(double angle){
    int steps = angleToSteps(abs(angle));
    if (angle > 0){
        stepForward(steps);
    }
    else if (angle < 0) {
        reverseStep(steps);
    }
    else{
        return;
    }
}
void Motor::oscillate(double maxAngle, int loops){
    //TODO: check that motor begins at start pos
    
    int maxSteps = angleToSteps(maxAngle);
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
}
void Motor::enableMotor(){
    digitalWrite(_ENPin, LOW);
}
void Motor::reset(){
    return;
}


MotorController::MotorController(Motor& inner_motor, Motor& outer_motor) {
    _inner_motor = inner_motor;
    _outer_motor = outer_motor;
}
void MotorController::point(double elevation, double azimuth){
    double coords[] = {sin(elevation) * cos(azimuth), sin(elevation)*sin(azimuth), cos(elevation) };
    double theta = asin(-coords[1]);
    double phi = asin(coords[0] / cos(theta));
    double outerRot = theta - _outer_motor.getPos();
    double innerRot = phi - _inner_motor.getPos();
    _outer_motor.setPos(theta);
    _inner_motor.setPos(phi);
    _outer_motor.moveMotor(outerRot);
    _inner_motor.moveMotor(innerRot);
}