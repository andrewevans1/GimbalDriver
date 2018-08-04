#include "Arduino.h"
#include "MotorDriver.h"

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
int Motor::angleToSteps(double angle, int resolution){

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
    int steps = angleToSteps(abs(angle), _resolution);
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
void Motor::oscillate(double maxAngle){
    return;
}
void Motor::enableMotor(){
    digitalWrite(_ENPin, LOW);
}
void Motor::reset(){
    return;
}
