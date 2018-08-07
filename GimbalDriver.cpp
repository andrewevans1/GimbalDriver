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
    _pos = 0;
}

double Motor::getPos(){
    Serial.print("current pos"); Serial.println(_pos);
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
      //Serial.println(String(i));
    }
    Serial.println("finished stepping forward");
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
    int steps = angle*(200.0/360.0)*_resolution; //motor has 200 finite steps per revolution
    Serial.println(String(steps));
    return steps;
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
    enableMotor();
    int steps = angleToSteps(abs(angle));
    Serial.println("steps " + String(steps));
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
    enableMotor();
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
    Serial.println("enabling");
    digitalWrite(_ENPin, LOW);
}
void Motor::reset(){
    digitalWrite(_stepPin, LOW);
    digitalWrite(_dirPin, LOW);
    digitalWrite(_MS1Pin, LOW);
    digitalWrite(_MS2Pin, LOW);
    digitalWrite(_ENPin, HIGH);
}

/*
MotorController::MotorController(Motor* inner_motor, Motor* outer_motor) {
    _inner_motor = inner_motor;
    _outer_motor = outer_motor;
}*/
MotorController::MotorController(Motor& inner_motor, Motor& outer_motor):
    _inner_motor(inner_motor),
    _outer_motor(outer_motor) {}
/*
void MotorController::point(double elevation, double azimuth){
    Serial.println(String(azimuth));
    //_outer_motor.setPos(0.0);
    //_inner_motor.setPos(0.0);
    Serial.print("elevation"); Serial.print(String(elevation));
    Serial.print("azimuth "); Serial.println(String(azimuth));
    double coords[] = {sin(elevation) * cos(azimuth), sin(elevation)*sin(azimuth), cos(elevation) };
    Serial.print("coord 1"); Serial.print(String(coords[0]));
    Serial.print("coord 2"); Serial.println(String(coords[1]));
    double theta = asin(-coords[1]);
    Serial.println(String(theta));
    double phi = asin(coords[0] / cos(theta));
    Serial.println(String(phi));
    double outerRot = theta - _outer_motor.getPos();
    double innerRot = phi - _inner_motor.getPos();
    Serial.print("outerRot "); Serial.print(String(outerRot));
    Serial.print(" innerRot "); Serial.println(String(innerRot));
    //_outer_motor.setPos(theta);
    //_inner_motor.setPos(phi);
    //_outer_motor.moveMotor(outerRot);
    _outer_motor.moveMotor(10);
    _inner_motor.moveMotor(10);
    //_inner_motor.moveMotor(innerRot);
}
*/
void MotorController::point(double elevation, double azimuth){
    double coords[] = {sin(elevation) * cos(azimuth), sin(elevation)*sin(azimuth), cos(elevation) };
    double theta = asin(-coords[1]);
    double phi = asin(coords[0] / cos(theta));
    double outerRot = theta - _outer_motor.getPos();
    double innerRot = phi - _inner_motor.getPos();
    _outer_motor.setPos(theta);
    _inner_motor.setPos(phi);
    Serial.print("outerRot "); Serial.print(String(outerRot));
    Serial.print(" innerRot "); Serial.println(String(innerRot));
    _outer_motor.moveMotor(outerRot);
    _inner_motor.moveMotor(innerRot);
}