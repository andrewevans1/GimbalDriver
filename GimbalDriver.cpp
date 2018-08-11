#include "Arduino.h"
#include "GimbalDriver.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

//TODO: take steps per revolution as constructor argument
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
    Serial.print("current pos "); Serial.println(_pos);
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
    _pos += steps/_resolution;
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
    _pos -= steps/_resolution;
}
int Motor::angleToSteps(double angle){
    int steps = angle*(2048.0/360.0)*_resolution; //motor has 2080 finite steps per revolution according to data sheet
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
    disableMotor();
}
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
void Motor::enableMotor(){
    Serial.println("enabling");
    digitalWrite(_ENPin, LOW);
}
void Motor::disableMotor(){
    Serial.println("disabling");
    digitalWrite(_ENPin, HIGH);
}
void Motor::reset(){
    digitalWrite(_stepPin, LOW);
    digitalWrite(_dirPin, LOW);
    //digitalWrite(_MS1Pin, LOW);
    //digitalWrite(_MS2Pin, LOW);
    digitalWrite(_ENPin, HIGH);
}

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
    tmp_str += String(_pos);
    tmp_str += "\nResolution: ";
    tmp_str += String(_resolution);
    return(tmp_str);
}

MotorController::MotorController(Motor& inner_motor, Motor& outer_motor):
    _inner_motor(inner_motor),
    _outer_motor(outer_motor) {}

void MotorController::point(double elevation, double azimuth){
    double coords[] = {sin(elevation) * cos(azimuth), sin(elevation)*sin(azimuth), cos(elevation) };
    double theta = asin(-coords[1]);
    double phi = asin(coords[0] / cos(theta));
    theta = theta * RAD_TO_DEG;
    phi = phi * RAD_TO_DEG;
    double outerRot = theta - _outer_motor.getPos();
    double innerRot = phi - _inner_motor.getPos();
    Serial.print("outerRot "); Serial.print(String(outerRot));
    Serial.print(" innerRot "); Serial.println(String(innerRot));
    _outer_motor.moveMotor(outerRot);
    _inner_motor.moveMotor(innerRot);
}

void MotorController::oscillate(double maxAngle, int loops){
    _inner_motor.oscillate(maxAngle, loops);
    _outer_motor.oscillate(maxAngle, loops);
}

void MotorController::spin(double elevation, int loops){
    //move to starting position
    point(elevation, 0);
    //begin oscillations
    int max_steps = _inner_motor.angleToSteps(elevation);
    int step_size = 10; //set smaller for smoother movement
    for (int i = 0; i < loops; i++){
        //TODO: might get truncation errors here
        for (int j = 0; j < max_steps/step_size; j++){
            _inner_motor.moveMotor(step_size);
            _outer_motor.moveMotor(-step_size);
        }
        for (int j = 0; j < max_steps/step_size; j++){
            _inner_motor.moveMotor(-step_size);
            _outer_motor.moveMotor(-step_size);
        }
        for (int j = 0; j < max_steps/step_size; j++){
            _inner_motor.moveMotor(-step_size);
            _outer_motor.moveMotor(step_size);
        }
        for (int j = 0; j < max_steps/step_size; j++){
            _inner_motor.moveMotor(step_size);
            _outer_motor.moveMotor(step_size);
        }
    }
}

void MotorController::reset(){
    _inner_motor.moveMotor(-_inner_motor.getPos());
    _outer_motor.moveMotor(-_outer_motor.getPos());
}