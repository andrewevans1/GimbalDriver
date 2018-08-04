#ifndef GimbalDriver_h
#define GimbalDriver_h

#include "Arduino.h"

class Motor 
{
    public:
        Motor(int stepPin, int dirPin, int MS1Pin, int MS2Pin, int ENPin);
        void setResolution(int resolution);
        double getPos();
        void setPos(double pos);
        void moveMotor(double angle);
        void oscillate(double maxAngle, int loops);
        void enableMotor();
        void reset();
    private:
        int _stepPin;
        int _dirPin;
        int _MS1Pin;
        int _MS2Pin;
        int _ENPin;
        int _resolution;
        int _pos;

        void stepForward(int steps);
        void reverseStep(int steps);
        int angleToSteps(double angle);
};

class MotorController 
{
    public:
        MotorController(Motor inner_motor, Motor outer_motor);
        void point(double elevation, double azimuth);

};

#endif
