#ifndef GimbalDriver_h
#define GimbalDriver_h

#include "Arduino.h"

class Motor 
{
    public:
        Motor(int stepPin, int dirPin, int MS1Pin, int MS2Pin, int ENPin, double maxAngle);
        void setResolution(int resolution);
        double getPos();
        void moveMotor(double angle);
        void oscillate(double maxAngle, int loops);
        void enableMotor();
        void disableMotor();
        void reset();
        String getState();
        double checkValidity(double angle);
        int angleToSteps(double angle);
        double stepsToAngle(int steps);
    private:
        int _stepPin;
        int _dirPin;
        int _MS1Pin;
        int _MS2Pin;
        int _ENPin;
        int _resolution;
        double _angle;
        double _maxAngle;

        void stepForward(int steps);
        void reverseStep(int steps);
        void setPos(double pos);
};

class MotorController 
{
    public:
        MotorController(Motor* inner_motor, Motor* outer_motor);
        void point(double elevation, double azimuth);
        void oscillate(double maxAngle, int loops);
        void spin(double elevation, int loops);
        void reset();
        Motor* _inner_motor;
        Motor* _outer_motor;
};

#endif
