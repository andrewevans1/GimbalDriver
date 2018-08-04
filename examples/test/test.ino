#include <MotorDriver.h>

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

Motor inner(STEP_1, DIR_1, MS1_1, MS2_1, EN_1);
Motor outer(STEP_2, DIR_2, MS1_2, MS2_2, EN_2);

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  inner.moveMotor(10);
  outer.moveMotor(-5.3);
}
