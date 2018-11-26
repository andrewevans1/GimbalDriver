# Gimbal-Driver

## Motor Class
The Motor class is built to control the Sparkfun Easy Driver microstepping motor controller. It handles all motor movements, and keeps track of supposed motor positions. In addition, it checks to make sure the motor position is within the bounds of safe movement applicable to your device. In my case, the structure I have built allows for about 30 degrees on either side.

## MotorController Class
The MotorController class is designed for a 2 axis gimbal, where the two stepper motors are perpendicular to one another, and the inner one is moutned on the outer one's axis of rotation. Knowing this, it is possible to input spherical or cartesian coordinates and have the MotorController figure out how much the inner and outer motors each need to move to point in that direction. On top of that there are a few novelty functions like oscillate and spin that demonstrate the range of motion of the gimbal.

## Using the Examples
There are two examples, gimbalDriver and gimbalDriverNano. The main difference is just the pins, as I used the former with an arduino uno and the latter with an arduino nano. The code for the nano is the most up to date, so I will explain it here.

## gimbalDriverNano.ino
### Globals
Constants: constants are defined for pin assignments and program values.
Motors are initialized as instances of Motor Class, MotorClass is created to control Motors.

### Setup
Initialize serial communication with 9600 baud rate, for use with arduino IDE's serial monitor or other similar program.
Accelerometer uses 3v3 output, so analog input on the nano is set to external reference of 3v3.
Print command list for the human commander's convenience.

### Loop
Check the switch position and therefore operation mode by seeing if the BALANCE_MODE or SPIN_DEMO pins are high. 
If BALANCE_MODE: run one iteration of `balance` function. Check input from the accelerometer and respond to level the gimbal. User can rotate the gimbal and see it rotate in response.
Else if SPIN_DEMO: run the `spin` function. Gimbal attempts to sweep out a circle at the input elevation (assigned here to be 25 degrees from the pole). This performs one loop. Note, this movement is not smooth. Something about switching back and forth rapidly between which motor is being driven has proved to be an issue for the motor controllers. 
Else: Niehter balance nor spin modes are selected, so the gimbal waits on user input to move. These are the commands:

| Input command string             | Description                                                                                                                                                                                                                                                                |
|----------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `point [phi] [theta]`            | Point gimbal to point on unit sphere defined by spherical coordinates phi being angle in degrees from the pole, and theta being angle from x-axis in degrees                                                                                                               |
| `point [x] [y] [z]`              | Point gimbal in direction of vector <x,y,z>. Values normalized automatically, so magnitude doesn't matter -- note this doesn't actually work with gimbalDriverNano command input, which only accepts three arguments currently, only used with the accelerometer right now |
| `spin [phi] [loops]`             | Spin through circle at angle from pole.                                                                                                                                                                                                                                    |
| `balance [loops]`                | Act in balance mode for set number of loops                                                                                                                                                                                                                                |
| `move [inner/outer] [angle]`     | Rotate single motor angle in degrees. Accepts positive and negative.                                                                                                                                                                                                       |
| `set [inner/outer] [resolution]` | set motor driver resolution either full step, half step, quarter step or eighth step with 1,2,4,8 respectively.                                                                                                                                                            |
| `get [inner/outer]`              | return state of motor (angular position, resolution, and pin assignments)                                                                                                                                                                                                  |
| `reset`                          | return both motors to rotation angle of 0 degrees - i.e. return to start position                                                                                                                                                                                          |
