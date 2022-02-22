#include <math.h>
#include <FEHLCD.h>
#include <FEHMotor.h>


#define PI 3.142857
#define DEFAULT_STOP_TIME 200
#define DEFAULT_SPEED 0.6

// Cosntants to initialize wheels
const int NUM_WHEELS = 3;
const double WHEEL_DIRECTIONS[NUM_WHEELS][2] = {
    {1,0},
    {-.5, -.866},
    {-.5, .866}
};
FEHMotor drivetrain[NUM_WHEELS] = {
    FEHMotor(FEHMotor::Motor0,9.0),
    FEHMotor(FEHMotor::Motor1,9.0),
    FEHMotor(FEHMotor::Motor2,9.0)
};


// Set velocity based on an x, y vector (each from 0 to 1)
void setVelocity (double velocity[2]) {
    for (int i = 0; i < NUM_WHEELS; i++) {
        // Get the scalar projection of the velocity onto the wheel direction
        double proj = WHEEL_DIRECTIONS[i][0] * velocity[0] + WHEEL_DIRECTIONS[i][1] * velocity[1];
        int percent = (int)(proj * 100);
        drivetrain[i].SetPercent(percent);
    }
    
}

// Set velocity based on an angle (in radians clockwise from positive y axis) and a speed from 0 to 1
void setVelocity(double angle, double speed) {
    double direction[2] = {sin(angle) * speed, cos(angle) * speed};
    setVelocity(direction);
}

// Set velocity based on an angle (in degrees clockwise from positive y axis) and a speed from 0 to 1
void setVelocity(int degrees, double speed) {
    double angle = degrees * PI / 180;
    setVelocity(angle, speed);
}
 

// Turn at a speed between -1 and 1 for a time in miliseconds
// Optionally, stopAfter and set a time in miliseconds to stop for
void turn(double speed, int time, bool stopAfter = true, int stopTime = DEFAULT_STOP_TIME) {
    int percent = (int)(speed * 100);
    for (int i = 0; i < NUM_WHEELS; i++) {
        drivetrain[i].SetPercent(percent);
    }
    Sleep(time);
    if (stopAfter) {
        stop(stopTime);
    }
}

// Turn at the specified direction for a time in miliseconds
// Optionally, stopAfter and set a time in miliseconds to stop for
void turn(bool turnRight, int time, bool stopAfter = true, int stopTime = DEFAULT_STOP_TIME) {
    double speed = DEFAULT_SPEED;
    if (!turnRight) speed *= -1;
    turn(speed, time, stopAfter, stopTime);
}






// Drive at a velocity vector for a time in miliseconds
// Optionally, stopAfter and set a time in miliseconds to stop for
void drive(double velocity[2], int time, bool stopAfter = true, int stopTime = DEFAULT_STOP_TIME) {
    setVelocity(velocity);
    Sleep(time);
    if (stopAfter) {
        stop(stopTime);
    }
}


// Drive at an angle (in degrees clockwise from positive y) for a time in miliseconds
// Optionally, set a speed (from 0 to 1), decide to stopAfter, and set a time in miliseconds to stop for
void drive (int degrees, int time, double speed = DEFAULT_SPEED, bool stopAfter = true, int stopTime = DEFAULT_STOP_TIME) {
    setVelocity(degrees, speed);
    Sleep(time);
    if (stopAfter) {
        stop(stopTime);
    }
}





// Stop the drivetrain
void stop() {
    for (int i = 0; i < NUM_WHEELS; i++) {
        drivetrain[i].Stop();
    }
}

// Stop and wait for a time in miliseconds
void stop(int time) {
    stop();
    Sleep(time);
}



// Test funcitons -----------------------------------------------------


// Drive in a circle
void driveInCircle(int speed, int timePerDegree) {
    for (int angle = 0; angle < 360; angle++) {
        drive(angle, timePerDegree, speed, false);
    }
    stop(DEFAULT_STOP_TIME);
}

// Test a few drive functions
void testBot() {
    drive(0, 2000);
    drive(90, 2000);
    drive(270, 2000);
    drive(180, 2000, true, 2000);
    driveInCircle(15, .7);
    turn(true, 2000);
    turn(false, 2000);
}

int main(void)
{
    testBot();
	return 0;
}
