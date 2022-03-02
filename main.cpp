#include <math.h>
#include <FEHLCD.h>
#include <FEHMotor.h>
#include <FEHIO.h>

#define PI 3.14159
#define DEFAULT_STOP_TIME 200
#define DEFAULT_SPEED 0.4

// Encoder things
#define WHEEL_DIAMETER 2.5
#define COUNTS_PER_ROTATION 318
#define NUM_WHEELS 3

// color thresholds for CdS cell
#define RED_LOWER 0.9
#define RED_UPPER 1.4
#define BLUE_LOWER 1.55
#define BLUE_UPPER 2.1

// Cosntants to initialize wheels
const double WHEEL_DIRECTIONS[NUM_WHEELS][2] = {
    {-1,0},
    {.5, .866},
    {.5, -.866}
};
FEHMotor drivetrain[NUM_WHEELS] = {
    FEHMotor(FEHMotor::Motor0,9.0),
    FEHMotor(FEHMotor::Motor1,9.0),
    FEHMotor(FEHMotor::Motor2,9.0)
};
AnalogInputPin CDS(FEHIO::P0_0);
DigitalEncoder encoders[NUM_WHEELS] = {
    DigitalEncoder(FEHIO::P3_0),
    DigitalEncoder(FEHIO::P3_2),
    DigitalEncoder(FEHIO::P3_4)
};

// Encode CPI
const int COUNTS_PER_INCH = COUNTS_PER_ROTATION / (PI * WHEEL_DIAMETER);


// Helper Math Functions
double projectOntoWheel (double a[2], int curWheel) {
    return a[0] * WHEEL_DIRECTIONS[curWheel][0] + a[1] * WHEEL_DIRECTIONS[curWheel][1];
}



// Movement Functions  ----------------------------------------------------

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


// Set velocity based on an x, y vector (each from 0 to 1)
void setVelocity (double velocity[2]) {
    for (int i = 0; i < NUM_WHEELS; i++) {
        // Get the scalar projection of the velocity onto the wheel direction
        double proj = projectOntoWheel(velocity, i);
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
    int percent = (int)(-speed * 100);
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




// Encoder Functions -------------------------------------------------

void driveDistance(int angle, double inches) {
    const double numCounts = inches * COUNTS_PER_INCH;
    int totalCounts = 0;

    double direction[2] = {sin(angle * PI / 180.), cos(angle * PI / 180.)};
    for (int i = 0; i < NUM_WHEELS; i++) {
        totalCounts += abs((int)(projectOntoWheel(direction, i) * numCounts));

        // Reset each encoder
        encoders[i].ResetCounts();
    }
    // Set the velocity
    setVelocity(angle, DEFAULT_SPEED);
    
    // Wait for counts to reach total counts
    int counts = 0;
    while (counts < totalCounts) {
        counts = 0;
        for (int i = 0; i < NUM_WHEELS; i++) {
            counts += abs((int)encoders[i].Counts());
        }
        Sleep(5);
    }
    stop();
    
    
}

// CdS Functions ---------------------------------------------------------
 

 // Enum for colors

enum COLORS {
    NONE, BLUE_LIGHT, RED_LIGHT
};

 // gets the color of loight using the CdS cell
 // returns 1 for blue, and 2 for red, and 0 for neither
 COLORS getColor(){
     COLORS ret=NONE;
     double color = CDS.Value();
     if(color >= RED_LOWER && color <= RED_UPPER) {
         ret=RED_LIGHT;
     }
     else if(color<=BLUE_UPPER && color>=BLUE_LOWER){
         ret=BLUE_LIGHT;
     }

     return ret;
 }

// Test funcitons -----------------------------------------------------


// Drive in a circle
void driveInCircle(double speed, int timePerDegree) {
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
    drive(180, 2000, DEFAULT_SPEED, true, 2000);
    driveInCircle(.7, 15);
    turn(true, 2000);
    turn(false, 2000);
}



// Functions for performance ----------------------------------------

void driveToLight() {
    //dr)
}


int main(void)
{
    //testBot();
    

    /*
        Performance Test 1
            ***MAKE SURE ROBOT IS IN THE RIGHT STARTING POSITION***
    */

   
   
   // Wait for starting light
   while (getColor() != RED_LIGHT) {
       Sleep(5);
   }
   

    // drive to light
    driveDistance(278, 16.6);
    Sleep(DEFAULT_STOP_TIME);
    // get light value (1=blue,2=red)
    COLORS color=getColor();
    LCD.Clear();
    // Hit correct button
    Sleep(DEFAULT_STOP_TIME);
    if(color==BLUE_LIGHT){
        // hit blue
        LCD.WriteLine("Blue");
        drive(160,700);
        drive(180+160,700);
    }else{
        // hit red
        LCD.WriteLine("Red");
        drive(224,900);
        drive(180+224,900);
        turn(true, 100);
    }

    // Move to ramp
    driveDistance(40, 10.);

    // Go up ramp
    turn(true, 70);
    driveDistance(0, 20.);
    Sleep(DEFAULT_STOP_TIME);

    // Go back down ramp
    driveDistance(180, 20.);

    return 0;
}   
