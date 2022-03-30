#include <math.h>
#include <FEHLCD.h>
#include <FEHMotor.h>
#include <FEHIO.h>
#include <FEHServo.h>
#include <FEHSD.h>
#include <FEHRPS.h>
#include <FEHBuzzer.h>
#include <FEHBattery.h>
#include <stdlib.h>

#define PI 3.14159
#define DEFAULT_STOP_TIME 100
#define DEFAULT_SPEED 0.4
#define DEFAULT_TURN 0.2

// Acceptable error
#define DEFAULT_ERROR 0.4
#define DEFAULT_ANGLE_ERROR 2

// Encoder things
#define WHEEL_DIAMETER 2.5
#define COUNTS_PER_ROTATION 318
#define BODY_RADIUS 4.2
#define NUM_WHEELS 3

// color thresholds for CdS cell
#define RED_LOWER 0.9
#define RED_UPPER 1.4
#define BLUE_LOWER 1.55
#define BLUE_UPPER 2.1

// Servo constants
#define SERVO_MIN 800
#define SERVO_MAX 2400

// Movement update speed
#define CLOCK .005

// RPS thing
#define QR_ANGLE 0

// Cosntants to initialize wheels

const double WHEEL_ANGLES[NUM_WHEELS] = {
    PI, PI/3, -PI/3
};
double WHEEL_DIRECTIONS[NUM_WHEELS][2];

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

FEHServo arm(FEHServo::Servo7);

// Encode CPI
const int COUNTS_PER_INCH = COUNTS_PER_ROTATION / (PI * WHEEL_DIAMETER);
const double COUNTS_PER_DEGREE = COUNTS_PER_INCH * (2 * PI * BODY_RADIUS) / 360;


// Location/RPS variables
// Latency in milliseconds
// double latency = 0;
double x = 0;
double y = 0;
double theta = 0;
// Last direction for each wheel, either 1 or negative 1, to figure out the sign of the counts 
int lastDirection[NUM_WHEELS];
/*
// Node structure for DEQUE
typedef struct Node {
    struct Node * next = nullptr;
    struct Node * prev = nullptr;
    // Small distance travelled
    double ds = 0;
    // Angle traveled at relative to the robot
    double angle = 0;
    // Small angle turned
    double dtheta = 0;
} Node;
Node * first;
Node * last;
int dequeSize;

*/
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
 
 // Set translational and rotational velocities, given an angle in degrees (absolute ccw from x-axis), a speed, and a turn speed
 void setVelocityAndTurn(double degrees, double speed, double turnSpeed) {
    double angle = (degrees - theta + QR_ANGLE) * PI / 180;
    double direction[2] = {cos(angle) * speed, sin(angle) * speed};
    for (int i = 0; i < NUM_WHEELS; i++) {
        // Get the scalar projection of the velocity onto the wheel direction
        double proj = projectOntoWheel(direction, i);
        int percent = (int)((proj + turnSpeed) * 100);
        drivetrain[i].SetPercent(percent);
        if (percent > 0) {
            lastDirection[i] = 1;
        } else {
            lastDirection[i] = -1;
        }
    }
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
// Drive at a given angle in degrees for a given distance in inches
// Optionally, give a speed from 0 to 1
void driveDistance(int angle, double inches, double speed = DEFAULT_SPEED) {
    const double numCounts = inches * COUNTS_PER_INCH;
    int totalCounts = 0;

    double direction[2] = {sin(angle * PI / 180.), cos(angle * PI / 180.)};
    for (int i = 0; i < NUM_WHEELS; i++) {
        totalCounts += abs((int)(projectOntoWheel(direction, i) * numCounts));

        // Reset each encoder
        encoders[i].ResetCounts();
    }
    // Set the velocity
    setVelocity(angle, speed);
    
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

// Turn a given angle using the encoders
// Optionally, give a speed from 0 to 1
void turnDegrees(int degrees, double speed = DEFAULT_SPEED) {
    const double totalCounts = fabs(degrees * COUNTS_PER_DEGREE * NUM_WHEELS);
    int actualSpeed = round(speed * 100);
    if (degrees > 0) actualSpeed *= -1;
    for (int i = 0; i < NUM_WHEELS; i++) {

        // Reset each encoder
        encoders[i].ResetCounts();
        
    }
    // Set the velocity
    for (int i = 0; i < NUM_WHEELS; i++) {

        // Reset each encoder
        drivetrain[i].SetPercent(actualSpeed);
        
    }
    
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


// RPS Functions --------------------------------------------------------h

double normalize(double num) {
    if (num == 0) return 0;
    const double CUTOFF = 25;
    const double MIN = 0.32;
    if (num > CUTOFF) return 1;
    else if (num < -CUTOFF) return -1;
    else if (num > 0) {
        double m = (1 - MIN) / CUTOFF;
        return m * num + MIN;
    } else {
        double m = (-MIN + 1) / CUTOFF;
        return m * num - MIN;
    }
}


// Go to a specific x, y, and heading using RPS and the encoders
// Optionally, pick a maximum error (in inches) and a translational speed and turn speed to drive at
void goTo(double toX, double toY, double toHeading, double error = DEFAULT_ERROR, double angleError = DEFAULT_ANGLE_ERROR, double speed = DEFAULT_SPEED, double turnSpeed = DEFAULT_TURN) {
    const double squaredError = error * error;

    // FEHFile * fptr = SD.FOpen("angles.txt", "w");

    // Last enocder counts
    double lastCounts[NUM_WHEELS];
    
    // Angle moved at for the last frame, relative to heading
    double lastAngle = 0;
    
    // Reset every encoder
    for (int i = 0; i < NUM_WHEELS; i++) {
        encoders[i].ResetCounts();
        lastCounts[i] = 0;
        lastDirection[i] = 1;
    }

    // Last updated rps values
    double lastX = RPS.X();
    double lastY = RPS.Y();
    double lastHeading = RPS.Heading();
    x = lastX;
    y = lastY;
    theta = lastHeading;

    
    /*
    // Reset the deque
    Node * cur = first;
    while (cur != nullptr) {
        cur->angle = 0;
        cur->ds = 0;
        cur->dtheta = 0;
        cur = cur->next;
    }
    */ 
    double errorStartTime = -1;
    while (true) {
        double startTime = TimeNow();
        
        // Check if RPS has updated in the last frame
        if (RPS.X() != lastX || RPS.Y() != lastY || RPS.Heading() != lastHeading) {

            // LCD.Clear();
            // LCD.Write("First if");
            // RPS has updated
            // update "last" values
            lastX = RPS.X();
            lastY = RPS.Y();
            lastHeading = RPS.Heading();
            // LCD.WriteLine(lastX);
            // If in deadzone or out of the area, break out of the loop
            if (lastX < 0)  {
                if (errorStartTime == -1)
                    errorStartTime = TimeNow();
                else {
                    if (TimeNow() - errorStartTime >= 0.4) {
                        stop();
                        return;
                    }
                }
                
            } else {
                errorStartTime = -1;
                // Update the current x and y by going through the deque
                x = lastX;
                y = lastY;
                theta = lastHeading;
                // Node * cur = first;
                // LCD.Write("beginning deque update while loop");
                /*while (cur != nullptr) {
                    x += cur->ds * cos((theta + cur->angle + cur->dtheta / 2) * PI / 180);
                    y += cur->ds * sin((theta + cur->angle + cur->dtheta / 2) * PI / 180);
                    theta += cur->dtheta;
                    cur = cur->next;
                }*/
                }
            }
        
        // LCD.Write("First if complete");

        // Figure out distance traveled from last sleep using encoder counts
        /*double dCounts[NUM_WHEELS];
        int totalCounts = 0;
        for (int i = 0; i < NUM_WHEELS; i++) {
            dCounts[i] = lastDirection[i] * (encoders[i].Counts() - lastCounts[i]);
            lastCounts[i] = encoders[i].Counts();
            totalCounts += dCounts[i];
        }
        double dtheta = totalCounts * 1.0 / NUM_WHEELS / COUNTS_PER_DEGREE;

        
        // Assume you were moving at the correct angle and find distance
        double ds = 0;
        int num = 0;
        for (int i = 0; i < NUM_WHEELS; i++) {
            // Ignore the angular motion
            dCounts[i] -= round(totalCounts / 3.0);
            // Find the distance according to this wheel
            if (fabs(cos(WHEEL_ANGLES[i] - lastAngle * PI /180)) > 0.05) {
                ds += fabs(dCounts[i] / COUNTS_PER_INCH/cos(WHEEL_ANGLES[i] - lastAngle * PI /180));
                num++;
            }
        }
        if (num != 0)
            ds /= num;
        

        // Add it to the deque (write over the first element)
        first->dtheta = dtheta;
        first->angle = lastAngle;
        first->ds = ds;

        // Put the first element of the deque at the end
        last->next = first;
        first->prev = last;
        first = first->next;
        last = first->prev;
        last->next = nullptr;
        first->prev = nullptr;

        // Update current position
        x += ds * cos((theta + lastAngle) * PI / 180);
        y += ds * sin((theta + lastAngle) * PI / 180);
        theta += dtheta;
        */
        // Set the speed
        double dx = toX - x;
        double dy = toY - y;
        double rSquared = dx * dx + dy * dy;
        double dAngle = toHeading - theta;
        while (fabs(dAngle) > 180) {
            if (dAngle < -180) {
                dAngle += 360;
            } else if (dAngle > 180) {
                dAngle -= 360;
            }
        }
        // If you are close enough, then stop
        if (rSquared < squaredError && fabs(dAngle) < angleError)  {
            // LCD.WriteLine("Broken");
            stop();
            return;
        }

        // Get speeds (step with tanh so that you slow down when near the desired value)
        double v = speed * normalize(rSquared);
        
        double vTurn = turnSpeed * normalize(dAngle / 3);
        double absoluteAngle = atan2(dy, dx) * 180 / PI;

        // LCD.Clear();
        // LCD.WriteLine(rSquared);
        // LCD.WriteLine(dAngle);
        // SD.FPrintf(fptr, "dx: %f, dy: %f, dangle: %f, ds: %f, angle: %f, dtheta: %f\n",x, y, dAngle,  ds, absoluteAngle, dtheta);
        // Set the actual speed
        setVelocityAndTurn(absoluteAngle, v, vTurn);

        // Update lastAngle to the relativeAngle
        lastAngle = absoluteAngle - theta;

        // Wait a bit
        double timeElapsed = TimeNow() - startTime;
        if (timeElapsed < CLOCK)
            Sleep(CLOCK - timeElapsed);

    }

    // Stop the motors
    stop();
}


//Go to with an array for x y and heading
void goTo(double to[3], double error = DEFAULT_ERROR, double angleError = DEFAULT_ANGLE_ERROR, double speed = DEFAULT_SPEED, double turnSpeed = DEFAULT_TURN) {
    goTo(to[0], to[1], to[2], error, angleError, speed, turnSpeed);
}

// CdS Functions ---------------------------------------------------------
 

 // Enum for colors

enum COLORS {
    NONE, BLUE_LIGHT, RED_LIGHT
};

 // gets the color of loight using the CdS cell
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
    driveInCircle(.6, 5);
    turn(true, 2000);
    turn(false, 2000);
}



// Functions for performance ----------------------------------------



void testSpeeds() {
    FEHFile* fptr = SD.FOpen("speeds.txt", "w");
    SD.FPrintf(fptr, "Percent\tTotal_Encoder_Counts\n");
    for (int i = 0; i <= 60; i+=5) {
        for (int j = 0; j < 3; j++) {
            encoders[j].ResetCounts();
        }
        turn(i / 100.0, 1000, true, 500);
        int totalCounts = 0;
        for (int j = 0; j < 3; j++) {
            totalCounts += encoders[j].Counts();
        }
        SD.FPrintf(fptr, "%d\t%d\n", i, totalCounts);

    }
    SD.FClose(fptr);
}


// Initialize everything
void initializeBot() {
    for (int i = 0; i < NUM_WHEELS; i++) {
        WHEEL_DIRECTIONS[i][0] = cos(WHEEL_ANGLES[i]);
        WHEEL_DIRECTIONS[i][1] = sin(WHEEL_ANGLES[i]);
    }
    arm.SetMin(SERVO_MIN);
    arm.SetMax(SERVO_MAX);
    arm.SetDegree(180);
    RPS.InitializeTouchMenu();
    // Find latency
    /*double curY = RPS.Y();
    setVelocity(0,0.5);
    double startTime = TimeNow();
    while (RPS.Y() - curY <= 0.2);
    latency = TimeNow() - startTime;
    stop();

    
    // Initialize the Deque
    dequeSize = round(latency / CLOCK);
    first = (Node * )malloc(sizeof(Node));
    first->next = nullptr;
    last = first;
    for (int i = 1; i < dequeSize; i++) {
        Node * newNode = (Node * )malloc(sizeof(Node));
        
        newNode->next = first;
        first->prev = newNode;
        first = newNode;
    }
    first->prev = nullptr;
    */

}

// Store the current RPS values in global position varaibles
void setRPSVals() {
    x = RPS.X();
    y = RPS.Y();
    theta = RPS.Heading();
}

// Set things
const int NUM_LOCATIONS = 4;
double locations[NUM_LOCATIONS][3];
int SERVO_LOCATIONS[NUM_LOCATIONS] = {
    180, 20, 30, 180
};

void setLocations () {
    for (int i = 0; i < NUM_LOCATIONS; i++) {
        
        int x, y;
        while(!LCD.Touch(&x, &y)) {
            arm.SetDegree(SERVO_LOCATIONS[i]);
            LCD.Clear();
            LCD.Write("Click for location ");
            LCD.WriteLine(i + 1);
            LCD.WriteLine("X,Y,Heading:");
            LCD.WriteLine(RPS.X());
            LCD.WriteLine(RPS.Y());
            LCD.WriteLine(RPS.Heading());
            Sleep(20);
        }
        while(LCD.Touch(&x, &y));
        locations[i][0] = RPS.X();
        locations[i][1] = RPS.Y();
        locations[i][2] = RPS.Heading();
        if (locations[i][0] < 0) i--;
        Sleep(1000);
        LCD.Touch(&x, &y);
    }
    arm.SetDegree(180);
}

// We are the champions song
const int SONG_LENGTH = 55;
int song[][2] = {
    {FEHBuzzer::C4, 4},
    {FEHBuzzer::B3, 1},
    {FEHBuzzer::C4, 1},
    {FEHBuzzer::B3, 2},
    {FEHBuzzer::G3, 3},
    {FEHBuzzer::E3, 1},
    {FEHBuzzer::A3, 2},
    {FEHBuzzer::E3, 2},
    {-1, 1},
    {FEHBuzzer::E3, 1},
    {FEHBuzzer::F3, 1},
    {-1, 1},
    {FEHBuzzer::F3, 1},
    {FEHBuzzer::G3, 1},
    {-1, 1},
    {FEHBuzzer::G3, 1},
    {FEHBuzzer::C4, 4},
    {FEHBuzzer::D4, 1},
    {FEHBuzzer::E4, 1},
    {FEHBuzzer::G4, 2},
    {FEHBuzzer::B3, 2},
    {FEHBuzzer::A3, 1},
    {FEHBuzzer::B3, 1},
    {FEHBuzzer::A3, 2},
    {-1, 1},
    {FEHBuzzer::A3, 1},
    {FEHBuzzer::Af3, 1},
    {FEHBuzzer::G3, 1},
    {FEHBuzzer::Gf3, 5},
    {-1, 1},
    {FEHBuzzer::A3, 3},
    {FEHBuzzer::G3, 2},
    {FEHBuzzer::A3, 1},
    {FEHBuzzer::G3, 3},
    {FEHBuzzer::F3, 3},
    {FEHBuzzer::F4, 3},
    {FEHBuzzer::E4, 2},
    {FEHBuzzer::F4, 1},
    {FEHBuzzer::E4, 3},
    {FEHBuzzer::D4, 3},
    {FEHBuzzer::E4, 3},
    {FEHBuzzer::C4, 2},
    {FEHBuzzer::F4, 1},
    {FEHBuzzer::E4, 3},
    {FEHBuzzer::C4, 2},
    {FEHBuzzer::F4, 1},
    {FEHBuzzer::Ef4, 3},
    {FEHBuzzer::C4, 2},
    {FEHBuzzer::F4, 1},
    {FEHBuzzer::Ef4, 3},
    {FEHBuzzer::C4, 3},
    {-1, 4},
    {FEHBuzzer::Bf3, 1},
    {FEHBuzzer::G3, 1},
    {FEHBuzzer::C4, 12}

};


// Play a song given a 2d array of ints (frequency and beats for each note), length of the array, and bpm
void playSong(int tones[][2], int length, int bpm) {
    // Milliseconds per beat
    const int MILLIS_PB = 1.0 / bpm * 60 * 1000;
    for (int i = 0; i < length; i++) {
        // Get the time
        int millis = tones[i][1] * MILLIS_PB;

        // If frequency is -1, rest
        if (tones[i][0] == -1) {
            Sleep(millis);
        } else {
            // Otherwise, play the tone
            Buzzer.Tone(tones[i][0], millis);
        }
    }
}



// Final task, do everything
void challenge() {
    // Step 1: Deposit tray ----------------------------------------------
    // Drive to the ramp
    driveDistance(-50, 17);
    turnDegrees(-25);
    
    // Drive up the ramp
    driveDistance(0, 26, 0.6);

    // Drive to the sink
    driveDistance(-90,12, .6);
    drive(180, 600);
    
    // Deposit tray
    arm.SetDegree(50);

    Sleep(200);
    arm.SetDegree(180);
    
    // Drive away a bit
    driveDistance(80, 4);

    // Step 2: flip ice cream lever --------------------------------------
    // Go to area
    goTo(locations[0], .2, 2, .3);

    // Read and move to the correct lever
    switch (RPS.GetIceCream()) {
        case 0:
            driveDistance(150, 7);
            break;
        case 1:
            driveDistance(180, 5);
            break;
        case 2:
            driveDistance(-150, 7);
            break;
    }
    // Flip
    arm.SetDegree(80);
    Sleep(800);
    arm.SetDegree(180);

    // Drive away a bit
    driveDistance(0, 6, .6);

    // Step 3: Burger flip ---------------------------------------------------
    // Go to burger
    arm.SetDegree(15);
    goTo(locations[1], .2, 1);

    // Flip it
    arm.SetDegree(70);
    Sleep(500);
    turn(true, 500);
    turn(false, 100);

    // Bring the stove back
    drive(0., 250, 0.5, false);
    drive(-90, 350, 0.5, false);
    drive(180, 250, 0.5, false);
    drive(90, 350, 0.5, false);

    // Drive away
    driveDistance(45, 5, .6);

    // Step 4: Unflip ice cream ------------------------------------------------  
    // Go to the area  
    arm.SetDegree(0);
    goTo(locations[0], .2, 2, .3);

    // Read and move to the correct lever
    switch (RPS.GetIceCream()) {
        case 0:
            driveDistance(150, 6);
            break;
        case 1:
            driveDistance(180, 4.5);
            break;
        case 2:
            driveDistance(-150, 6);
            break;
    }

    // Flip
    arm.SetDegree(120);
    Sleep(500);
    driveDistance(0, 2, .2);
    arm.SetDegree(0);

    // Drive away a bit
    driveDistance(0, 4, .6);

    // Step 5: Slide receipt ----------------------------------------
    // Go to the receipt
    arm.SetDegree(30);
    goTo(locations[2]);
    
    drive(180, 500);

    // Hook the receipt
    turnDegrees(15);

    // Pull receipt
    driveDistance(-90, 6, .6);

    // Step 6: Click the correct jukebox button -----------------------------------
    // Drive away from receipt
    driveDistance(0, 2, .6);
    arm.SetDegree(180);

    // Go to ramp
    driveDistance(-90, 6, .6);
    
    // Drive down the ramp
    driveDistance(180, 20, .6);

    // Drive towards the light
    driveDistance(225, 6, .6);

    // Go to the light
    goTo(locations[3], DEFAULT_ERROR, 1.0);

    // get light value
    COLORS color=getColor();
    
    // Hit correct button
    Sleep(DEFAULT_STOP_TIME);
    if(color==BLUE_LIGHT){
        // hit blue
        LCD.WriteLine("Blue");
        drive(160,700);
        
    }else{
        // hit red
        LCD.WriteLine("Red");
        driveDistance(250,5);
        drive(180, 400);
        turn(true, 100, false);
        drive(180, 50, false);
        
    }
    // Drive away a bit
    driveDistance(45, 6, .6);

    // Step 7: press final button ---------------------------
    goTo(21., 16., 48., .5, 3., .4);
    setVelocity(180, DEFAULT_SPEED);
    
}


// Do a celebration dance!!!!!!!
void celebrationDance() {
    int colors[] = {RED, ORANGE, YELLOW, GREEN, BLUE, INDIGO, VIOLET};
    int j = 0;
    LCD.SetBackgroundColor(colors[j]);
    LCD.Clear();
    j++;
    j %= 7;
    
    for (int i = 0; i < NUM_WHEELS; i++) {
        WHEEL_DIRECTIONS[i][0] = cos(WHEEL_ANGLES[i]);
        WHEEL_DIRECTIONS[i][1] = sin(WHEEL_ANGLES[i]);
    }
    arm.SetMin(SERVO_MIN);
    arm.SetMax(SERVO_MAX);
    arm.SetDegree(180);
    driveDistance(0, 3, .6);
    LCD.SetBackgroundColor(colors[j]);
    LCD.Clear();
    j++;
    j %= 7;
    arm.SetDegree(0);
    driveDistance(180, 3, .6);
    LCD.SetBackgroundColor(colors[j]);
    LCD.Clear();
    j++;
    j %= 7;
    arm.SetDegree(100);
    driveDistance(90, 1.5, .6);
    LCD.SetBackgroundColor(colors[j]);
    LCD.Clear();
    j++;
    j %= 7;
    driveDistance(-90, 3, .6);
    LCD.SetBackgroundColor(colors[j]);
    LCD.Clear();
    j++;
    j %= 7;
    driveDistance(90, 1.5, .6);
    LCD.SetBackgroundColor(colors[j]);
    LCD.Clear();
    j++;
    j %= 7;
    turnDegrees(30, .6);
    LCD.SetBackgroundColor(colors[j]);
    LCD.Clear();
    j++;
    j %= 7;
    turnDegrees(-60, .6);
    LCD.SetBackgroundColor(colors[j]);
    LCD.Clear();
    j++;
    j %= 7;
    arm.SetDegree(180);
    turnDegrees(340, .6);
    LCD.SetBackgroundColor(colors[j]);
    LCD.Clear();
    j++;
    j %= 7;
    driveInCircle(.6,5);
    LCD.SetBackgroundColor(colors[j]);
    LCD.Clear();
    j++;
    j %= 7;
    Sleep(100);
    arm.SetDegree(0);
    turn(true, 300);
    LCD.SetBackgroundColor(colors[j]);
    LCD.Clear();
    j++;
    j %= 7;
    arm.SetDegree(180);
    turn(false, 300);
    LCD.SetBackgroundColor(colors[j]);
    LCD.Clear();
    j++;
    j %= 7;

    // Play the song
    playSong(song, SONG_LENGTH, 400);
}


// Test to make sure everything works
void testSensors () {
    for (int i = 0; i < NUM_WHEELS; i++) {
        WHEEL_DIRECTIONS[i][0] = cos(WHEEL_ANGLES[i]);
        WHEEL_DIRECTIONS[i][1] = sin(WHEEL_ANGLES[i]);
    }
    LCD.WriteLine(Battery.Voltage());
    arm.SetMin(SERVO_MIN);
    arm.SetMax(SERVO_MAX);
    arm.SetDegree(180);
    Sleep(500);
    arm.SetDegree(0);
    Sleep(500);
    
    for (int i = 0; i < 200; i++) {
        LCD.Clear();
        LCD.WriteLine(CDS.Value());
        Sleep(10);
    }
    for (int i = 0; i < NUM_WHEELS; i++) {
        encoders[i].ResetCounts();
        drivetrain[i].SetPercent(40);
        Sleep(400);
        stop();
        LCD.Clear();
        LCD.WriteLine(encoders[i].Counts());
    }
    
}


int main(void)
{

    // celebrationDance();
    
    //testSensors();

    initializeBot();
    
    setLocations(); 
    
    // Wait for starting light
    LCD.Clear();
    LCD.WriteLine("Waiting for touch...");
    int x, y;
    while(!LCD.Touch(&x, &y));
    while(LCD.Touch(&x, &y));
    LCD.Clear();
    while (getColor() != RED_LIGHT) {
        Sleep(5);
    }

    setRPSVals();
    challenge();
    // celebrationDance();

    /*
        Performance Test 1
            ***MAKE SURE ROBOT IS IN THE RIGHT STARTING POSITION***
    */

   /*
   
   
   
   

    // drive to light
    driveDistance(278, 16.6);
    Sleep(DEFAULT_STOP_TIME);
    // get light value
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


    */

    // Performance task 2 -------------------------------------------
    
    // Drive to the ramp
    /*
    driveDistance(-50, 17);
    turn(false, 260);
    
    // Drive up the ramp
    driveDistance(0, 24, 0.6);

    // Drive to the sink
    driveDistance(-90,12);
    
    // Deposit tray
    arm.SetDegree(50);

    Sleep(200);
    arm.SetDegree(180);

    // Drive to the receipt
    driveDistance(90, 23);

    // Bring down the arm and drive up to the receipt
    arm.SetDegree(30);
    driveDistance(176, 7.4);
    
    // Pull receipt
    driveDistance(-90, 6);

    // Touch the burger
    driveDistance(30, 19);
    drive(20, 1500, .2);
    */
    
    /*LCD.WriteLine("started!");
    // goTo(17.0, 23.0, 0);
    goTo(locations[0]);
    driveDistance(0, 20, 0.6);
    arm.SetDegree(0);
    // goTo(26.7, 57.7, 180);
    goTo(locations[1]);
    arm.SetDegree(80);
    Sleep(500);
    turn(true, 500);
    turn(false, 100);
    drive(0, 300);
    drive(-90, 400);
    drive(180, 300);
    drive(90, 400);
    arm.SetDegree(180);
    goTo(locations[2], .2, 2, .3);
    driveDistance(180, 5);
    arm.SetDegree(80);
    LCD.WriteLine("Done!");*/
    // Performance task 4 --------------------------------------
    /*
    goTo(locations[0]);
    driveDistance(0, 20, 0.6);
    goTo(locations[1], .2, 2, .3);
    switch (RPS.GetIceCream()) {
        case 0:
            LCD.WriteLine("Vanilla");
            driveDistance(150, 7);
            break;
        case 1:
            LCD.WriteLine("Twist");
            driveDistance(180, 5.6);
            break;
        case 2:
            LCD.WriteLine("Chocolate");
            driveDistance(-150, 7);
            break;
    }
    arm.SetDegree(80);

    Sleep(7000);
    arm.SetDegree(180);
    driveDistance(0, 3);
    arm.SetDegree(0);
    Sleep(500);
    driveDistance(180, 2.4);
    arm.SetDegree(120);
    Sleep(600);
    arm.SetDegree(0);
    Sleep(100);
    driveDistance(0, 6);
    arm.SetDegree(180);

    goTo(locations[2]);
    driveDistance(0, 15);
    goTo(locations[3]);
    drive(180, 2000);
    */
   
   
}   

