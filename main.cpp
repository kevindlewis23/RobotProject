#include <math.h>
#include <FEHLCD.h>
#include <FEHMotor.h>
#include <FEHIO.h>
#include <FEHServo.h>
#include <FEHSD.h>
#include <FEHRPS.h>
#include <stdlib.h>

#define PI 3.14159
#define DEFAULT_STOP_TIME 200
#define DEFAULT_SPEED 0.4
#define DEFAULT_TURN 0.2

// Acceptable error
#define DEFAULT_ERROR 0.3
#define DEFAULT_ANGLE_ERROR 1

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
#define SERVO_MAX 2300

// Movement update speed
#define CLOCK .005

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

FEHServo arm(FEHServo::Servo7);

// Encode CPI
const int COUNTS_PER_INCH = COUNTS_PER_ROTATION / (PI * WHEEL_DIAMETER);
const double COUNTS_PER_DEGREE = COUNTS_PER_INCH * (2 * PI * BODY_RADIUS) / 360;


// Location/RPS variables
// Latency in milliseconds
double latency = 0;
double x = 0;
double y = 0;
double theta = 0;


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
    double angle = (degrees - theta) * PI / 180;
    double direction[2] = {cos(angle) * speed, sin(angle) * speed};
    for (int i = 0; i < NUM_WHEELS; i++) {
        // Get the scalar projection of the velocity onto the wheel direction
        double proj = projectOntoWheel(direction, i);
        int percent = (int)((proj + turnSpeed) * 100);
        drivetrain[i].SetPercent(percent);
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


// RPS Functions --------------------------------------------------------

// Go to a specific x, y, and heading using RPS and the encoders
// Optionally, pick a maximum error (in inches) and a translational speed and turn speed to drive at
void goTo(double toX, double toY, double toHeading, double error = DEFAULT_ERROR, double angleError = DEFAULT_ANGLE_ERROR, double speed = DEFAULT_SPEED, double turnSpeed = DEFAULT_TURN) {
    const double squaredError = error * error;

    // Last enocder counts
    double lastCounts[NUM_WHEELS];
    // Last direction for each wheel, either 1 or negative 1, to figure out the sign of the counts 
    int lastDirection[NUM_WHEELS];
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

    

    // Reset the deque
    Node * cur = first;
    while (cur != nullptr) {
        cur->angle = 0;
        cur->ds = 0;
        cur->dtheta = 0;
        cur = cur->next;
    }

    while (true) {
        
        // Check if RPS has updated in the last frame
        if (RPS.X() != lastX || RPS.Y() != lastY || RPS.Heading() != lastHeading) {
            // RPS has updated
            // update "last" values
            lastX = RPS.X();
            lastY = RPS.Y();
            lastHeading = RPS.Heading();
            // If in deadzone or out of the area, break out of the loop
            if (lastX < 0) break;

            // Update the current x and y by going through the deque
            x = lastX;
            y = lastY;
            theta = lastHeading;
            Node * cur = first;
            while (cur != nullptr) {
                x += cur->ds * cos(theta + cur->angle);
                y += cur->ds * sin(theta + cur->angle);
                theta += cur->dtheta;
                cur = cur->next;
            }
        }

        // Figure out distance traveled from last sleep using encoder counts
        double dCounts[NUM_WHEELS];
        int totalCounts = 0;
        for (int i = 0; i < NUM_WHEELS; i++) {
            dCounts[i] = lastDirection[i] * (encoders[i].Counts() - lastCounts[i]);
            lastCounts[i] = encoders[i].Counts();
            totalCounts += dCounts[i];
        }
        double dtheta = totalCounts * 1.0 / NUM_WHEELS / COUNTS_PER_DEGREE;

        
        // Assume you were moving at the correct angle and find distance
        double ds = 0;
        for (int i = 0; i < NUM_WHEELS; i++) {
            // Ignore the angular motion
            dCounts[i] -= round(totalCounts / 3.0);
            // Find the distance according to this wheel
            ds += dCounts[i]/cos(lastAngle);
        }
        ds /= NUM_WHEELS;
        

        // Add it to the deque
        first->dtheta = dtheta;
        first->angle = lastAngle;
        first->ds = ds;

        last->next = first;
        first->prev = last;
        first = first->next;
        last->next->next = nullptr;
        last = first->prev;
        first->prev = nullptr;

        // Update current position
        x += ds * cos(theta + lastAngle);
        y += ds * sin(theta + lastAngle);
        theta += dtheta;

        // Set the speed
        double dx = toX - x;
        double dy = toY - y;
        double rSquared = dx * dx + dy * dy;
        double dAngle = toHeading - theta;
        // If you are close enough, then stop
        if (rSquared < squaredError && fabs(dAngle) < angleError) break;

        // Get speeds (step with tanh so that you slow down when near the desired value)
        double v = speed * tanh(rSquared);
        double vTurn = turnSpeed * tanh(dAngle);
        double absoluteAngle = atan2(dy, dx) * 180 / PI;

        // Set the actual speed
        setVelocityAndTurn(absoluteAngle, v, vTurn);

        // Update lastAngle to the relativeAngle
        lastAngle = absoluteAngle - theta;

        // Wait a bit
        Sleep(CLOCK);

    }

    // Stop the motors
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
    arm.SetMin(SERVO_MIN);
    arm.SetMax(SERVO_MAX);
    arm.SetDegree(180);
    RPS.InitializeTouchMenu();
    // Find latency
    double y = RPS.Y();
    setVelocity(0,0.5);
    double startTime = TimeNow();
    while (RPS.Y() - y <= 0.3);
    latency = TimeNow() - startTime;
    stop();

    
    // Initialize the Deque
    dequeSize = round(latency / CLOCK);
    first = (Node * )malloc(sizeof(Node));
    last = first;
    for (int i = 1; i < dequeSize; i++) {
        Node * newNode = (Node * )malloc(sizeof(Node));
        newNode->next = first;
        first->prev = newNode;
        first = newNode;
    }

}

// Store the current RPS values in global position varaibles
void setRPSVals() {
    x = RPS.X();
    y = RPS.Y();
    theta = RPS.Heading();
}

int main(void)
{
   

    initializeBot();
    // Wait for starting light
    while (getColor() != RED_LIGHT) {
        Sleep(5);
    }

    setRPSVals();

    /*
        Performance Test 1
            ***MAKE SURE ROBOT IS IN THE RIGHT STARTING POSITION***
    */

   /*
   
   
   
   

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


    */

    // Performance task 2 -------------------------------------------
    
    // Drive to the ramp
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


}   
