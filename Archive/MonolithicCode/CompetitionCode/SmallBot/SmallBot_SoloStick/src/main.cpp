#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
motor FL = motor(PORT11, ratio18_1, false);

motor FR = motor(PORT12, ratio18_1, true);

motor RL = motor(PORT13, ratio18_1, false);

motor RR = motor(PORT14, ratio18_1, true);

motor CL = motor(PORT19, ratio6_1, false);

motor CR = motor(PORT20, ratio6_1, true);

digital_out ConvLift = digital_out(Brain.ThreeWirePort.A);
controller Controller1 = controller(primary);


// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed(); 
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

/*                                           
__     __  _____  __  __      __     __  ____  
\ \   / / | ____| \ \/ /      \ \   / / | ___| 
 \ \ / /  |  _|    \  /        \ \ / /  |___ \ 
  \ V /   | |___   /  \         \ V /    ___) |
   \_/    |_____| /_/\_\         \_/    |____/ 

 ____   __  __   ___   _      _          ____    ___   _____ 
/ ___| |  \/  | / _ \ | |    | |        | __ )  / _ \ |_   _|
\___ \ | |\/| |/ /_\ \| |    | |        |  _ \ | | | |  | |  
 ___) || |  | ||  _  || |___ | |___     | |_) || |_| |  | |  
|____/ |_|  |_|| | | ||_____||_____|    |____/  \___/   |_|  
               \_| |_/                                       

*/

#include "vex.h"

using namespace vex;

vex::task batteryTask;

motor_group leftDrive(FL, RL);
motor_group rightDrive(FR, RR);
motor_group conveyorMotors(CL, CR);

void arcadeMode();
void tankMode();
void intakeControl();
void conveyorLift();
void preAutonomous();
void autonomous();
void userControl();
int batteryMonitor();
int main();
enum class selected {
  SOLO,
  SPLIT,
  TANK
};

// User Settings:
selected driveMode = selected::SOLO; // SOLO = solo arcade, SPLIT = split arcade, TANK = tank drive.
bool conveyorUp = false;
double conveyorSpeed = 100;
double rampRate = 4;

void arcadeMode(double rawFwd, double rawTrn) {
  // Four Motor Arcade Drive
  static double prevLeft = 0;
  static double prevRight = 0;
  
  // Deadband
  // Sticks are never at true 0, this prevents robot creep.
  if (fabs(rawFwd) < 5) rawFwd = 0;
  if (fabs(rawTrn) < 5) rawTrn = 0;

  // Input Shaping
  // Makes small joystick movements more precise.
  double fwdInput = (pow((rawFwd), 3)) / 10000; // Squaring would remove the negative sign, so multiplying by |input| preserves it.
  double trnInput = (pow((rawTrn), 3)) / 10000;

  // Curvature Scaling
  // Scales turning to be proportional to forward/reverse speed. Forces smoother turning and stability at higher speeds (prevents robot jerking sideways when turn input changed) while still allowing precise turns at low speeds.
  double trnScale = fabs(rawFwd) / 100;
  trnScale = fmax(trnScale, 0.2);
  trnInput *= trnScale;

  // Mix Inputs -> Outputs
  double leftOutput = fwdInput + trnInput;
  double rightOutput = fwdInput - trnInput;

  // Normalize Outputs
  // If either side exceeds motor limits (-100 to 100), scales both outputs down proportionally so that power differential ratio and turn behavior remain the same.
  double mostOutput = fmax(fabs(leftOutput), fabs(rightOutput));
  
  if (mostOutput > 100) {
    leftOutput = leftOutput * 100 / mostOutput;
    rightOutput = rightOutput * 100 / mostOutput;
  }

  // Power Ramping
  // Right now, the center of gravity of the robot sits behind the rear wheels, which makes the robot wheelie when going forward.
  // Left Ramp
  if (leftOutput > prevLeft + rampRate) {
    leftOutput = prevLeft + rampRate;
  } else if (leftOutput < prevLeft - rampRate) {
    leftOutput = prevLeft - rampRate;
  }
  // Right Ramp
  if (rightOutput > prevRight + rampRate) {
    rightOutput = prevRight + rampRate;
  } else if (rightOutput < prevRight - rampRate) {
    rightOutput = prevRight - rampRate;
  }

  prevLeft = leftOutput;
  prevRight = rightOutput;

  leftDrive.spin(forward, leftOutput, percent);
  rightDrive.spin(forward, rightOutput, percent);
}

void tankMode(double leftStick, double rightStick) {
// 2 Motor Tank Drive Code.
  double leftInput = leftStick;
  double rightInput = rightStick;

  // Deadband
  // Sticks are never at true 0, this prevents robot creep.
  if (fabs(leftInput) < 5) leftInput = 0;
  if (fabs(rightInput) < 5) rightInput = 0;

  // Input Shaping
  // Makes small joystick movements more precise.
  leftInput = (pow((leftInput), 3)) / 10000; // Squaring would remove the negative sign, so multiplying by |input| preserves it.
  rightInput = (pow((rightInput), 3)) / 10000;

  leftDrive.spin(forward, leftInput, percent);
  rightDrive.spin(forward, rightInput, percent);
}

void intakeControl() {
  bool L1 = Controller1.ButtonL1.pressing();
  bool R1 = Controller1.ButtonR1.pressing();
  
  if (R1) {
    conveyorMotors.spin(forward, conveyorSpeed, percent);
    return;
  }

  if (L1) {
    conveyorMotors.spin(reverse, conveyorSpeed, percent);
    return;
  }

  conveyorMotors.stop();
}

void conveyorLift() {
  bool buttonUp = Controller1.ButtonUp.pressing();
  static bool pressedUp = false;

  if (buttonUp && !pressedUp) {
    conveyorUp = !conveyorUp;
    ConvLift.set(conveyorUp);
  }

  pressedUp = buttonUp;
}

void preAutonomous(void) {
  vexcodeInit();
  Brain.Screen.clearScreen();
  Brain.Screen.print("Running Preautonomous Code...");

  rightDrive.setStopping(coast);
  leftDrive.setStopping(coast);
  conveyorMotors.setStopping(coast);
  batteryTask = vex::task(batteryMonitor); // Starts batteryMonitor task.
  leftDrive.setVelocity(100, percent);
  rightDrive.setVelocity(100, percent);

  wait(1, seconds);
}

void autonomous(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("autonomous code");
}

void userControl(void) {
  Brain.Screen.clearScreen();
  while (true) {
    // Set Axis Variables for Cleaner Code
    double axis1 = Controller1.Axis1.position();
    double axis2 = Controller1.Axis2.position();
    double axis3 = Controller1.Axis3.position();
    double axis4 = Controller1.Axis4.position();

    // Switch Between Drive Modes According to User Settings
    switch(driveMode) {
      case selected::SOLO:
        arcadeMode(axis3, axis4);
        break;

      case selected::SPLIT:
        arcadeMode(axis3, axis1);
        break;
      
      case selected::TANK:
        tankMode(axis3, axis2);
        break;
    }

    intakeControl();
    conveyorLift();

    wait(20, msec);
  }
}

int batteryMonitor() {
  bool warned50 = false;
  bool warned30 = false;
  bool warned20 = false;
  // For conditions to check whether warning already sent to controller first, so the controller isn't just constantly vibrating. - Ryan

  while (true) {
  
    double batteryLevel = Brain.Battery.capacity();

    // Check in reverse order, because if it checks that the battery is below 50% and it's true, it will just check that condition and the loop will continue. - Ryan
    if (batteryLevel <= 20.0 && !warned20) {
      
      Controller1.rumble(rumbleLong);
      warned20 = true;

    } else if (batteryLevel <= 30.0 && !warned30) {
      
      Controller1.rumble(rumbleShort);
      warned30 = true;

    } else if (batteryLevel <= 50.0 && !warned50) {
      
      Controller1.rumble(rumblePulse);
      warned50 = true;

    }
    wait(1000, msec); // Loop terminates every second so as not to hog CPU. - Ryan
  }
  return 0;
}

int main() {
  competition Competition;
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);
  preAutonomous(); // Runs preAutonomous.

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}