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
controller Controller1 = controller(primary);
motor LF = motor(PORT20, ratio6_1, false);

motor LM = motor(PORT19, ratio18_1, false);

motor LR = motor(PORT18, ratio6_1, false);

motor RF = motor(PORT10, ratio6_1, true);

motor RM = motor(PORT9, ratio18_1, true);

motor RR = motor(PORT8, ratio6_1, true);

motor ConveyorMotorR = motor(PORT7, ratio18_1, true);

motor ConveyorMotorL = motor(PORT6, ratio18_1, true);

motor ScoreMotor = motor(PORT17, ratio18_1, false);

digital_out Descorer = digital_out(Brain.ThreeWirePort.G);
digital_out Lift = digital_out(Brain.ThreeWirePort.H);


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

   ____   ___   ____      ____    ___   _____ 
  | __ ) |_ _| / ___|    | __ )  / _ \ |_   _|
  |  _ \  | | | |  _     |  _ \ | | | |  | |  
  | |_) | | | | |_| |    | |_) || |_| |  | |  
  |____/ |___| \____|    |____/  \___/   |_|                                          

*/

// Include the V5 Library
#include "vex.h"

// Allows for easier use of the VEX Library
using namespace vex;
vex::task batteryTask;

motor_group leftDrive(LF, LM, LR);
motor_group rightDrive(RF, RM, RR);
motor_group conveyorMotors(ConveyorMotorL, ConveyorMotorR);

void arcadeMode();
void tankMode();
void conveyorControl();
void scoreControl();
void descorerControl();
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
selected driveMode = selected::SPLIT; // SOLO = solo arcade, SPLIT = split arcade, TANK = tank drive.
bool liftDown = false;   // Allows us to keep track of state of Lift Pneumatic.
bool descorerUp = false; // Allows us to keep track of state of Descorer Pneumatic.
double rampRate = 4;


void arcadeMode(double rawFwd, double rawTrn) {
  // Six Motor Arcade Drive
  static double prevLeft = 0;
  static double prevRight = 0;

  // Reverse inputs
  // For the life of me, I cannot figure out why, but the inputs are reversed so forward = reverse and left = right. This reverses the behavior.
  rawFwd = -rawFwd;
  rawTrn = -rawTrn;
  
  // Deadband
  // Sticks are never at true 0, this prevents robot creep.
  if (fabs(rawFwd) < 5) rawFwd = 0;
  if (fabs(rawTrn) < 5) rawTrn = 0;

  // Input Shaping
  // Makes small joystick movements more precise.
  double fwdInput = rawFwd * fabs(rawFwd) / 100; // Squaring would remove the negative sign, so multiplying by |input| preserves it.
  double trnInput = rawTrn * fabs(rawTrn) / 100;

  // Curvature Scaling
  // Scales turning to be proportional to forward/reverse speed. Forces smoother turning and stability at higher speeds (prevents robot jerking sideways when turn input changed) while still allowing precise turns at low speeds.
  double trnScale = fabs(rawFwd) / 100;
  trnScale = fmax(trnScale, 0.4);
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

  // Output Power -> Motors
  leftDrive.spin(forward, leftOutput, percent);
  rightDrive.spin(forward, rightOutput, percent);
}

void tankMode(double leftStick, double rightStick) {
  // 6 Motor Tank Drive
  double leftInput = leftStick;
  double rightInput = rightStick;
  static double leftOutput = 0;
  static double rightOutput = 0;
  static double prevLeft = 0;
  static double prevRight = 0;

  // Deadband
  // Sticks are never at true 0, this prevents robot creep.
  if (fabs(leftInput) < 5) leftInput = 0;
  if (fabs(rightInput) < 5) rightInput = 0;

  // Input Shaping
  // Makes small joystick movements more precise.
  leftInput = (pow((leftInput), 3)) / 10000; // Squaring would remove the negative sign, so multiplying by |input| preserves it.
  rightInput = (pow((rightInput), 3)) / 10000;
  
  // Normalize Outputs
  // If either side exceeds motor limits (-100 to 100), scales both outputs down proportionally so that power differential ratio and turn behavior remain the same.
  double mostOutput = fmax(fabs(leftInput), fabs(rightInput));
  
  if (mostOutput > 100) {
    leftInput = leftInput * 100 / mostOutput;
    rightInput = rightInput * 100 / mostOutput;
  }

  leftOutput = leftInput;
  rightOutput = rightInput;

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

void conveyorControl() {
  bool R1 = Controller1.ButtonR1.pressing();
  bool L1 = Controller1.ButtonL1.pressing();
  conveyorMotors.setVelocity(100, percent);

  if (R1) {
    conveyorMotors.spin(forward);
  } else if (L1) {
    conveyorMotors.spin(reverse);
  } else {
    conveyorMotors.stop();
  }

}

void scoreControl() {
  bool R2 = Controller1.ButtonR2.pressing();
  bool L2 = Controller1.ButtonL2.pressing();
  ScoreMotor.setVelocity(100, percent);

  if (R2) {
    ScoreMotor.spin(forward); 
  } else if (L2) {
    ScoreMotor.spin(reverse); 
  } else { 
    ScoreMotor.stop();
  }
}

void descorerControl() {
  bool buttonUp = Controller1.ButtonUp.pressing();
  static bool pressedUp = false;

  if (buttonUp && !pressedUp) {
    descorerUp = !descorerUp;
    Descorer.set(descorerUp); 
  }

  pressedUp = buttonUp;
}

void liftControl() {
  bool buttonDown = Controller1.ButtonDown.pressing();
  static bool pressedDown = false;

  if (buttonDown && !pressedDown) {
    liftDown = !liftDown;
    Lift.set(liftDown); 
  }

  pressedDown = buttonDown;
}

void preAutonomous(void) {
  vexcodeInit();
  Brain.Screen.clearScreen();
  Brain.Screen.print("Running Preautonomous Code...");

  rightDrive.setStopping(coast);
  leftDrive.setStopping(coast);
  conveyorMotors.setStopping(coast);


  batteryTask = vex::task(batteryMonitor); // Starts batteryMonitor task.

  wait(1, seconds);
}

void autonomous(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("autonomous code");

  leftDrive.setVelocity(80, percent);
  rightDrive.setVelocity(80, percent);
  conveyorMotors.setVelocity(100, percent);
  ScoreMotor.setVelocity(100, percent);
  
  leftDrive.spinFor(forward, 2406, degrees, false);
  rightDrive.spinFor(forward, 2406, degrees, false);
  }

void userControl(void) {
  Brain.Screen.clearScreen();
  while(true) {
    // Set Axis Variables for Cleaner Code
    double axis1 = Controller1.Axis1.position();
    double axis2 = Controller1.Axis2.position();
    double axis3 = Controller1.Axis3.position();
    double axis4 = Controller1.Axis4.position();

    // Switch Between Drive Modes According to User Settings
    switch (driveMode) {
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
    
    // Other Functions
    conveyorControl();
    scoreControl();
    descorerControl();
    liftControl();
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