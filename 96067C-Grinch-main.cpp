/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Moises                                                      */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Intake               motor_group   8, 9            
// PneumaticsA          digital_out   A               
// PneumaticsB          digital_out   B               
// PneumaticsC          digital_out   C               
// Inertial             inertial      10              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"


using namespace vex;


// A global instance of competition
competition Competition;

brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor IntakeMotorA = motor(PORT8, ratio6_1, false);
motor IntakeMotorB = motor(PORT9, ratio6_1, false);
motor_group Intake = motor_group(IntakeMotorA, IntakeMotorB);
digital_out PneumaticsA = digital_out(Brain.ThreeWirePort.A);
digital_out PneumaticsB = digital_out(Brain.ThreeWirePort.B);
digital_out PneumaticsC = digital_out(Brain.ThreeWirePort.C);
inertial Inertial = inertial(PORT10);


// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

motor leftMotorA = motor(PORT11, ratio6_1, true);
motor leftMotorB = motor(PORT5, ratio6_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT6, ratio6_1, false);
motor rightMotorB = motor(PORT7, ratio6_1, false);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 299.24, 330, 241, mm, 1);
motor LiftMotorA = motor(PORT1, ratio6_1, false);
motor LiftMotorB = motor(PORT2, ratio6_1, true);
motor_group Lift = motor_group(LiftMotorA, LiftMotorB);

void pre_auton() {
  // Start calibration
  Inertial.calibrate();
  Brain.Screen.print("Calibrating Inertial...");

  // Wait for calibration to complete
  while (Inertial.isCalibrating()) {
    wait(100, msec);
  }

  Brain.Screen.clearScreen();
  Brain.Screen.print("Inertial Calibration Complete!");
}

/**
 * Drive the robot in reverse for a specific speed and duration.
 * @param speed The speed of the drivetrain (percentage).
 * @param duration The distance to travel (mm).
 */
void driveReverse(double speed, double duration) {
  Drivetrain.setDriveVelocity(speed, pct);
  Drivetrain.driveFor(directionType::rev, duration, mm);
}

void driveForward(double speed, double duration) {
  Drivetrain.setDriveVelocity(speed, pct);
  Drivetrain.driveFor(directionType::fwd, duration, mm);
}



void turnToAngle(double targetAngle) {
  const double kP = 0.4;    // Proportional gain constant
  const double kI = 0.0008; // Integral gain constant
  const double kD = 0.24;    // Derivative gain constant

  double error;
  double previousError = 0;
  double integral = 0;
  double derivative;

  while (true) {
    error = targetAngle - Inertial.rotation(); // Calculate the current error
    integral += error; // Accumulate the error for integral term
    derivative =
        error - previousError; // Calculate change in error for derivative term

    // Calculate turning speed using PID control formula
    double turnSpeed = (error * kP) + (integral * kI) + (derivative * kD);

    // Clamp turnSpeed to avoid excessive speeds
    if (turnSpeed > 100)
      turnSpeed = 100;
    if (turnSpeed < -100)
      turnSpeed = -100;

    // Spin motors in opposite directions to turn
    LeftDriveSmart.spin(directionType::fwd, turnSpeed, pct);
    RightDriveSmart.spin(directionType::rev, turnSpeed, pct);

    // Stop turning if the error is within 1 degree
    if (fabs(error) < 1) {
      LeftDriveSmart.stop();
      RightDriveSmart.stop();
      break;
    }

    previousError = error; // Store the current error for the next loop
    task::sleep(20);       // Small delay to prevent CPU overload
  }
}



void autonomous(void) {
  vexcodeInit();
   Intake.setVelocity(100, pct);

  driveReverse(40, 800);
  turnToAngle(-25);
  driveReverse(12.5, 310);
  PneumaticsA.set(true); // Activate pneumatics to secure the goal
  PneumaticsB.set(true);
  task::sleep(200); // Wait for pneumatics to fully engage
  Intake.spin(reverse);
  wait(.75, seconds);
  Intake.spin(fwd);
  wait(.25, seconds);
  turnToAngle(-105); // Turn to face the next ring
  Intake.spin(reverse);
  driveForward(40,700);
  turnToAngle(-167);
  driveForward(60, 600); // Move towards the third ring
  wait(.3, seconds);
  driveReverse(40, 125);
  turnToAngle(-142.5); // Final alignment 
  driveForward(40, 310); // move towards third ring and collect 4
  wait(2.5, seconds);
  Intake.stop();// stop intke after collecting the fourth ring
  driveReverse(40, 300); // move back , away from the line
  turnToAngle(-233); // turn to face the ladder
  driveForward(40, 575); // drive and hit the ldder
  
 
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
// Target positions for lift
int liftUpPosition = 1600;  // Adjust this to your desired height in decidegrees
int liftDownPosition = 0;  // Original 0 degree position

bool L1Pressed = false;
bool L2Pressed = false;

void controlLift() {
    // Lift Up to set height
    if (Controller1.ButtonL1.pressing() && !L1Pressed) {
        L1Pressed = true;
        L2Pressed = false;
        Lift.spinToPosition(liftUpPosition, degrees, 80, velocityUnits::pct, true);
    }
    
    // Reset Down to 0 position
    if (Controller1.ButtonL2.pressing() && !L2Pressed) {
        L2Pressed = true;
        L1Pressed = false;
        Lift.spinToPosition(liftDownPosition, degrees, 80, velocityUnits::pct, true);
    }
}
void usercontrol(void) {
  // User control code here, inside the loop
  bool pneumaticsClosed = false;
  bool armClosed = false;
  

  
  // User control code here, inside the loop
  while (1) {
    controlLift();
        Lift.stop(hold);
        

    Drivetrain.setDriveVelocity(70, pct);
    Drivetrain.setTurnVelocity(20, pct);
    Intake.setVelocity(80, pct);

    int leftStickX = Controller1.Axis1.position();
    int leftStickY = Controller1.Axis3.position();

    Drivetrain.arcade(leftStickY, leftStickX);
    if (Controller1.ButtonR1.pressing()) {
      Intake.spin(reverse);
    } else if (Controller1.ButtonR2.pressing()) {
      Intake.spin(fwd);
    } else {
      Intake.stop(coast);
    }

    if (Controller1.ButtonA.pressing()) {
      if (pneumaticsClosed) {
        PneumaticsA.set(true); // Extend pneumatics
        PneumaticsB.set(true);
        pneumaticsClosed = false; // Update state to open
      } else {
        PneumaticsA.set(false); // Retract pneumatics
        PneumaticsB.set(false);
        pneumaticsClosed = true; // Update state to closed
      }
      this_thread::sleep_for(500); // Debounce delay ensures debounce and
                                   // prevents rapid state toggling.
    }

    if (Controller1.ButtonX.pressing()) {
      if (armClosed) {
        PneumaticsC.set(true);
        armClosed = false;
      } else {
        PneumaticsC.set(false);
        armClosed = true;
      }
      this_thread::sleep_for(500);
    }

    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
