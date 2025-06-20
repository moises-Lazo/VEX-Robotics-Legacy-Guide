/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Moises Lazo                                               */
/*    Created:      Thu Dec 8 2025                                            */
/*    Last Edited:  Wed Feb 21 2025                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/


#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);

digital_out PneumaticsA = digital_out(Brain.ThreeWirePort.A);
digital_out PneumaticsB = digital_out(Brain.ThreeWirePort.B);
digital_out PneumaticsC = digital_out(Brain.ThreeWirePort.C);


// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

// Define motors and motor groups for the drivetrain
motor leftMotorA = motor(PORT10, ratio6_1, true);
motor leftMotorB = motor(PORT9, ratio6_1, false);
motor leftMotorC = motor(PORT7, ratio6_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);

motor rightMotorA = motor(PORT2, ratio6_1, false);
motor rightMotorB = motor(PORT3, ratio6_1, true);
motor rightMotorC = motor(PORT6, ratio6_1, false);
motor_group RightDriveSmart =
    motor_group(rightMotorA, rightMotorB, rightMotorC);

// Create drivetrain object with true dimensions and gearing

drivetrain Drivetrain =
    drivetrain(LeftDriveSmart, RightDriveSmart, 299.24, 304.8, 228.6, mm, 1);

// Define intake motor group
motor IntakeMotorA = motor(PORT8, ratio6_1, true);
motor IntakeMotorB = motor(PORT4, ratio6_1, false);
motor_group Intake = motor_group(IntakeMotorA, IntakeMotorB);

// Define additional sensors and devices
inertial Inertial = inertial(PORT5);
distance Distance = distance(PORT19);
optical Optical = optical(PORT20);

// calibrate inertial sensor prior to the comencment of autonomous
void pre_auton() { 
  // Start calibration
  Inertial.calibrate();
  Brain.Screen.print("Calibrating Inertial...");

  // Wait for calibration to complete
  while (Inertial.isCalibrating()) {
    wait(100, msec);
  }

  Brain.Screen.clearScreen();
  Brain.Screen.print(" Calibration Complete!");
}


// Functions to drive the robot forward and reverse at a given speed and duration
void driveForward(double speed, double duration) {
  Drivetrain.setDriveVelocity(speed, pct);
  Drivetrain.driveFor(directionType::fwd, duration, mm);
}

void driveReverse(double speed, double duration) {
  Drivetrain.setDriveVelocity(speed, pct);
  Drivetrain.driveFor(directionType::rev, duration, mm);
}

// PID-controlled turning function
void turnToAngle(double targetAngle) {
  const double kP = 0.5;    // Proportional gain constant
  const double kI = 0.0015; // Integral gain constant
  const double kD = 0.3;    // Derivative gain constant

  double error;
  double previousError = 0;
  double integral = 0;
  double derivative;

  while (true) {
    error = targetAngle - Inertial.rotation(); // Calculate the current error
    integral += error; // Accumulate the error for integral term
    derivative = error - previousError; // Calculate change in error for derivative term

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
   
  
  // Initialize intake settings

  Intake.setVelocity(95, pct);
  driveReverse(40, 850); // Drive backwards towards the goal
  turnToAngle(-27);       // Align hook with the mobile goal
    driveReverse(20, 300); // Drive backwards towards the goal

  PneumaticsA.set(true);  // Activate pneumatics to secure the goal
  PneumaticsB.set(true);
  task::sleep(200); // Wait for pneumatics to fully engage

  turnToAngle(-105); // Turn to face the next ring
  Intake.spin(fwd);
  driveForward(40,600);
  wait(.2,seconds);
  Intake.spin(reverse);
  wait(.075,seconds);

  Intake.stop();
  turnToAngle(-167);// Align for the third ring
  Intake.spin(fwd);   
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

/* The follwing void controlls the intake if an opposing colored ring is
 * detected in the intake system */

/* An optical sensor will detect what colored ring is midway within the intake
 * system */

/* If it detects an opposing color the distance sensor located at the top of the
 * system will wait until its within 40 mm distance to stop the intake for a
 * brief second*/

/* stoping the Intake for a brief second will allow the ring to be thrown out
 * using its momentum */

// Manual control of intake system with color-based rejection
bool manualIntake() {
  if (Controller1.ButtonR1.pressing()) {
    Intake.spin(directionType::fwd, 80, velocityUnits::pct);
    return true;
  } else if (Controller1.ButtonR2.pressing()) {
    Intake.spin(directionType::rev, 80, velocityUnits::pct);
    return true;
  } else {
    Intake.stop(coast);
    return false;
  }
}

// Automatic ring handling based on optical sensor input
void HandleRings() {
    Optical.setLightPower(100);
    Optical.setLight(ledState::on);
    if (Optical.color() == color::red ) {
      Intake.spin(fwd, 100, pct);
      waitUntil(Distance.objectDistance(mm) < 50);
        Intake.stop();
        wait(1, seconds);
        Intake.spin(fwd, 100, pct);
    }
      else{
          task::sleep(20);
         }
    }

void usercontrol(void) {
  bool pneumaticsClosed = false;    // Tracks the state of goal hook pneumatics
  bool armPneumaticsClosed = false; // Tracks the state of arm pneumatics

  while (1) {
    manualIntake();
    
    wait(20, msec);

    // Set drivetrain velocity for forward/backward motion (70% of max speed)
    Drivetrain.setDriveVelocity(70, percent);
    // Set turning velocity for left/right rotation (20% of max speed)
    Drivetrain.setTurnVelocity(20, percent);

    // Read joystick input for arcade control
    int leftStickX =
        Controller1.Axis1.position(); // Horizontal joystick position
    int leftStickY = Controller1.Axis3.position(); // Vertical joystick position

    // Arcade control for drivetrain (forward/backward and turn)
    Drivetrain.arcade(leftStickY, leftStickX);

    // Pneumatics control for goal hook
    if (Controller1.ButtonA.pressing()) {
      if (pneumaticsClosed) {
        PneumaticsA.set(true);
        PneumaticsB.set(true);
        pneumaticsClosed = false;
      } else {
        PneumaticsA.set(false);
        PneumaticsB.set(false);
        pneumaticsClosed = true;
      }
      this_thread::sleep_for(500); // Debounce delay
    }

    // Pneumatics control for arm
    if (Controller1.ButtonX.pressing()) {
      if (armPneumaticsClosed) {
        PneumaticsC.set(true);
        armPneumaticsClosed = false;
      } else {
        PneumaticsC.set(false);
        armPneumaticsClosed = true;
      }
      this_thread::sleep_for(500); // Debounce delay
    }

    wait(20, msec); // Prevent CPU overutilization by limiting loop rate
  }
}

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
