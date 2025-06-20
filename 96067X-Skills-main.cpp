/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:        main.cpp                                                 */
/*    Author:        Moises Lazo                                              */
/*    Team:          96067X                                                   */
/*    Competition:   Skills USA 3/29/2025                                     */
/*    Created Date:  3/14/2025                                                */
/*    Last Modified: 3/24/2025                                                */
/*                                                                            */
/*    Description:  Competition Code for VEX V5 Skills Challenge              */
/*                  - 6-Motor Drivetrain with Precision PID Turning           */
/*                  - Dual Pneumatic Lift System with Safety Limit            */
/*                  - Hook Mechanism for Game Object Manipulation             */
/*                  - Ring Intake System with Reverse Capability              */
/*                  - 1 minute Autonomous Scoring Routine                     */
/*----------------------------------------------------------------------------*/

// ---- VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port]         Description
// Controller1          controller                   Primary V5 Controller
// PushH                digital_out   H              Right Lift Pneumatic
// PushG                digital_out   G              Left Lift Pneumatic
// Hook                 digital_out   F              Scoring Hook Pneumatic
// LimSwitch            limit         A              Limit Switch Used With Lady Brown
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
using namespace vex;

// Competition Control Instance
competition Competition;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
digital_out PushH = digital_out(Brain.ThreeWirePort.H);
digital_out PushG = digital_out(Brain.ThreeWirePort.G);
digital_out Hook = digital_out(Brain.ThreeWirePort.F);
limit LimSwitch = limit(Brain.ThreeWirePort.A);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

// Left Drive Motors (6:1 Gear Ratio)
motor leftF = motor(PORT7, ratio6_1, true);      // Front Left (Reversed)
motor leftTB = motor(PORT10, ratio6_1, false);   // Top Back Left
motor leftBB = motor(PORT9, ratio6_1, true);     // Bottom Back Left (Reversed)
motor_group leftDrive = motor_group(leftF, leftTB, leftBB);

// Right Drive Motors (6:1 Gear Ratio)
motor rightF = motor(PORT3, ratio6_1, false);    // Front Right
motor rightTB = motor(PORT2, ratio6_1, true);    // Top Back Right (Reversed)
motor rightBB = motor(PORT1, ratio6_1, false);   // Bottom Back Right
motor_group rightDrive = motor_group(rightF, rightTB, rightBB);

/* Drivetrain Specifications:

   - Wheelbase: 297mm (Distance between front and back wheels)
   - Track Width: 310mm (Distance between left and right wheels)
   - Wheel Diameter: 325mm
   - External Gear Ratio: 1.33
  
*/

drivetrain Drivetrain = drivetrain(leftDrive, rightDrive, 297, 310, 325, mm, 1.33);

/*---------------------------------------------------------------------------*/
/*                          SENSOR CONFIGURATION                             */
/*---------------------------------------------------------------------------*/

inertial Inertial = inertial(PORT20);             // IMU for Heading Control
rotation rotationSensor = rotation(PORT11, true);// Lady Brown Position Tracking (not used)

/*---------------------------------------------------------------------------*/
/*                          SUBSYSTEM CONFIGURATION                          */
/*---------------------------------------------------------------------------*/

motor Intake = motor(PORT6, ratio6_1, true);     // Ring Intake System
motor lb = motor(PORT8, ratio36_1, false);      // "Lady Brown" Scoring Arm (36:1 Gear Ratio)

/*---------------------------------------------------------------------------*/
/*                          PRE-AUTONOMOUS INITIALIZATION                    */
/*---------------------------------------------------------------------------*/

/**
   Initializing sensors and resets all mechanisms
    - Calibrates IMU (2 second process)
    - Set all pneumatics to default states
   Ensures proper motor configurations
 */
void pre_auton(void) {
  vexcodeInit();                                // VEX Library Initialization
  Inertial.calibrate();                         // IMU Calibration
  while (Inertial.isCalibrating()) {            // Block until calibration completes
    wait(50, msec);
  }
  
  // Initialize Pneumatics
  PushG.set(false);
  PushH.set(false);
  Hook.set(true);
}

/*---------------------------------------------------------------------------*/
/*                          AUTONOMOUS CONTROL FUNCTIONS                     */
/*---------------------------------------------------------------------------*/

/*
   PID-Controlled Turning Algorithm
    targetAngle Desired heading in degrees 
    Features:
    - Tunable PID constants (kP=0.4, kI=0.0008, kD=0.24)
    - Anti-windup protection through speed clamping
    - 1-degree tolerance exit condition
    - Continuous angle correction
 */
void turnToAngle(double targetAngle) {
  // PID Constants
  const double kP = 0.4;    // Proportional - Main correction factor
  const double kI = 0.0008; // Integral - Eliminates steady-state error
  const double kD = 0.24;   // Derivative - Reduces overshoot

  // Control Variables
  double error = 0;
  double previousError = 0;
  double integral = 0;
  double derivative = 0;

  do {
    // Calculate PID Terms
    error = targetAngle - Inertial.rotation();
    integral += error;
    derivative = error - previousError;
    previousError = error;

    // Calculate Output
    double turnSpeed = (error * kP) + (integral * kI) + (derivative * kD);

    // Output Clamping 
       if (turnSpeed > 100)   
      turnSpeed = 100;
    if (turnSpeed < -100)
      turnSpeed = -100;

    // Differential Drive Execution
    leftDrive.spin(fwd, turnSpeed, pct);
    rightDrive.spin(reverse, turnSpeed, pct);

  } while (fabs(error) > 1.0);  // Exit when within 1 degree

  leftDrive.stop();
  rightDrive.stop();
}

/*
   Controlled Forward Movement
   Percentage velocity (0-100%)
   distance Travel distance in millimeters
 */
void driveFWD(double speed, double distance) {
  Drivetrain.setDriveVelocity(speed, pct);
  Drivetrain.setStopping(brake);               // Brake mode for precision
  Drivetrain.driveFor(fwd, distance, mm);
}

/**
  Controlled Reverse Movement
  speed Percentage velocity (0-100%)
  distance Travel distance in millimeters
 */
void driveREV(double speed, double distance) {
  Drivetrain.setDriveVelocity(speed, pct);
  Drivetrain.setStopping(brake);               // Brake mode for precision
  Drivetrain.driveFor(reverse, distance, mm);
}

/*
   Executes Scoring Sequence
    - Rotates Lady Brown to 220° position
    - Lowers until limit switch engages
    - Holds position to prevent play
 */
void Score() {
  lb.spinToPosition(220, degrees);  // Spin to score position
  wait(250, msec);                  // Stabilization pause
  lb.spin(reverse);                 //  Descent
  waitUntil(LimSwitch.pressing());  // Stop when limSwitch is pressed (safe position where rings can be placed for accurate scoring)
  lb.stop(hold);                    // Maintain position 
}

/*---------------------------------------------------------------------------*/
/*                          AUTONOMOUS ROUTINE                               */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // System Initialization
  lb.setVelocity(100, pct);
  Intake.setVelocity(100, pct);

  // Autonomous Routine Sequence
  Score();                    
  driveFWD(50, 150);         
  turnToAngle(6);            
  Intake.spin(reverse);       
  driveFWD(80, 1800);        
  turnToAngle(28);            
  driveFWD(80, 1950);        
  turnToAngle(15);           
  Intake.stop();              
  PushG.set(true);            
  PushH.set(true);           
}

/*---------------------------------------------------------------------------*/
/*                          DRIVER CONTROL                                   */
/*---------------------------------------------------------------------------*/

/*
    Main Driver Control Loop
    - Arcade Drive (Left Y + Right X)
    - Dual-button Intake Control
    - Single-button Lift Operation
    - Toggle Pneumatic Controls
 */
void usercontrol(void) {
  // State Variables
  bool liftDeployed = false;
  bool hookDeployed = false;

  // System Configuration
  Intake.setVelocity(100, pct);
  lb.setVelocity(100, pct);
  

  // Main Control Loop
  while (true) {
    // DRIVETRAIN - Split Arcade Control
    int leftY = Controller1.Axis3.position();
    int rightX = Controller1.Axis1.position();

    Drivetrain.arcade(leftY, rightX);

    // INTAKE CONTROL - R1/R2 Buttons
    if (Controller1.ButtonR1.pressing()) {
      Intake.spin(reverse);  // Intake
    } 
    else if (Controller1.ButtonR2.pressing()) {
      Intake.spin(fwd);     // Outake
    } 
    else {
      Intake.stop(brake);              // Brake when idle
    }

    // LIFT CONTROL - L1 Button
    if (Controller1.ButtonL1.pressing()) {
      Score();                         // Reuse autonomous routine
      this_thread::sleep_for(200);     // Debounce delay
    }

    // PNEUMATIC LIFT TOGGLE - X Button
    if (Controller1.ButtonX.pressing()) {
      liftDeployed = !liftDeployed;
      PushG.set(liftDeployed);
      PushH.set(liftDeployed);
      this_thread::sleep_for(200);     // Debounce delay
    }

    // HOOK TOGGLE - A Button
    if (Controller1.ButtonA.pressing()) {
      hookDeployed = !hookDeployed;
      Hook.set(hookDeployed);
      this_thread::sleep_for(200);     // Debounce delay
    }

    wait(20, msec);  // Reduce CPU Usage
  }
}

/*---------------------------------------------------------------------------*/
/*                          MAIN PROGRAM EXECUTION                           */
/*---------------------------------------------------------------------------*/

int main() {
  // Competition Callbacks
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Initialization
  pre_auton();

  // Prevent Exit
  while (true) {
    wait(100, msec);
  }
}