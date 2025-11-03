/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       USER                                                      */
/*    Created:      11/3/2025, 1:22:31 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

double Cos_move(double dis, double v, double target, double m){
  timer limit_time;
  timer timer2;
  limit_time.reset();
  timer2.reset();

  // Define your own Motor
  L1.resetPosition();
  L2.resetPosition();
  L3.resetPosition();
  R1.resetPosition();
  R2.resetPosition();
  R3.resetPosition();

  // PID variable 
  double v_now = 0;
  int cnt = 0;
  double rpm_now = 0;
  double error_now = 0; // Kp
  double error_last = 0; // Kd
  double error_INT = 0; // Ki

  // PID parameter
  double Kp = 2.2;
  double Ki = 0.00001;
  double Kd = 0.5;

  // Calculation
  double mag = dis*19.339/0.8265; // distance to degrees
  double t = fabs(dis/v*1000); // total time takes to run

  double one;
  double two;
  double three;
  double four;
  double five;
  double six;

  // Running 
  while ((one+two+three+four) / 4<abs(mag) && limit_time.time(msec)<t+100){
    one = abs(L1.position(deg));
    two = abs(L2.position(deg));
    three = abs(R1.position(deg));
    four = abs(R2.position(deg));
    five = abs(L3.position(deg));
    six = abs(R3.position(deg));

    // PID
    error_now = target - Inertial4.rotation(degrees);
    error_INT = error_now + error_INT;
    rpm_now = fmax(-600, fmin(Kp*(error_now) + Ki*error_INT + Kd*(error_now - error_last) ,600));

    // =======
    double ti = limit_time.time(msec);
    double y_final = 0.5*mag*(1-cos((ti+10)*pi/t));
    double y_initial = 0.5*mag*(1-cos((ti)*pi/t));
    double last_y_final;
    double last_y_initial;

    while((one+two+three+four)/4<fabs(y_final)&&limit_time.time(msec)<t+100){ // total distance travel && time limit
      one = fabs(L1.position(deg));
      two = fabs(L2.position(deg));
      three = fabs(R1.position(deg));
      four = fabs(R2.position(deg));
      five = fabs(L3.position(deg));
      six = fabs(R3.position(deg));

      if(limit_time.time(msec)>t){
        L1.spin(forward,-(last_y_final-last_y_initial)/10*c-rpm_now,rpm); //rpm_now is for PID speed
        L2.spin(forward,-(last_y_final-last_y_initial)/10*c-rpm_now,rpm);
        L3.spin(forward,-(last_y_final-last_y_initial)/10*c-rpm_now,rpm);
        R1.spin(forward,(last_y_final-last_y_initial)/10*c-rpm_now,rpm);
        R2.spin(forward,(last_y_final-last_y_initial)/10*c-rpm_now,rpm);
        R3.spin(forward,(last_y_final-last_y_initial)/10*c-rpm_now,rpm);
      }
      else{
        L1.spin(forward,-(y_final-y_initial)/10*c-rpm_now,rpm);
        L2.spin(forward,-(y_final-y_initial)/10*c-rpm_now,rpm);
        // L3.spin(forward,-(y_final-y_initial)/10*c-rpm_now,rpm);
        R1.spin(forward,(y_final-y_initial)/10*c-rpm_now,rpm);
        R2.spin(forward,(y_final-y_initial)/10*c-rpm_now,rpm);
        // R3.spin(forward,(y_final-y_initial)/10*c-rpm_now,rpm);
        last_y_final = y_final;
        last_y_initial = y_initial;
      }
    }
  }
  L1.stop(brake);
  L2.stop(brake);
  L3.stop(brake);
  R1.stop(brake);
  R2.stop(brake);
  R3.stop(brake);
  return 0;
}


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
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
