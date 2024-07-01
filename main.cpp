#include "vex.h"
using namespace vex;

competition Competition;

motor lift(PORT1, gearSetting::ratio18_1, false);
digital_out wings(Brain.ThreeWirePort.A);
digital_out left_wing(Brain.ThreeWirePort.B);
digital_out right_wing(Brain.ThreeWirePort.C);

void moveForward(double x) {
  right_drive.spinFor(fwd, x, turns, false);
  left_drive.spinFor(fwd, x, turns);
}

void moveForward(double x, bool z) {
  right_drive.spinFor(fwd, x, turns, false);
  left_drive.spinFor(fwd, x, turns, z);
}

void moveBack(double x) {
  right_drive.spinFor(reverse, x, turns, false);
  left_drive.spinFor(reverse, x, turns);
}

void moveBack(double x, bool z) {
  right_drive.spinFor(reverse, x, turns, false);
  left_drive.spinFor(reverse, x, turns, z);
}

void turnLeft(double x) {
  left_drive.spinFor(reverse, x, turns, false);
  right_drive.spinFor(fwd, x, turns);
}

void turnLeft(double x, bool z) {
  left_drive.spinFor(reverse, x, turns, false);
  right_drive.spinFor(fwd, x, turns, z);
}

void turnRight(double x) {
  left_drive.spinFor(fwd, x, turns, false);
  right_drive.spinFor(reverse, x, turns);
}

void turnRight(double x, bool z) {
  left_drive.spinFor(fwd, x, turns, false);
  right_drive.spinFor(reverse, x, turns, z);
}

void rightDegrees(double x) {
  left_drive.spinFor(fwd, x, degrees, false);
  right_drive.spinFor(reverse, x, degrees);
}

void leftDegrees(double x) {
  left_drive.spinFor(reverse, x, degrees, false);
  right_drive.spinFor(fwd, x, degrees);
}

void moveForwardPID(double targetDistance) {
  double kP = 0.6;
  double kI = 0.04;
  double kD = 0.3;
  double error = targetDistance;
  double previousError = 0;
  double integral = 0;
  double derivative;
  double motorPower;

  left_drive.resetPosition();
  right_drive.resetPosition();

  while (fabs(error) > 0.1) {
    error = targetDistance - left_drive.position(turns);
    integral += error;
    derivative = error - previousError;
    motorPower = (kP * error) + (kI * integral) + (kD * derivative);

    left_drive.spin(fwd, motorPower, pct);
    right_drive.spin(fwd, motorPower, pct);

    previousError = error;
    wait(20, msec);
  }

  left_drive.stop();
  right_drive.stop();
}

void leftPID(double targetDegrees) {
  double kP = 0.6;
  double kI = 0.04;
  double kD = 0.3;
  double error = -targetDegrees;
  double previousError = 0;
  double integral = 0;
  double derivative;
  double motorPower;

  inertial.resetRotation();

  while (fabs(error) > 1) {
    error = targetDegrees - inertial.rotation(degrees);
    integral += error;
    derivative = error - previousError;
    motorPower = (kP * error) + (kI * integral) + (kD * derivative);

    left_drive.spin(fwd, motorPower, pct);
    right_drive.spin(reverse, motorPower, pct);

    previousError = error;
    wait(20, msec);
  }

  left_drive.stop();
  right_drive.stop();
}

void rightPID(double targetDegrees) {
  double kP = 0.6;
  double kI = 0.04;
  double kD = 0.3;
  double error = targetDegrees;
  double previousError = 0;
  double integral = 0;
  double derivative;
  double motorPower;

  inertial.resetRotation();

  while (fabs(error) > 1) {
    error = targetDegrees - inertial.rotation(degrees);
    integral += error;
    derivative = error - previousError;
    motorPower = (kP * error) + (kI * integral) + (kD * derivative);

    left_drive.spin(fwd, motorPower, pct);
    right_drive.spin(reverse, motorPower, pct);

    previousError = error;
    wait(20, msec);
  }

  left_drive.stop();
  right_drive.stop();
}

void setLeftWing(bool state) {
  wings.left_wing.set(state);
}

void setRightWing(bool state) {
  wings.right_wing.set(state);
}

void pre_auton(void) {
  vexcodeInit();

  right_drive.setStopping(brake);
  left_drive.setStopping(brake);
  right_drive.setVelocity(40, pct);
  left_drive.setVelocity(40, pct);
}

void autonomous(void) {


  
}

void usercontrol(void) {
  while (true) {
    left_drive.spin(fwd, Controller1.Axis3.position(pct), pct);
    right_drive.spin(fwd, Controller1.Axis2.position(pct), pct);
    wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  
  while (true) {
    wait(100, msec);
  }
}
