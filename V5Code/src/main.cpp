d#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>

pros::Motor LF(1, pros::MotorGearset::green);
pros::Motor RF(10, pros::MotorGearset::green);
pros::Motor LB(11, pros::MotorGearset::green);
pros::Motor RB(20, pros::MotorGearset::green);

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Imu imu(9);

// ---- Constants ----
static constexpr double WHEEL_DIAMETER_MM = 101.6;
static constexpr double PI = 3.14159265358979323846;
static constexpr double WHEEL_CIRC_MM = WHEEL_DIAMETER_MM * PI;

// PROS motor get_position() returns degrees by default
static constexpr double DEGREES_PER_REV = 360.0;

// X-drive geometry (45° wheel axis)
static constexpr double SQRT2 = 1.4142135623730951;

// Prevent dead output that can't overcome stiction
static constexpr int MIN_POWER = 12;

// Control loop timing
static constexpr int LOOP_MS = 10;

// ---- Helpers ----
static int applyDeadzone(int v, int dz = 8) {
    return (std::abs(v) < dz) ? 0 : v;
}

static int clamp127(int v) {
    if (v > 127) return 127;
    if (v < -127) return -127;
    return v;
}

static int sgn(int x) {
    return (x > 0) - (x < 0);
}

// ---- Drive I/O ----
// Keep YOUR existing motor direction conventions:
void setDrive(int lf, int rf, int lb, int rb) {
    LF.move(-clamp127(lf));
    RF.move( clamp127(rf));
    LB.move(-clamp127(lb));
    RB.move( clamp127(rb));
}

void stopDrive() {
    setDrive(0, 0, 0, 0);
}

void resetDriveEncoders() {
    LF.tare_position();
    RF.tare_position();
    LB.tare_position();
    RB.tare_position();
}

// wheel travel mm -> motor degrees
double mmToMotorDeg(double mm) {
    return (mm / WHEEL_CIRC_MM) * DEGREES_PER_REV;
}

// These are “drive axis estimators” based on your mixing.
// They output motor degrees (since get_position() is degrees).
double avgForwardDeg() {
    return (LF.get_position() + RF.get_position() + LB.get_position() + RB.get_position()) / 4.0;
}

double avgStrafeDeg() {
    return (LF.get_position() - RF.get_position() - LB.get_position() + RB.get_position()) / 4.0;
}

// ---- IMU / Angle math ----
double wrapDeg180(double deg) {
    while (deg >= 180.0) deg -= 360.0;
    while (deg <  -180.0) deg += 360.0;
    return deg;
}

// Convert 0..360 heading into -180..180 for sane error math
double imuHeading180() {
    return wrapDeg180(imu.get_heading());
}

double headingErrorDeg(double targetDeg) {
    return wrapDeg180(targetDeg - imuHeading180());
}

// ---- Motion ----
void turnToHeading(double targetDeg, int timeoutMs = 2000) {
    const double kP = 2.2;
    const double kD = 9.0;

    double lastErr = headingErrorDeg(targetDeg);
    int start = pros::millis();
    int lastT = start;

    while (pros::millis() - start < timeoutMs) {
        int now = pros::millis();
        double dt = (now - lastT) / 1000.0;
        if (dt <= 0) dt = LOOP_MS / 1000.0;
        lastT = now;

        double err = headingErrorDeg(targetDeg);
        double deriv = (err - lastErr) / dt;
        lastErr = err;

        if (std::abs(err) < 1.0) break;

        int power = (int)(kP * err + kD * deriv);
        power = clamp127(power);

        if (std::abs(power) < MIN_POWER) {
            power = MIN_POWER * sgn(power);
        }

        setDrive(power, -power, power, -power);
        pros::delay(LOOP_MS);
    }

    stopDrive();
    pros::delay(40);
}

// Drive forward mm while holding heading.
// IMPORTANT FIX: multiply mm by SQRT2 because wheels are at 45°.
void driveMmHoldHeading(double mm, double headingDeg, int timeoutMs = 3000) {
    resetDriveEncoders();

    const double targetDeg = mmToMotorDeg(mm * SQRT2);

    const double kP_dist = 0.55;
    const double kD_dist = 2.0;
    const double kP_head = 1.6;

    double lastErr = targetDeg;
    int start = pros::millis();
    int lastT = start;

    while (pros::millis() - start < timeoutMs) {
        int now = pros::millis();
        double dt = (now - lastT) / 1000.0;
        if (dt <= 0) dt = LOOP_MS / 1000.0;
        lastT = now;

        double pos = avgForwardDeg();
        double err = targetDeg - pos;
        double deriv = (err - lastErr) / dt;
        lastErr = err;

        // close enough (degrees of motor rotation)
        if (std::abs(err) < 10.0) break;

        int base = (int)(kP_dist * err + kD_dist * deriv);
        base = clamp127(base);

        if (std::abs(base) < MIN_POWER) {
            base = MIN_POWER * sgn(base);
        }

        int turnFix = (int)(kP_head * headingErrorDeg(headingDeg));
        turnFix = clamp127(turnFix);

        setDrive(base + turnFix, base - turnFix, base + turnFix, base - turnFix);
        pros::delay(LOOP_MS);
    }
    stopDrive();
    pros::delay(40);
}

// Strafe mm while holding heading.
// IMPORTANT FIX: multiply mm by SQRT2 because wheels are at 45°.
void strafeMmHoldHeading(double mm, double headingDeg, int timeoutMs = 3500) {
    resetDriveEncoders();

    const double targetDeg = mmToMotorDeg(mm * SQRT2);

    const double kP_dist = 0.65;
    const double kD_dist = 2.2;
    const double kP_head = 1.8;

    double lastErr = targetDeg;
    int start = pros::millis();
    int lastT = start;

    while (pros::millis() - start < timeoutMs) {
        int now = pros::millis();
        double dt = (now - lastT) / 1000.0;
        if (dt <= 0) dt = LOOP_MS / 1000.0;
        lastT = now;

        double pos = avgStrafeDeg();
        double err = targetDeg - pos;
        double deriv = (err - lastErr) / dt;
        lastErr = err;

        if (std::abs(err) < 10.0) break;

        int base = (int)(kP_dist * err + kD_dist * deriv);
        base = clamp127(base);

        if (std::abs(base) < MIN_POWER) {
            base = MIN_POWER * sgn(base);
        }

        int turnFix = (int)(kP_head * headingErrorDeg(headingDeg));
        turnFix = clamp127(turnFix);

        // X-drive strafe mix 
        setDrive(base + turnFix, -base - turnFix, -base + turnFix, base - turnFix);
        pros::delay(LOOP_MS);
    }

    stopDrive();
    pros::delay(40);
}

// ---- PROS lifecycle ----
void initialize() {
    imu.reset();
    while (imu.is_calibrating()) {
        pros::delay(20);
    }
}

void autonomous() {

}

void opcontrol() {
    imu.reset();
    while (true) {
        int fwd    = applyDeadzone(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int strafe = applyDeadzone(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
        int rot    = applyDeadzone(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        
        int lf = fwd + strafe + rot;
        int rf = fwd - strafe - rot;
        int lb = fwd - strafe + rot;
        int rb = fwd + strafe - rot;

        setDrive(lf, rf, lb, rb);
        pros::delay(LOOP_MS);
    }
}
// End of file
