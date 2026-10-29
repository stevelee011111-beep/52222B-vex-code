#include <algorithm>
#include <cmath>
#include "lemlib/timer.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"

void lemlib::Chassis::move(int left, int right, int time, bool async) {
    left = std::clamp(left, -127, 127);
    right = std::clamp(right, -127, 127);
    time = std::abs(time);

    lemlib::Timer timer(time);

    if (time == 0) {
        if (left != 0) {
            drivetrain.leftMotors->move(left);
        } else {
            drivetrain.leftMotors->brake();
        }
        if (right != 0) {
            drivetrain.rightMotors->move(right);
        } else {
            drivetrain.rightMotors->brake();
        }
    } else {
        this->requestMotionStart();
        // were all motions cancelled?
        if (!this->motionRunning) 
            return;
        // if the function is async, run it in a new task
        if (async) {
            pros::Task task([&]() { move(left, right, time, false); });
            this->endMotion();
            pros::delay(10);
            return;
        }
        
        while (!timer.isDone() && this->motionRunning) {
            drivetrain.leftMotors->move(left);
            drivetrain.rightMotors->move(right);
            pros::delay(5);
        }
        drivetrain.leftMotors->brake();
        drivetrain.rightMotors->brake();
        this->endMotion();
    }
}