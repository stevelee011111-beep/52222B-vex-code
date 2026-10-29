#include "main.h"
#include "config.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/timer.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include "autons.h"
#include "pixlib/functionControl.hpp"

bool autonSelected = false;
bool updateScreen = true;
int autonIndex = 0;

std::vector<std::string> autonPrograms = {"L-7Blocks", "L_2Goals", "R-7Blocks", "R_2Goals", "R_soloAWP", "Skill", "None"};

int autonSize = (int)autonPrograms.size();

void initialize() {
    controller.clear();
    pros::lcd::initialize(); // initialize brain screen
    intake.setBrakeMode();
    chassis.calibrate(); // calibrate sensors

    // make sure imu calibrated
    while (imu.is_calibrating()) {
        pros::delay(10);
    }

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // thread to for brain screen and position logging

    pros::Task screenTask([&]() {
        while (true) {
            // print robot optical information to the robot screen
            // pros::lcd::print(0, "proximity: %d", optical.get_proximity());
            // pros::lcd::print(1, "hue: %.2f", optical.get_hue());
            // pros::lcd::print(2, "saturation: %.2f", optical.get_saturation());

            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

void disabled() {}

void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

void autonomous() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    autonSelected = true;

    switch (autonIndex) {
    case 0:
        left_7Blocks();
        break;
    case 1:
        left_2Goals();
        break;
    case 2:
        right_7Blocks();
        break;
    case 3:
        right_2Goals();
        break;
    case 4:
        right_soloAWP();
        break;
    case 5:
        skills();
        break;
    default:
        none();
        break;
    }
}

void opcontrol() {
    lemlib::Timer autonSelectTimer(240000);
    int timerNow = 0, timerPrev = 0;

    while (!autonSelected || autonSelectTimer.isDone()) {
        timerNow = autonSelectTimer.getTimePassed();
        if (updateScreen || timerNow / 1000 != timerPrev / 1000 || timerNow < 10) {
            pros::Task screenTask([&]() {
                controller.clear();
                pros::delay(75);
                controller.print(0, 0, "Prog: %s", autonPrograms[autonIndex]);
                pros::delay(75);
                controller.print(1, 0, "Heading: %.2f", chassis.getPose().theta);
                pros::delay(75);
                controller.print(2, 0, "X: %.1f Y: %.1f", chassis.getPose().x, chassis.getPose().y);
                pros::delay(75);
            });
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            autonIndex--;
            autonIndex = autonIndex < 0 ? 0 : autonIndex;
            updateScreen = true;
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            autonIndex++;
            autonIndex = autonIndex >= autonPrograms.size() ? autonPrograms.size() - 1 : autonIndex;
            updateScreen = true;
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            if (sensors.imu != nullptr) {
                imu.tare_rotation();
            }

            if (sensors.horizontal1 != nullptr) {
                horizontalEnc.reset_position();
            } else {
                leftMotors.tare_position_all();
                rightMotors.tare_position_all();
            }

            if (sensors.vertical1 != nullptr) {
                verticalEnc.reset_position();
            } 
        } else {
            updateScreen = false;
        } 

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            autonSelected = true;
            pros::delay(75);
            controller.clear();
            pros::delay(75);
        }
        timerPrev = timerNow;
        pros::delay(20);
    }

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    lemlib::Timer opcontrolTimer(600000);
    timerNow = 0;
    timerPrev = 0;

    int throttleValue, turnValue; 
    bool intakebtn, bottombtn, midbtn, scorebtn;
    bool preloadbtn, hookbtn, midhookbtn;
    bool longgoalbtn = false;

    while (true) {
        throttleValue = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        throttleValue += lemlib::sgn(throttleValue) * std::abs(std::clamp((int)controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), -15, 15));
        turnValue = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        turnValue += lemlib::sgn(turnValue) * std::abs(std::clamp((int)controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), -15, 15));
        
        intakebtn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bottombtn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        midbtn = intakebtn && bottombtn;
        scorebtn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        
        hookbtn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        preloadbtn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        longgoalbtn = (intakebtn && !bottombtn) ? false : (scorebtn || midbtn) ? true : longgoalbtn;
        midhookbtn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);

        chassis.arcade(throttleValue, turnValue, false, 0.51);
        intake.operatorControl(intakebtn, bottombtn, midbtn, scorebtn);

        preload.switchControl(preloadbtn);
        hook.switchControl(hookbtn);
        midDisScore.switchControl(midhookbtn);
        longGoal.buttonControl(longgoalbtn);
        intakeLift.buttonControl(bottombtn && !intakebtn);
        scoreLift.buttonControl(midbtn);

        timerNow = opcontrolTimer.getTimePassed();
        if (timerNow / 15000 != timerPrev / 15000 || timerNow < 20)
            pros::Task opcontrolSreenTask([&]() { infoDisplay(timerNow); });
        timerPrev = timerNow;

        pros::delay(20);
    }
}