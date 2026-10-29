#pragma once

#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/adi.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pixlib/functionControl.hpp"

extern pros::Controller controller;

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::Motor intakeMotor;
extern pros::Motor scoreMotor; 

extern pros::adi::Port preloadPiston;
extern pros::adi::Port hookPiston;
extern pros::adi::Port midDisScorePiston;
extern pros::adi::Port longGoalPiston;
extern pros::adi::Port intakePiston;
extern pros::adi::Port scorePiston;

extern pros::Imu imu;

extern pros::Rotation horizontalEnc;
extern pros::Rotation verticalEnc;

extern lemlib::TrackingWheel horizontal;
extern lemlib::TrackingWheel vertical;

extern lemlib::Drivetrain drivetrain;

extern lemlib::OdomSensors sensors;

extern lemlib::ControllerSettings linearController;
extern lemlib::ControllerSettings headingController;
extern lemlib::ControllerSettings angularController;

extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;

extern lemlib::Chassis chassis;
extern pixlib::Intake intake;
extern pixlib::Piston preload;
extern pixlib::Piston hook;
extern pixlib::Piston midDisScore;
extern pixlib::Piston longGoal;
extern pixlib::Piston odomLift;
extern pixlib::Piston intakeLift;
extern pixlib::Piston scoreLift;

void infoDisplay(int timeNow);