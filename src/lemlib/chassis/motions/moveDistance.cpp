#include <cmath>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/pose.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.hpp"
#include "pros/misc.hpp"

// 1、获取垂直定位轮数据
// 2、确定垂直定位轮目标值

void lemlib::Chassis::moveDistance(float targetDistance, float targetHeading, int timeout, MoveDistanceParams params, bool async) {
    params.maxSpeed = std::fabs(params.maxSpeed);
    params.minSpeed = std::fabs(params.minSpeed);
    params.earlyExitRange = std::fabs(params.earlyExitRange);

    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) 
        return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { moveDistance(targetDistance, targetHeading, timeout, params, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    // reset PIDs and exit conditions
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    headingPID.reset();

    // initialize vars used between iterations

    float lastVerticalPos;

    if (sensors.vertical1 != nullptr) {
        lastVerticalPos = sensors.vertical1->getDistanceTraveled();
    } else if (sensors.vertical2 != nullptr) {
        lastVerticalPos = sensors.vertical2->getDistanceTraveled();
    } else {
        // 读取左侧所有电机编码器数值并取平均数，精度更高
        std::vector<pros::MotorGears> leftGearSets = drivetrain.leftMotors->get_gearing_all();
        std::vector<pros::MotorGears> rightGearSets = drivetrain.rightMotors->get_gearing_all();
        std::vector<double> leftPositions = drivetrain.leftMotors->get_position_all();
        std::vector<double> rightPositions = drivetrain.rightMotors->get_position_all();
        std::vector<float> leftDistances;
        std::vector<float> rightDistances;
        for (int i = 0; i < drivetrain.leftMotors->size(); i++) {
            float in;
            switch (leftGearSets[i]) {
                case pros::MotorGears::red: in = 100; break;
                case pros::MotorGears::green: in = 200; break;
                case pros::MotorGears::blue: in = 600; break;
                default: in = 200; break;
            }
            leftDistances.push_back(leftPositions[i] * (drivetrain.wheelDiameter * M_PI) * (drivetrain.rpm / in));

            switch (rightGearSets[i]) {
                case pros::MotorGears::red: in = 100; break;
                case pros::MotorGears::green: in = 200; break;
                case pros::MotorGears::blue: in = 600; break;
                default: in = 200; break;
            }
            rightDistances.push_back(rightPositions[i] * (drivetrain.wheelDiameter * M_PI) * (drivetrain.rpm / in));
        }
        lastVerticalPos =  (lemlib::avg(leftDistances) + lemlib::avg(rightDistances)) / 2;
    }

    distTraveled = 0; 
    Timer timer(timeout);
    bool close = false;
    float prevLateralOut = 0; // previous lateral power
    float prevHeadingOut = 0; // previous angular power
    const int compState = pros::competition::get_status();
    std::optional<bool> prevSide = std::nullopt;

    float target = params.forwards ? lastVerticalPos + targetDistance : lastVerticalPos - targetDistance;

    // if (!params.forwards) 
    //     targetHeading = std::fmod(targetHeading + 180, 360); // backwards movement

    // main loop
    while (!timer.isDone() && ((!lateralSmallExit.getExit() && !lateralLargeExit.getExit()) || !close) &&
           this->motionRunning) {
        // update position
        const Pose pose = getPose();
        float verticalPos;

        if (sensors.vertical1 != nullptr) {
            verticalPos = sensors.vertical1->getDistanceTraveled();
        } else if (sensors.vertical2 != nullptr) {
            verticalPos = sensors.vertical2->getDistanceTraveled();
        } else {
            std::vector<pros::MotorGears> leftGearSets = drivetrain.leftMotors->get_gearing_all();
            std::vector<pros::MotorGears> rightGearSets = drivetrain.rightMotors->get_gearing_all();
            std::vector<double> leftPositions = drivetrain.leftMotors->get_position_all();
            std::vector<double> rightPositions = drivetrain.rightMotors->get_position_all();
            std::vector<float> leftDistances;
            std::vector<float> rightDistances;
            for (int i = 0; i < drivetrain.leftMotors->size(); i++) {
                float in;
                switch (leftGearSets[i]) {
                    case pros::MotorGears::red: in = 100; break;
                    case pros::MotorGears::green: in = 200; break;
                    case pros::MotorGears::blue: in = 600; break;
                    default: in = 200; break;
                }
                leftDistances.push_back(leftPositions[i] * (drivetrain.wheelDiameter * M_PI) * (drivetrain.rpm / in));

                switch (rightGearSets[i]) {
                    case pros::MotorGears::red: in = 100; break;
                    case pros::MotorGears::green: in = 200; break;
                    case pros::MotorGears::blue: in = 600; break;
                    default: in = 200; break;
                }
                rightDistances.push_back(rightPositions[i] * (drivetrain.wheelDiameter * M_PI) * (drivetrain.rpm / in));
            }
            verticalPos =  (lemlib::avg(leftDistances) + lemlib::avg(rightDistances)) / 2;
        }

        // update distance traveled
        distTraveled += std::fabs(verticalPos - lastVerticalPos);

        lastVerticalPos = verticalPos;

        // calculate distance to the target point
        const float distTarget = target - verticalPos;

        // check if the robot is close enough to the target to start settling
        if (std::fabs(distTarget) < 5 && close == false) {
            close = true;
            params.maxSpeed = fmax(fabs(prevLateralOut), 60);
        }

        // motion chaining
        const bool side = params.forwards ? target >= verticalPos + params.earlyExitRange : target <= verticalPos - params.earlyExitRange;
        
        if (prevSide == std::nullopt) 
            prevSide = side;
        const bool sameSide = side == prevSide;
        // exit if close
        if (!sameSide && params.minSpeed != 0) 
            break;
        prevSide = side;

        // calculate error
        // const float adjustedRobotTheta = params.forwards ? std::fmod(pose.theta, 360) : std::fmod(pose.theta + 180, 360);
        const float headingError = angleError(targetHeading, pose.theta, false);
        float lateralError = target - verticalPos;
 
        // update exit conditions
        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);

        // get output from PIDs
        float lateralOut = lateralPID.update(lateralError);
        float headingOut = headingPID.update(headingError);
        if (close) headingOut = 0;

        // apply restrictions on angular speed
        headingOut = std::clamp(headingOut, -params.maxSpeed, params.maxSpeed);
        headingOut = slew(headingOut, prevHeadingOut, angularSettings.slew);

        // apply restrictions on lateral speed
        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);
        // constrain lateral output by max accel
        // but not for decelerating, since that would interfere with settling
        if (!close) 
            lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);

        // prevent moving in the wrong direction
        if (params.forwards && !close) 
            lateralOut = std::fmax(lateralOut, 0);
        else if (!params.forwards && !close) 
            lateralOut = std::fmin(lateralOut, 0);

        // constrain lateral output by the minimum speed
        if (params.forwards && lateralOut < fabs(params.minSpeed) && lateralOut > 0) 
            lateralOut = fabs(params.minSpeed);
        if (!params.forwards && -lateralOut < fabs(params.minSpeed) && lateralOut < 0)
            lateralOut = -fabs(params.minSpeed);

        // update previous output
        prevHeadingOut = headingOut;
        prevLateralOut = lateralOut;
        
        infoSink()->debug("Heading Out: {}, Lateral Out: {}", headingOut, lateralOut);

        // ratio the speeds to respect the max speed
        float leftPower = lateralOut + headingOut;
        float rightPower = lateralOut - headingOut;
        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        // move the drivetrain
        move(leftPower, rightPower);

        // delay to save resources
        pros::delay(10);
        
    }
    
    // stop the drivetrain
    move(0, 0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->endMotion();
}