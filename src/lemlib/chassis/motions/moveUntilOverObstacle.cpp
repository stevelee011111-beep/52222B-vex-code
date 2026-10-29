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

void lemlib::Chassis::moveUntilOverObstacle(float theta, int timeout, MoveUntilOverObstacleParams params, bool async) {
    params.basicSpeed = std::fabs(params.basicSpeed);
    params.obstacleSpeed = std::fabs(params.obstacleSpeed);

    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) 
        return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { moveUntilOverObstacle(theta, timeout, params, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    // reset PIDs and exit conditions
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
    Timer levelTimer(timeout);
    bool close = false;
    float prevLateralOut = 0; // previous lateral power
    float prevHeadingOut = 0; // previous angular power
    const int compState = pros::competition::get_status();
    std::optional<bool> prevSide = std::nullopt;
    RobotState currentState = RobotState::STATE_DRIVING;

    // main loop
    while (!timer.isDone() && ((currentState != RobotState::STATE_CROSSED) || !close) &&
           this->motionRunning) {
        // update position
        float currentRoll = sensors.imu->get_roll();
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

        // check if the robot is close enough to the target to start settling
        if (currentState == RobotState::STATE_ON_OBSTACLE && fabs(currentRoll) <= params.levelAngle && close == false) {
            close = true;
        }

        // motion chaining
        const bool side = currentState == RobotState::STATE_CROSSED;
        
        if (prevSide == std::nullopt) 
            prevSide = side;
        const bool sameSide = side == prevSide;
        // exit if close
        if (!sameSide && params.motionChaining) 
            break;
        prevSide = side;

        // calculate error
        // const float adjustedRobotTheta = params.forwards ? std::fmod(pose.theta, 360) : std::fmod(pose.theta + 180, 360);
        const float headingError = angleError(theta, pose.theta, false);
 
        // update exit conditions
        float lateralOut = currentState == RobotState::STATE_ON_OBSTACLE ? params.obstacleSpeed : params.basicSpeed;
        // get output from PIDs
        float headingOut = headingPID.update(headingError);

        if (close) headingOut = 0;

        // apply restrictions on angular speed
        headingOut = std::clamp(headingOut, -params.maxHedingTuning, params.maxHedingTuning);
        headingOut = slew(headingOut, prevHeadingOut, angularSettings.slew);

        // prevent moving in the wrong direction
        if (params.forwards && !close) 
            lateralOut = std::fmax(lateralOut, 0);
        else if (!params.forwards && !close) 
            lateralOut = std::fmin(lateralOut, 0);

        // constrain lateral output by the minimum speed
        if (params.forwards && lateralOut < fabs(params.basicSpeed) && lateralOut > 0) 
            lateralOut = fabs(params.basicSpeed);
        if (!params.forwards && -lateralOut < fabs(params.basicSpeed) && lateralOut < 0)
            lateralOut = -fabs(params.basicSpeed);

        // update previous output
        prevHeadingOut = headingOut;
        prevLateralOut = lateralOut;
        
        infoSink()->debug("Heading Out: {}, Lateral Out: {}", headingOut, lateralOut);

        // ratio the speeds to respect the max speed
        float leftPower = lateralOut + headingOut;
        float rightPower = lateralOut - headingOut;

        // move the drivetrain
        if (currentState == RobotState::STATE_DRIVING) {
            levelTimer.reset();
            move(leftPower, rightPower);
            if (fabs(currentRoll) > params.obstacleAngle) {
                currentState = RobotState::STATE_ON_OBSTACLE;
            }
        } else if (currentState == RobotState::STATE_ON_OBSTACLE) {
            move(leftPower, rightPower);
            if (fabs(currentRoll) < params.levelAngle) {
                if (levelTimer.getTimePassed() > 500) {
                    currentState = RobotState::STATE_CROSSED;
                }
            } else {
                levelTimer.reset();
            }
        } else {
            move(0, 0);
        }
            
        // delay to save resources
        pros::delay(10);
        
    }
    
    // stop the drivetrain
    move(0, 0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->endMotion();
}