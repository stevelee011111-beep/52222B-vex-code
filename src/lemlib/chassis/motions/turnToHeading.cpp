#include <cmath>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "pros/misc.hpp"

void lemlib::Chassis::turnToHeading(float theta, int timeout, TurnToHeadingParams params, bool async) {
    params.maxSpeed = std::fabs(params.maxSpeed);
    params.minSpeed = std::fabs(params.minSpeed);
    params.earlyExitRange = std::fabs(params.earlyExitRange);

    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) 
        return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { turnToHeading(theta, timeout, params, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    float targetTheta;
    float deltaTheta;
    float motorPower;
    float prevMotorPower = 0;
    float startTheta = getPose().theta;
    bool settling = false;
    std::optional<float> prevRawDeltaTheta = std::nullopt;
    std::optional<float> prevDeltaTheta = std::nullopt;
    std::uint8_t compState = pros::competition::get_status();
    distTraveled = 0;
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();

    // main loop
    while (!timer.isDone() && !angularLargeExit.getExit() && !angularSmallExit.getExit() && this->motionRunning) {
        // update variables
        Pose pose = getPose();

        // update completion vars
        distTraveled = std::fabs(angleError(pose.theta, startTheta, false));

        targetTheta = theta;

        // check if settling
        const float rawDeltaTheta = angleError(targetTheta, pose.theta, false);
        if (prevRawDeltaTheta == std::nullopt) 
            prevRawDeltaTheta = rawDeltaTheta;

        if (sgn(rawDeltaTheta) != sgn(prevRawDeltaTheta)) 
            settling = true;

        prevRawDeltaTheta = rawDeltaTheta;

        // calculate deltaTheta
        if (settling) 
            deltaTheta = angleError(targetTheta, pose.theta, false);
        else 
            deltaTheta = angleError(targetTheta, pose.theta, false, params.direction);
        
        if (prevDeltaTheta == std::nullopt) 
            prevDeltaTheta = deltaTheta;

        // motion chaining
        if (params.minSpeed != 0 && std::fabs(deltaTheta) < params.earlyExitRange) 
            break;
        if (params.minSpeed != 0 && sgn(deltaTheta) != sgn(prevDeltaTheta)) 
            break;

        // calculate the speed
        motorPower = angularPID.update(deltaTheta);
        angularLargeExit.update(deltaTheta);
        angularSmallExit.update(deltaTheta);

        // cap the speed
        motorPower = std::clamp(motorPower, -params.maxSpeed, params.maxSpeed);

        if (fabs(deltaTheta) > 20) 
            motorPower = slew(motorPower, prevMotorPower, angularSettings.slew);

        if (motorPower < 0 && motorPower > -params.minSpeed) 
            motorPower = -params.minSpeed;
        else if (motorPower > 0 && motorPower < params.minSpeed) 
            motorPower = params.minSpeed;

        prevMotorPower = motorPower;

        infoSink()->debug("Turn Motor Power: {} ", motorPower);

        // move the drivetrain
        move(motorPower, -motorPower);

        pros::delay(10);
    }

    // stop the drivetrain
    move(0, 0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->endMotion();
}