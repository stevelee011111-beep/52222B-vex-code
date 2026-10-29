#pragma once

#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "lemlib/timer.hpp"

namespace pixlib {
/**
 * @brief Parameters for intake functions
 */
struct intakeParams {
    /** whether the intake motors brake or not. false by default */
    bool brake = false;
    /** the maximum speed the intake motors can travel at. Value between 0-127. 127 by default */
    float maxSpeed = 127;
    /** the minimum speed the intake motors can travel at. Value between 0-127. 0 by default. 
     *  Attention: if minSpeed set to value not equal 0, the motors will spin at the minimum speed, 
     *  even no button is pressed   
     */
    float minSpeed = 0;
    /** reverse time for block selection */
    float cycleTime = 20;
};

enum class OutSide {
    TOP,
    MID, 			
    BOTTOM 
};

/**
 * @brief class containing constants for intake control
 */
class Intake {
    public:
        Intake(pros::Motor *intakeMotor, pros::Motor *scoreMotor);
        Intake(pros::Motor *intakeMotor, pros::Motor *scoreMotor, pros::Optical *optical);
        Intake(pros::Motor *intakeMotor, pros::Motor *scoreMotor, pros::Optical *optical, pros::Distance *distance);
        pros::Motor *intakeMotor;
        pros::Motor *scoreMotor;
        pros::Optical *optical = nullptr;
        pros::Distance *distance = nullptr;

        lemlib::Timer midTimer;
        /**
        * @brief Sets brake mode of the intake motors
        *
        * @param mode Mode to set the intake motors to
        */
        void setBrakeMode(pros::motor_brake_mode_e mode = pros::E_MOTOR_BRAKE_COAST);

        /**
        * @brief get the Color of optical sensor detected. 
        *        blue return 1, red return 0, no color return -1. 
        *
        */
        int getColor();

        /**
        * @brief Control the intake during the autonomous using distance
        *
        * @param Value of the intake time.
        * @param async whether the function should be run asynchronously. true by default
        * @param params struct to simulate named parameters
        */
        void intakeUntilNear(int timeout, intakeParams params = {}, bool async = true);

        /**
        * @brief Control the intake during the autonomous 
        *
        * @param Value of the intake time. 0 by default
        * @param async whether the function should be run asynchronously. false by default
        * @param params struct to simulate named parameters
        */
        void intake(int time = 0, intakeParams params = {}, bool async = true);

        /**
        * @brief Control the outlet during the autonomous 
        *
        * @param Value of the intake time. 0 by default
        * @param async whether the function should be run asynchronously. false by default
        * @param params struct to simulate named parameters
        */
        void outlet(OutSide side, int time = 0, intakeParams params = {}, bool async = true);

        /**
        * @brief Control the stop during the autonomous 
        *
        * @param whether brake mode hold or not. false by default
        */
        void stop(bool brake = false);

        /**
        * @brief Control the intake during the driver using the button control
        * scheme. In this control scheme one joystick button controls the intake,
        * while the other joystick button controls the robot's outlet

        * @param Value of the intakeButton. Takes an input in 1 or 0;
        * @param Value of the outletButton. Takes an input in 1 or 0;
        * @param Value of the allianceColor. Takes an input in 1 or 0 or -1;
        * @param Value of the onlyIntakeButton. Takes an input in 1 or 0;
        * @param Value of the dropRingButton. Takes an input in 1 or 0;
        * @param params struct to simulate named parameters
        */
        void operatorControl(bool intakeButton, bool bottomButton, bool midButton, bool scoreButton, intakeParams params = {});

        /**
         * @brief Wait until the robot has completed the path
         */
        void waitUntilDone();

        /**
         * @brief Cancels the currently running motion.
         * If there is a queued motion, then that queued motion will run.
         */
        void cancelMotion();

        /**
         * @brief Cancels all motions, even those that are queued.
         * After this, the chassis will not be in motion.
         */
        void cancelAllMotions();

        /**
         * @return whether a motion is currently running
         */
        bool isInMotion() const;

    protected:
        /**
         * @brief Indicates that this motion is queued and blocks current task until this motion reaches front of queue
         */
        void requestMotionStart();

        /**
         * @brief Dequeues this motion and permits queued task to run
         */
        void endMotion();

        bool motionRunning = false;
        bool motionQueued = false;

        float distTraveled = 0;
        
    private:
        pros::Mutex mutex;
};

/**
 * @brief class containing constants for piston control
 */
class Piston {
    public:
        Piston(pros::adi::Port *piston);
        pros::adi::Port *piston;

        bool pistonMode = false;
        bool pistonStatus = true;
        
        bool autoDoneStatus = true;

        /**
        * @brief Extends the piston, if not already extended.
        *
        * @param Extend the piston or not. extend if true and retract false;
        * @param params struct to simulate named parameters;
        */
        void extend(bool extended);

        /**
        * @brief Control the piston during the driver using the switch control
        * scheme. In this control scheme one joystick button controls the piston extend,
        * and press again the piston retract 

        * @param Value of the toggleButton. Takes an input in 1 or 0;
        * @param whether autonomous done;
        */
        void switchControl(bool extendButton, bool autoDone = false);

        /**
        * @brief Control the piston during the driver using the button control
        * scheme. In this control scheme one joystick button controls the piston extend,
        * while release the button make piston retract

        * @param Value of the extendButton. piston extend if 1, retract if 0;
        */
        void buttonControl(bool extendButton);
};
} // namespace FSlib