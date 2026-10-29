#include "pixlib/functionControl.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "lemlib/timer.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

pixlib::Intake::Intake(pros::Motor *intakeMotor, pros::Motor *scoreMotor)
    : intakeMotor(intakeMotor),
      scoreMotor(scoreMotor),
      midTimer(120000) {}

pixlib::Intake::Intake(pros::Motor *intakeMotor, pros::Motor *scoreMotor, pros::Optical *optical) 
    : intakeMotor(intakeMotor),
      scoreMotor(scoreMotor),
      optical(optical),
      midTimer(120000) {}

pixlib::Intake::Intake(pros::Motor *intakeMotor, pros::Motor *scoreMotor, pros::Optical *optical, pros::Distance *distance) 
    : intakeMotor(intakeMotor),
      scoreMotor(scoreMotor),
      optical(optical),
      distance(distance),
      midTimer(120000) {}

void pixlib::Intake::setBrakeMode(pros::motor_brake_mode_e mode) {
    intakeMotor->set_brake_mode(mode);
    scoreMotor->set_brake_mode(mode);
}

int pixlib::Intake::getColor() {
    int currentColor = -1;
    int maxCount = 0;
    double hue;
    std::vector<int> colors;
    optical->set_integration_time(10);
    
    if (optical->get_proximity() == 255) {
        for (int i = 0; i < 6; i++) {
            hue = optical->get_hue();
            if ((hue >= 200 && hue <= 260)) {
                colors.push_back(1);
            } else if ((hue <= 20 || hue >= 340)) {
                colors.push_back(0);
            } else {
                colors.push_back(-1);
            }
            pros::delay(1);
        }
        
        for (int num : colors) {
            int count = std::count(colors.begin(), colors.end(), num);
            if (count > maxCount) {
                maxCount = count;
                currentColor = num;
            }
        }
        colors.clear();
    } else {
        currentColor = -1;
    }
    return currentColor;
}

void pixlib::Intake::intakeUntilNear(int timeout, intakeParams params, bool async) {
    if (params.brake) {
        setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    } else {
        setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    }

    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) 
        return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { intakeUntilNear(timeout, params, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    distTraveled = 0;
    lemlib::Timer timer(timeout);

    intakeMotor->move(params.maxSpeed);
    scoreMotor->move(params.maxSpeed);

    while(!timer.isDone() && optical->get_proximity() != 255 && this->motionRunning)
        pros::delay(10);
    
    if (params.minSpeed != 0) {
        intakeMotor->move(params.minSpeed);
        scoreMotor->move(params.minSpeed);
    } else {
        intakeMotor->brake();
        scoreMotor->brake();
    }
    distTraveled = -1;
    this->endMotion();
}

void pixlib::Intake::intake(int time, intakeParams params, bool async) {
    if (params.brake) {
        setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    } else {
        setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    }

    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) 
        return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { intake(time, params, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }
    
    distTraveled = 0;
    lemlib::Timer timer(time);
    lemlib::Timer blockTimer(time);

    if (time == 0) {
        intakeMotor->move(params.maxSpeed);
        scoreMotor->move(params.maxSpeed);
    } else {
        while (!timer.isDone() && this->motionRunning) {
            intakeMotor->move(params.maxSpeed);
            scoreMotor->move(params.maxSpeed);
            pros::delay(10);
        }
        
        if (params.minSpeed != 0) {
            intakeMotor->move(params.minSpeed);
            scoreMotor->move(params.minSpeed);
        } else {
            intakeMotor->brake();
            scoreMotor->brake();
        }
    }
    distTraveled = -1;
    this->endMotion();
}

void pixlib::Intake::outlet(OutSide side, int time, intakeParams params, bool async) {
    if (params.brake) {
        setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    } else {
        setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    }

    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) 
        return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { outlet(side, time, params, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }
    
    distTraveled = 0;
    lemlib::Timer timer(time);

    if (time == 0) {
        if (side == OutSide::BOTTOM) {
            intakeMotor->move(-params.maxSpeed * 0.8);
            scoreMotor->move(-params.maxSpeed);
        } else if (side == OutSide::MID) {
            intakeMotor->move(params.maxSpeed);
            scoreMotor->move(params.maxSpeed * 0.8);
        } else {
            intakeMotor->move(params.maxSpeed);
            scoreMotor->move(params.maxSpeed);
        }
    } else {
        while (!timer.isDone() && this->motionRunning) {
            if (side == OutSide::BOTTOM) {
                intakeMotor->move(-params.maxSpeed * 0.75);
                scoreMotor->move(-params.maxSpeed);
            } else if (side == OutSide::MID) {
                intakeMotor->move(params.maxSpeed);
                scoreMotor->move(params.maxSpeed);
            } else {
                intakeMotor->move(params.maxSpeed);
                scoreMotor->move(params.maxSpeed);
            }
            pros::delay(10);
        }
        
        
        if (params.minSpeed != 0) {
            if (side == OutSide::BOTTOM) {
                intakeMotor->move(-params.minSpeed);
                scoreMotor->move(-params.minSpeed);
            } else if (side == OutSide::MID) {
                intakeMotor->move(params.minSpeed);
                scoreMotor->move(-params.minSpeed);
            } else {
                intakeMotor->move(params.minSpeed);
                scoreMotor->move(params.minSpeed);
            }
        } else {
            intakeMotor->brake();
            scoreMotor->brake();
        }
    }
    distTraveled = -1;
    this->endMotion();
}

void pixlib::Intake::stop(bool brake) {
    if (brake) {
        setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    } else {
        setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    }
    cancelMotion();
    intakeMotor->brake();
    scoreMotor->brake();
}

void pixlib::Intake::operatorControl(bool intakeButton, bool bottomButton, bool midButton, bool scoreButton, intakeParams params) {
    params.maxSpeed = std::fabs(params.maxSpeed);
    params.minSpeed = std::fabs(params.minSpeed);
    float currentSpeed;
    
    if (params.brake) {
        setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    }

    if (midButton) {
        currentSpeed = params.maxSpeed - (midTimer.getTimePassed() / params.cycleTime);
        if (currentSpeed > params.maxSpeed * 0.8)
            currentSpeed = params.maxSpeed * 0.8;
        if (currentSpeed < params.maxSpeed * 0.5)
            currentSpeed = params.maxSpeed * 0.5;
        
        intakeMotor->move(params.maxSpeed);
        scoreMotor->move(currentSpeed);
    } else if (intakeButton) {
        midTimer.reset();
        intakeMotor->move(params.maxSpeed);
        scoreMotor->move(params.maxSpeed);
    } else if (bottomButton) {
        midTimer.reset();
        intakeMotor->move(-params.maxSpeed * 0.75);
        scoreMotor->move(-params.maxSpeed);
    } else if (scoreButton) {
        midTimer.reset();
        intakeMotor->move(params.maxSpeed);
        scoreMotor->move(params.maxSpeed);
    } else {
        midTimer.reset();
        if (params.minSpeed != 0) {
            intakeMotor->move(params.minSpeed);
            scoreMotor->move(params.minSpeed);
        } else {
            intakeMotor->brake();
            scoreMotor->brake();
        }
    }
}

void pixlib::Intake::waitUntilDone() {
    do pros::delay(10);
    while (distTraveled != -1);
}

void pixlib::Intake::requestMotionStart() {
    if (this->isInMotion()) this->motionQueued = true; // indicate a motion is queued
    else this->motionRunning = true; // indicate a motion is running

    // wait until this motion is at front of "queue"
    this->mutex.take(TIMEOUT_MAX);

    // this->motionRunning should be true
    // and this->motionQueued should be false
    // indicating this motion is running
}

void pixlib::Intake::endMotion() {
    // move the "queue" forward 1
    this->motionRunning = this->motionQueued;
    this->motionQueued = false;

    // permit queued motion to run
    this->mutex.give();
}

void pixlib::Intake::cancelMotion() {
    this->motionRunning = false;
    pros::delay(10); // give time for motion to stop
}

void pixlib::Intake::cancelAllMotions() {
    this->motionRunning = false;
    this->motionQueued = false;
    pros::delay(10); // give time for motion to stop
}

bool pixlib::Intake::isInMotion() const { return this->motionRunning; }

pixlib::Piston::Piston(pros::adi::Port *piston) {
    this->piston = piston;
}

void pixlib::Piston::extend(bool extended) {
    piston->set_value(extended);
}

void pixlib::Piston::switchControl(bool extendButton, bool autoDone) {
    if (autoDone) {
        if (autoDoneStatus) {
            pistonMode = true;
            autoDoneStatus = false;
        }
    }

    if (extendButton) {
        if (pistonStatus) {
            pistonMode = !pistonMode;
            pistonStatus = false;
        }  
    } else {
        pistonStatus = true;
    }

    piston->set_value(pistonMode);
}

void pixlib::Piston::buttonControl(bool extendButton) {
    piston->set_value(extendButton);
}
