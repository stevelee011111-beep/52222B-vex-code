#include "config.h"
#include "lemlib/chassis/chassis.hpp"
#include <cmath>
#include <cstdio>
#include <sys/_intsup.h>
#include "autons.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pixlib/functionControl.hpp"

void left_7Blocks() {
    chassis.setPose(0, 0, 65);
    // 直线前进靠近3球，并完成吸取
    intake.intake(2500);
    chassis.moveToPoint(18.55, 8.35, 1000, {.minSpeed = 40, .earlyExitRange = 0.01});
    chassis.waitUntil(16.05);
    preload.extend(true);
    
    // 转向并移动至长桥前预备打球点
    chassis.turnToPoint(4, 34.97, 2000, {.minSpeed = 30, .earlyExitRange = 0.5});
    chassis.moveToPoint(4, 34.97, 1500, {.maxSpeed = 80, .minSpeed = 15, .earlyExitRange = 0.1});
    // 朝向联队站位围挡方向移动一段距离，放下长桥撞击板，吸取填装桶内填装
    chassis.turnToHeading(270, 2000, {.minSpeed = 50, .earlyExitRange = 1});
    chassis.waitUntilDone();
    intake.cancelMotion();
    intake.intake(1600);
    chassis.moveDistance(9.75, 270, 800, {.maxSpeed = 50, .minSpeed = 30, .earlyExitRange = 3});
    chassis.move(50, 50, 600.00, true);
    
    // 后退至打球位并等待完成打球动作
    chassis.moveDistance(27.375, 270, 1500, {.forwards = false, .maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 0.1});
    chassis.waitUntil(21);
    longGoal.extend(true);
    chassis.waitUntilDone();
    intake.cancelMotion();
    preload.extend(false);
    chassis.move(-60, -60, 500, true);
    intake.outlet(pixlib::OutSide::TOP, 1600);
    intake.waitUntilDone();
    // 移动到通道位，准备拉球
    // chassis.moveDistance(6.375, 270, 1200, {.maxSpeed = 80, .minSpeed = 50, .earlyExitRange = 0.01});
    // chassis.turnToHeading(230, 1200, {.minSpeed = 60, .earlyExitRange = 1});
    // chassis.moveDistance(5.55, 230, 1200, {.forwards = false, .minSpeed = 50, .earlyExitRange = 0.01});
    chassis.turnToHeading(325, 2000, {.minSpeed = 60, .earlyExitRange = 1});
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 1200, {.minSpeed = 80, .earlyExitRange = 1});
    chassis.waitUntilDone();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveDistance(23.25, 270, 6500, {.forwards = false, .maxSpeed = 75});
}

void left_2Goals() {
    chassis.setPose(0, 0, 65);
    // 直线前进靠近3球，并完成吸取
    intake.intake(2500);
    chassis.moveToPoint(18.55, 8.35, 900, {.minSpeed = 40, .earlyExitRange = 0.01});
    chassis.waitUntil(16.05);
    preload.extend(true);
    
    // 转向并移动至长桥前预备打球点
    chassis.turnToPoint(4, 33.75, 2000, {.minSpeed = 30, .earlyExitRange = 0.5});
    chassis.moveToPoint(4, 33.75, 1500, {.maxSpeed = 80, .minSpeed = 15, .earlyExitRange = 0.1});
    // 朝向联队站位围挡方向移动一段距离，放下长桥撞击板，吸取填装桶内填装
    chassis.turnToHeading(270, 2000, {.minSpeed = 50, .earlyExitRange = 1});
    chassis.waitUntilDone();
    intake.cancelMotion();
    intake.intake(1600);
    chassis.moveDistance(9.75, 270, 800, {.maxSpeed = 50, .minSpeed = 30, .earlyExitRange = 3});
    chassis.move(50, 50, 600.00, true);
    
    // 后退至打球位并等待完成打球动作
    chassis.moveDistance(27.375, 270, 1500, {.forwards = false, .maxSpeed = 80, .minSpeed = 30, .earlyExitRange = 0.1});
    chassis.waitUntil(21);
    longGoal.extend(true);
    chassis.waitUntilDone();
    intake.cancelMotion();
    preload.extend(false);
    chassis.move(-60, -60, 400, true);
    intake.outlet(pixlib::OutSide::TOP, 560);
    intake.waitUntilDone();

    // 前进，准备打中桥
    chassis.moveDistance(6, 270, 2000, {.maxSpeed = 60, .minSpeed = 30, .earlyExitRange = 0.01});
    chassis.turnToHeading(315, 2000, {.minSpeed = 30, .earlyExitRange = 1});
    chassis.waitUntil(20);
    longGoal.extend(false);
    chassis.moveToPose(44.05, -2.55, 315, 4000, {.forwards = false, .maxSpeed = 80, .minSpeed = 40});

    // 打中桥
    chassis.waitUntilDone();
    scoreLift.extend(true);
    intake.outlet(pixlib::OutSide::MID, 3500);
    intake.waitUntilDone();

    // 前进一段，堵桥
    chassis.moveDistance(6, 315, 2000, {.maxSpeed = 60, .minSpeed = 30, .earlyExitRange = 0.01});
    chassis.waitUntilDone();
    midDisScore.extend(true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveDistance(6, 315, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 0.01});
}

void right_7Blocks() {
    chassis.setPose(0, 0, 115);
    // 直线前进靠近3球，并完成吸取
    intake.intake(2500);
    chassis.moveToPoint(18.55, -8.35, 900, {.minSpeed = 40, .earlyExitRange = 0.01});
    chassis.waitUntil(16.05);
    preload.extend(true);
    
    // 转向并移动至长桥前预备打球点
    chassis.turnToPoint(4, -33.75, 2000, {.minSpeed = 30, .earlyExitRange = 0.5});
    chassis.moveToPoint(4, -33.75, 1500, {.maxSpeed = 80, .minSpeed = 15, .earlyExitRange = 0.1});
    // 朝向联队站位围挡方向移动一段距离，放下长桥撞击板，吸取填装桶内填装
    chassis.turnToHeading(270, 2000, {.minSpeed = 50, .earlyExitRange = 1});
    chassis.waitUntilDone();
    intake.cancelMotion();
    intake.intake(1750);
    chassis.moveDistance(9.75, 270, 800, {.maxSpeed = 50, .minSpeed = 30, .earlyExitRange = 3});
    chassis.move(50, 50, 600.00, true);
    
    // 后退至打球位并等待完成打球动作
    chassis.moveDistance(27.375, 270, 1500, {.forwards = false, .maxSpeed = 75, .minSpeed = 30, .earlyExitRange = 0.1});
    chassis.waitUntil(21);
    longGoal.extend(true);
    chassis.waitUntilDone();
    intake.cancelMotion();
    preload.extend(false);
    chassis.move(-60, -60, 500, true);
    intake.outlet(pixlib::OutSide::TOP, 1600);
    intake.waitUntilDone();
    // 移动到通道位，准备拉球
    // chassis.moveDistance(6.375, 270, 1200, {.maxSpeed = 80, .minSpeed = 50, .earlyExitRange = 0.01});
    // chassis.turnToHeading(230, 1200, {.minSpeed = 60, .earlyExitRange = 1});
    // chassis.moveDistance(5.55, 230, 1200, {.forwards = false, .minSpeed = 50, .earlyExitRange = 0.01});
    chassis.turnToHeading(325, 2000, {.minSpeed = 60, .earlyExitRange = 1});
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 1200, {.minSpeed = 80, .earlyExitRange = 1});
    chassis.waitUntilDone();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveDistance(23.25, 270, 6500, {.forwards = false, .maxSpeed = 75});
}

void right_2Goals() {
    chassis.setPose(0, 0, 115);
    // 直线前进靠近3球，并完成吸取
    intake.intake(2500);
    chassis.moveToPoint(18.55, -8.35, 900, {.minSpeed = 40, .earlyExitRange = 0.01});
    chassis.waitUntil(16.05);
    preload.extend(true);
    
    // 转向并移动至长桥前预备打球点
    chassis.turnToPoint(4, -33.05, 2000, {.minSpeed = 30, .earlyExitRange = 0.5});
    chassis.moveToPoint(4, -33.05, 1500, {.maxSpeed = 80, .minSpeed = 15, .earlyExitRange = 0.1});
    // 朝向联队站位围挡方向移动一段距离，放下长桥撞击板，吸取填装桶内填装
    chassis.turnToHeading(270, 2000, {.minSpeed = 50, .earlyExitRange = 1});
    chassis.waitUntilDone();
    intake.cancelMotion();
    intake.intake(1500);
    chassis.moveDistance(9.75, 270, 800, {.maxSpeed = 50, .minSpeed = 30, .earlyExitRange = 3});
    chassis.move(50, 50, 600.00, true);
    
    // 后退至打球位并等待完成打球动作
    chassis.moveDistance(27.375, 270, 1500, {.forwards = false, .maxSpeed = 75, .minSpeed = 30, .earlyExitRange = 0.1});
    chassis.waitUntil(21);
    longGoal.extend(true);
    chassis.waitUntilDone();
    intake.cancelMotion();
    preload.extend(false);
    intake.outlet(pixlib::OutSide::TOP, 560);
    intake.waitUntilDone();

    // 前进，准备打中桥
    chassis.moveDistance(6, 270, 2000, {.maxSpeed = 60, .minSpeed = 30, .earlyExitRange = 0.01});
    chassis.turnToHeading(45, 2000, {.minSpeed = 30, .earlyExitRange = 1});
    chassis.waitUntil(20);
    longGoal.extend(false);
    chassis.moveToPose(42.65, 3.55, 45, 4000, {.maxSpeed = 80, .minSpeed = 30});

    // 打中桥
    chassis.waitUntilDone();
    intakeLift.extend(true);
    intake.outlet(pixlib::OutSide::BOTTOM, 3500);
    intake.waitUntilDone();

    // 前进一段，堵桥
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveDistance(6, 45, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 0.01});
    chassis.waitUntilDone();
    intakeLift.extend(false);
    chassis.moveDistance(8, 45, 2000, {.maxSpeed = 60, .minSpeed = 30, .earlyExitRange = 0.01});
}

void right_soloAWP() {
    chassis.setPose(0, 0, 0);
    // 撞击联队队友，使其离开停泊区并取其车尾预装球
    chassis.moveToPoint(0, 4.75, 3000, {.minSpeed = 60, .earlyExitRange = 1});
    intake.intake(1500);
    // 移动至启动位右侧填装桶，右转取填装
    chassis.moveToPoint(4, -26.75, 4000, {.forwards = false, .minSpeed = 5});
    chassis.turnToHeading(270, 2500, {.minSpeed = 30, .earlyExitRange = 15});
    chassis.waitUntilDone();
    preload.extend(true);
    chassis.moveDistance(10, 270, 2000, {.maxSpeed = 90, .minSpeed = 31, .earlyExitRange = 4});
    chassis.waitUntilDone();
    chassis.cancelMotion();
    intake.intake(2500);
    chassis.move(50, 50, 725, true);

    // 后退至打球位并等待完成打球动作
    chassis.moveDistance(27.375, 272, 1500, {.forwards = false, .maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 0.25});
    chassis.waitUntil(21);
    longGoal.extend(true);
    chassis.waitUntilDone();
    intake.cancelMotion();
    preload.extend(false);
    intake.outlet(pixlib::OutSide::TOP, 1000);
    intake.waitUntilDone();

    // 前进，右转并移动至右侧3球前方，将3球吸入
    chassis.moveDistance(6.5, 270, 1400, {.maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 0.5});
    chassis.turnToPoint(23.25, -18, 3000, {.maxSpeed = 80, .minSpeed = 30, .earlyExitRange = 0.5});
    chassis.waitUntilDone();
    longGoal.extend(false);
    intake.intake(6000);
    chassis.moveToPoint(23.25, -18, 1000, {.maxSpeed = 80, .minSpeed = 30, .earlyExitRange = 0.75});
    chassis.waitUntil(14.85);
    preload.extend(true);
    
    chassis.turnToPoint(26.05, 29.875, 1500, {.minSpeed = 40, .earlyExitRange = 1});
    chassis.waitUntil(30);
    preload.extend(false);
    chassis.moveToPoint(26.05, 29.875, 4000, {.maxSpeed = 100, .minSpeed = 5});
    chassis.waitUntil(28.65);
    preload.extend(true);

    // 转向并后退移动至中桥
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.turnToPoint(39.5, 21.45, 2000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 0.5});
    chassis.moveToPoint(39.5, 21.45, 1500, {.forwards = false, .maxSpeed = 80, .minSpeed = 30, .earlyExitRange = 0.5});
    chassis.waitUntil(11.25);
    intake.cancelMotion();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    // 中桥吐球
    scoreLift.extend(true);
    intake.outlet(pixlib::OutSide::MID, 1000);
    intake.waitUntilDone();
    scoreLift.extend(false);
    intake.intake(6000);
    
    // 移动到左侧填装桶，取填装
    chassis.turnToPoint(10, 51.15, 2000, {.maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 0.25});
    chassis.moveToPoint(10, 51.15, 3000, {.minSpeed = 5});
    chassis.turnToHeading(270, 2500, {.minSpeed = 30, .earlyExitRange = 0.5});
    chassis.waitUntilDone();
    preload.extend(true);
    chassis.moveDistance(10.5, 270, 1000, {.maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 2});
    chassis.move(50, 50, 700, true);

    // 后退至打球位并等待完成打球动作
    chassis.moveDistance(27.875, 270, 1500, {.forwards = false, .maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 0.25});
    chassis.waitUntil(21);
    longGoal.extend(true);
    chassis.waitUntilDone();
    intake.cancelMotion();
    
    preload.extend(false);
    intake.outlet(pixlib::OutSide::TOP, 1300);
    intake.waitUntilDone();
}

void skills() {
    chassis.setPose(0, 0, 180);

    // 移动至启动位右侧填装桶，右转取填装
    chassis.moveToPoint(0, -25.55, 4000, {.maxSpeed = 80, .minSpeed = 20, .earlyExitRange = 0.01});
    preload.extend(true);
    intake.intake(3540);
    chassis.turnToHeading(270, 2500, {.maxSpeed = 80, .minSpeed = 30});
    chassis.waitUntilDone();
    
    hook.extend(true);
    pros::delay(100);
    chassis.moveDistance(10, 270, 2000, {.maxSpeed = 60, .minSpeed = 31, .earlyExitRange = 7});
    chassis.move(50, 50, 1800, true);
    // chassis.move(-50, -50, 200, true);
    // chassis.move(50, 50, 800, true);
    chassis.waitUntilDone();

    // 后退，打4个红色球
    chassis.moveDistance(29.875, 270, 1500, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    intake.cancelMotion();
    longGoal.extend(true);
    intake.outlet(pixlib::OutSide::TOP, 720);
    intake.waitUntilDone();
    preload.extend(false);

    // 左转进长桥右侧通道
    chassis.moveDistance(8, 270, 2500, {.minSpeed = 30});
    chassis.turnToHeading(180, 2000, {.maxSpeed = 60, .minSpeed = 30});
    chassis.moveDistance(4.25, 180, 2000, {.maxSpeed = 60, .minSpeed = 40});
    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 2000, {.minSpeed = 40});
    
    // 通过通道，移动至长桥另一侧前
    chassis.moveDistance(67.5, 90, 4000, {.maxSpeed = 70, .minSpeed = 10});
    chassis.waitUntil(4);
    // intake.intake(2000);
    longGoal.extend(false);
    chassis.turnToPoint(97, -37.15, 3000, {.maxSpeed = 60, .minSpeed = 30, .earlyExitRange = 1});
    chassis.moveToPoint(97, -37.15, 3000, {.maxSpeed = 56, .minSpeed = 30, .earlyExitRange = 0.2});
    chassis.turnToHeading(90, 2000, {.maxSpeed = 80, .minSpeed = 30, .earlyExitRange = 0.5});

    // 后退，将车内剩余球打入长桥
    chassis.moveDistance(17.5, 90, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 30});
    chassis.waitUntilDone();
    longGoal.extend(true);
    intake.outlet(pixlib::OutSide::TOP, 1200);
    intake.waitUntilDone();

    // 前进，取出填装桶内填装
    preload.extend(true);
    longGoal.extend(false);
    chassis.moveDistance(14, 90, 2000, {.maxSpeed = 80, .minSpeed = 20, .earlyExitRange = 0.01});
    chassis.waitUntil(20);
    

    intake.intake(3000);
    chassis.move(55, 50, 1800, true);
     chassis.move(-40, -40, 200, true);
     chassis.move(40, 40, 800, true);
    chassis.waitUntilDone();

    // 后退，将车内球打入长桥
    chassis.moveDistance(29.875, 90, 1500, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    intake.cancelMotion();
    longGoal.extend(true);
    intake.outlet(pixlib::OutSide::TOP, 2000);
    intake.waitUntilDone();
    preload.extend(false);

    // 左转，取4球区球
    chassis.moveDistance(8, 90, 2000, {.maxSpeed = 80, .minSpeed = 30, .earlyExitRange = 0.1});
    chassis.turnToPoint(76.9, -4, 3000, {.maxSpeed = 60, .minSpeed = 30, .earlyExitRange = 0.25});
    chassis.waitUntilDone();
    longGoal.extend(false);
    intake.intake(6000);
    chassis.moveToPoint(76.9, -4, 4000, {.maxSpeed = 56, .minSpeed = 30});
    chassis.waitUntil(14);
    preload.extend(true);

    // 转向至另一半区4球，并取其4球
    chassis.turnToPoint(77.25, 40.95, 2000, {.maxSpeed = 80, .minSpeed = 30});
    chassis.waitUntil(20);
    preload.extend(false);
    chassis.moveToPoint(77.25, 40.95, 4000, {.maxSpeed = 60});
    chassis.waitUntil(32.25);
    preload.extend(true);
    chassis.waitUntilDone();
    pros::delay(100);

    // 转向至中桥低桥，将车内球吐出 69.140 32.927
    chassis.turnToPoint(67.3, 26.00, 2000, {.maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 0.25});
    chassis.waitUntil(20);
    preload.extend(false);
    chassis.moveToPoint(67.3, 26.00, 2000, {.maxSpeed = 60, .minSpeed = 30});
    
    chassis.waitUntil(4);
    intakeLift.extend(true);
    intake.cancelMotion();
    intake.outlet(pixlib::OutSide::BOTTOM, 2100);
    intake.waitUntilDone();

    // 后退且确保车内球全部吐出，移动至蓝方右侧填装桶
    intake.outlet(pixlib::OutSide::BOTTOM, 2000, {.maxSpeed = 200});
    chassis.turnToPoint(96, 64.20, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 30});
    chassis.moveToPoint(96 , 64.20, 5000, {.forwards = false, .maxSpeed = 56, .minSpeed = 30});
     chassis.waitUntilDone();
    intakeLift.extend(false);
    chassis.turnToHeading(90, 2000, {.maxSpeed = 80, .minSpeed = 10, .earlyExitRange = 5});
   
    preload.extend(true);

    // 取出填装
    chassis.moveDistance(10, 90, 2000, {.maxSpeed = 60, .minSpeed = 30, .earlyExitRange = 0.1});
    intake.intake(3000);
    chassis.move(43, 40, 1800, true);
    // chassis.move(-40, -40, 200, true);
    // chassis.move(40, 40, 800, true);
    chassis.waitUntilDone();

    // 后退，将3颗红球打入长桥
    chassis.moveDistance(30, 90, 1400, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    intake.cancelMotion();
    longGoal.extend(true);
    intake.outlet(pixlib::OutSide::TOP, 560);
    intake.waitUntilDone();
    preload.extend(false);

    // 左转至通道
    chassis.moveDistance(10, 90, 1600, {.minSpeed = 30});
    chassis.turnToHeading(180, 2500, {.maxSpeed = 60, .minSpeed = 30});
    chassis.moveDistance(5.05, 180, 2500, {.maxSpeed = 60, .minSpeed = 40});
    chassis.swingToHeading(270, lemlib::DriveSide::RIGHT, 1500, {.minSpeed = 40});

    // 通过通道并移动至长桥前
    chassis.moveDistance(68, 270, 4000, {.maxSpeed = 60});
    chassis.waitUntil(4);
    intake.intake(2000);
    longGoal.extend(false);
    chassis.turnToPoint(6, 67.95, 3000, {.maxSpeed = 80, .minSpeed = 50, .earlyExitRange = 0.5});
    chassis.moveToPoint(6, 67.95, 2000, {.maxSpeed = 60});
    chassis.turnToHeading(270, 2000, {.maxSpeed = 60, .minSpeed = 30});

    // 将车内剩余球打入长桥
    chassis.moveDistance(20, 270, 1000, {.forwards = false, .maxSpeed = 60, .minSpeed = 30});
    chassis.waitUntilDone();
    longGoal.extend(true);
    intake.outlet(pixlib::OutSide::TOP, 1300);
    intake.waitUntilDone();
    
    // 前进，取出填装
    preload.extend(true);
    longGoal.extend(false);
    chassis.moveDistance(16, 270, 2000, {.maxSpeed = 60, .minSpeed = 30, .earlyExitRange = 0.01});
    chassis.waitUntil(6);
    
    chassis.waitUntilDone();
    intake.intake(3000);
    chassis.move(45, 40, 1800, true);
     chassis.move(-40, -40, 200, true);
     chassis.move(40, 40, 800, true);
    chassis.waitUntilDone();

    // 后退，将车内球打入长桥
    chassis.moveDistance(31, 270, 1500, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    intake.cancelMotion();
    longGoal.extend(true);
    intake.outlet(pixlib::OutSide::TOP, 2200);
    intake.waitUntilDone();
    preload.extend(false);

    //
    chassis.moveDistance(15, 270, 2000, {.maxSpeed = 80, .minSpeed = 30});
    chassis.turnToHeading(207.5, 2000);
    chassis.moveDistance(31, 207.5, 4000, {.maxSpeed = 80, .minSpeed = 30, .earlyExitRange = 0.5});
    chassis.swingToHeading(185, lemlib::DriveSide::LEFT, 2000, {.maxSpeed = 80, .minSpeed = 50});
    chassis.waitUntilDone();
    longGoal.extend(false);
    preload.extend(true);
    intake.intake(6000);
    // chassis.moveUntilOverObstacle(185, 4000);
    chassis.moveDistance(18, 185, 8000, {.maxSpeed = 80, .minSpeed = 60});
    chassis.waitUntil(17);
    preload.extend(false);
}

void none() {

}