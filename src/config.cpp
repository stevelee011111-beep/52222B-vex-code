#include "config.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/util.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pixlib/functionControl.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// 电机设置：（负数代表对应端口电机反转）
// 底盘左侧电机组
pros::MotorGroup leftMotors({-12, -13, -14}, pros::MotorGearset::blue);
// 底盘右侧电机组
pros::MotorGroup rightMotors({17, 18, 19}, pros::MotorGearset::blue);

// 吸块电机组
pros::Motor intakeMotor(-11, pros::MotorGearset::green);  // 底部吸块
pros::Motor scoreMotor(-20, pros::MotorGearset::blue);    // 顶部得分

// 气杆设置：（例：三线拓展器在1，电磁阀在A，则格式为{1, 'A'}；无三线拓展器则直接写对应字母三线口，如'A'）
// 取填装气杆
pros::adi::Port preloadPiston('H', pros::E_ADI_DIGITAL_OUT);
// 长桥消分气杆
pros::adi::Port hookPiston('C', pros::E_ADI_DIGITAL_OUT);
// 中桥消分气杆
pros::adi::Port midDisScorePiston('D', pros::E_ADI_DIGITAL_OUT);
// 长桥挡球气杆
pros::adi::Port longGoalPiston('F', pros::E_ADI_DIGITAL_OUT);
// 车头抬起气杆
pros::adi::Port intakePiston('E', pros::E_ADI_DIGITAL_OUT);
// 打球抬降气杆
pros::adi::Port scorePiston('G', pros::E_ADI_DIGITAL_OUT);

// 传感器设置：
// 惯性传感器（陀螺仪）
pros::Imu imu(1);
// 旋转传感器（水平定位轮），方向反转
pros::Rotation horizontalEnc(3); 
// 旋转传感器（垂直定位轮），方向正转
pros::Rotation verticalEnc(-2);

// 以下设置在不修改车体定位轮位置、底盘车轮位置的前提下，无需修改！

// 水平定位轮设置
lemlib::TrackingWheel horizontal(
    &horizontalEnc,                  // 水平定位轮
    lemlib::Omniwheel::NEW_2, // 定位轮直径
    -3.125 // 安装距旋转中心垂直方向距离，正数即安装在中心前方，负数后方
);

// 垂直定位轮设置
lemlib::TrackingWheel
    vertical(&verticalEnc,                    // 垂直定位轮
             lemlib::Omniwheel::NEW_2, // 定位轮直径
             1.125 // 安装距旋转中心水平方向距离，正数即安装在中心右方，负数左方
    );

// 底盘设置
lemlib::Drivetrain
    drivetrain(&leftMotors, &rightMotors,
               11.375,                       // 左右轮间距
               lemlib::Omniwheel::NEW_325, // 驱动轮直径
               450,                        // 驱动轮转速
               358,
               2 // 底盘横向飘移数值，有牵引胶轮通用为8，无牵引胶轮通用为2
    );

// 定位轮设置
lemlib::OdomSensors
    sensors(&vertical, // 1号垂直定位轮，无则设置为
                     // nullptr，使用底盘左侧电机编码器实现垂直方向定位
            nullptr, // 2号垂直定位轮，无则设置为
                     // nullptr，使用底盘右侧电机编码器实现垂直方向定位
            &horizontal, // 1号水平定位轮，不使用则设置为 nullptr
            nullptr, // 2号水平定位轮，不使用则设置为 nullptr 
            &imu     // 惯性传感器
    );

// 移动 PID 参数设置
lemlib::ControllerSettings linearController(10, 0, 8, 3, 1, 100, 3, 500, 20);
// 偏航 PID 参数设置
lemlib::ControllerSettings headingController(2.7, 0, 12, 3, 0, 0, 0, 0, 0);
// 转向 PID 参数设置
lemlib::ControllerSettings angularController(2, 0, 10, 3, 1, 100, 3, 500, 20);

// 油门曲率输入参数设置
lemlib::ExpoDriveCurve throttleCurve(5,   // 摇杆死区范围
                                     10,   // 最小输出功率
                                     1.019    // 曲率系数
);

// 转向曲率输入参数设置
lemlib::ExpoDriveCurve steerCurve(5,      // 摇杆死区
                                  10,      // 最小输出功率
                                  1.019       // 曲率系数
);

// 构造底盘实例
lemlib::Chassis chassis(drivetrain, 
                        sensors, 
                        linearController, 
                        headingController,
                        angularController, 
                        &throttleCurve, 
                        &steerCurve);

// 构造吸取实例
pixlib::Intake intake(&intakeMotor, &scoreMotor);

// 构造气动实例
pixlib::Piston preload(&preloadPiston);
pixlib::Piston hook(&hookPiston);
pixlib::Piston midDisScore(&midDisScorePiston);
pixlib::Piston longGoal(&longGoalPiston);
pixlib::Piston intakeLift(&intakePiston);
pixlib::Piston scoreLift(&scorePiston);

void infoDisplay(int timeNow) {
    controller.clear();
    pros::delay(75);
    controller.print(0, 0, "Temp:%.0f°C", (lemlib::average(drivetrain.leftMotors->get_temperature_all()) + lemlib::average(drivetrain.rightMotors->get_temperature_all())) / 2);
    pros::delay(75);
    controller.print(1, 0, "Brain:%.0f%%", pros::battery::get_capacity());
    pros::delay(75);
    controller.print(2, 0, "Ctrl:%d%%", controller.get_battery_capacity());
    pros::delay(75);
}