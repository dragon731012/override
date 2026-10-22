#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "robodash/api.h"

pros::MotorGroup left_motors({-5, -3, -1});
pros::MotorGroup right_motors({6, 4, 2});

lemlib::Drivetrain drivetrain(&left_motors, 
                              &right_motors, 
                              9.7, 
                              lemlib::Omniwheel::NEW_275, 
                              450, 
                              2 
);

pros::Imu imu(11);

//pros::Rotation hr1(7);
pros::Rotation vr2(-14);

//lemlib::TrackingWheel horizontal_tracking_wheel(&hr1, 2.0, 0.5);
lemlib::TrackingWheel vertical_tracking_wheel(&vr2, 2.0, 0);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, 
                            nullptr, 
                            nullptr/*&horizontal_tracking_wheel*/, 
                            nullptr, 
                            &imu
);

// LATERAL PID (Driving Straight)
lemlib::ControllerSettings lateral_controller(
    10,   // kP
    0,    // kI
    3,    // kD
    3,    // anti windup
    1,    // small error range
    100,  // small error timeout
    3,    // large error range
    500,  // large error timeout
    20    // slew
);

lemlib::ControllerSettings angular_controller(
    4.5, // kP
    0,   // kI
    35,  // kD
    3,   // anti windup
    1,   // small error range
    100, // small error timeout
    3,   // large error range
    500, // large error timeout
    0    // slew
);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

//motors
pros::Motor color_switcher(15);
pros::Motor claw(16);
bool clawOpened = true;
pros::Motor intake(18);
pros::Motor lift(17);

//pnumatics
/*pros::adi::DigitalOut tongue(1);
bool tongueExtended = false;*/

//sensors
pros::Optical color_sensor(12);


int allianceColor = 0;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

void disabled() {

}

/*void useTongue() {
    tongueExtended=!tongueExtended;
    tongue.set_value(tongueExtended);
}*/

void toAlliance() {
    double hue = color_sensor.get_hue();
    // gets color (0 = red, 1 = yellow, 2 = blue)
    // I <3 ternary operators
    int color = (hue < 20 || hue > 340) ? (0) : ((hue >= 40 && hue <= 60) ? (1) : ((hue >= 200 && hue <= 240) ? (2) : (-1)));

    if (color == allianceColor) color_switcher.move_velocity(0);
    else color_switcher.move_velocity(100);
}

void toggleClaw() {
    if (clawOpened) claw.move_relative(90, 100);
    else claw.move_relative(-90, 100);
    clawOpened = !clawOpened;
}

void soloawp() {
    chassis.setPose(0,0,-90);
}

void nomove() {

}

void skills() {
    chassis.setPose(0,0,-90);
}

rd::Selector selector({
    {"No Move", nomove},
    {"Skills", skills},
    {"Solo AWP", soloawp}
});


void initialize() {
    chassis.calibrate(); 
    selector.focus();  

    //pros::lcd::initialize();

    left_motors.set_brake_mode(MOTOR_BRAKE_BRAKE);
    right_motors.set_brake_mode(MOTOR_BRAKE_BRAKE);
    lift.set_brake_mode(MOTOR_BRAKE_HOLD);
    claw.set_brake_mode(MOTOR_BRAKE_HOLD);
}

void competition_initialize() {

}

void autonomous() {
    if (selector.get_auton().has_value()) {
        selector.run_auton();
    } else {
        //fallback program here
    }
}

void opcontrol() {
    left_motors.set_brake_mode(MOTOR_BRAKE_BRAKE);
    right_motors.set_brake_mode(MOTOR_BRAKE_BRAKE);

	while (true) {
        /*if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            forward = !forward;
        }*/

        //buttons
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) toAlliance();
        else color_switcher.move_velocity(0);

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) intake.move_velocity(-100);
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) intake.move_velocity(100);
        else intake.move_velocity(0);

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) lift.move_velocity(100);
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) lift.move_velocity(-100);
        else lift.move_velocity(0);
        
        //driving
        int RightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int LeftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        chassis.arcade(LeftY,RightX);

        pros::delay(25);
    }
}
