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

int current = 0;
bool forward = true;

//motors
pros::Motor intake1(15);
pros::Motor intake2(-16);

double intake2mult = 1;
bool intake2forward = true;

//pnumatics
pros::adi::DigitalOut tongue(1);
bool tongueExtended = false;
pros::adi::DigitalOut wing(2);
bool wingExtended = false;
pros::adi::DigitalOut hood(4);
bool hoodExtended = false;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

void disabled() {

}

void useTongue() {
    tongueExtended=!tongueExtended;
    tongue.set_value(tongueExtended);
}

void useWing() {
    wingExtended=!wingExtended;
    wing.set_value(wingExtended);
}

void useHood() {
    hoodExtended=!hoodExtended;
    hood.set_value(hoodExtended);
}

void spin(int val) {
    intake1.move_velocity(6*val);

	if (intake2forward){
        intake2.move_velocity(6*val * intake2mult);
	} else {
	    intake2.move_velocity(-6*val * intake2mult);
	}
}

void storeMode() {
    if (hoodExtended) useHood();
    intake2forward = true;
    intake2mult = 1;
}

void scoreMode() {
    if (!hoodExtended) useHood();
    intake2forward = true;
    intake2mult = 1;
}

void sballrush(int m) {
    chassis.setPose(0,0,0);

    int goalPos = -32*m;
    int matchloaderOffset = 1.5*m;
    
    //move from start
    storeMode();
    chassis.moveToPoint(-0.1,22,1000,{.maxSpeed=100});

    //move to the balls and pick them up
    spin(-100);
    chassis.moveToPoint(-6.8*m,41,3500,{.maxSpeed=40});

    //go in front of matchloader
    chassis.moveToPoint(goalPos + matchloaderOffset,16,1700,{.maxSpeed=100});
    storeMode();
    spin(-70);

    //turn towards the matchloader
    chassis.turnToHeading(180,1000,{.maxSpeed=40});
    storeMode();
    spin(-100);
    useTongue();

    //use the matchloader
    chassis.moveToPoint(goalPos + matchloaderOffset,0,800,{.maxSpeed=60});
    chassis.moveToPoint(goalPos + matchloaderOffset,9,800,{.maxSpeed=60});
    chassis.moveToPoint(goalPos + matchloaderOffset,0,800,{.maxSpeed=60});

    //move towards the goal
    chassis.moveToPoint(goalPos, 40,1000,{.forwards = false,.maxSpeed=80});
    spin(0);

    //outtake
    pros::delay(800);
    scoreMode();
    useTongue();
    spin(-100);
    pros::delay(2300);

    //push balls
    chassis.moveToPoint(goalPos, 20,1000,{.forwards = true,.maxSpeed=80});
    storeMode();
    chassis.moveToPoint(goalPos, 40,1000,{.forwards = false,.maxSpeed=127});
    pros::delay(800);
    scoreMode();

    //wing
    /*if (m == 1) {
        useWing();
        chassis.moveToPoint(goalPos - (m*15), 20,1000,{.forwards = true,.maxSpeed=127});
        chassis.turnToHeading(190, 700, {.maxSpeed=127});
        chassis.moveToPoint(goalPos - (m*17), 35,1000,{.forwards = false,.maxSpeed=80});
        chassis.turnToHeading(170, 700, {.maxSpeed=127});
        chassis.moveToPoint(goalPos - (m*5), 40,1000,{.forwards = false,.maxSpeed=80});
        useWing();
        chassis.moveToPoint(goalPos - (m*5), 52,1000,{.forwards = false,.maxSpeed=127});
    }*/
}

void sballrushl() {sballrush(1);}

void sballrushr() {sballrush(-1);}

void soloawp() {
    chassis.setPose(0,0,-90);

    //push other robot
    chassis.moveToPoint(-12, 0, 1000, {.maxSpeed=127});

    //go to matchloader
    chassis.moveToPoint(31.2, 0, 1000, {.forwards = false, .maxSpeed=127});
    chassis.turnToHeading(174, 700, {.maxSpeed=127});
    useTongue();
    spin(-100);
    storeMode();
    chassis.moveToPoint(42, -15, 1000, {.maxSpeed=80});
    pros::delay(1500);

    //go to goal and outtake
    chassis.moveToPoint(42.5, 30, 1500, {.forwards = false, .maxSpeed=127});
    scoreMode();
    spin(-100);
    pros::delay(1500);

    //get other balls
    chassis.moveToPoint(39.3, 15, 500, {.forwards = true, .maxSpeed=127});
    chassis.turnToHeading(-40, 800,{.maxSpeed=127});
    storeMode();
    spin(-100);
    chassis.moveToPoint(20, 30, 2000, {.forwards = true, .maxSpeed=127});

    //score in goal
    chassis.moveToPoint(-49.5, 4, 3000, {.forwards = true, .maxSpeed=127});
    chassis.turnToHeading(174, 700, {.maxSpeed=127});
    chassis.moveToPoint(-49.5, 30, 1200, {.forwards = false, .maxSpeed=127});
    scoreMode();
    useTongue();
    spin(-100);
    pros::delay(1500);

    //get other balls
    chassis.moveToPoint(-47.5, 15, 500, {.forwards = true, .maxSpeed=127});
    chassis.turnToHeading(40, 800,{.maxSpeed=127});
    storeMode();
    spin(-100);
    chassis.moveToPoint(-20, 30, 2000, {.forwards = true, .maxSpeed=127});
    storeMode();
    chassis.turnToHeading(-45, 800,{.maxSpeed=127});
    chassis.moveToPoint(-5, 40, 500, {.forwards = true, .maxSpeed=127});
    intake2forward = false;
}

void nomove() {

}

void skills() {
    spin(-1);
}

void moveinch() {
    spin(-10);
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,1.5, 1000, {.maxSpeed=80});
}

rd::Selector selector({
    {"Seven Ball Rush (Left)", sballrushl},
    {"Seven Ball Rush (Right)", sballrushr},
    {"No Move", nomove},
    {"Skills", skills},
    {"Move Inch", moveinch},
    {"Solo AWP", soloawp}
});


void initialize() {
    chassis.calibrate(); 
    selector.focus();  

    //pros::lcd::initialize();

    left_motors.set_brake_mode(MOTOR_BRAKE_BRAKE);
    right_motors.set_brake_mode(MOTOR_BRAKE_BRAKE);
}

void competition_initialize() {

}

void autonomous() {
    if (selector.get_auton().has_value()) {
        selector.run_auton();
    } else {
        sballrushl();
    }
}

void opcontrol() {
    left_motors.set_brake_mode(MOTOR_BRAKE_BRAKE);
    right_motors.set_brake_mode(MOTOR_BRAKE_BRAKE);

	while (true) {
        //switching direction
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            forward = !forward;
        }
        
        //driving
        int RightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int LeftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        chassis.arcade(LeftY,RightX);

        pros::delay(25);
    }
}
