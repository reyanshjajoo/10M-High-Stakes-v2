#include "main.h"
#include "lemlib/api.hpp"
#include "constants.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

// COLOR SORT VALUES (200-240 Blue, 5-45 Red)
int LOWER_COLOR_RANGE = 5;
int UPPER_COLOR_RANGE = 45;

// init motorgroups
pros::MotorGroup left_motors({LEFT_MOTOR_FRONT, LEFT_MOTOR_TOP, LEFT_MOTOR_BACK});
pros::MotorGroup right_motors({RIGHT_MOTOR_FRONT, RIGHT_MOTOR_TOP, RIGHT_MOTOR_BACK});

// init motors
pros::Motor intake(INTAKE_PORT);
pros::Motor hook(HOOK_PORT, pros::MotorGearset::blue);
pros::Motor lb(LB_PORT);

// init pneumatics
pros::adi::DigitalOut clamp(CLAMP_PORT);
pros::adi::DigitalOut doinker(DOINKER_PORT);

// init sensors
pros::Imu imu(IMU_PORT);
pros::Optical optical(OPTICAL_PORT);

// init controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

//tracking wheels
pros::Rotation horizontal_encoder(HORTIZONTAL_TRACKING_ROTATION_PORT);
pros::Rotation vertical_encoder(VERTICAL_TRACKING_ROTATION_PORT);

//TODO: update offset
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -2.5);


// init drivetrain
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12.24, // track width
                              lemlib::Omniwheel::NEW_325, // wheel diameter
                              450, // drivetrain rpm
                              4 // horizontal drift is 2 (for now)
);

// init PID controllers
lemlib::ControllerSettings lateral_controller(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              4, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// init input curve
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                      10, // minimum output where drivetrain will move out of 127
                                      1.019 // expo curve gain
);

lemlib::ExpoDriveCurve steer_curve(6, // joystick deadband out of 127
                                   15, // minimum output where drivetrain will move out of 127
                                   1.01 // expo curve gain
);

//init odom
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have one
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// init chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);

bool clamp_down = false;
bool doinker_down = false;
bool color_sort = true;
float target_position = 0;

// init intake control with color sorting
void intake_control() {
    while (true) {
        // if the R1 button is pressed, start the intake and color sort (200-240 Blue, 5-45 Red)
        if ((optical.get_hue() > LOWER_COLOR_RANGE && optical.get_hue() < UPPER_COLOR_RANGE) && color_sort){
            intake.move_velocity(0);
            hook.move_velocity(0);
            pros::delay(51);
        }
        // move intake forward when R1 pressed, move backward when R2 pressed
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { 
            intake.move_velocity(600);
            hook.move_velocity(600);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intake.move_velocity(-600);
            hook.move_velocity(-600);
        }
        // if neither button is pressed, stop the intake
        else {
            intake.move_velocity(0);
            hook.move_velocity(0);
        }
        pros::delay(20);  // Small delay to prevent overload/freezing
    }
}


void pneumatic_control() {
	while (true) {
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {  // when L1 pressed, toggle clamp and display position
			clamp_down = !clamp_down;
			clamp.set_value(clamp_down);
            if (clamp_down){
                controller.print(0, 0, "DOWN");
            } else {
                controller.print(0, 0, "UP  ");
            }
		} 
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){ // when B pressed, toggle color sort and display
            color_sort = !color_sort;
            if (color_sort){
                controller.print(1, 0, "SORT  ");
            } else {
                controller.print(1, 0, "NOSORT");
            }
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) { // when L2 pressed, toggle doinker
			doinker_down = !doinker_down;
			doinker.set_value(doinker_down);
		} 

        // delay to save resources
		pros::delay(20); 
	}
}

//control ladybrown
void lb_control(){
    while (true) {
        // if left or right pressed move LB to loading, if down pressed move it down, or if up pressed move it up
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) || controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            target_position = -83.5;
            lb.move_absolute(-410, 400);
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            target_position = 0;
            lb.move_absolute(0, 400);
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
            lb.move_absolute(-2000, 400);
        }
        // delay to save resources
        pros::delay(20);
    }
}

void initialize() {
	pros::lcd::initialize(); // initialize brain screen
	chassis.calibrate(); // calibrate sensors
    lb.tare_position(); // calibrate lb motor
    optical.set_integration_time(20); // calibrate optical
    optical.set_led_pwm(50); // turn on optical LED
    lb.set_brake_mode(pros::MotorBrake::hold); // set LB motor to hold
	 pros::Task screen_task([&]() {
        while (true) {
            // print robot info to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(5, "Color Hue: %f", optical.get_hue()); // hue
            pros::lcd::print(6, "LB Position %f", lb.get_position()); // lb position

            // delay to save resources
            pros::delay(20);
        }
    });
}

// color sort specifically for autons
void auton_color_sort() {
    while (true){
        if (optical.get_hue() > LOWER_COLOR_RANGE && optical.get_hue() < UPPER_COLOR_RANGE){
            hook.move_velocity(0);
            pros::delay(51);
            hook.move_velocity(600);
            intake.move_velocity(200);
        }
        pros::delay(20);
    }
}

// autonomous code
void autonomous() {
    chassis.setPose(0, 0, 0);
    clamp.set_value(false);
}

void opcontrol() {
    // start all tasks
    pros::Task intake_task(intake_control);
    pros::Task pneumatic_task(pneumatic_control);
    pros::Task lb_task(lb_control);
    
    while (true) {
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(20);
    }
}