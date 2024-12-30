#include "main.h"
#include "lemlib/api.hpp"
#include "constants.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rotation.hpp"


// init motorgroups
pros::MotorGroup left_motors({LEFT_MOTOR_1, LEFT_MOTOR_2, LEFT_MOTOR_3});
pros::MotorGroup right_motors({RIGHT_MOTOR_1, RIGHT_MOTOR_2, RIGHT_MOTOR_3});

// init motors
pros::Motor intake(INTAKE_PORT, pros::MotorGearset::blue);
pros::Motor hook(HOOK_PORT, pros::MotorGearset::blue);

// init pneumatics
pros::adi::DigitalOut clamp_pneumatic(CLAMP_PORT);
pros::adi::DigitalOut doinker(DOINKER_PORT);

// init sensors
pros::Imu imu(IMU_PORT);
pros::Optical optical(OPTICAL_PORT);
pros::Rotation lb_rotation(LB_ROTATION_PORT);
pros::Rotation vertical_encoder(VERTICAL_TRACKING_ROTATION_PORT);
pros::Rotation horizontal_encoder(HORTIZONTAL_TRACKING_ROTATION_PORT);

//init controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// init drivetrain
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12.24, // track width
                              lemlib::Omniwheel::NEW_325, // wheel diameter
                              450, // drivetrain rpm
                              2 // horizontal drift is 2 (for now)
);

// init tracking wheels
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

//init odom
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// init PID
// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
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

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

//init chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);


void initialize() {
	pros::lcd::initialize(); // initialize brain screen
	chassis.calibrate(); // calibrate sensors
	while (true) {
        // print measurements from the rotation sensor
        pros::lcd::print(0, "Vertical: %i", vertical_encoder.get_position());
		pros::lcd::print(1, "Horizontal: %i", horizontal_encoder.get_position());
		pros::lcd::print(2, "Ladybrown: %i", lb_rotation.get_position());

		// print robot location to the brain screen
		pros::lcd::print(3, "X: %f", chassis.getPose().x); // x
		pros::lcd::print(4, "Y: %f", chassis.getPose().y); // y
		pros::lcd::print(5, "Theta: %f", chassis.getPose().theta); // heading
		// delay to save resources
		pros::delay(20);
    }
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	while (true) {
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

        // delay to save resources
        pros::delay(25);
	};
}