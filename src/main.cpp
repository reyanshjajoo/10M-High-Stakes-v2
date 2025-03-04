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

int RED_RING_COLOR = 14;
int BLUE_RING_COLOR = 206;

int color_to_sort = RED_RING_COLOR;

int LOWER_COLOR_RANGE = color_to_sort-40;
int UPPER_COLOR_RANGE = color_to_sort+40;

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

// init tracking wheels
pros::Rotation horizontal_encoder(HORTIZONTAL_TRACKING_ROTATION_PORT);
pros::Rotation vertical_encoder(VERTICAL_TRACKING_ROTATION_PORT);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0.425);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -0.0866);


// init drivetrain
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12.23696614, // track width
                              lemlib::Omniwheel::NEW_325, // wheel diameter
                              450, // drivetrain rpm
                              2 // horizontal drift is 2 since we have traction wheels
);

// init PID controllers
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              5, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(2.73, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              14, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// init input curve
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                      10, // minimum output where drivetrain will move out of 127
                                      1.019 // expo curve gain
);

lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
    10, // minimum output where drivetrain will move out of 127
    1.019 // expo curve gain
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

// init control bools
bool clamp_down = false;
bool doinker_down = false;
bool color_sort = true;

// intake control with color sorting
void intake_control() {
    while (true) {
        // if the R1 button is pressed, start the intake and color sort (200-240 Blue, 5-45 Red)
        if ((optical.get_hue() > LOWER_COLOR_RANGE && optical.get_hue() < UPPER_COLOR_RANGE) && color_sort){
            intake.move_velocity(0);
            hook.move_velocity(0);
            pros::delay(50);
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
        pros::delay(20);  // delay to prevent overload/freezing
    }
}

// pneumatic control
void pneumatic_control() {
	while (true) {
        // when L1 pressed, toggle clamp and display position
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			clamp_down = !clamp_down;
			clamp.set_value(clamp_down);
            if (clamp_down){
                controller.print(0, 0, "DOWN");
            } else {
                controller.print(0, 0, "UP  ");
            }
		} 
        // when B pressed, toggle color sort and display
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            color_sort = !color_sort;
            if (color_sort){
                controller.print(1, 0, "SORT  ");
            } else {
                controller.print(1, 0, "NOSORT");
            }
        }
        // when L2 pressed, toggle doinker
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
			doinker_down = !doinker_down;
			doinker.set_value(doinker_down);
		} 

        // delay to save resources
		pros::delay(20); 
	}
}

// control ladybrown
void lb_control(){
    while (true) {
        // if left pressed move LB to loading, if down pressed move it down, if up pressed move it up, or if right pressed move it all the way down
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
            lb.move_absolute(LB_MID, 400);
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            lb.move_absolute(LB_DOWN, 400);
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
            lb.move_absolute(LB_UP, 400);
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            lb.move_absolute(LB_TIP, 400);
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
            pros::lcd::print(3, "Color Hue: %f", optical.get_hue()); // hue
            pros::lcd::print(4, "LB Position %f", lb.get_position()); // lb position

            pros::lcd::print(5, "Vertical Sensor: %i", vertical_encoder.get_position()); // vertical sensor
            pros::lcd::print(6, "Horizontal Sensor: %i", horizontal_encoder.get_position()); // horizontal sensor

            // delay to save resources
            pros::delay(20);
        }
    });
}

// color sort specifically for autons
bool stop_after_sorting = false;
void auton_color_sort() {
    while (true){
        if (optical.get_hue() > LOWER_COLOR_RANGE && optical.get_hue() < UPPER_COLOR_RANGE){
            if (stop_after_sorting){
                pros::delay(100);
            }
            hook.move_velocity(0);
            pros::delay(50);
            if (!stop_after_sorting){
                hook.move_velocity(600);
            }
        }
        pros::delay(20);
    }
}

// autonomous code
void autonomous() {
    chassis.setPose(0, 0, 0);
    clamp.set_value(false);
    
    // SKILLS CODE
    // chassis.setPose(-63.5, 0, 90);
    // //score on alliance stake
    // hook.move_velocity(600);
    // pros::delay(250);
    // hook.brake();
    // // clamp goal
    // chassis.moveToPose(-47.5, 9, 0, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);
    // chassis.moveToPose(-47.5, -19, 0, 3000, {.forwards=false,.horizontalDrift = 8, .lead = 0.3, .maxSpeed=65, .minSpeed=15}, false);
    // pros::delay(250);
    // clamp.set_value(true);
    // pros::delay(250);
    // // get rings and go to wall stake
    // hook.move_velocity(600);
    // intake.move_velocity(600);
    // chassis.moveToPose(-23.5, -23.5, 90, 3000, {.horizontalDrift = 8, .lead = 0.3}, false); // ring 1
    // chassis.moveToPose(0, -59, 180, 3000, {.horizontalDrift = 8, .lead = 0.3}, false); // ring 2
    // lb.move_absolute(LB_MID, 400);
    // pros::delay(1000);
    // // score on wall stake
    // chassis.moveToPose(0, -64, 180, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);
    // hook.brake();
    // intake.brake();
    // lb.move_absolute(LB_UP, 400);
    // chassis.moveToPose(0, -48, 180, 3000, {.forwards=false, .horizontalDrift = 8, .lead = 0.3}, false);
    // lb.move_absolute(LB_DOWN, 400);
    // //grab rest of rings + corner
    // hook.move_velocity(600);
    // intake.move_velocity(600);
    // chassis.turnToHeading(270, 3000, {}, false);
    // chassis.moveToPose(-59, -47, 270, 3000, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed=90}, false);
    // chassis.turnToHeading(125, 3000, {}, false);
    // chassis.moveToPose(-47, -59, 125, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);
    // chassis.moveToPose(-62, -61, 45, 3000, {.forwards=false,.horizontalDrift = 8, .lead = 0.3}, false);
    // clamp.set_value(false);
    // //grab next goal
    // chassis.moveToPose(-47.5, 0, 0, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);
    // chassis.moveToPose(-47.5, 19, 180, 3000, {.forwards=false, .horizontalDrift = 8, .lead = 0.3, .maxSpeed=65, .minSpeed=15}, false);
    // pros::delay(250);
    // clamp.set_value(true);
    // pros::delay(250);
    // // get rings and go to wall stake
    // hook.move_velocity(600);
    // intake.move_velocity(600);
    // chassis.moveToPose(-23.5, 23.5, 90, 3000, {.horizontalDrift = 8, .lead = 0.3}, false); // ring 1
    // chassis.moveToPose(0, 59, 0, 3000, {.horizontalDrift = 8, .lead = 0.3}, false); // ring 2
    // lb.move_absolute(LB_MID, 400);
    // pros::delay(1000);
    // // score on wall stake
    // chassis.moveToPose(0, 64, 0, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);
    // hook.brake();
    // intake.brake();
    // lb.move_absolute(LB_UP, 400);
    // chassis.moveToPose(0, 48, 0, 3000, {.forwards=false, .horizontalDrift = 8, .lead = 0.3,}, false);
    // lb.move_absolute(LB_DOWN, 400);
    // //grab rest of rings + corner
    // hook.move_velocity(600);
    // intake.move_velocity(600);
    // chassis.turnToHeading(270, 3000, {}, false);
    // chassis.moveToPose(-59, 47, 270, 3000, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed=90}, false);
    // chassis.turnToHeading(125, 3000, {}, false);
    // chassis.moveToPose(-47, 59, 45, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);
    // chassis.moveToPose(-62, 61, 125, 3000, {.forwards=false,.horizontalDrift = 8, .lead = 0.3}, false);
    // clamp.set_value(false);
    // //go to far goal with ring in LB
    // lb.move_absolute(LB_MID, 400);
    // chassis.moveToPose(23.5, 47.5, 90, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);
    // chassis.moveToPose(47, 4.5, 0, 3000, {.forwards = false, .horizontalDrift = 8, .lead = 0.3, .maxSpeed=65, .minSpeed=15}, false);
    // pros::delay(250);
    // clamp.set_value(true);
    // pros::delay(250);
    // //alliance stake
    // chassis.moveToPose(64, 0, 90, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);
    // chassis.moveToPose(55.5, 0, 90, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);
    // hook.brake();
    // intake.brake();
    // lb.move_absolute(LB_ALLIANCE_STAKE, 400);
    // chassis.moveToPose(47, 0, 90, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);
    // lb.move_absolute(LB_DOWN, 400);
    // hook.move_velocity(600);
    // intake.move_velocity(600);
    // //grab rings
    // chassis.moveToPose(47, -47, 180, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);
    // chassis.moveToPose(24, -47, 270, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);
    // chassis.moveToPose(24, -24, 0, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);
    // chassis.moveToPose(0, 0, 315, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);
    // chassis.moveToPose(24, 24, 45, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);
    // chassis.moveToPoint(24, 47, 3000, {.earlyExitRange=6}, false);
    // chassis.moveToPose(47, 59, 90, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);
    // pros::delay(800);
    // hook.brake();
    // intake.brake();
    // //put full goal in corner
    // chassis.moveToPose(62, 62, 225, 3000, {.forwards = false, .horizontalDrift = 8, .lead = 0.3}, false);
    // clamp.set_value(false);
    // //put last goal in corner
    // chassis.moveToPoint(40, 23, 3000, {.earlyExitRange=4}, false);
    // chassis.moveToPose(59, -14, 180, 3000, {.forwards=false, .horizontalDrift = 8, .lead = 0.3}, false);
    // chassis.moveToPose(56.5, -14.5, 170, 3000, {.forwards=false, .horizontalDrift = 8, .lead = 0.3}, false);
    // chassis.moveToPose(63.5, -59, 170, 3000, {.forwards=false, .horizontalDrift = 8, .lead = 0.3}, false);
    // //hang
    // lb.move_absolute(LB_UP, 400);
    // chassis.moveToPose(15, -15, 315, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);
    

    //SWAP
    pros::Task auton_color_sort_task(auton_color_sort);
    chassis.setPose(54, 15, 135);
    lb.move_absolute(LB_ALLIANCE_STAKE, 600);
    pros::delay(1000);
    chassis.moveToPoint(49, 20, 1000, {.forwards=false, .earlyExitRange=8}, false);
    lb.move_absolute(-LB_MID, 600);
    chassis.moveToPose(22, 24.5, 120, 1500, {.forwards = false, .horizontalDrift = 8, .lead = 0.3, .maxSpeed=60, .minSpeed=15,}, false);
    pros::delay(250);
    clamp.set_value(true);
    pros::delay(200);
    lb.tare_position();
    intake.move_velocity(600);
    hook.move_velocity(600);
    chassis.turnToHeading(335, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPose(4, 37, 335, 1500, {.horizontalDrift = 8, .lead = 0.3,}, false);
    chassis.moveToPose(3, 55, 0, 2000, {.horizontalDrift = 8, .lead = 0.3,.maxSpeed=40}, false);
    pros::delay(300);
    chassis.moveToPose(24, 48, 90, 1500, {.horizontalDrift = 8, .lead = 0.3,}, false);
    pros::delay(300);
    hook.move_velocity(300);
    chassis.moveToPose(42, 13, 180, 1500, {.horizontalDrift = 8, .lead = 0.3,.earlyExitRange=4}, false);
    clamp.set_value(false);
    stop_after_sorting = true;
    chassis.moveToPose(42, -15, 180, 1500, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed=33}, false);
    stop_after_sorting=false;
    chassis.moveToPose(20, -24.5, 60, 1500, {.forwards = false, .horizontalDrift = 8, .lead = 0.3, .maxSpeed=60, .minSpeed=15,}, false);
    pros::delay(250);
    clamp.set_value(true);
    pros::delay(500);
    hook.move_velocity(600);
    lb.move_absolute(LB_UP-500, 400);
    chassis.swingToHeading(315, lemlib::DriveSide::LEFT, 800, {}, false);
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