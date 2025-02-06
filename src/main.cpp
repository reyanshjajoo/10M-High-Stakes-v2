#include "main.h"
#include <iostream>
#include "lemlib/api.hpp"
#include "constants.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

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
pros::Imu imu2(IMU2_PORT);
pros::Optical optical(OPTICAL_PORT);
pros::Rotation lb_encoder(LB_ROTATION_PORT);

// init controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

lemlib::PID lb_pid(1, 0, 1);

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
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu2 // inertial sensor
);

// init chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors, // No tracking wheels
                        &throttle_curve, 
                        &steer_curve
);

bool clamp_down = false;
bool doinker_down = false;
bool color_sort = true;
float target_position = 0;

// Initialize intake control with color sorting
void intake_control() {
    while (true) {
        // If the R1 button is pressed, start the intake (200-240 Blue, 5-45 Red)
        if ((optical.get_hue() > 200 && optical.get_hue() < 240) && color_sort){
            intake.move_velocity(0);
            hook.move_velocity(0);
            pros::delay(51);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move_velocity(600);  // Move intake forward
            hook.move_velocity(600);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intake.move_velocity(-600);
            hook.move_velocity(-600);
        }
        // If neither button is pressed, stop the intake
        else {
            intake.move_velocity(0); // Stop the intake
            hook.move_velocity(0);
        }


        pros::delay(20);  // Small delay to prevent controller overload
    }
}


void pneumatic_control() {
	while (true) {
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			clamp_down = !clamp_down;
			clamp.set_value(clamp_down);
            if (clamp_down){
                controller.print(0, 0, "DOWN");
            } else {
                controller.print(0, 0, "UP  ");
            }
		} 
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            color_sort = !color_sort;
            if (color_sort){
                controller.print(1, 0, "SORT  ");
            } else {
                controller.print(1, 0, "NOSORT");
            }
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
			doinker_down = !doinker_down;
			doinker.set_value(doinker_down);
		} 
		pros::delay(20);
	}
}

void lb_pid_task() {
    while (true) {
        double current_position = lb_encoder.get_position(); // Read motor encoder position
        double output = lb_pid.update(current_position-target_position); // Calculate PID output
        lb.move(output); // Apply output to the motor

        pros::delay(20); // Allow the system to update
    }
}

void lb_control(){
    while (true) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) || controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            target_position = -83.5;
            lb.move_absolute(-410, 400);
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            target_position = 0;
            lb.move_absolute(0, 400);
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
            lb.move_absolute(-2000, 400);
        }
        pros::delay(20);
    }
}

void initialize() {
	pros::lcd::initialize(); // initialize brain screen
	chassis.calibrate(); // calibrate sensors
    lb.tare_position();
    optical.set_integration_time(20);
    optical.set_led_pwm(50);
    lb_encoder.reset_position();
    lb.set_brake_mode(pros::MotorBrake::hold);
	// lb_encoder.reset();
	// optical.set_led_pwm(100);
	// optical.set_integration_time(20);
	 pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(4, "Hook Draw: %f", hook.get_current_draw()); // draw
            pros::lcd::print(5, "Color Hue: %f", optical.get_hue()); // hue
            pros::lcd::print(6, "LB Position %f", lb.get_position());
            pros::lcd::print(6, "Hook Velocity %f", hook.get_actual_velocity());

            std::cout << "Position: " << lb_encoder.get_position() << std::endl;
            // delay to save resources
            pros::delay(20);
        }
    });
}
void anti_jam() {
    while (true){
        if (hook.get_actual_velocity() == 0){
            hook.move_velocity(-100);
            pros::delay(50);
            hook.move_velocity(400);
        }
        if (optical.get_hue() > 200 && optical.get_hue() < 240){
            hook.move_velocity(0);
            pros::delay(51);
        }
        pros::delay(20);
    }
}

void autonomous() {
    chassis.setPose(0, 0, 0);
    clamp.set_value(false);

    // RED GOAL RUSH
    chassis.turnToHeading(-18, 1000,{.minSpeed=50, .earlyExitRange=12}, false);
    intake.move_velocity(600);
    chassis.moveToPoint(-15, 43, 2000, {}, false);
    doinker.set_value(true);
    pros::delay(300);
    chassis.moveToPose(-15, 10, 0, 2000, {.forwards=false, .minSpeed=35}, false);
    doinker.set_value(false);
    pros::delay(200);
    chassis.turnToHeading(-145, 1000, {.minSpeed=50, .earlyExitRange=12}, false);
    chassis.moveToPoint(-6, 23 , 1000, {.forwards=false, .minSpeed=25}, false);
    clamp.set_value(true);
    pros::delay(250);
    hook.move_velocity(600);
    intake.move_velocity(600);
    pros::delay(1000);
    // chassis.turnToHeading(0, 1000, {.minSpeed=40}, false);
    //chassis.turnToHeading(0, 1000,{.minSpeed=50, .earlyExitRange=12}, false);
    clamp.set_value(false);
    hook.brake();
    chassis.moveToPose(-40, 36.5, 90, 3000, {.forwards=false, .minSpeed=30}, false);
    clamp.set_value(true);
    pros::delay(250);
    clamp_down = true;
    hook.move_velocity(600);
    pros::delay(250);
    pros::Task anti_jam_task(anti_jam);
    chassis.moveToPose(-63, 5, -145, 3000, {}, false);
    pros::delay(1000);
    // doinker.set_value(true);
    chassis.turnToHeading(0, 1000);




    //blue 2 ring
    // chassis.moveToPoint(0, -40, 3000, {.forwards=false, .maxSpeed=75}, false);
    // pros::delay(200);
    // clamp_down = true;
    // clamp.set_value(true);
    // pros::delay(200);
    // intake.move_velocity(600);
    // hook.move_velocity(600);
    // pros::delay(1000);
    // chassis.turnToHeading(90, 3000);
    // chassis.moveToPoint(23, -38, 3000);
    // pros::delay(3000);
    // intake.move_velocity(0);
    // hook.move_velocity(0);
    // chassis.turnToHeading(-90, 3000, {}, false);
    // chassis.moveToPoint(-17, -38, 3000, {}, false);
    // chassis.turnToHeading(-135, 3000, {}, false);
    // lb.move_absolute(-2000, 400);


    //red 2 ring
    // chassis.moveToPoint(0, -40, 3000, {.forwards=false, .maxSpeed=75}, false);
    // pros::delay(200);
    // clamp_down = true;
    // clamp.set_value(true);
    // pros::delay(200);
    // intake.move_velocity(600);
    // hook.move_velocity(600);
    // pros::delay(1000);
    // chassis.turnToHeading(-90, 3000);
    // chassis.moveToPoint(-23, -38, 3000);
    // pros::delay(3000);
    // intake.move_velocity(0);
    // hook.move_velocity(0);
    // chassis.turnToHeading(90, 3000, {}, false);
    // chassis.moveToPoint(27, -38, 3000, {}, false);
    // lb.move_absolute(-2000, 400);

    //go back
    // pros::delay(10000);
    // chassis.moveToPoint(0, -20, 3000, {.forwards=false});

    //NEW red left
    // chassis.setPose(-4,0, -45);
    // lb.move_absolute(-410, 600);
    // pros::delay(200);
    // intake.move_velocity(600);
    // hook.move_velocity(300);
    // pros::delay(500);
    // hook.brake();
    // pros::delay(400);
    // lb.move_absolute(-3200, 120);
    // pros::delay(1800);
    // lb.move_absolute(0, 600);
    // pros::delay(500);
    // chassis.moveToPose(6, -38, 0, 2000, {.forwards=false, .horizontalDrift = 8, .lead = 0.3, .minSpeed=15, .earlyExitRange=2}, false);
    // pros::delay(300);
    // clamp.set_value(true);
    // clamp_down=true;
    // pros::delay(250);
    // hook.move_velocity(600);
    // chassis.moveToPose(30, -35, 97, 1500, {.earlyExitRange=2}, false);
    // pros::Task anti_jam_task(anti_jam);
    // chassis.moveToPose(36, -53.3, 180, 1500, {}, false);
    // chassis.moveToPose(36, -35, 180, 1500, {.forwards=false, .earlyExitRange=2}, false);
    // chassis.moveToPose(24, -54.5, 180, 1900, {}, false);
    // pros::delay(250);
    // chassis.moveToPose(24, -35, 180, 1500, {.forwards=false, .earlyExitRange=2}, false);
    // lb.move_absolute(-2000, 600);
    // chassis.moveToPose(-10, -38, -90, 1500, {.earlyExitRange=2}, false);

    //NEW blue right
    // chassis.setPose(4,0, 45);
    // lb.move_absolute(-410, 600);
    // pros::delay(200);
    // intake.move_velocity(600);
    // hook.move_velocity(300);
    // pros::delay(500);
    // hook.brake();
    // pros::delay(400);
    // lb.move_absolute(-3200, 120);
    // pros::delay(1800);
    // lb.move_absolute(0, 600);
    // pros::delay(500);
    // chassis.moveToPose(-6, -38, 0, 2000, {.forwards=false, .horizontalDrift = 8, .lead = 0.3, .minSpeed=15, .earlyExitRange=2}, false);
    // pros::delay(300);
    // clamp.set_value(true);
    // clamp_down=true;
    // pros::delay(250);
    // hook.move_velocity(600);
    // pros::delay(250);
    // pros::Task anti_jam_task(anti_jam);
    // chassis.moveToPose(-28, -38, -100, 1500, {.earlyExitRange=2}, false);
    // chassis.moveToPose(-36, -54, -180, 1500, {}, false);
    // chassis.moveToPose(-36, -35, -180, 1500, {.forwards=false, .earlyExitRange=2}, false);
    // chassis.moveToPose(-24, -54, -180, 1900, {}, false);
    // pros::delay(250);
    // chassis.moveToPose(-24, -35, -180, 1500, {.forwards=false, .earlyExitRange=2}, false);
    // lb.move_absolute(-2000, 600);
    // chassis.moveToPose(10, -38, 90,1500, {.earlyExitRange=2}, false);

    // SKILLS
    // chassis.moveToPoint(0, 0.8, 3000, {}, false);
    // hook.move_velocity(600);
    // intake.move_velocity(600);
    // pros::delay(250);
    // chassis.moveToPoint(-5, 16, 3000, {.earlyExitRange=2}, false);
    // hook.move_velocity(0);
    // chassis.moveToPoint(23, 15, 3000, {.forwards=false, .maxSpeed=65}, false);
    // pros::delay(250);
    // clamp.set_value(true);
    // pros::delay(250);
    // hook.move_velocity(600);
    // chassis.moveToPoint(20, 37, 3000, {.earlyExitRange=2}, false);
    // chassis.moveToPoint(52, 60, 3000, {}, false);
    // chassis.moveToPoint(42, 89, 3000, {});
    // lb.move_absolute(-410, 400);
    // pros::delay(1000);
    // chassis.moveToPose(62, 59, 90, 3000, {}, false);
    // pros::delay(500);
    // chassis.moveToPose(67, 59, 90, 1000, {});
    // hook.move_velocity(0);
    // lb.move_absolute(-2000, 400);
    // //chassis.moveToPose(59, 58, 90, 1000, {});
    // //pros::delay(1000);
    // pros::delay(300);
    // hook.move_velocity(600);
    // chassis.moveToPose(59, 59, 90, 3000, {.forwards=false}, false);
    // lb.move_absolute(0, 400);
    // chassis.moveToPoint(50, 59, 1000, {.forwards=false}, false);
    // chassis.moveToPose(53, 37, 180, 3000, {.maxSpeed=65}, false);
    // chassis.moveToPoint(50, -2, 3000, {.maxSpeed=65}, false);
    // pros::delay(500);
    // chassis.moveToPoint(50, 30, 3000, {.forwards=false}, false);
    // chassis.moveToPoint(60, 10, 3000, {}, false);
    // pros::delay(1200);
    // chassis.turnToHeading(-45, 3000);
    // //hook.move_velocity(0);
    // chassis.moveToPoint(62, 0, 800, {.forwards=false}, false);
    // clamp.set_value(false);
    // pros::delay(250);
    // hook.move_velocity(600);
    // chassis.moveToPoint(10, 13, 3000, {.earlyExitRange=2}, false);
    // hook.brake();
    // chassis.moveToPose(-36, 15, 90, 3000, {.forwards=false, .maxSpeed=65}, false);
    // pros::delay(250);
    // clamp.set_value(true);
    // pros::delay(250);
    // hook.move_velocity(600);
    // chassis.moveToPoint(-32, 37, 3000, {.earlyExitRange=2}, false);
    // chassis.moveToPoint(-62, 60, 3000, {}, false);
    // pros::delay(500);
    // chassis.moveToPoint(-55, 89, 3000, {});
    // pros::delay(1000);
    // lb.move_absolute(-410, 400);
    // chassis.moveToPose(-73, 69, -90, 3000, {}, false);
    // pros::delay(500);
    // chassis.moveToPose(-82, 69, -90, 1000, {});
    // hook.move_velocity(0);
    // //chassis.moveToPose(-65, 65, -90, 1000, {});
    // lb.move_absolute(-2000, 400);
    // //pros::delay(1000);
    // pros::delay(300);
    // hook.move_velocity(600);
    // chassis.moveToPose(-68, 69, -90, 1000, {.forwards=false}, false);
    // lb.move_absolute(0, 400);
    // chassis.moveToPoint(-51, 69, 1000, {.forwards=false}, false);
    // chassis.moveToPose(-50, 37, 180, 3000, {.maxSpeed=65}, false);
    // chassis.moveToPoint(-49, 5, 3000, {.maxSpeed=65}, false);
    // pros::delay(500);
    // chassis.moveToPoint(-54, 30, 3000, {.forwards=false}, false);
    // chassis.moveToPoint(-64, 15, 3000, {}, false);

    // pros::delay(1200);
    // chassis.turnToHeading(45, 3000);
    // chassis.moveToPoint(-80, -2, 800, {.forwards=false}, false);
    // clamp.set_value(false);
    // chassis.moveToPoint(-62, 60, 3000, {.earlyExitRange=2});
    // hook.move_velocity(0);
    // intake.move_velocity(600);
    // chassis.moveToPoint(-42, 90, 4000, {}, false);
    // // hook.move_velocity(200);
    // // while (true){
    // //     if (optical.get_hue() > 5 & optical.get_hue() < 45){
    // //         hook.move_velocity(0);
    // //         break;
    // //     }
    // //     pros::delay(10);
    // // }
    // // chassis.moveToPose(-18, 110, 210 , 4000, {.forwards=false}, false);
    // // pros::delay(250);
    // // clamp.set_value(true);
    // // pros::delay(250);
    // // chassis.moveToPoint(19, 89, 3000, {}, false);
    // // chassis.moveToPoint(62, 125, 3000, {.forwards=false}, false);
    // // clamp.set_value(false);
    // // pros::delay(250);
    // // chassis.moveToPoint(19, 110, 3000, {.earlyExitRange=2}, false);
    // // chassis.moveToPose(-18, 120, 90, 3000, {.earlyExitRange=2}, false);
    // // chassis.moveToPoint(-80, 135, 3000, {}, false);
    // chassis.moveToPose(-20, 110, 210 , 4000, {.forwards=false}, false);
    // pros::delay(250);
    // clamp.set_value(true);
    // pros::delay(250);
    // hook.move_velocity(600);
    // //pros::delay(750);
    // chassis.turnToHeading(60, 3000, {}, false);
    // clamp.set_value(false);
    // intake.brake();
    // hook.brake();
    // chassis.moveToPoint(88, 135, 2000, {.minSpeed=100}, false);
    // // chassis.moveToPoint(19, 110, 3000, {.earlyExitRange=2}, false);
    // //chassis.moveToPoint(60, 120, 1000, {.earlyExitRange=2}, false);
    // chassis.moveToPoint(50, 135, 2000, {.forwards=false, .minSpeed=140}, false);
    // chassis.moveToPoint(0, 128, 2000, {.forwards=false, .minSpeed=140}, false);
    // chassis.moveToPoint(-89, 135, 2000, {.forwards=false, .minSpeed=140}, false);
}   

void opcontrol() {
    // Start intake control in a separate task to manage intake behavior
    pros::Task intake_task(intake_control);
    pros::Task pneumatic_task(pneumatic_control);
    // pros::Task lb_pid_task_task(lb_pid_task);
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