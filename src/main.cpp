#include "main.h"
#include "lemlib/api.hpp"
#include "constants.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pose.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

// COLOR SORT VALUES (200-240 Blue, 5-45 Red)

int RED_RING_COLOR = -10;
int BLUE_RING_COLOR = 190;

int color_to_sort = BLUE_RING_COLOR;

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

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, 2, 0.425);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, 2, -0.0866);


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

lemlib::ControllerSettings angular_controller(1.9, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              13.5, // derivative gain (kD)
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
            //pros::delay(12);
            intake.move_velocity(0);
            hook.move_velocity(0);
            pros::delay(100);
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

//allianceStake macro

void alliance_stake() {
    while (true) {
        //if "x" is pressed start alliance stake macro
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
            chassis.setPose(0,0,0);
            chassis.moveToPose(0,-6,0,2000, {.forwards = false});
            lb.move_absolute(LB_ALLIANCE_STAKE, 400);
            pros::delay(1000);
            chassis.moveToPose(0,-25,0,2000, {.forwards = false});
            pros::delay(500);
            lb.move_absolute(LB_DOWN, 400);
        }
        pros::delay(20);
    }
}

void initialize() {
	pros::lcd::initialize(); // initialize brain screen
	chassis.calibrate(); // calibrate sensors
    lb.tare_position(); // calibrate lb motor
    optical.set_integration_time(10); // calibrate optical
    optical.set_led_pwm(50); // turn on optical LED
    lb.set_brake_mode(pros::MotorBrake::hold); // set LB motor to hold
	 pros::Task screen_task([&]() {
        while (true) {
            // print robot info to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Color Hue: %f", optical.get_hue()); // hue
            pros::lcd::print(4, "Ring Proximity: %i", optical.get_proximity()); // hue
            pros::lcd::print(5, "LB Position %f", lb.get_position()); // lb position
            pros::lcd::print(6, "Vertical Sensor: %i", vertical_encoder.get_position()); // vertical sensor
            pros::lcd::print(7, "Horizontal Sensor: %i", horizontal_encoder.get_position()); // horizontal sensor

            // delay to save resources
            pros::delay(20);
        }
    });
}

// color sort specifically for autons
bool stop_after_sorting = false;
void auton_color_sort() {
    while (true){
        if ((optical.get_hue() > LOWER_COLOR_RANGE && optical.get_hue() < UPPER_COLOR_RANGE)){
            if (stop_after_sorting){
                //pros::delay(100);
            }
            hook.move_velocity(0);
            pros::delay(100);
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
    pros::delay(500);
    chassis.setPose(-64, -6, 315);
    //score on alliance stake (OLD)
    // lb.move_absolute(LB_UP, 600);
    // pros::delay(1000);
    // chassis.moveToPose(-51, -20, 315, 1000, {.forwards=false,.horizontalDrift = 8, .lead = 0.3, .maxSpeed=65, .minSpeed=15}, false);
    // lb.move_absolute(-LB_MID-15, 600);

    //score on alliance stake (NEW)
    lb.move_absolute(LB_MID, 600);
    pros::delay(200);
    hook.move_velocity(600);
    pros::delay(700);
    hook.move_velocity(0);
    lb.move_absolute(LB_ALLIANCE_STAKE, 600);
    pros::delay(1000);
    chassis.moveToPose(-51, -20, 315, 1000, {.forwards=false,.horizontalDrift = 8, .lead = 0.3, .maxSpeed=65, .minSpeed=15}, false);
    lb.move_absolute(LB_DOWN, 600);

    // clamp goal
    pros::delay(250);
    clamp.set_value(true);
    pros::delay(250);
    // get rings and go to wall stake
    hook.move_velocity(600);
    intake.move_velocity(600);
    chassis.moveToPose(-27.5, -13.5, 90, 2000, {.horizontalDrift = 8, .lead = 0.3, .earlyExitRange=6}, false); // ring 1
    lb.tare_position();
    lb.move_absolute(LB_MID, 400);
    chassis.moveToPose(-8.5, -42, 180, 2000, {.horizontalDrift = 8, .lead = 0.3, .earlyExitRange = 6}, false); 
    chassis.moveToPoint(-8.5, -52, 1500, {}, true); // ring 2
    pros::delay(800);
    // score on wall stake
    // chassis.setPose(chassis.getPose().x, chassis.getPose().y, 180);
    hook.move_velocity(0);
    intake.brake();
    lb.move_absolute(LB_UP, 400);
    pros::delay(1000);
    lb.move_absolute(LB_DOWN, 400);
    chassis.moveToPose(-8.5, -38, 180, 1000, {.forwards=false, .horizontalDrift = 8, .lead = 0.3}, false);
    // //grab rest of rings + corner
    hook.move_velocity(600);
    intake.move_velocity(600);
    chassis.turnToHeading(270, 600, {}, false);
    chassis.moveToPose(-70, -39, 270, 3000, {.horizontalDrift = 8, .lead = 0.1, .maxSpeed=70}, false);
    chassis.setPose(chassis.getPose().x, chassis.getPose().y, 270);
    chassis.moveToPose(-65, -39, 270, 3000, {.forwards=false, .horizontalDrift = 8, .lead = 0.3,}, false);
    pros::delay(300);
    chassis.turnToHeading(125, 600, {}, false);
    chassis.moveToPose(-51, -54, 125, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    chassis.moveToPose(-70, -59, 45, 1000, {.forwards=false,.horizontalDrift = 8, .lead = 0.3}, false);
    hook.move_velocity(0);
    clamp.set_value(false);
    
    //grab next goal
    chassis.moveToPose(-62, -24, 0, 3000, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed=75, .earlyExitRange=8}, false);
    chassis.moveToPose(-60, 22, 180, 3000, {.forwards=false, .horizontalDrift = 8, .lead = 0.3, .maxSpeed=65, .minSpeed=10}, false);
    pros::delay(150);
    clamp.set_value(true);
    pros::delay(250);
    // get rings and go to wall stake
    hook.move_velocity(600);
    intake.move_velocity(600);
    chassis.moveToPose(-39, 24, 90, 3000, {.horizontalDrift = 8, .lead = 0.3, .earlyExitRange=6}, false); // ring 1
    chassis.moveToPose(-18, 50, 0, 2000, {.horizontalDrift = 8, .lead = 0.3}, false); 
    lb.move_absolute(LB_MID, 400);
    chassis.moveToPoint(-18, 70, 1700, {}, true); // ring 2
    pros::delay(800);
    // score on wall stake
    hook.move_velocity(0);
    intake.brake();
    lb.move_absolute(LB_UP, 400);
    pros::delay(1000);
    lb.move_absolute(LB_DOWN, 400);
    chassis.moveToPose(-18, 48, 0, 1000, {.forwards=false, .horizontalDrift = 8, .lead = 0.3}, false);
    //grab rest of rings + corner
    hook.move_velocity(600);
    intake.move_velocity(600);
    chassis.turnToHeading(270, 600, {}, false);
    chassis.moveToPose(-92, 48, 270, 2000, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed=70}, false);
    chassis.setPose(chassis.getPose().x, chassis.getPose().y, 270);
    //chassis.moveToPose(-87, 46, 270, 3000, {.forwards=false, .horizontalDrift = 8, .lead = 0.3,}, false);
    pros::delay(300);
    chassis.turnToHeading(45, 600, {}, false);
    chassis.moveToPose(-64, 63, 45, 1000, {.horizontalDrift = 8, .lead = 0.3}, false); //last ring
    chassis.moveToPose(-87, 66, 125, 1000, {.forwards=false,.horizontalDrift = 8, .lead = 0.3}, false);//corner drop
    pros::delay(100);
    hook.brake();
    clamp.set_value(false);

    //go to far goal with ring in LB
    chassis.moveToPose(1.5, 53.5, 90, 3000, {.horizontalDrift = 8, .lead = 0.3});
    chassis.waitUntil(10);
    //lb.move_absolute(LB_MID, 400);
    hook.move_velocity(600);
    intake.move_velocity(600);
    chassis.waitUntilDone();
    chassis.moveToPose(32, 0, 330, 3000, {.forwards = false, .horizontalDrift = 8, .lead = 0.3, .maxSpeed=65, .minSpeed=15}, false);
    pros::delay(250);
    clamp.set_value(true);
    pros::delay(250);

    //alliance stake
    //chassis.moveToPose(35.5, 9.25, 90, 3000, {.horizontalDrift = 8, .lead = 0.1}, false);      //move into stake
    // 
    //chassis.moveToPose(29.5, 11.5, 90, 3000, {.forwards=true, .horizontalDrift = 8, .lead = 0.3}, false); //move away
    //hook.brake();
    //intake.brake();
    //lb.move_absolute(LB_ALLIANCE_STAKE, 400);
    //pros::delay(1000);
    chassis.moveToPose(23, 11.5,180, 3000, {.forwards=false, .horizontalDrift = 8, .lead = 0.3}, false);// back out
    //lb.move_absolute(LB_DOWN, 400);
    //hook.move_velocity(600);
    //intake.move_velocity(600);
    //grab rings
    chassis.moveToPose(26, -38, 180, 3000, {.horizontalDrift = 8, .lead = 0.3}, false); // ring 1
    chassis.moveToPose(4, -42, 270, 3000, {.horizontalDrift = 8, .lead = 0.3}, false); // ring 2
    chassis.moveToPose(7, -21, 0, 3000, {.horizontalDrift = 8, .lead = 0.3}); // ring 3
    pros::delay(2500);
    hook.move_velocity(0);
    chassis.moveToPose(-20, 5, 315, 3000, {.horizontalDrift = 8, .lead = 0.3}, false); // ring 4 (ladder)
    chassis.moveToPose(4, 28, 45, 3000, {.horizontalDrift = 8, .lead = 0.3});   //ring 5
    pros::delay(1000);
    hook.move_velocity(600);
    // chassis.moveToPoint(4, 47, 3000, {.earlyExitRange=6}, false);
    chassis.moveToPose(22.5, 55, 0, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);   //ring 6
    chassis.moveToPose(22, 75, 0, 1000, {.horizontalDrift = 8, .lead = 0.3}, false);
    intake.move_velocity(-600);     //to plow rings

    
    //put full goal in  
    chassis.moveToPose(34, 75, 270, 1500, {.forwards = false, .horizontalDrift = 8, .lead = 0.3}, false);
    clamp.set_value(false);
    //put last goal in corner
    chassis.moveToPoint(20, 27, 3000, {.earlyExitRange=8}, false);
    chassis.moveToPoint(65, -32, 3000, {.earlyExitRange=10}, false);
    //chassis.moveToPose(56.5, -34.5, 170, 3000, { .horizontalDrift = 8, .lead = 0.3}, false);
    chassis.moveToPoint(65, -70, 3000);
    //hang
    //lb.move_absolute(LB_UP, 400);
    //chassis.moveToPose(15, -15, 315, 3000, {.horizontalDrift = 8, .lead = 0.3}, false);

    // SWP BLUE
    // pros::Task auton_color_sort_task(auton_color_sort);
    // chassis.setPose(54, 15, 135);
    // lb.move_absolute(LB_ALLIANCE_STAKE, 600);
    // pros::delay(1000);
    // chassis.moveToPoint(49, 20, 1000, {.forwards=false, .earlyExitRange=8}, false);
    // lb.move_absolute(-LB_MID, 600);
    // chassis.moveToPose(22, 24.5, 120, 1500, {.forwards = false, .horizontalDrift = 8, .lead = 0.3, .maxSpeed=60, .minSpeed=25,}, false);
    // pros::delay(200);
    // clamp.set_value(true);
    // pros::delay(200);
    // lb.tare_position();
    // intake.move_velocity(600);
    // hook.move_velocity(600);
    // chassis.turnToHeading(335, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    // chassis.moveToPose(3, 39, 335, 1500, {.horizontalDrift = 8, .lead = 0.3,}, false);
    // chassis.moveToPose(2, 55, 0, 2000, {.horizontalDrift = 8, .lead = 0.3,.maxSpeed=40}, false);
    // pros::delay(300);
    // chassis.moveToPose(18, 53, 90, 1500, {.horizontalDrift = 8, .lead = 0.3,}, false);
    // hook.move_velocity(300);
    // chassis.moveToPose(42, 13, 180, 1500, {.horizontalDrift = 8, .lead = 0.3,.earlyExitRange=4}, false);
    // clamp.set_value(false);
    // stop_after_sorting = true;
    // intake.move_velocity(600);
    // chassis.moveToPose(42, -15, 180, 1500, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed=33}, false);
    // stop_after_sorting=false;
    // chassis.moveToPose(20, -24.5, 60, 1500, {.forwards = false, .horizontalDrift = 8, .lead = 0.3, .maxSpeed=60, .minSpeed=25,}, false);
    // pros::delay(250);
    // clamp.set_value(true);
    // pros::delay(500);
    // hook.move_velocity(600);
    // lb.move_absolute(LB_UP-500, 400);
    // chassis.swingToHeading(315, lemlib::DriveSide::LEFT, 800, {}, false);


    
    // OLDring rush BLUE
    // pros::Task auton_color_sort_task(auton_color_sort);
    // chassis.setPose(52, 39, 270);
    // chassis.moveToPoint(29, 39, 3000, {.minSpeed=40}, false);
    // intake.move_velocity(600);
    // chassis.moveToPose(7, 39, 277, 3000, {}, false);
    // //chassis.moveToPose(7, 39 , 294, 600, {.maxSpeed=40}, false);
    // //chassis.turnToHeading(287, 1000);
    // doinker.set_value(true);
    // pros::delay(500);
    // chassis.moveToPoint(20, 23, 3000, {.forwards=false, .maxSpeed=60}, false);
    // doinker.set_value(false);
    // pros::delay(200);
    // clamp.set_value(true);
    // pros::delay(200);
    // hook.move_velocity(600);
    // chassis.moveToPose(20, 55 , 0, 3000, {.minSpeed=30}, false);
    // chassis.moveToPose(7, 50, 270, 3000, {.maxSpeed=40}, false);
    // chassis.moveToPose(5, 5, 45, 3000, {.maxSpeed=40}, false);

    //BLUE RING RUSH
//     pros::Task auton_color_sort_task(auton_color_sort);
//     chassis.setPose(-11,4.5,208);
//     // chassis.moveToPoint(-14.5,0.5,3000);
//     // lb.move_absolute(LB_ALLIANCE_STAKE,400);
//     // pros::delay(1000);
//     chassis.moveToPoint(0,30,3000, {.forwards = false, .maxSpeed = 60});     //goal 
//    // lb.move_absolute(-LB_MID,400);
//     pros::delay(1250);
//     clamp.set_value(true);
//     intake.move_velocity(600);
//     lb.tare_position();
//     hook.move_velocity(600);
//     chassis.moveToPose(12,46,70,3000, {.horizontalDrift = 8, .lead = 0.3, .earlyExitRange = 4});    //ring 1
//     chassis.moveToPose(24,47,90,3000,{.horizontalDrift = 8, .lead = 0.3, .earlyExitRange = 4});      //ring 2
//     chassis.moveToPose(24,34,180,3000, {.horizontalDrift = 8, .lead = 0.3});       //ring 3
//     //chassis.moveToPoint(20,0,3000, {.forwards = true, .earlyExitRange = 5});//push out top ring
//     chassis.moveToPose(43,-10,135,3000, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed = 30}); //intial corner
//     chassis.moveToPose(45,-20,135, 300, {.minSpeed = 130});     //ram
//     chassis.moveToPoint(30,4,3000, {.forwards = false});
//     chassis.moveToPoint(34,-4,3000);
//     chassis.moveToPoint(22,8,300, {.forwards = false});

    


    //RED RING RUSH
    // pros::Task auton_color_sort_task(auton_color_sort);
    // chassis.setPose(11,4.5,152);
    // chassis.moveToPoint(14.5,0.5,3000);
    // lb.move_absolute(LB_ALLIANCE_STAKE,400);
    // pros::delay(1000);
    // chassis.moveToPoint(0,30,3000, {.forwards = false, .maxSpeed = 60});     //goal 
    // lb.move_absolute(-LB_MID,400);
    // pros::delay(1250);
    // clamp.set_value(true);
    // intake.move_velocity(600);
    // hook.move_velocity(600);
    // chassis.moveToPose(-12,41,290,3000, {.horizontalDrift = 8, .lead = 0.3, .earlyExitRange = 4});    //ring 1
    // chassis.moveToPose(-24,42,270,3000,{.horizontalDrift = 8, .lead = 0.3, .earlyExitRange = 4});      //ring 2
    // chassis.moveToPose(-24,30,180,3000, {.horizontalDrift = 8, .lead = 0.3});       //ring 3
    // //chassis.moveToPoint(-20,0,3000, {.forwards = true, .earlyExitRange = 5});//push out top ring
    // chassis.moveToPose(-43,-10,225,3000, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed = 30}); //intial corner
    // chassis.moveToPose(-45,-20,225, 300, {.minSpeed = 130});     //ram
    // chassis.moveToPoint(-30,4,3000, {.forwards = false});
    // chassis.moveToPoint(-34,-4,3000);
    // chassis.moveToPoint(-22,8,300, {.forwards = false});

    //BLUE GOAL SIDE
    // pros::Task auton_color_sort_task(auton_color_sort);
    // chassis.setPose(-24,24,180);
    // chassis.moveToPoint(-24,44,3000, {.forwards = false, .maxSpeed = 60});  //mogo
    // pros::delay(1000);
    // clamp.set_value(true);
    // pros::delay(1000);
    // intake.move_velocity(600);
    // hook.move_velocity(600);
    // chassis.moveToPose(-48,48,270,3000,{.horizontalDrift = 8, .lead = 0.3});    //ring 1
    // pros::delay(750);
    // intake.move_velocity(-600); //reverse intake to plow ring
    // chassis.moveToPose(-48,18,180,3000, {.horizontalDrift = 8, .lead = 0.3});   
    // chassis.moveToPoint(-48,24,3000, {.forwards = false});  //reverse to optimal angle + renable intake
    // intake.move_velocity(600);
    // chassis.moveToPose(-63,7,225,3000,{.horizontalDrift = 8, .lead = 0.3, .maxSpeed = 30});    //slow approach
    // chassis.moveToPoint(-66,4,1500, {.minSpeed = 130});    //ram to go all the way in
    // chassis.moveToPoint(-54,16,1000, {.forwards = false});  //reverse
    // chassis.moveToPoint(-58,12,1000);
    // pros::delay(2000);
    // lb.move_absolute(LB_UP+600, 400);
    // chassis.moveToPose(-16,56,45, 3000, {.horizontalDrift = 8, .lead = 0.3});

    //tuned red goal SIDE
    // pros::Task auton_color_sort_task(auton_color_sort);
    // chassis.setPose(24,24,180);
    // chassis.moveToPoint(24,44,3000, {.forwards = false, .maxSpeed = 60});  //mogo
    // pros::delay(1000);
    // clamp.set_value(true);
    // pros::delay(1000);
    // intake.move_velocity(600);
    // hook.move_velocity(600);
    // chassis.moveToPose(48,45,90,3000,{.horizontalDrift = 8, .lead = 0.3});    //ring 1
    // pros::delay(1000);
    // intake.move_velocity(-600); //reverse intake to plow ring
    // chassis.moveToPose(48,18,180,3000, {.horizontalDrift = 8, .lead = 0.3});   
    // chassis.moveToPoint(48,24,3000, {.forwards = false});  //reverse to optimal angle + renable intake
    // intake.move_velocity(600);
    // chassis.moveToPose(63,5,135,3000,{.horizontalDrift = 8, .lead = 0.3, .maxSpeed = 30});    //slow approach
    // chassis.moveToPoint(66,2,1500, {.minSpeed = 130});    //ram to go all the way in
    // //chassis.moveToPoint(54,16,1000, {.forwards = false});  //reverse
    // // chassis.moveToPoint(58,12,1000);
    // pros::delay(2000);
    // lb.move_absolute(LB_UP, 400);
    // chassis.moveToPose(16,56,315, 3000, {.horizontalDrift = 8, .lead = 0.3});

    // //RED GOAL SIDE
    // pros::Task auton_color_sort_task(auton_color_sort);
    // chassis.setPose(24,24,180);
    // chassis.moveToPoint(24,44,3000, {.forwards = false, .maxSpeed = 60});  //mogo
    // pros::delay(500);
    // clamp.set_value(true);
    // intake.move_velocity(600);
    // hook.move_velocity(600);
    // chassis.moveToPose(48,48,270,3000,{.horizontalDrift = 8, .lead = 0.3});    //ring 1
    // pros::delay(750);
    // intake.move_velocity(-600); //reverse intake to plow ring
    // chassis.moveToPose(48,18,180,3000, {.horizontalDrift = 8, .lead = 0.3});   
    // chassis.moveToPoint(48,24,3000, {.forwards = false});  //reverse to optimal angle + renable intake
    // intake.move_velocity(600);
    // chassis.moveToPose(60,12,135,3000,{.horizontalDrift = 8, .lead = 0.3, .maxSpeed = 30});    //slow approach
    // chassis.moveToPoint(64,8,3000, {.minSpeed = 130});    //ram to go all the way in
    // chassis.moveToPoint(54,18,3000, {.forwards = false});  //reverse
    // chassis.moveToPoint(60,12,3000);
    // pros::delay(2000);
    // lb.move_absolute(LB_UP, 400);
    // chassis.moveToPose(16,56,315, 3000, {.horizontalDrift = 8, .lead = 0.3});

    //SWP RED
    // pros::Task auton_color_sort_task(auton_color_sort);
    // chassis.setPose(-57, 23, 225);
    // lb.move_absolute(LB_ALLIANCE_STAKE, 600);
    // pros::delay(1000);
    // chassis.moveToPoint(-49, 20, 1000, {.forwards=false, .earlyExitRange=8}, false);
    // lb.move_absolute(-LB_MID, 600);
    // chassis.moveToPose(-22, 24.5, 240, 1500, {.forwards = false, .horizontalDrift = 8, .lead = 0.3, .maxSpeed=60, .minSpeed=15,}, false);
    // pros::delay(250);
    // clamp.set_value(true);
    // pros::delay(200);
    // lb.tare_position();
    // intake.move_velocity(600);
    // hook.move_velocity(600);
    // // double check CW_CLOCKWISE
    // chassis.turnToHeading(25, 800, {.direction=lemlib::AngularDirection::CW_CLOCKWISE});
    // chassis.moveToPose(-4, 37, 25, 1500, {.horizontalDrift = 8, .lead = 0.3,}, false);
    // chassis.moveToPose(-3, 55, 0, 2000, {.horizontalDrift = 8, .lead = 0.3,.maxSpeed=40}, false);
    // pros::delay(300);
    // chassis.moveToPose(-24, 48, 270, 1500, {.horizontalDrift = 8, .lead = 0.3,}, false);
    // pros::delay(300);
    // hook.move_velocity(300);
    // chassis.moveToPose(-42, 13, 180, 1500, {.horizontalDrift = 8, .lead = 0.3,.earlyExitRange=4}, false);
    // clamp.set_value(false);
    // stop_after_sorting = true;
    // chassis.moveToPose(-42, -15, 180, 1500, {.horizontalDrift = 8, .lead = 0.3, .maxSpeed=33}, false);
    // stop_after_sorting=false;
    // chassis.moveToPose(-20, -24.5, 300, 1500, {.forwards = false, .horizontalDrift = 8, .lead = 0.3, .maxSpeed=60, .minSpeed=15,}, false);
    // pros::delay(250);
    // clamp.set_value(true);
    // pros::delay(500);
    // hook.move_velocity(600);
    // lb.move_absolute(LB_UP-500, 400);
    // // double check RIGHT
    // chassis.swingToHeading(45, lemlib::DriveSide::RIGHT, 800, {}, false);
}
void opcontrol() {
    // // ONLY UNCOMMENT FOR SKILLS
    // chassis.setPose(-64, -6, 315);
    // //score on alliance stake
    // lb.move_absolute(LB_ALLIANCE_STAKE, 600);
    // pros::delay(1000);
    // lb.move_absolute(-LB_MID-15, 600);
    // pros::delay(300);
    // chassis.moveToPose(-51, -20, 315, 1000, {.forwards=false,.horizontalDrift = 8, .lead = 0.3, .maxSpeed=65, .minSpeed=15}, false);
    // // clamp goal
    // pros::delay(250);
    // clamp.set_value(true);
    // pros::delay(250);
    // lb.tare_position();


    pros::Task intake_task(intake_control);
    pros::Task pneumatic_task(pneumatic_control);
    pros::Task lb_task(lb_control);
    pros::Task alliance_task(alliance_stake);
    
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