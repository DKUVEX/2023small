#include "main.h"


// pros::Task auto_task (auto_task_fn, NULL, TASK_PRIORITY_DEFAULT,
//         TASK_STACK_DEPTH_DEFAULT, "auto_task");
// pros::Task rc_update_task (rc_update_task_fn, NULL, TASK_PRIORITY_MAX,
//         TASK_STACK_DEPTH_DEFAULT, "rc_update_task");

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");

    pros::lcd::register_btn1_cb(on_center_button);

    pros::Task sensor_task (sensor_task_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT,
                TASK_STACK_DEPTH_DEFAULT, "sensor_task");
    pros::Task chassis_task (chassis_task_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT,
                TASK_STACK_DEPTH_DEFAULT, "chassis_task");
    pros::Task functional_task (functional_task_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT,
                TASK_STACK_DEPTH_DEFAULT, "functional_task");
    // pros::Task auto_task (auto_task_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT,
    //             TASK_STACK_DEPTH_DEFAULT, "auto_task");
    // pros::Task rc_update_task (rc_update_task_fn, (void*)"PROS", TASK_PRIORITY_MAX,
    //             TASK_STACK_DEPTH_DEFAULT, "rc_update_task");
    // auto_task.suspend();
    // rc_update_task.suspend();
    //pros::Task tracking (tracking_1_fn,  (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "tracking");

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {

    std::cout << "auto task runs" << std::endl;

    pros::Task::delay(2000);
    auto_init(&auto_control);

    // turn_relative(-90, &auto_control);
    // turn_relative(90, &auto_control);

    // move_vertical_relative_speed(-1, &auto_control);

    // move_horizontal_relative(0.5, &auto_control);

//      while(true){
//         int now_time = pros::millis();
//         if ((init_time - now_time)>50*1000) {
// //auto_control.functional_status->extension_motor = E_FUNCTIONAL_MOTOR_STATUS_BACKWARD;

//         }
//         if((init_time - now_time)>50*1000+500){
//             auto_control.functional  _status->flywheel = E_FLYWHEEL_STATUS_OFF;
//             break;
//         }
//     }
    // move_time(1,1000,&auto_control);
    
    // // pros::Task::delay(10);
    // // rotate_roller(500, &auto_control);
    // pros::Task::delay(50);
    // // pros::Task::delay(5);
    // move_time(-1,150,&auto_control);
    // pros::Task::delay(50);
    // kick_out(&auto_control);
    // // turn_right_relative(80, &auto_control);
    // // pros::Task::delay(50);
    // // move_time(1,2000,&auto_control);
    // // // pros::Task::delay(5);
    // // // rotate_roller(500, &auto_control);
    // // // move_front_relative(0.8, &auto_control);
    // // pros::Task::delay(50);
    // // move_time(-1,700,&auto_control);
    // // pros::Task::delay(50);
    // // turn_right_relative(45, &auto_control);
    // // // rotate_roller(500, &auto_control);
    // // move_front_relative(-1, &auto_control);
    // // pros::Task::delay(50);
    // // turn_left_relative(90, &auto_control);
    
    // // pros::Task::delay(37*1000);
    // // pros::Mutex auto_mutex;
    // // auto_mutex.take();
    // // {
    // //     auto_control.functional_status->extension_gpio = FUNCTIONAL_LIFT_HIGH_STATE;
    // // }
    // // auto_mutex.give();
    // // pros::Task::delay(0.5*1000);
    // // auto_mutex.take();
    // // {
    // //     auto_control.functional_status->extension_gpio = FUNCTIONAL_LIFT_LOW_STATE;
    // // }
    // // auto_mutex.give();
    
    // //     pros::Task::delay(1000);
    // // move_back_relative(0.05,&auto_control);
    // //     pros::Task::delay(1000);
    // // turn_left_relative(120, &auto_control);
    // //     pros::Task::delay(1000);
    // // rotate_roller(180, &auto_control);
    // // turn_relative(-120, 3&auto_control);
    // // move_relative(-0.2, &auto_control);
    // // move_relative(-0.2, &auto_control);
    // // turn_relative(90, &auto_control);
    // // move_relative(0.5, &auto_control);
    // // turn_relative(90, &auto_control);

    
    // // kick_out(&auto_control);

    
    // // pros:: Task this_task = pros::Task::current();
    // // this_task.remove();

    // // if (rc_update_task.get_state() != pros::E_TASK_STATE_SUSPENDED)
    // // {
    // //     rc_update_task.suspend();
    // // }


    // // pros::Task auto_task (auto_task_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT,
    // //         TASK_STACK_DEPTH_DEFAULT, "auto_task");
    // // auto_task.resume();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_mtr(1);
	pros::Motor right_mtr(2);

    pros::Task rc_update_task (rc_update_task_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT,
                TASK_STACK_DEPTH_DEFAULT, "rc_update_task");
    // if (auto_task.get_state() != pros::E_TASK_STATE_SUSPENDED)
    // {
    //     auto_task.suspend();
    // }
    // rc_update_task.resume();
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		left_mtr = left;
		right_mtr = right;

		pros::delay(20);
	}
}
