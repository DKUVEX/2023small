#include "dku/opcontrol/rc_update_task.hpp"
#include "dku/chassis_task.hpp"
#include "dku/functional_task.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

rc_update_t controller_update;

/**
  * @brief          "controller_update" valiable initialization, include pid initialization, remote control data point initialization, chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "controller_update" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"controller_update"变量，包括pid初始化， 遥控器指针初始化，底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     init:"controller_update"变量指针.
  * @retval         none
  */
//TODO: not finish yet
static void rc_update_init(rc_update_t* init);

/**
  * @brief          "controller_update" valiable initialization, include pid initialization, remote control data point initialization, chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "controller_update" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"controller_update"变量，包括pid初始化， 遥控器指针初始化，底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     init:"controller_update"变量指针.
  * @retval         none
  */
//TODO: not finish yet
static void rc_update_init(rc_update_t* init)
{
    init->update_RC = get_remote_control_point();
    init->chassis_voltage = get_chassis_voltage_point();
    init->functional_status = get_functional_device_status();
}

/**
  * @brief          rc update task, osDelay RC_UPDATE_CONTROL_TIME_MS (2ms) 
  * @param[in]      param: null
  * @retval         none
  */
/**
  * @brief          底盘任务，间隔 RC_UPDATE_CONTROL_TIME_MS 2ms
  * @param[in]      param: 空
  * @retval         none
  */
void rc_update_task_fn(void* param) 
{
    std::cout << "rc update task runs" << std::endl;
    pros::Task::delay(RC_UPDATE_TASK_INIT_TIME);
    rc_update_init(&controller_update);
    std::uint32_t now = pros::millis();
    while (true) {
        // Do opcontrol things
        pros::Mutex chassis_mutex;
        chassis_mutex.take();
        {
            controller_update.chassis_voltage[0] = controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)
                                        +controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)
                                        +controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            controller_update.chassis_voltage[1] = controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)
                                        -controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)
                                        +controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            controller_update.chassis_voltage[2] = controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)
                                        -controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)
                                        -controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);        
            controller_update.chassis_voltage[3] = controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)
                                        +controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)
                                        -controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);        
        }
        chassis_mutex.give();
        pros::Mutex fucntional_mutex;
        fucntional_mutex.take();
        {
            //TODO:can be improved
            if ((controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_R1))
                && !controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_L2))
            {
                controller_update.functional_status->intake_motor = E_FUNCTIONAL_MOTOR_STATUS_FORWARD;
            }
            else if ((controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_R2))
                    && !controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_L2))
            {
                controller_update.functional_status->intake_motor = E_FUNCTIONAL_MOTOR_STATUS_BACKWARD;
            }
            else if (!(controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_R2))
                    && !(controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_R1)))
            {
                controller_update.functional_status->roller_motor = E_FUNCTIONAL_MOTOR_STATUS_OFF;
                controller_update.functional_status->intake_motor = E_FUNCTIONAL_MOTOR_STATUS_OFF;
            }
            if ((controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_Y))) {
                controller_update.functional_status->roller_motor = E_FUNCTIONAL_MOTOR_STATUS_FORWARD;
            }
            else if ((controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))) {
                controller_update.functional_status->roller_motor = E_FUNCTIONAL_MOTOR_STATUS_BACKWARD;
            }
            if (controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                controller_update.functional_status->index_motor = E_FUNCTIONAL_MOTOR_STATUS_BACKWARD;
            }
            else if (!controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                controller_update.functional_status->index_motor = E_FUNCTIONAL_MOTOR_STATUS_OFF;                
            }{
                controller_update.functional_status->index_motor = E_FUNCTIONAL_MOTOR_STATUS_OFF;
            }
            if(controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_B)) 
            {
                controller_update.functional_status->flywheel = E_FLYWHEEL_STATUS_OFF;
            }
            else if(controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_L2) 
                    && controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_R1)) 
            {
                controller_update.functional_status->flywheel = E_FLYWHEEL_STATUS_SPEED_HIGH;
            }
            else if(controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_L2) 
                    && controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_R2)) 
            {
                controller_update.functional_status->flywheel = E_FLYWHEEL_STATUS_SPEED_LOW;
            }
            if (controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_UP) ) {
                controller_update.functional_status->gas_gpio = FUNCTIONAL_LIFT_HIGH_STATE;
            }
            if (controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) ) {
                controller_update.functional_status->gas_gpio = FUNCTIONAL_LIFT_LOW_STATE;
            }
            if (controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_L1) ) {
                controller_update.functional_status->intake_motor = E_FUNCTIONAL_MOTOR_STATUS_BACKWARD;
                controller_update.functional_status->index_motor = E_FUNCTIONAL_MOTOR_STATUS_FORWARD;
            }

            if (controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_UP) ) {
                controller_update.functional_status->extension_motor = E_FUNCTIONAL_MOTOR_STATUS_BACKWARD;
            }
            else {
                controller_update.functional_status->extension_motor = E_FUNCTIONAL_MOTOR_STATUS_OFF;
            }
            if (controller_update.update_RC->get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) ) {
                controller_update.now_time = pros::millis();
                if ((controller_update.now_time - controller_update.op_start_time)>60*1000) {
                    controller_update.functional_status->extension_gpio = FUNCTIONAL_LIFT_LOW_STATE;
                }
            }
        }
        fucntional_mutex.give();
        if (pros::competition::is_connected() && controller_update.op_start_flag == false) {
            controller_update.op_start_time = pros::millis();
            controller_update.op_start_flag = true;
        }
        // printf("%d\n", chassis_motor_voltage[0] );
        pros::Task::delay_until(&now, CHASSIS_CONTROL_TIME_MS);
    }
}