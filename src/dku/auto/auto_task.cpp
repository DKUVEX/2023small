/**
 * ****************************(C) COPYRIGHT 2023 Blue Bear****************************
 * @file       auto_task.cpp
 * @brief      
 * 
 * @note       
 * @history:
 *   Version   Date            Author          Modification    Email
 *   V1.0.0    Mar-03-2023     Tianyi Zhang    1. start        tz137@duke.edu/shadow_rogue@qq.com
 * 
 * @verbatim
 * ==============================================================================
 * 
 * ==============================================================================
 * @endverbatim
 * ****************************(C) COPYRIGHT 2023 Blue Bear****************************
 */
#include "dku/auto/auto_task.hpp"
#include "dku/functional_task.hpp"
#include "pros/rtos.hpp"
#include <cstdint>


auto_control_t auto_control;

/**
 * @brief           auto control init
 * @param[in,out]   init:
 * @retval          null
 */
void auto_init(auto_control_t* init);


/**
 * @brief           let robot turn to specific point
 * @param[in]       target_x: aimed position x
 * @param[in]       target_y: aimed position y
 * @param[in,out]   turn: find current position,give chassis voltage
 * @retval          null
 */
void turn_to(double target_x, double target_y, auto_control_t* turn);

/**
 * @brief           let robot turn to specific point
 * @param[in]       target_angle: aimed position x
 * @param[in,out]   turn: find current position,give chassis voltage
 * @retval          null
 */
void turn_relative(double target_angle, auto_control_t* turn);

/**
 * @brief           let robot turn to specific point
 * @param[in]       direction: direction, -1 or +1
 * @param[in]       time: a period of time
 * @param[in,out]   turn: find current position,give chassis voltage
 * @retval          null
 */
void turn_time(double direction, double time, auto_control_t* turn);

/**
 * @brief           let robot move to specific point
 * @param[in]       target_x: aimed position x
 * @param[in]       target_y: aimed position y
 * @param[in,out]   move: find current position,give chassis voltage
 * @retval          null
 */
void move_to(double target_x, double target_y, auto_control_t* move);

/**
 * @brief           let robot turn to specific point
 * @param[in]       direction: direction, -1 or +1
 * @param[in]       time: a period of time
 * @param[in,out]   turn: find current position,give chassis voltage
 * @retval          null
 */
void move_time(double direction, double time, auto_control_t* move);

/**
 * @brief           kick out 3 plates
 * @param[in,out]   kick: change the voltage of index
 * @retval          null
 */
void kick_out(auto_control_t* kick);


/**
 * @brief           auto control init
 * @param[in,out]   init:
 * @retval          null
 */
void auto_init(auto_control_t* init)
{
    init->chassis_voltage = get_chassis_voltage_point();
    init->functional_status = get_functional_device_status();
    pros::Mutex init_mutex;
    init_mutex.take();
    {
        init->functional_status->flywheel = E_FLYWHEEL_STATUS_SPEED_HIGH;
        init->functional_status->intake_motor = E_FUNCTIONAL_MOTOR_STATUS_FORWARD;
        init->functional_status->roller_motor = E_FUNCTIONAL_MOTOR_STATUS_BACKWARD;
    }
    init_mutex.give();
    // init->current_pos.current_x = 
    // init->current_pos.current_y = 
    // init->current_pos.current_dir = 
}

/**
 * @brief           let robot turn to specific point
 * @param[in]       target_x: aimed position x
 * @param[in]       target_y: aimed position y
 * @param[in,out]   turn: find current position,give chassis voltage
 * @retval         
 */
void turn_to(double target_x, double target_y, auto_control_t* turn)
{
    std::int32_t analog_right_x = 0; //simulate the joystick
    double rad = atan2f((target_x - turn->current_pos.current_x), (target_y - turn->current_pos.current_y));
    double angle = rad*(180/PI);
    while (abs((int)(turn->current_pos.current_dir - angle))>=1) {
        analog_right_x = 127;
        pros::Mutex turn_mutex;
        turn_mutex.take();
        {
            turn->chassis_voltage[0] = analog_right_x;
            turn->chassis_voltage[1] = analog_right_x;
            turn->chassis_voltage[2] = -analog_right_x;
            turn->chassis_voltage[3] = -analog_right_x;
        }
        turn_mutex.give();
    }
}

/**
 * @brief           let robot turn to specific point
 * @param[in]       target_angle: aimed position x
 * @param[in,out]   turn: find current position,give chassis voltage
 * @retval          null
 */
void turn_relative(double target_angle, auto_control_t* turn)
{

}

/**
 * @brief           let robot turn to specific point
 * @param[in]       direction: direction, -1 or +1
 * @param[in]       time: a period of time, unit:ms
 * @param[in,out]   turn: find current position,give chassis voltage
 * @retval          null
 */
void turn_time(double direction, double time, auto_control_t* turn)
{
    std::int32_t analog_right_x = 127;
    pros::Mutex turn_mutex;
    turn_mutex.take();
    {
        turn->chassis_voltage[0] = analog_right_x*direction;
        turn->chassis_voltage[1] = analog_right_x*direction;
        turn->chassis_voltage[2] = -analog_right_x*direction;
        turn->chassis_voltage[3] = -analog_right_x*direction;
    }
    turn_mutex.give();
    pros::delay(time);
    analog_right_x = 0;
    turn_mutex.take();
    {
        turn->chassis_voltage[0] = analog_right_x*direction;
        turn->chassis_voltage[1] = analog_right_x*direction;
        turn->chassis_voltage[2] = -analog_right_x*direction;
        turn->chassis_voltage[3] = -analog_right_x*direction;
    }
    turn_mutex.give();
}
/**
 * @brief           let robot move to specific point
 * @param[in]       target_x: aimed position x
 * @param[in]       target_y: aimed position y
 * @param[in,out]   move: find current position,give chassis voltage
 * @retval         
 */
void move_to(double target_x, double target_y, auto_control_t* move)
{
    turn_to(target_x, target_y, move);
    std::int32_t analog_left_y = 0; //simulate the joystick
    double distance = sqrt(pow((target_x-move->current_pos.current_x),2) + pow((target_y-move->current_pos.current_y),2));
    while (abs((int)distance)>=0.05) {
        int analog_left_y = 127;
        pros::Mutex move_mutex;
        move_mutex.take();
        {
            move->chassis_voltage[0] = analog_left_y;
            move->chassis_voltage[1] = analog_left_y;
            move->chassis_voltage[2] = analog_left_y;
            move->chassis_voltage[3] = analog_left_y;
        }
        move_mutex.give();
    }
}

/**
 * @brief           let robot turn to specific point
 * @param[in]       direction: direction, -1 or +1
 * @param[in]       time: a period of time
 * @param[in,out]   turn: find current position,give chassis voltage
 * @retval          null
 */
void move_time(double direction, double time, auto_control_t* move)
{
    std::int32_t analog_left_y = 127;
    pros::Mutex turn_mutex;
    turn_mutex.take();
    {
        move->chassis_voltage[0] = analog_left_y*direction;
        move->chassis_voltage[1] = analog_left_y*direction;
        move->chassis_voltage[2] = analog_left_y*direction;
        move->chassis_voltage[3] = analog_left_y*direction;
    }
    turn_mutex.give();
    pros::delay(time);
    analog_left_y = 0;
    turn_mutex.take();
    {
        move->chassis_voltage[0] = analog_left_y*direction;
        move->chassis_voltage[1] = analog_left_y*direction;
        move->chassis_voltage[2] = analog_left_y*direction;
        move->chassis_voltage[3] = analog_left_y*direction;
    }
    turn_mutex.give();
}

/**
  * @brief          auto task. auto task is not a while true loop
  * @param[in]      param: null
  * @retval         none
  */
/**
  * @brief          自动任务
  * @param[in]      param: 空
  * @retval         none
  */
void auto_task_fn(void* param)
{
    std::cout << "auto task runs" << std::endl;
    // pros::Task::delay(AUTO_TASK_INIT_TIME);
    auto_init(&auto_control);
    // std::uint32_t now = pros::millis();
    // while (true) {
    //     pros::Task::delay_until(&now, AUTO_TASK_TIME_MS);
    // }
    move_time(FORWARD, 500, &auto_control);
    move_time(BACKWARD, 500, &auto_control);
}
/**
 * @brief           kick out 3 plates
 * @param[in,out]   kick: change the voltage of index
 * @retval          null
 */
void kick_out(auto_control_t* kick)
{
    pros::Mutex kick_mutex;
    kick_mutex.take();
    {
        kick->functional_status->index_motor = E_FUNCTIONAL_MOTOR_STATUS_FORWARD;
    }
    kick_mutex.give();
    pros::delay(5000);
    kick_mutex.take();
    {
        kick->functional_status->index_motor = E_FUNCTIONAL_MOTOR_STATUS_OFF;
    }
    kick_mutex.give();
}

/**
 * @brief            get the current status struck(x,y,direction)
 * @param[out]       null
 * @return           current_status_t*
 * @retval           
 */
current_status_t* get_current_status_pointer(void)
{
    return &auto_control.current_pos;
}