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
#include "dku/sensor_task.hpp"
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
 * @brief           let robot turn to specific point
 * @param[in]       direction: direction, -1 left or +1 right
 * @param[in]       time: a period of time
 * @param[in,out]   turn: find current position,give chassis voltage
 * @retval          null
 */
void horizontal_time(double direction, double time, auto_control_t* move);

/**
 * @brief           kick out 3 plates
 * @param[in,out]   kick: change the voltage of index
 * @retval          null
 */
void kick_out(auto_control_t* kick);



/**
 * @brief           rotate the roller
 * @param[in]       time: the rotate time, unit: ms
 * @param[in,out]   rotate: the control pointer
 * @retval          null
 */
void rotate_roller(std::int32_t time , auto_control_t* rotate);

void move_relative(double target_distance, auto_control_t* move);

/**
 * @brief           auto control init
 * @param[in,out]   init:
 * @retval          null
 */
void auto_init(auto_control_t* init)
{
   init->chassis_voltage = get_chassis_voltage_point();
    init->functional_status = get_functional_device_status();
    init->sensor_data = get_sensor_data_point();
    pros::Mutex init_mutex;
    init_mutex.take();
    {
        // printf("here2");
        init->functional_status->flywheel = E_FLYWHEEL_STATUS_OFF;
        init->functional_status->intake_motor = E_FUNCTIONAL_MOTOR_STATUS_FORWARD;
        // init->functional_status->roller_motor = E_FUNCTIONAL_MOTOR_STATUS_BACKWARD;
// printf("1");
    }
    init_mutex.give();
// printf("2");


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
        int analog_left_y = 12;
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
void move_relative(double target_distance, auto_control_t* move)
{
    double v0 = 0;
    double vt = 0;
    double x = 0;
    double a = 0;
    pros::Mutex turn_mutex;
    std::int32_t analog_left_y = 70;
    std::uint32_t now_2 = pros::millis();
    while (x < target_distance) {
        turn_mutex.take();
        {
            move->chassis_voltage[0] = analog_left_y;
            move->chassis_voltage[1] = analog_left_y;
            move->chassis_voltage[2] = analog_left_y;
            move->chassis_voltage[3] = analog_left_y;
        }
        turn_mutex.give();
        a = -move->sensor_data->gps_front_data.gps_acc.y;
        v0 = vt;
        vt = v0 + a*(AUTO_TASK_TIME_MS/100.0);
        x += v0*(AUTO_TASK_TIME_MS/100.0) + 0.5*a*(AUTO_TASK_TIME_MS/100.0)*(AUTO_TASK_TIME_MS/100.0);
        printf("distance %lf", x);
        pros::lcd::print(1, "distance: %lf", x);
        pros::lcd::print(2, "acc: %lf", move->sensor_data->gps_front_data.gps_acc.y);
        pros::Task::delay_until(&now_2, AUTO_TASK_TIME_MS);
    }
    turn_mutex.take();
    {
        move->chassis_voltage[0] = 0;
        move->chassis_voltage[1] = 0;
        move->chassis_voltage[2] = 0;
        move->chassis_voltage[3] = 0;
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
    horizontal_time(0.1, 1, &auto_control);
//    move_relative(0.1, &auto_control);
    // pros::delay(500);

    rotate_roller(0.2, &auto_control);

   
    // std::uint32_t now = pros::millis();
    // while (true) {
    //     pros::Task::delay_until(&now, AUTO_TASK_TIME_MS);
    // }

    // std::int32_t now_1 = pros::millis();
    // horizontal_time(FORWARD, 1310, &auto_control);
    // move_time(BACKWARD, 105, &auto_control);
    // pros::Mutex auto_mutex;
    // auto_mutex.take();
    // {
    //     auto_control.functional_status->roller_motor = E_FUNCTIONAL_MOTOR_STATUS_FORWARD;
    // }
    // auto_mutex.give();
    // pros::delay(200);
    // auto_mutex.take();
    // auto_control.functional_status->roller_motor = E_FUNCTIONAL_MOTOR_STATUS_OFF;
    // auto_mutex.give();
    // pros::delay(320);
    
    // move_time(FORWARD, 300, &auto_control);
    // turn_time(BACKWARD, 380, &auto_control);
    // pros::delay(1000);
    // move_time(BACKWARD, 560, &auto_control);
    
    // auto_mutex.take();
    // auto_control.functional_status->roller_motor = E_FUNCTIONAL_MOTOR_STATUS_BACKWARD;
    // auto_mutex.give();
    
    // pros::delay(500);

    // auto_mutex.take();
    // auto_control.functional_status->roller_motor = E_FUNCTIONAL_MOTOR_STATUS_OFF;
    // auto_mutex.give();
    // pros::delay(300);
    // move_time(FORWARD, 300, &auto_control);
    // turn_time(FORWARD, 180, &auto_control);
    // pros::delay(1000);
    // move_time(BACKWARD, 100, &auto_control);

    // while (true) {
    //     std::int32_t now_2 = pros::millis();
    //     if (now_2 - now_1 >= 50*1000) {
    //         auto_control.functional_status->extension_motor = E_FUNCTIONAL_MOTOR_STATUS_BACKWARD;
    //         pros::delay(1500);
    //         auto_control.functional_status->extension_motor = E_FUNCTIONAL_MOTOR_STATUS_OFF;
    //     }
        
    // }
    
    
}


void rotate_roller(std::int32_t time , auto_control_t* rotate)
{
    pros::Mutex rotate_mutex;
    rotate_mutex.take();
    {
        rotate->functional_status->roller_motor = E_FUNCTIONAL_MOTOR_STATUS_BACKWARD;
    }
    rotate_mutex.give();
    pros::delay(time);
    rotate_mutex.take();
    {
        rotate->functional_status->roller_motor = E_FUNCTIONAL_MOTOR_STATUS_OFF;
    }
    rotate_mutex.give();
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

/**
 * @brief           let robot turn to specific point
 * @param[in]       direction: direction, -1 left or +1 right
 * @param[in]       time: a period of time
 * @param[in,out]   turn: find current position,give chassis voltage
 * @retval          null
 */
void horizontal_time(double direction, double time, auto_control_t* move)
{
    std::int32_t analog_left_x = 70;
    pros::Mutex turn_mutex;
    turn_mutex.take();
    {
        move->chassis_voltage[0] = analog_left_x*direction;
        move->chassis_voltage[1] = -analog_left_x*direction;
        move->chassis_voltage[2] = -analog_left_x*direction;
        move->chassis_voltage[3] = analog_left_x*direction*1.1;
    }
    turn_mutex.give();
    pros::delay(time);
    analog_left_x = 0;
    turn_mutex.take();
    {
        move->chassis_voltage[0] = analog_left_x*direction;
        move->chassis_voltage[1] = -analog_left_x*direction;
        move->chassis_voltage[2] = -analog_left_x*direction;
        move->chassis_voltage[3] = analog_left_x*direction;
    }
    turn_mutex.give();
}




