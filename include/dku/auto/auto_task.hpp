/**
 * ****************************(C) COPYRIGHT 2023 Blue Bear****************************
 * @file       auto_task.hpp
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
#ifndef AUTO_TASK_H
#define AUTO_TASK_H

#include "api.h"
#include "dku/functional_task.hpp"
#include "dku/chassis_task.hpp"
#include "dku/sensor_task.hpp"
#include <cmath>
//in the beginning of task ,wait a time
//任务开始空闲一段时间
#define AUTO_TASK_INIT_TIME       5000
//auto task control time  2ms
//自动任务控制间隔 2ms
#define AUTO_TASK_TIME_MS          SENSOR_CONTROL_TIME_MS
#define PI (3.1415926535)
#define FORWARD 1
#define BACKWARD -1
#define STOP 0
typedef struct {
    double current_x;
    double current_y;
    double current_dir;
}current_status_t;

typedef struct {
    std::int32_t *chassis_voltage;
    functional_device_status_t *functional_status;
    current_status_t current_pos; 
    sensor_data_t *sensor_data;
}auto_control_t;

extern auto_control_t auto_control;
/**
  * @brief          finctional task, osDelay AUTO_TASK_TIME_MS (2ms) 
  * @param[in]      param: null
  * @retval         none
  */
/**
  * @brief          竞技任务，间隔 AUTO_TASK_TIME_MS 2ms
  * @param[in]      param: 空
  * @retval         none
  */
void auto_task_fn(void* param);

/**
 * @brief           get the current status struck(x,y,direction)
 * @param[out]       null
 * @return          current_status_t*
 * @retval         
 */





current_status_t* get_current_status_pointer(void);

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
void turn_right_relative(double target_angle, auto_control_t* turn);

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
 * @brief           move a relative distance
 * @param[in]       target_distance: aimed distance
 * @param[in,out]   move: find current position,give chassis voltage
 * @retval          null
 */
void move_front_relative(double target_distance, auto_control_t* move);

void move_back_relative(double target_distance, auto_control_t* move);

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

/**
 
 
 * @brief           auto control init
 * @param[in,out]   init:
 * @retval          null
 */

void move_horizontal_right_relative(double target_distance, auto_control_t* move);

void move_horizontal_left_relative(double target_distance, auto_control_t* move);

void move_front_speed_relative(double target_distance, auto_control_t* move, std::int32_t analog_left_y);

void turn_left_relative(double target_angle, auto_control_t* turn);

#endif