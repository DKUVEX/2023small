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
#define AUTO_TASK_INIT_TIME       300
//auto task control time  2ms
//自动任务控制间隔 2ms
#define AUTO_TASK_TIME_MS          2
#define PI (3.1415926535)
#define FORWARD 1
#define BACKWARD -1
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
// void auto_task_fn(void* param);
void auto_task_fn(void* param);

/**
 * @brief           get the current status struck(x,y,direction)
 * @param[out]       null
 * @return          current_status_t*
 * @retval         
 */
current_status_t* get_current_status_pointer(void);
#endif