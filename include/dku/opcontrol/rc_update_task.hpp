/**
 * ****************************(C) COPYRIGHT 2023 Blue Bear****************************
 * @file       rc_update_task.hpp
 * @brief      update controller status
 * 
 * @note       
 * @history:
 *   Version   Date            Author          Modification    Email
 *   V1.0.0    Feb-27-2023     Tianyi Zhang    1. start        tz137@duke.edu/shadow_rogue@qq.com
 * 
 * @verbatim
 * ==============================================================================
 * 
 * ==============================================================================
 * @endverbatim
 * ****************************(C) COPYRIGHT 2023 Blue Bear****************************
 */
#ifndef RC_UPDATE_TASK_H
#define RC_UPDATE_TASK_H

#include "api.h"
#include "dku/remote_control.hpp"
#include "dku/chassis_task.hpp"
#include "dku/functional_task.hpp"
#include <cstdint>

//in the beginning of task ,wait a time
//任务开始空闲一段时间
#define RC_UPDATE_TASK_INIT_TIME       300
//functional task control time  2ms
//竞技任务控制间隔 2ms
#define RC_UPDATE_CONTROL_TIME_MS      2
#define FUNCTIONAL_LIFT_HIGH_STATE LOW
#define FUNCTIONAL_LIFT_LOW_STATE HIGH

typedef struct {
    pros::Controller *update_RC; //遥控器指针, the point to remote control
    std::int32_t *chassis_voltage;
    functional_device_status_t *functional_status;
    std::uint32_t now_time;
    std::uint32_t op_start_time;
    bool op_start_flag = false;
}rc_update_t;
// extern std::int32_t chassis_motor_voltage[4];
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
void rc_update_task_fn(void* param);

#endif