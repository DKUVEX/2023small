/**
  ****************************(C) COPYRIGHT 2023 Blue Bear****************************
  * @file       senator_task.hpp
  * @brief      sensor data update task,
  *             传感器数据更新任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Feb-17-2023     Tianyi          1. start
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 Blue Bear****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "api.h"
#include "pros/gps.hpp"
#include "pros/optical.hpp"

//in the beginning of task ,wait a time
//任务开始空闲一段时间
#define SENSOR_TASK_INIT_TIME 300
//sensor task control time  2ms
//传感器任务控制间隔 2ms
#define SENSOR_CONTROL_TIME_MS 2

typedef struct {
    pros::GPS *gps_data;
    pros::Optical *optical_data;
}sensor_fetch_t;
/**
  * @brief          sensor task, osDelay SENSOR_CONTROL_TIME_MS (2ms) 
  * @param[in]      param: null
  * @retval         none
  */
/**
  * @brief          传感器任务，间隔 SENSOR_CONTROL_TIME_MS 2ms
  * @param[in]      param: 空
  * @retval         none
  */
void sensor_task_fn(void* param);

#endif