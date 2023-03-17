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
#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include "api.h"
#include "pros/gps.hpp"
#include "pros/optical.hpp"

//in the beginning of task ,wait a time
//任务开始空闲一段时间
#define SENSOR_TASK_INIT_TIME 300
//sensor task control time  2ms
//传感器任务控制间隔 2ms
#define SENSOR_CONTROL_TIME_MS 5

#define SENSOR_GPS_FRONT_PORT 11
#define SENSOR_GPS_BACK_PORT 1

#define SENSOR_GPS_FRONT_OFFSET_X 0.01
#define SENSOR_GPS_FRONT_OFFSET_Y 0.01
#define SENSOR_GPS_FRONT_INITIAL_X 0.01
#define SENSOR_GPS_FRONT_INITIAL_Y 0.01
#define SENSOR_GPS_FRONT_INITIAL_HEADING 0

#define SENSOR_GPS_BACK_OFFSET_X 0.01
#define SENSOR_GPS_BACK_OFFSET_Y 0.01
#define SENSOR_GPS_BACK_INITIAL_X 0.01
#define SENSOR_GPS_BACK_INITIAL_Y 0.01
#define SENSOR_GPS_BACK_INITIAL_HEADING 0

#define SENSOR_GPS_DATA_RATE 5

typedef struct {
    pros::GPS *gps_pointer;
    pros::c::gps_status_s_t gps_pos;
    pros::c::gps_gyro_s_t gps_gyro;
    pros::c::gps_accel_s_t gps_acc;
}gps_all_t;
typedef struct {
    
    gps_all_t gps_front_data;
    gps_all_t gps_back_data;
    pros::Optical *optical_data;
}sensor_data_t;
/**
  * @brief          sensor task, osDelay SENSOR_CONTROL_TIME_MS (2ms) 
  * @param[in]      param: null
  * @retval         none
  */
/**
  * @brief          传感器任务，间隔 SENSOR_CONTROL_TIME_MS 5ms
  * @param[in]      param: 空
  * @retval         none
  */
void sensor_task_fn(void* param);
/**
  * @brief          get sensor data point
  * @param[in]      none
  * @retval         sensor data point
  */
/**
  * @brief          获取传感器数据指针
  * @param[in]      none
  * @retval         传感器数据指针
  */
sensor_data_t *get_sensor_data_point(void);

#endif