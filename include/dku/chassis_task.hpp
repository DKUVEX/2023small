/**
  ****************************(C) COPYRIGHT 2023 Blue Bear****************************
  * @file       chassis_task.hpp
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Feb-12-2023     Tianyi          1. start
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 Blue Bear****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include <cmath>
#include <cstdint>
#include <iostream>
#include "api.h"

//in the beginning of task ,wait a time
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 300
//chassis task control time  2ms
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//the reduction gear ratio of chassis motors
//底盘电机减速比
#define CHASSIS_MOTOR_GEAR_RATIO (pros::E_MOTOR_GEARSET_18)
#define CHASSIS_MOTOR_ENCODER_UNIT (pros::E_MOTOR_ENCODER_DEGREES)

#define CHASSIS_MOTOR_LA_PORT 2
#define CHASSIS_MOTOR_LB_PORT 3
#define CHASSIS_MOTOR_RA_PORT 9
#define CHASSIS_MOTOR_RB_PORT 8

typedef struct {
    pros::Motor *motor_status;
    std::float_t speed;
    std::uint16_t give_current;
}chassis_motor_t;

typedef struct {
    pros::Controller *chassis_RC; //底盘使用的遥控器指针, the point to remote control
    chassis_motor_t motor_chassis[4];   //chassis motor data.底盘电机数据
}chassis_move_t;


/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      param: null
  * @retval         none
  */
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      param: 空
  * @retval         none
  */
void chassis_task_fn(void* param);
/**
  * @brief          get chassis voltage point
  * @param[in]      none
  * @retval         chassis voltage data point
  */
/**
  * @brief          获取底盘电压数据指针
  * @param[in]      none
  * @retval         底盘电压数据指针
  */
std::int32_t *get_chassis_voltage_point(void);
#endif