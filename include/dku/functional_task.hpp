/**
  ****************************(C) COPYRIGHT 2023 Blue Bear****************************
  * @file       functional_behaviour.hpp
  * @brief      the specific competing task, changes for each season
  *             具体的竞技任务，随赛季改变
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
#ifndef FUNCTIONAL_task_H
#define FUNCTIONAL_task_H

#include "api.h"
#include "dku/remote_control.hpp"
#include "dku/control/pid.hpp"
#include <cstdint>
//in the beginning of task ,wait a time
//任务开始空闲一段时间
#define FUNCTIONAL_TASK_INIT_TIME       300
//functional task control time  2ms
//竞技任务控制间隔 2ms
#define FUNCTIONAL_CONTROL_TIME_MS      200

#define INTAKE_MOTOR_PORT               7
#define FLYWHEEL_MOTOR_PORT             6
#define INDEX_MOTOR_PORT                10
#define ROLLER_MOTOR_PORT               4
#define GAS_GPIO_PORT                   1 // port A

#define FUNCTION_MOTOR_GEAR_RATIO       (pros::E_MOTOR_GEARSET_18)
#define FUCTION_MOTOR_ENCODER_UNIT      (pros::E_MOTOR_ENCODER_DEGREES)
#define FUNCTIONAL_MOTOR_MAX_SPEED         200 // The motor has the 18 Gearset
#define FUNCTIONAL_MOTOR_ZERO_SPEED        0 // The motor has the 18 Gearset
#define FUNCTIONAL_MOTOR_MAX_VOLTAGE       12000 // The motor has the 18 Gearset
#define FUNCTIONAL_MOTOR_ZERO_VOLTAGE      0 // The motor has the 18 Gearset

#define FUNCTIONAL_LIFT_HIGH_STATE LOW
#define FUNCTIONAL_LIFT_LOW_STATE HIGH
#define MAX_FLEWHEEL_MOTOR_VOLTAGE 11800 
//flywheel speed close-loop PID params, max out and max iout
//飞轮 PID参数以及 PID最大输出，积分输出
#define FLYWHEEL_SPEED_PID_KP        1
#define FLYWHEEL_SPEED_PID_KI        0
#define FLYWHEEL_SPEED_PID_KD        0
#define FLYWHEEL_SPEED_PID_MAX_OUT   MAX_FLEWHEEL_MOTOR_VOLTAGE
#define FLYWHEEL_SPEED_PID_MAX_IOUT  200
//arm motor speed PID
typedef enum flywheel_status_e
{
    E_FLYWHEEL_STATUS_OFF = 0,
    E_FLYWHEEL_STATUS_SPEED_LOW,
    E_FLYWHEEL_STATUS_SPEED_HIGH,
} flywheel_status_e_t;

typedef struct {
    pros::Motor *motor_status;
    std::float_t speed;
    std::int32_t give_voltage; // pid output
    std::int32_t set_voltage;  // pid input
    pid_type_def speed_pid;
}functional_motor_t;

typedef struct {
    pros::Controller *functional_RC; //竞技功能使用的遥控器指针, the point to remote control
    functional_motor_t motor_flywheel;   // motor data.电机数据
    functional_motor_t motor_index;   // motor data.电机数据
    functional_motor_t motor_intake;   // motor data.电机数据
    functional_motor_t motor_roller;   // motor data.电机数据
    pros::ADIPort *gas_gpio;           // control gas
}functional_behaviour_t;

/**
  * @brief          finctional task, osDelay FUNCTIONAL_CONTROL_TIME_MS (2ms) 
  * @param[in]      param: null
  * @retval         none
  */
/**
  * @brief          竞技任务，间隔 FUNCTIONAL_CONTROL_TIME_MS 2ms
  * @param[in]      param: 空
  * @retval         none
  */
void functional_task_fn(void* param);

#endif