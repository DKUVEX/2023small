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
#include "dku/pid.hpp"
//in the beginning of task ,wait a time
//任务开始空闲一段时间
#define FUNCTIONAL_TASK_INIT_TIME       300
//functional task control time  2ms
//竞技任务控制间隔 2ms
#define FUNCTIONAL_CONTROL_TIME_MS      2
//flywheel max motor control current
//飞轮最大can发送电流值
#define MAX_FLYWHEEL_MOTOR_VOLTAGE 12000.0f

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

#define FLYWHEEL_MOTOR_SPEED_PID_KP         100.0f
#define FLYWHEEL_MOTOR_SPEED_PID_KI         10.0f
#define FLYWHEEL_MOTOR_SPEED_PID_KD         0.0f
#define PITCH_PAW_SPEED_PID_MAX_OUT         MAX_FLYWHEEL_MOTOR_VOLTAGE
#define PITCH_PAW_SPEED_PID_MAX_IOUT        2000.0f

typedef enum flywheel_status_e
{
    E_FLYWHEEL_STATUS_OFF = 0,
    E_FLYWHEEL_STATUS_SPEED_LOW,
    E_FLYWHEEL_STATUS_SPEED_HIGH,
} flywheel_status_e_t;

typedef struct {
    pros::Motor *motor_status;
    std::float_t speed;
    float set_voltage;
    float gave_voltage;
    pid_type_def motor_speed_pid;
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