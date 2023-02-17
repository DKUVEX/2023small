/**
  ****************************(C) COPYRIGHT 2023 Blue Bear****************************
  * @file       functional_behaviour.cpp
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

#include "dku/functional_task.hpp"
#include "pros/misc.h"


functional_behaviour_t functional_behaviour;
pros::Motor intake_motor(INTAKE_MOTOR_PORT, FUNCTION_MOTOR_GEAR_RATIO, false, FUCTION_MOTOR_ENCODER_UNIT);
pros::Motor index_motor(INDEX_MOTOR_PORT, FUNCTION_MOTOR_GEAR_RATIO, true, FUCTION_MOTOR_ENCODER_UNIT);
pros::Motor roller_motor(ROLLER_MOTOR_PORT, FUNCTION_MOTOR_GEAR_RATIO, false, FUCTION_MOTOR_ENCODER_UNIT);

pros::Motor flywheel_motor(FLYWHEEL_MOTOR_PORT, false); //fly wheel do not have gear

/**
  * @brief          "functional_behaviour" valiable initialization, include pid initialization, remote control data point initialization, chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     functional_behaviour_init: "functional_behaviour" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"functional_behaviour"变量，包括pid初始化， 遥控器指针初始化，底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     functional_behaviour_init:"functional_behaviour"变量指针.
  * @retval         none
  */
//TODO: not finish yet
static void functional_init(functional_behaviour_t *functional_behaviour_init);

/**
  * @brief          concrol the behaviour of fly wheel
  * @param[in]      int: flywheel_status
  * @retval         none
  */
/**
  * @brief          飞轮控制函数
  * @param[in]      int: flywheel_status
  * @retval         none
  */
//TODO: not finish yet
static void flywheel_move(functional_motor_t *flywheel_motor, int flywheel_status);

/**
  * @brief          concrol the behaviour of fly wheel
  * @param[in]      int: flywheel_status
  * @retval         none
  */
/**
  * @brief          飞轮控制函数
  * @param[in]      int: flywheel_status
  * @retval         none
  */
//TODO: not finish yet
static void flywheel_move(functional_motor_t *flywheel_motor, int flywheel_status)
{
    if (flywheel_status == E_FLYWHEEL_STATUS_OFF)
    {
        flywheel_motor->motor_status->move_voltage(FUNCTIONAL_MOTOR_ZERO_VOLTAGE);
    }
    else if (flywheel_status == E_FLYWHEEL_STATUS_SPEED_HIGH) {
        flywheel_motor->motor_status->move_voltage(FUNCTIONAL_MOTOR_MAX_VOLTAGE);        
    }
    else if (flywheel_status == E_FLYWHEEL_STATUS_SPEED_LOW) {
        flywheel_motor->motor_status->move_voltage(FUNCTIONAL_MOTOR_MAX_VOLTAGE*0.5);        
    }
}
/**
  * @brief          "functional_behaviour" valiable initialization, include pid initialization, remote control data point initialization, chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     functional_behaviour_init: "functional_behaviour" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"functional_behaviour"变量，包括pid初始化， 遥控器指针初始化，底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     functional_behaviour_init:"functional_behaviour"变量指针.
  * @retval         none
  */
//TODO: not finish yet
static void functional_init(functional_behaviour_t *functional_behaviour_init)
{
    flywheel_motor.set_encoder_units(FUCTION_MOTOR_ENCODER_UNIT);
    functional_behaviour_init->functional_RC = get_remote_control_point();
    functional_behaviour_init->motor_index.motor_status = &index_motor;
    functional_behaviour_init->motor_intake.motor_status = &intake_motor;
    functional_behaviour_init->motor_roller.motor_status = &roller_motor;
    functional_behaviour_init->motor_flywheel.motor_status = &flywheel_motor;
}
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
void functional_task_fn(void* param)
{

    std::cout << "Functional task runs" << std::endl;
    pros::Task::delay(FUNCTIONAL_TASK_INIT_TIME);
    functional_init(&functional_behaviour);
    static int flywheel_status = E_FLYWHEEL_STATUS_OFF;
    std::uint32_t now = pros::millis();
    while (true) {
        if ((functional_behaviour.functional_RC->get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            && !functional_behaviour.functional_RC->get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            functional_behaviour.motor_intake.motor_status->move_velocity(FUNCTIONAL_MOTOR_MAX_SPEED);
        }
        else if ((functional_behaviour.functional_RC->get_digital(pros::E_CONTROLLER_DIGITAL_R2))
                 && !functional_behaviour.functional_RC->get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            functional_behaviour.motor_intake.motor_status->move_velocity(-FUNCTIONAL_MOTOR_MAX_SPEED);
        }
        else if (!(functional_behaviour.functional_RC->get_digital(pros::E_CONTROLLER_DIGITAL_R2))
                 && !(functional_behaviour.functional_RC->get_digital(pros::E_CONTROLLER_DIGITAL_R1)))
        {
            functional_behaviour.motor_intake.motor_status->move_velocity(-FUNCTIONAL_MOTOR_ZERO_SPEED);
        }        // functional_behaviour.motor_intake.motor_status->move_velocity(FUNCTIONAL_MOTOR_MAX_SPEED
        //                                                              *(functional_behaviour.functional_RC->get_digital(pros::E_CONTROLLER_DIGITAL_R1)
        //                                                               -functional_behaviour.functional_RC->get_digital(pros::E_CONTROLLER_DIGITAL_R2)
        //                                                               )
        //                                                              *functional_behaviour.functional_RC->get_digital(pros::E_CONTROLLER_DIGITAL_L2));
        functional_behaviour.motor_roller.motor_status->move_velocity(FUNCTIONAL_MOTOR_MAX_SPEED
                                                                     *functional_behaviour.functional_RC->get_digital(pros::E_CONTROLLER_DIGITAL_Y));
        functional_behaviour.motor_index.motor_status->move_velocity(FUNCTIONAL_MOTOR_MAX_SPEED
                                                                     *functional_behaviour.functional_RC->get_digital(pros::E_CONTROLLER_DIGITAL_L1));
        if(functional_behaviour.functional_RC->get_digital(pros::E_CONTROLLER_DIGITAL_B)) 
        {
            flywheel_status = E_FLYWHEEL_STATUS_OFF;
        }
        else if(functional_behaviour.functional_RC->get_digital(pros::E_CONTROLLER_DIGITAL_L2) 
                && functional_behaviour.functional_RC->get_digital(pros::E_CONTROLLER_DIGITAL_R1)) 
        {
            flywheel_status = E_FLYWHEEL_STATUS_SPEED_HIGH;
        }
        else if(functional_behaviour.functional_RC->get_digital(pros::E_CONTROLLER_DIGITAL_L2) 
                && functional_behaviour.functional_RC->get_digital(pros::E_CONTROLLER_DIGITAL_R2)) 
        {
            flywheel_status = E_FLYWHEEL_STATUS_SPEED_LOW;
        }
        flywheel_move(&functional_behaviour.motor_flywheel,flywheel_status);
        pros::Task::delay_until(&now, FUNCTIONAL_CONTROL_TIME_MS);
    }
}
