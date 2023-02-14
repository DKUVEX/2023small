/**
  ****************************(C) COPYRIGHT 2023 Blue Bear****************************
  * @file       chassis.cpp
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
#include "dku/chassis_task.hpp"
#include "dku/remote_control.hpp"


//底盘运动数据 chassis movement data
chassis_move_t chassis_move;
pros::Motor chassis_motor_la(CHASSIS_MOTOR_LA_PORT, CHASSIS_MOTOR_GEAR_RATIO, false, CHASSIS_MOTOR_ENCODER_UNIT);
pros::Motor chassis_motor_lb(CHASSIS_MOTOR_LB_PORT, CHASSIS_MOTOR_GEAR_RATIO, false, CHASSIS_MOTOR_ENCODER_UNIT);
pros::Motor chassis_motor_ra(CHASSIS_MOTOR_RA_PORT, CHASSIS_MOTOR_GEAR_RATIO, true, CHASSIS_MOTOR_ENCODER_UNIT);
pros::Motor chassis_motor_rb(CHASSIS_MOTOR_RB_PORT, CHASSIS_MOTOR_GEAR_RATIO, true, CHASSIS_MOTOR_ENCODER_UNIT);

/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
//TODO: not finish yet
static void chassis_init(chassis_move_t *chassis_move_init);


/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
//TODO: not finish yet
static void chassis_init(chassis_move_t *chassis_move_init)
{
    chassis_move_init->chassis_RC = get_remote_control_point();
    chassis_move_init->motor_chassis[0].motor_status = &chassis_motor_la;
    chassis_move_init->motor_chassis[1].motor_status = &chassis_motor_lb;
    chassis_move_init->motor_chassis[2].motor_status = &chassis_motor_ra;
    chassis_move_init->motor_chassis[3].motor_status = &chassis_motor_rb;

}
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
void chassis_task_fn(void* param) {
    std::cout << "Chassis task runs" << std::endl;
    pros::Task::delay(CHASSIS_TASK_INIT_TIME);
    chassis_init(&chassis_move);
    std::uint32_t now = pros::millis();
    while (true) {
        // Do opcontrol things
        chassis_move.motor_chassis[0].motor_status->move(chassis_move.chassis_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)
                                                        +chassis_move.chassis_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)
                                                        -chassis_move.chassis_RC->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        chassis_move.motor_chassis[1].motor_status->move(chassis_move.chassis_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)
                                                        -chassis_move.chassis_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)
                                                        -chassis_move.chassis_RC->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        chassis_move.motor_chassis[2].motor_status->move(chassis_move.chassis_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)
                                                        -chassis_move.chassis_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)
                                                        +chassis_move.chassis_RC->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        chassis_move.motor_chassis[3].motor_status->move(chassis_move.chassis_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)
                                                        +chassis_move.chassis_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)
                                                        +chassis_move.chassis_RC->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        pros::Task::delay_until(&now, CHASSIS_CONTROL_TIME_MS);
    }
}