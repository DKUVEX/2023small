/**
 * ****************************(C) COPYRIGHT 2023 Blue Bear****************************
 * @file       auto_task.cpp
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
#include "dku/auto/auto_task.hpp"
#include "dku/chassis_task.hpp"

auto_control_t auto_control;

/**
 * @brief           auto control init
 * @param[in,out]   init:
 * @retval         null
 */
void auto_init(auto_control_t* init);


/**
 * @brief           let robot move to specific point
 * @param[in]       current_x: current x position
 * @param[in]       current_y: current y position
 * @param[in]       target_x: aimed position x
 * @param[in]       target_y: aimed position y
 * @param[out]      move: give chassis voltage
 * @retval         
 */
void move_to(double current_x, double current_y, double target_x, double target_y, auto_control_t* move);

/**
 * @brief           auto control init
 * @param[in,out]   init:
 * @retval         null
 */
void auto_init(auto_control_t* init)
{
    init->chassis_voltage = get_chassis_voltage_point();
}

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
void auto_task_fn(void* param)
{

}