/**
  ****************************(C) COPYRIGHT 2023 Blue Bear****************************
  * @file       remote_control.cpp
  * @brief      remote control,
  *             遥控
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Feb-12-2023     Tianyi          1. start
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT COPYRIGHT 2023 Blue Bear****************************
  */
#include "dku/remote_control.hpp"


//remote control data 
//遥控器控制变量
pros::Controller rc_ctrl(pros::E_CONTROLLER_MASTER);
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
pros::Controller *get_remote_control_point(void)
{
    return &rc_ctrl;
}