/**
  ****************************(C) COPYRIGHT 2023 Blue Bear****************************
  * @file       senator_task.cpp
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
#include "dku/sensor_task.hpp"

/**
  * @brief          "sensor_fetch" valiable initialization, include internal and external sensors
  * @param[out]     sensor_fetch_t: "sensor_fetch_init" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"sensor_fetch"变量，包括各内置、外置传感器数据初始化
  * @param[out]     sensor_fetch_t:"sensor_fetch_init"变量指针.
  * @retval         none
  */
//TODO: not finish yet
static void sensor_init(sensor_fetch_t *sensor_fetch_init);

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
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      param: null
  * @retval         none
  */
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      param: 空
  * @retval         none
  */
void chassis_task_fn(void* param) 
{

}
