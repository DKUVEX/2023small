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
#include "pros/gps.hpp"
#include "pros/llemu.hpp"
#include <iostream>
#include <ostream>

sensor_data_t sensor_fetch;
pros::GPS gps_front(SENSOR_GPS_FRONT_PORT, SENSOR_GPS_FRONT_INITIAL_X, SENSOR_GPS_FRONT_INITIAL_Y, 
                    SENSOR_GPS_FRONT_INITIAL_HEADING, SENSOR_GPS_FRONT_OFFSET_X, SENSOR_GPS_FRONT_OFFSET_Y);
pros::GPS gps_back(SENSOR_GPS_BACK_PORT, SENSOR_GPS_BACK_INITIAL_X, SENSOR_GPS_BACK_INITIAL_Y, 
                    SENSOR_GPS_BACK_INITIAL_HEADING, SENSOR_GPS_BACK_OFFSET_X, SENSOR_GPS_BACK_OFFSET_Y);

/**
  * @brief          update gps data, include position, gyroscope and accelerate
  * @param[out]     gps_all_t: "gps_update" valiable point
  * @retval         none
  */
/**
  * @brief          更新gps数据，包括位置, 陀螺仪, 加速度
  * @param[out]     gps_all_t:"gps_update"变量指针.
  * @retval         none
  */
static void gps_data_update(gps_all_t *gps_update);

/**
  * @brief          "sensor_fetch" valiable initialization, include internal and external sensors
  * @param[out]     sensor_data_t: "sensor_fetch_init" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"sensor_fetch"变量，包括各内置、外置传感器数据初始化
  * @param[out]     sensor_data_t:"sensor_fetch_init"变量指针.
  * @retval         none
  */
//TODO: not finish yet
static void sensor_init(sensor_data_t *sensor_fetch_init);


/**
  * @brief          update gps data, include position, gyroscope and accelerate
  * @param[out]     gps_all_t: "gps_update" valiable point
  * @retval         none
  */
/**
  * @brief          更新gps数据，包括位置, 陀螺仪, 加速度
  * @param[out]     gps_all_t:"gps_update"变量指针.
  * @retval         none
  */
static void gps_data_update(gps_all_t *gps_update)
{
    gps_update->gps_pos = gps_update->gps_pointer->get_status();
    gps_update->gps_gyro = gps_update->gps_pointer->get_gyro_rate();
    gps_update->gps_acc = gps_update->gps_pointer->get_accel();
}

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
static void sensor_init(sensor_data_t *sensor_fetch_init)
{
    sensor_fetch_init->gps_front_data.gps_pointer = &gps_front;
    sensor_fetch_init->gps_back_data.gps_pointer = &gps_back;
    sensor_fetch_init->gps_front_data.gps_pointer->set_data_rate(SENSOR_GPS_DATA_RATE);
    sensor_fetch_init->gps_back_data.gps_pointer->set_data_rate(SENSOR_GPS_DATA_RATE);
    gps_data_update(&sensor_fetch_init->gps_front_data);
    gps_data_update(&sensor_fetch_init->gps_back_data);
}
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
sensor_data_t *get_sensor_data_point(void)
{
    return &sensor_fetch;
}
/**
  * @brief          sensor task, osDelay SENSOR_CONTROL_TIME_MS (5ms) 
  * @param[in]      param: null
  * @retval         none
  */
/**
  * @brief          传感器任务，间隔 SENSOR_CONTROL_TIME_MS 5ms
  * @param[in]      param: 空
  * @retval         none
  */
void sensor_task_fn(void* param)
{
    std::cout << "sensor task runs" << std::endl;
    pros::Task::delay(SENSOR_TASK_INIT_TIME);
    sensor_init(&sensor_fetch);
    std::uint32_t now = pros::millis();
    while (true) {
        gps_data_update(&sensor_fetch.gps_front_data);
        gps_data_update(&sensor_fetch.gps_back_data);
        /*test code*/
        //TODO: add a polt on pc to see the trend of gyro and acc(transfer com data to some apps?)
        // pros::lcd::print(0, "position x: %lf", sensor_fetch.gps_front_data.gps_pos.x);
        // pros::lcd::print(1, "position y: %lf", sensor_fetch.gps_front_data.gps_pos.y);
        // pros::lcd::print(2, "heading: %lf", sensor_fetch.gps_front_data.gps_pos.yaw);
        // pros::lcd::print(3, "gyro_x: %lf", sensor_fetch.gps_front_data.gps_gyro.x);
        // pros::lcd::print(4, "gyro_y: %lf", sensor_fetch.gps_front_data.gps_gyro.y);
        // pros::lcd::print(5, "acc_x: %lf", sensor_fetch.gps_front_data.gps_acc.x);
        // pros::lcd::print(6, "acc_y: %lf", sensor_fetch.gps_front_data.gps_acc.y);

        pros::Task::delay_until(&now, SENSOR_CONTROL_TIME_MS);
    }
}
