/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.hpp
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V2.0.0     Feb-24-2023     Tianyi          2. 适配VEX
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include <cstdint>
#define NULL 0
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    std::uint8_t mode;
    //PID 三参数 PID 3 parameters 
    std::int32_t Kp;
    std::int32_t Ki;
    std::int32_t Kd;

    std::int32_t max_out;  //最大输出 max output
    std::int32_t max_iout; //最大积分输出 max integrate output

    std::int32_t set;
    std::int32_t fdb;

    std::int32_t out;
    std::int32_t Pout;
    std::int32_t Iout;
    std::int32_t Dout;
    std::int32_t Dbuf[3];  //微分项 0最新 1上一次 2上上次 Differential item 0 latest 1 last 2 last last
    std::int32_t error[3]; //误差项 0最新 1上一次 2上上次 Error term 0 latest 1 last 2 last last

} pid_type_def;
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const std::int32_t PID[3], std::int32_t max_out, std::int32_t max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern std::int32_t PID_calc(pid_type_def *pid, std::int32_t ref, std::int32_t set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

#endif
