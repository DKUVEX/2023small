/**
  ****************************(C) COPYRIGHT 2023 Blue Bear****************************
  * @file       functional_behaviour.cpp
  * @brief      the specific competing task, changes for each season
  *             具体的竞技任务，随赛季改变
  * @note       
  * @history
  *  Version    Date            Author          Modification        Email
  *  V1.0.0     Feb-17-2023     Tianyi          1. start            tz137@duke.edu/shadow_rogue@qq.email
  *  V1.0.1     Feb-26-2023     Tianyi          1. cannot disable build-in Pid, give up custom pid

  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 Blue Bear****************************
*/

#include "dku/functional_task.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include <cstdint>

functional_behaviour_t functional_behaviour;
functional_device_status_t functional_devices;
pros::Motor intake_motor(INTAKE_MOTOR_PORT, FUNCTION_MOTOR_GEAR_RATIO, false, FUCTION_MOTOR_ENCODER_UNIT);
pros::Motor index_motor(INDEX_MOTOR_PORT, FUNCTION_MOTOR_GEAR_RATIO, true, FUCTION_MOTOR_ENCODER_UNIT);
pros::Motor roller_motor(ROLLER_MOTOR_PORT, FUNCTION_MOTOR_GEAR_RATIO, false, FUCTION_MOTOR_ENCODER_UNIT);

pros::Motor flywheel_motor(FLYWHEEL_MOTOR_PORT,FLYWHEEL_MOTOR_GEAR_RATIO, true, FLYWHEEL_MOTOR_ENCODER_UNIT); //fly wheel do not have gear
pros::Motor flywheel_motor_2(FLYWHEEL_MOTOR_2_PORT,FLYWHEEL_MOTOR_GEAR_RATIO, false, FLYWHEEL_MOTOR_ENCODER_UNIT); //fly wheel do not have gear

pros::ADIPort gas_GPIO(GAS_GPIO_PORT, pros::E_ADI_DIGITAL_OUT);
pros::ADIPort extension_GPIO(EXTENSION_GPIO_PORT, pros::E_ADI_DIGITAL_OUT);

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
static void functional_init(functional_behaviour_t* functional_behaviour_init);

/**
  * @brief          concrol the behaviour of fly wheel
  * @param[out]     flywheel_motor: "flywheel_motor" valiable point
  * @param[out]     flywheel_motor_2: "flywheel_motor_2" 变量指针
  * @param[in]      flywheel_status: flywheel speed status    
  * @retval         none
  */
/**
  * @brief          飞轮控制函数
  * @param[out]     flywheel_motor: "flywheel_motor" 变量指针
  * @param[out]     flywheel_motor_2: "flywheel_motor_2" 变量指针
  * @param[in]      flywheel_status: flywheel 速度状态指示
  * @retval         none
  */
//TODO: not finish yet
static void flywheel_move(functional_motor_t *flywheel_motor, functional_motor_t *flywheel_motor_2, int flywheel_status);

/**
 * @brief           速度控制函数，避开pid
 * @param[in,out]   flywheel_motor: 电机指针
 * @param[in]       speed_error: 加速减速误差
 * @retval         
 */
/**
 * @brief           speed control, without pid
 * @param[in,out]   flywheel_motor: motor point
 * @param[in]       speed_error: error of accelerate and deaccelerate
 * @retval         
 */
void flywheel_speed_control_withoutpid(functional_motor_t *motor, std::int32_t speed_error);


/**
  * @brief          concrol the behaviour of fly wheel
  * @param[out]     flywheel_motor: "flywheel_motor" valiable point
  * @param[out]     flywheel_motor_2: "flywheel_motor_2" 变量指针
  * @param[in]      flywheel_status: flywheel speed status    
  * @retval         none
  */
/**
  * @brief          飞轮控制函数
  * @param[out]     flywheel_motor: "flywheel_motor" 变量指针
  * @param[out]     flywheel_motor_2: "flywheel_motor_2" 变量指针
  * @param[in]      flywheel_status: flywheel 速度状态指示
  * @retval         none
  */
//TODO: not finish yet
static void flywheel_move(functional_motor_t *flywheel_motor, functional_motor_t *flywheel_motor_2, int flywheel_status)
{
    if (flywheel_status == E_FLYWHEEL_STATUS_OFF)
    {
        flywheel_motor->set_voltage = FUNCTIONAL_MOTOR_ZERO_VOLTAGE;
        flywheel_motor_2->set_voltage = FUNCTIONAL_MOTOR_ZERO_VOLTAGE;

        flywheel_motor->motor_status->move_voltage(flywheel_motor->set_voltage);
        flywheel_motor_2->motor_status->move_voltage(flywheel_motor_2->set_voltage);
    }
    else if (flywheel_status == E_FLYWHEEL_STATUS_SPEED_HIGH) {
        flywheel_motor->set_voltage = MAX_FLEWHEEL_MOTOR_VOLTAGE*0.5;
        flywheel_motor_2->set_voltage = MAX_FLEWHEEL_MOTOR_VOLTAGE*0.5;
        flywheel_speed_control_withoutpid(flywheel_motor, FLYWHEEL_CONTROL_ERROR);
        flywheel_speed_control_withoutpid(flywheel_motor_2, FLYWHEEL_CONTROL_ERROR);        
    }
    else if (flywheel_status == E_FLYWHEEL_STATUS_SPEED_LOW) {
        flywheel_motor->set_voltage = MAX_FLEWHEEL_MOTOR_VOLTAGE*0.35;
        flywheel_motor_2->set_voltage = MAX_FLEWHEEL_MOTOR_VOLTAGE*0.35;
        flywheel_speed_control_withoutpid(flywheel_motor, FLYWHEEL_CONTROL_ERROR);
        flywheel_speed_control_withoutpid(flywheel_motor_2, FLYWHEEL_CONTROL_ERROR);
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
    static const float flywheel_speed_pid[3] = {FLYWHEEL_SPEED_PID_KP, FLYWHEEL_SPEED_PID_KI, FLYWHEEL_SPEED_PID_KD};
    
    flywheel_motor.set_encoder_units(FUCTION_MOTOR_ENCODER_UNIT);
    functional_behaviour_init->functional_RC = get_remote_control_point();
    functional_behaviour_init->motor_index.motor_status = &index_motor;
    functional_behaviour_init->motor_intake.motor_status = &intake_motor;
    functional_behaviour_init->motor_roller.motor_status = &roller_motor;
    functional_behaviour_init->motor_flywheel.motor_status = &flywheel_motor;
    functional_behaviour_init->motor_flywheel_2.motor_status = &flywheel_motor_2;
    functional_behaviour_init->gas_gpio = &gas_GPIO;
    functional_behaviour_init->gas_gpio->set_value(FUNCTIONAL_LIFT_LOW_STATE);
    functional_behaviour_init->extension_gpio = &extension_GPIO;
    functional_behaviour_init->extension_gpio->set_value(FUNCTIONAL_LIFT_HIGH_STATE);

    PID_init(&functional_behaviour_init->motor_flywheel.speed_pid, PID_DELTA, flywheel_speed_pid, FLYWHEEL_SPEED_PID_MAX_OUT, FLYWHEEL_SPEED_PID_MAX_IOUT);
    PID_clear(&functional_behaviour_init->motor_flywheel.speed_pid);
    functional_behaviour_init->motor_intake.motor_status->set_voltage_limit(MAX_FLEWHEEL_MOTOR_VOLTAGE);
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
    std::uint32_t now = pros::millis();
    while (true) {
        //TODO: lots of ugly repeat codes
        if (functional_devices.intake_motor == E_FUNCTIONAL_MOTOR_STATUS_FORWARD)
        {
            functional_behaviour.motor_intake.motor_status->move_velocity(FUNCTIONAL_MOTOR_MAX_SPEED);
        }
        else if (functional_devices.intake_motor == E_FUNCTIONAL_MOTOR_STATUS_BACKWARD)
        {
            functional_behaviour.motor_intake.motor_status->move_velocity(-FUNCTIONAL_MOTOR_MAX_SPEED);
        }
        else if (functional_devices.intake_motor == E_FUNCTIONAL_MOTOR_STATUS_OFF)
        {
            functional_behaviour.motor_intake.motor_status->move_velocity(FUNCTIONAL_MOTOR_ZERO_SPEED);
        }        
        if (functional_devices.roller_motor == E_FUNCTIONAL_MOTOR_STATUS_FORWARD) {
            functional_behaviour.motor_roller.motor_status->move_velocity(FUNCTIONAL_MOTOR_MAX_SPEED);
        }
        else if (functional_devices.roller_motor == E_FUNCTIONAL_MOTOR_STATUS_OFF) {
            functional_behaviour.motor_roller.motor_status->move_velocity(FUNCTIONAL_MOTOR_ZERO_SPEED);
        }
        else if (functional_devices.roller_motor == E_FUNCTIONAL_MOTOR_STATUS_BACKWARD) {
            functional_behaviour.motor_roller.motor_status->move_velocity(-FUNCTIONAL_MOTOR_MAX_SPEED);
        }
        if (functional_devices.index_motor == E_FUNCTIONAL_MOTOR_STATUS_FORWARD) {
            functional_behaviour.motor_index.motor_status->move_velocity(FUNCTIONAL_MOTOR_MAX_SPEED*0.5);
        }
        else if (functional_devices.index_motor == E_FUNCTIONAL_MOTOR_STATUS_OFF) {
            functional_behaviour.motor_index.motor_status->move_velocity(FUNCTIONAL_MOTOR_ZERO_SPEED);
        }
        flywheel_move(&functional_behaviour.motor_flywheel, &functional_behaviour.motor_flywheel_2,functional_devices.flywheel);
        pros::motor_pid_full_s_t testpid;
        testpid = functional_behaviour.motor_flywheel.motor_status->get_pos_pid();
        // printf("kf is %d, kp is %d, ki is %d, kd is %d, filter is %d, limit is %d, threshod is %d, loopspeed is %d\n", &testpid.kf, &testpid.kp, &testpid.ki, &testpid.kd, &testpid.filter, &testpid.limit, &testpid.threshold, &testpid.loopspeed);
        
        // TODO need to be changed as enum
        if ( functional_devices.gas_gpio == FUNCTIONAL_LIFT_HIGH_STATE ) {
            functional_behaviour.gas_gpio->set_value(FUNCTIONAL_LIFT_HIGH_STATE);
        }
        if ( functional_devices.gas_gpio == FUNCTIONAL_LIFT_LOW_STATE ) {
            functional_behaviour.gas_gpio->set_value(FUNCTIONAL_LIFT_LOW_STATE);
        }
        if (functional_devices.extension_gpio == FUNCTIONAL_LIFT_LOW_STATE ) {
            functional_behaviour.extension_gpio->set_value(FUNCTIONAL_LIFT_LOW_STATE);
        }
        pros::Task::delay_until(&now, FUNCTIONAL_CONTROL_TIME_MS);
    }
}
/**
 * @brief           速度控制函数，避开pid
 * @param[in,out]   flywheel_motor: 电机指针
 * @param[in]       speed_error: 加速减速误差
 * @retval         
 */
/**
 * @brief           speed control, without pid
 * @param[in,out]   flywheel_motor: motor point
 * @param[in]       speed_error: error of accelerate and deaccelerate
 * @retval         
 */
void flywheel_speed_control_withoutpid(functional_motor_t *motor, std::int32_t speed_error)
{
    if ((motor->motor_status->get_voltage()-motor->set_voltage) > speed_error) {
        motor->motor_status->move_voltage(FUNCTIONAL_MOTOR_ZERO_VOLTAGE);
    }
    else if ((motor->motor_status->get_voltage()-motor->set_voltage) < speed_error) {
        motor->motor_status->move_voltage(MAX_FLEWHEEL_MOTOR_VOLTAGE);        
    } 
    else {
        motor->motor_status->move_voltage(motor->set_voltage);
    }
}
/**
 * @brief           get functional device status. off/forward/backward
 * @param[in]       none
 * @retval          functional device status point
 */
functional_device_status_t *get_functional_device_status(void)
{
    return &functional_devices;
}