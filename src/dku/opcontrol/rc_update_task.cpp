#include "dku/opcontrol/rc_update_task.hpp"
#include "dku/chassis_task.hpp"
#include "pros/rtos.hpp"

rc_update_t controller_update;

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
static void rc_update_init(rc_update_t* init);

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
static void rc_update_init(rc_update_t* init)
{
    init->update_RC = get_remote_control_point();
    init->chassis_voltage = get_chassis_voltage_point();
}

/**
  * @brief          rc update task, osDelay RC_UPDATE_CONTROL_TIME_MS (2ms) 
  * @param[in]      param: null
  * @retval         none
  */
/**
  * @brief          底盘任务，间隔 RC_UPDATE_CONTROL_TIME_MS 2ms
  * @param[in]      param: 空
  * @retval         none
  */
void rc_update_task_fn(void* param) 
{
    std::cout << "rc update task runs" << std::endl;
    pros::Task::delay(RC_UPDATE_TASK_INIT_TIME);
    rc_update_init(&controller_update);
    std::uint32_t now = pros::millis();
    while (true) {
        // Do opcontrol things
        pros::Mutex chassis_mutex;
        chassis_mutex.take();
        controller_update.chassis_voltage[0] = controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)
                                    +controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)
                                    +controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        controller_update.chassis_voltage[1] = controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)
                                    -controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)
                                    +controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        controller_update.chassis_voltage[2] = controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)
                                    -controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)
                                    -controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);        
        controller_update.chassis_voltage[3] = controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)
                                    +controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)
                                    -controller_update.update_RC->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);        
        chassis_mutex.give();
        // printf("%d\n", chassis_motor_voltage[0] );
        pros::Task::delay_until(&now, CHASSIS_CONTROL_TIME_MS);
    }
}