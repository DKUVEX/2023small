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
#include "dku/functional_task.hpp"
#include "dku/sensor_task.hpp"
#include "pros/rtos.hpp"
#include <cstdint>


auto_control_t auto_control;

/**
 * @brief           auto control init
 * @param[in,out]   init:
 * @retval          null
 */
void auto_init(auto_control_t* init);


/**
 * @brief           let robot turn to specific point
 * @param[in]       target_x: aimed position x
 * @param[in]       target_y: aimed position y
 * @param[in,out]   turn: find current position,give chassis voltage
 * @retval          null
 */
void turn_to(double target_x, double target_y, auto_control_t* turn);

/**
 * @brief           let robot turn to specific point
 * @param[in]       target_angle: aimed position x
 * @param[in,out]   turn: find current position,give chassis voltage
 * @retval          null
 */
void turn_relative(double target_angle, auto_control_t* turn);

/**
 * @brief           let robot turn to specific point
 * @param[in]       direction: direction, -1 or +1
 * @param[in]       time: a period of time
 * @param[in,out]   turn: find current position,give chassis voltage
 * @retval          null
 */
void turn_time(double direction, double time, auto_control_t* turn);

/**
 * @brief           let robot move to specific point
 * @param[in]       target_x: aimed position x
 * @param[in]       target_y: aimed position y
 * @param[in,out]   move: find current position,give chassis voltage
 * @retval          null
 */
void move_to(double target_x, double target_y, auto_control_t* move);

/**
 * @brief           let robot turn to specific point
 * @param[in]       direction: direction, -1 or +1
 * @param[in]       time: a period of time
 * @param[in,out]   turn: find current position,give chassis voltage
 * @retval          null
 */
void move_time(double direction, double time, auto_control_t* move);

/**
 * @brief           move a relative distance
 * @param[in]       target_distance: aimed distance
 * @param[in,out]   move: find current position,give chassis voltage
 * @retval          null
 */
void move_relative(double target_distance, auto_control_t* move);

/**
 * @brief           kick out 3 plates
 * @param[in,out]   kick: change the voltage of index
 * @retval          null
 */
void kick_out(auto_control_t* kick);


/**
 * @brief           auto control init
 * @param[in,out]   init:
 * @retval          null
 */
void auto_init(auto_control_t* init)
{

    init->chassis_voltage = get_chassis_voltage_point();
    init->functional_status = get_functional_device_status();
    init->sensor_data = get_sensor_data_point();
    pros::Mutex init_mutex;
    init_mutex.take();
    {
        // printf("here2");
        init->functional_status->flywheel = E_FLYWHEEL_STATUS_OFF;
        // init->functional_status->intake_motor = E_FUNCTIONAL_MOTOR_STATUS_FORWARD;
        // init->functional_status->roller_motor = E_FUNCTIONAL_MOTOR_STATUS_BACKWARD;
    }
    init_mutex.give();
    // printf("here3");
    // init->current_pos.current_x = 
    // init->current_pos.current_y = 
    // init->current_pos.current_dir = 
}

/**
 * @brief           let robot turn to specific point
 * @param[in]       target_x: aimed position x
 * @param[in]       target_y: aimed position y
 * @param[in,out]   turn: find current position,give chassis voltage
 * @retval         
 */
void turn_to(double target_x, double target_y, auto_control_t* turn)
{
    std::int32_t analog_right_x = 0; //simulate the joystick
    // double rad = atan2f((target_x - turn->current_pos.current_x), (target_y - turn->current_pos.current_y));
    double rad = atan2f((target_x - turn->sensor_data->gps_front_data.gps_pos.x), (target_y - turn->sensor_data->gps_front_data.gps_pos.y));
    // double rad = atan2f((target_x - turn->sensor_data->gps_front_data.gps_pointer->get_status().x), (target_y - turn->sensor_data->gps_front_data.gps_pointer->get_status().y));
    double angle = rad*(180/PI);
    pros::lcd::print(0, "heading: ", turn->sensor_data->gps_front_data.gps_pos.yaw); 
    // printf("heading: %lf\n", turn->sensor_data->gps_front_data.gps_pos.yaw);
    // while (abs((int)(turn->current_pos.current_dir - angle))>=1) {
    pros::Mutex turn_mutex;
    std::int32_t angle_difference = abs((int)(turn->sensor_data->gps_front_data.gps_pos.yaw - angle));
    // std::int32_t angle_difference = abs((int)(turn->sensor_data->gps_front_data.gps_pointer->get_status().yaw - angle));
    if (angle_difference > 180) {
        angle_difference = 360 - angle_difference;
    }
    std::uint32_t now_1 = pros::millis();
    while (angle_difference>=3) {

        analog_right_x = 30;
        turn_mutex.take();
        {
            turn->chassis_voltage[0] = analog_right_x;
            turn->chassis_voltage[1] = analog_right_x;
            turn->chassis_voltage[2] = -analog_right_x;
            turn->chassis_voltage[3] = -analog_right_x;
        }
        turn_mutex.give();
        pros::lcd::print(2, "ANGLE: %lf", angle); 
        printf("angle %5.5lf, yaw %5.5lf, yaw-angle %5d\n", angle, turn->sensor_data->gps_front_data.gps_pos.yaw, angle_difference);
        // printf("angle %5.5lf, yaw %5.5lf, yaw-angle %5d\n", angle, turn->sensor_data->gps_front_data.gps_pointer->get_status().yaw, angle_difference);
        rad = atan2f((target_x - turn->sensor_data->gps_front_data.gps_pos.x), (target_y - turn->sensor_data->gps_front_data.gps_pos.y));
        angle = rad*(180/PI);
        angle_difference = abs((int)(turn->sensor_data->gps_front_data.gps_pos.yaw - angle));
        // angle_difference = abs((int)(turn->sensor_data->gps_front_data.gps_pointer->get_status().yaw - angle));
        if (angle_difference > 180) {
            angle_difference = 360 - angle_difference;
        }
        pros::Task::delay_until(&now_1, AUTO_TASK_TIME_MS);
    }
    printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    turn_mutex.take();
    {
        turn->chassis_voltage[0] = 0;
        turn->chassis_voltage[1] = 0;
        turn->chassis_voltage[2] = 0;
        turn->chassis_voltage[3] = 0;
    }
    turn_mutex.give();
}

/**
 * @brief           let robot turn a relative angle
 * @param[in]       target_angle: aimed position x
 * @param[in,out]   turn: find current position,give chassis voltage
 * @retval          null
 */
void turn_relative(double target_angle, auto_control_t* turn)
{
    printf("turning");
    double angle = 0;
    std::int32_t analog_right_x = 0; //simulate the joystick
    analog_right_x = 30;
    pros::Mutex turn_mutex;
    double yaw_gyro = 0;
    std::uint32_t now_0 = pros::millis();
    while (angle < target_angle) {
        turn_mutex.take();
        {
            turn->chassis_voltage[0] = analog_right_x;
            turn->chassis_voltage[1] = analog_right_x;
            turn->chassis_voltage[2] = -analog_right_x;
            turn->chassis_voltage[3] = -analog_right_x;
        }
        yaw_gyro = -turn->sensor_data->gps_front_data.gps_gyro.z;
        pros::lcd::print(3, "z: %lf", turn->sensor_data->gps_front_data.gps_gyro.z);
        pros::lcd::print(4, "x: %lf", turn->sensor_data->gps_front_data.gps_gyro.x);
        pros::lcd::print(5, "y: %lf", turn->sensor_data->gps_front_data.gps_gyro.y);
        pros::lcd::print(6, "time: %d", pros::millis());

        angle += yaw_gyro * (AUTO_TASK_TIME_MS/100.0);
        printf("angle %lf", angle);
        pros::lcd::print(2, "angle: %lf", angle);
        // pros::Task::delay_until(&now_0, AUTO_TASK_TIME_MS);
    }
    turn_mutex.take();
    {
        turn->chassis_voltage[0] = 0;
        turn->chassis_voltage[1] = 0;
        turn->chassis_voltage[2] = 0;
        turn->chassis_voltage[3] = 0;
    }
    turn_mutex.give();
}

/**
 * @brief           let robot turn to specific point
 * @param[in]       direction: direction, -1 or +1
 * @param[in]       time: a period of time, unit:ms
 * @param[in,out]   turn: find current position,give chassis voltage
 * @retval          null
 */
void turn_time(double direction, double time, auto_control_t* turn)
{
    std::int32_t analog_right_x = 127;
    pros::Mutex turn_mutex;
    turn_mutex.take();
    {
        turn->chassis_voltage[0] = analog_right_x*direction;
        turn->chassis_voltage[1] = analog_right_x*direction;
        turn->chassis_voltage[2] = -analog_right_x*direction;
        turn->chassis_voltage[3] = -analog_right_x*direction;
    }
    turn_mutex.give();
    pros::delay(time);
    analog_right_x = 0;
    turn_mutex.take();
    {
        turn->chassis_voltage[0] = analog_right_x*direction;
        turn->chassis_voltage[1] = analog_right_x*direction;
        turn->chassis_voltage[2] = -analog_right_x*direction;
        turn->chassis_voltage[3] = -analog_right_x*direction;
    }
    turn_mutex.give();
}
/**
 * @brief           let robot move to specific point
 * @param[in]       target_x: aimed position x
 * @param[in]       target_y: aimed position y
 * @param[in,out]   move: find current position,give chassis voltage
 * @retval         
 */
void move_to(double target_x, double target_y, auto_control_t* move)
{
    turn_to(target_x, target_y, move);
    std::int32_t analog_left_y = 0; //simulate the joystick
    // double distance = sqrt(pow((target_x - move->current_pos.current_x),2) + pow((target_y - move->current_pos.current_y),2));
    double distance = sqrt(pow((target_x - move->sensor_data->gps_front_data.gps_pos.x),2) + pow((target_y - move->sensor_data->gps_front_data.gps_pos.y),2));
    printf("distance %5.5lf\n", distance);

    pros::Mutex move_mutex;
    while (abs((int)(distance*100))>=10) {
        int analog_left_y = 50;
        move_mutex.take();
        {
            move->chassis_voltage[0] = analog_left_y;
            move->chassis_voltage[1] = analog_left_y;
            move->chassis_voltage[2] = analog_left_y;
            move->chassis_voltage[3] = analog_left_y;
        }
        move_mutex.give();
        double distance = sqrt(pow((target_x - move->sensor_data->gps_front_data.gps_pos.x),2) + pow((target_y - move->sensor_data->gps_front_data.gps_pos.y),2));
        printf("distance %5.5lf\n", distance);
    }
    printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
    move_mutex.take();
    {
        move->chassis_voltage[0] = 0;
        move->chassis_voltage[1] = 0;
        move->chassis_voltage[2] = 0;
        move->chassis_voltage[3] = 0;
    }
    move_mutex.give();
}

/**
 * @brief           let robot turn to specific point
 * @param[in]       direction: direction, -1 or +1
 * @param[in]       time: a period of time
 * @param[in,out]   turn: find current position,give chassis voltage
 * @retval          null
 */
void move_time(double direction, double time, auto_control_t* move)
{
    std::int32_t analog_left_y = 70;
    pros::Mutex turn_mutex;
    turn_mutex.take();
    {
        move->chassis_voltage[0] = analog_left_y*direction;
        move->chassis_voltage[1] = analog_left_y*direction;
        move->chassis_voltage[2] = analog_left_y*direction;
        move->chassis_voltage[3] = analog_left_y*direction;
    }
    turn_mutex.give();
    pros::delay(time);
    analog_left_y = 0;
    turn_mutex.take();
    {
        move->chassis_voltage[0] = analog_left_y*direction;
        move->chassis_voltage[1] = analog_left_y*direction;
        move->chassis_voltage[2] = analog_left_y*direction;
        move->chassis_voltage[3] = analog_left_y*direction;
    }
    turn_mutex.give();
}

/**
 * @brief           move a relative distance
 * @param[in]       target_distance: aimed distance
 * @param[in,out]   move: find current position,give chassis voltage
 * @retval          null
 */
void move_relative(double target_distance, auto_control_t* move)
{
    double v0 = 0;
    double vt = 0;
    double x = 0;
    double a = 0;
    pros::Mutex turn_mutex;
    std::int32_t analog_left_y = 70;
    std::uint32_t now_2 = pros::millis();
    while (x < target_distance) {
        turn_mutex.take();
        {
            move->chassis_voltage[0] = analog_left_y;
            move->chassis_voltage[1] = analog_left_y;
            move->chassis_voltage[2] = analog_left_y;
            move->chassis_voltage[3] = analog_left_y;
        }
        turn_mutex.give();
        a = -move->sensor_data->gps_front_data.gps_acc.y;
        v0 = vt;
        vt = v0 + a*(AUTO_TASK_TIME_MS/100.0);
        x += v0*(AUTO_TASK_TIME_MS/100.0) + 0.5*a*(AUTO_TASK_TIME_MS/100.0)*(AUTO_TASK_TIME_MS/100.0);
        // printf("distance %lf", x);
        pros::lcd::print(1, "distance: %lf", x);
        pros::lcd::print(2, "acc: %lf", move->sensor_data->gps_front_data.gps_acc.y);
        pros::Task::delay_until(&now_2, AUTO_TASK_TIME_MS);
    }
    turn_mutex.take();
    {
        move->chassis_voltage[0] = 0;
        move->chassis_voltage[1] = 0;
        move->chassis_voltage[2] = 0;
        move->chassis_voltage[3] = 0;
    }
    turn_mutex.give();
}

/**
  * @brief          auto task. auto task is not a while true loop
  * @param[in]      param: null
  * @retval         none
  */
/**
  * @brief          自动任务
  * @param[in]      param: 空
  * @retval         none
  */
void auto_task_fn(void* param)
{
    std::cout << "auto task runs" << std::endl;

    pros::Task::delay(AUTO_TASK_INIT_TIME);
    auto_init(&auto_control);
    // std::uint32_t now = pros::millis();
    // while (true) {
    //     turn_to(1.8,1.8,&auto_control);
    //     pros::Task::delay_until(&now, AUTO_TASK_TIME_MS);
    // }

    // turn_to(0,0,&auto_control);
    turn_relative(0.785, &auto_control);

    // turn_to(0,0,&auto_control);
    // std::uint32_t now_a = pros::millis();
    // // std::uint32_t now = pros::millis();
    // // while (true) {
    // //     pros::Task::delay_until(&now, AUTO_TASK_TIME_MS);
    // // }
    // move_time(FORWARD, 350, &auto_control);
    // pros::delay(1000);
    // pros::Mutex auto_mutes;
    // auto_mutes.take();
    //     auto_control.functional_status->roller_motor = E_FUNCTIONAL_MOTOR_STATUS_BACKWARD;
    // auto_mutes.give();
    // pros::delay(220);
    // auto_mutes.take();
    //     auto_control.functional_status->roller_motor = E_FUNCTIONAL_MOTOR_STATUS_OFF;
    // auto_mutes.give();
    // pros::delay(1000);
    // move_time(BACKWARD, 200, &auto_control);
    // pros::delay(500);
    // turn_time(FORWARD, 340, &auto_control);
    // pros::delay(500);
    // move_time(FORWARD, 1100, &auto_control);
    // pros::delay(1000);

    // auto_mutes.take();
    //     auto_control.functional_status->roller_motor = E_FUNCTIONAL_MOTOR_STATUS_BACKWARD;
    // auto_mutes.give();
    // pros::delay(200);
    // auto_mutes.take();
    //     auto_control.functional_status->roller_motor = E_FUNCTIONAL_MOTOR_STATUS_OFF;
    // auto_mutes.give();

    // move_time(BACKWARD, 600, &auto_control);
    // turn_time(BACKWARD, 290, &auto_control);

    // // auto_control.functional_status->extension_gpio = FUNCTIONAL_LIFT_LOW_STATE;
    // while (true) {
    //     std::uint32_t now_time_b = pros::millis();
    //     if ((now_time_b - now_a)>50*1000) {
    //         auto_mutes.take();
    //             auto_control.functional_status->extension_gpio = FUNCTIONAL_LIFT_LOW_STATE;
    //         auto_mutes.give();
    //     }
    // }

}
/**
 * @brief           kick out 3 plates
 * @param[in,out]   kick: change the voltage of index
 * @retval          null
 */
void kick_out(auto_control_t* kick)
{
    pros::Mutex kick_mutex;
    kick_mutex.take();
    {
        kick->functional_status->index_motor = E_FUNCTIONAL_MOTOR_STATUS_FORWARD;
    }
    kick_mutex.give();
    pros::delay(5000);
    kick_mutex.take();
    {
        kick->functional_status->index_motor = E_FUNCTIONAL_MOTOR_STATUS_OFF;
    }
    kick_mutex.give();
}

/**
 * @brief            get the current status struck(x,y,direction)
 * @param[out]       null
 * @return           current_status_t*
 * @retval           
 */
current_status_t* get_current_status_pointer(void)
{
    return &auto_control.current_pos;
}