#include "pros/motors.h"
#include <sys/types.h>
#undef __ARM_NEON__
#undef __ARM_NEON

#include "dku/sensor_task.hpp"
#include "eigen/Eigen/Dense"
#include "pros/gps.hpp"
#include "pros/llemu.hpp"
#include <chrono>
#include <iostream>
#include <ostream>
#include "eigen/Eigen/src/Core/Matrix.h"
#include "eigen/Eigen/src/Core/GlobalFunctions.h"
#include "dku/algorithm/positioning.hpp"
#include <thread>

using namespace std;
sensor_data_t *sensor;
class GPS_data{
    private:
      double xx, yy, ax, ay;
      Vector<double, 2> gps_data_1; 
      Vector<double, 2> gps_acc;
    public:
      GPS_data(){
        xx=0;
        yy=0; ax=0; ay=0;
        gps_data_1 << xx, yy;
        gps_acc << ax, ay;
      }
      void setGPS(double a, double b){
        xx=a;
        yy=b;
        gps_data_1 << a, b;
      }
      Vector<double, 2>& get_GPS_data(){
        return gps_data_1;
      }
      void setACC(double a, double b){
        xx=a;
        yy=b;
        gps_acc<< a, b;
      }
      Vector<double, 2>& get_ACC_data(){
        return gps_acc;
      }
};


Vector<double, 4> stimated_location_1;
Vector<double, 4> get_stimated_location(){
    return stimated_location_1;
}
void set_stimated_location(Vector<double, 4> v){
    stimated_location_1=v;
}

void tracking_1_fn(void* param) {
    sensor = get_sensor_data_point();
    GPS_data newData;
    Vector<double, 4> stimated_location;

    std::uint32_t now = pros::millis();
    // cout << "location system works" << endl;
    newData.setGPS(0, 0);
    Matrix<double, 2, 2> angular_velocity;

    while (true) {

        newData.setACC(get_sensor_data_point()->gps_front_data.gps_acc.x, 
        get_sensor_data_point()->gps_front_data.gps_acc.y);
                angular_velocity << cos(0.01*get_sensor_data_point()->gps_front_data.gps_gyro.z), 
        -sin(0.01*get_sensor_data_point()->gps_front_data.gps_gyro.z), 
        sin(0.01*get_sensor_data_point()->gps_front_data.gps_gyro.z), 
        cos(0.01*get_sensor_data_point()->gps_front_data.gps_gyro.z);

        Vector<double, 2> zz = newData.get_GPS_data();
        kalmanFilter(stimated_location, newData.get_ACC_data(), zz, angular_velocity);
        newData.setGPS(stimated_location.w(), stimated_location.x());
        set_stimated_location(stimated_location);

        // cout << stimated_location << endl;
        // cout << "hola" << endl;
        // cout << get_sensor_data_point()->gps_front_data.gps_acc.x << endl;
        // cout << get_sensor_data_point()->gps_front_data.gps_acc.y << endl;
        // cout << "hola2" << endl;
        // Do opcontrol things
            pros::Task::delay_until(&now, 10);
    }

}

// int main(){
//     tracking();
//     return 0;
// }