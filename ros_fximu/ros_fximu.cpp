#include "fximu.h"
#include "ros_fximu.h"
#include "fximu/SensorData.h"

ros::NodeHandle nh;

fximu::SensorData msg_sensor;
ros::Publisher pub_sensor("imu/sensor_data", &msg_sensor);

ros::Time current_time;
ros::Time marked_time;

int main(void) {

    // initialize tiva-c @ 80mhz
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // init ports, hardware reset of sensor.
    init_system();

    // init i2c
    init_I2C2();

    // init interrupts
    init_interrupts();

    // TODO: fix this.
    // TODO: in hybrid mode, accelmag must be *2
    // initialize sensors
    init_sensors(AFSR_4G, GFSR_250PS, ODR_400HZ, ODR_200HZ);

    // init node
    /*
    nh.initNode();

    // advertise publisher
    nh.advertise(pub_sensor);
*/
    // Wait for connection to establish
    /*
    while(!nh.connected()) {
      nh.spinOnce();
      nh.getHardware()->delay(10);
    }*/

    // connection state
    bool nh_prev_state = false;

    while(1) {

        //current_time = nh.now();

        if(gyro_data_ready) {
            GyroGetData(FXAS21002C_ADDRESS, &gyroRD);
            AGGetData(FXOS8700_ADDRESS, &accelRD, &magRD);
            gyro_data_ready = false;
        }

        if(accelmag_data_ready) {
            //GyroGetData(FXAS21002C_ADDRESS, &gyroRD);
            AGGetData(FXOS8700_ADDRESS, &accelRD, &magRD);
            accelmag_data_ready = false;
        }

/*
        if(nh.connected() && !nh_prev_state) {
            nh_prev_state = true;
            nh.spinOnce();
            nh.getHardware()->delay(10);
            continue;
        }

        if(!nh.connected() && nh_prev_state) {
          nh_prev_state = false;
          nh.spinOnce();
          nh.getHardware()->delay(10);
          continue;
        }
*/
/*
        if(data_ready) {

            current_time = nh.now();

            double dt = current_time.toSec() - marked_time.toSec();
            marked_time = current_time;

            msg_sensor.ax = accelRD.x;
            msg_sensor.ay = accelRD.y;
            msg_sensor.az = accelRD.z;

            msg_sensor.gx = gyroRD.x;
            msg_sensor.gy = gyroRD.y;
            msg_sensor.gz = gyroRD.z;

            msg_sensor.mx = magRD.x;
            msg_sensor.my = magRD.y;
            msg_sensor.mz = magRD.z;

            pub_sensor.publish(&msg_sensor);
            nh.spinOnce();

        }*/
    }
}