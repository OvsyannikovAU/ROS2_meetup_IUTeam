#ifndef BNO085_IMU_H
#define BNO085_IMU_H

#include <stdbool.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>

// IMU configuration
#define I2C_PORT i2c0
#define SDA_PIN 0
#define SCL_PIN 1
#define BNO085_ADDRESS 0x4A

// IMU data structure
typedef struct {
    float quat_w, quat_x, quat_y, quat_z;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    float mag_x, mag_y, mag_z;  // Magnetometer values (μT)
    float heading;              // Compass heading (radians)
    float yaw;                  // Yaw angle (radians)
} IMUData;

// Function declarations
bool imu_init(rcl_node_t* node, rclc_support_t* support);
void imu_read();
void imu_publish();
void imu_cleanup();

#endif