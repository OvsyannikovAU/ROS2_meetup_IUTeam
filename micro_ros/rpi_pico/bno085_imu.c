#include "bno085_imu.h"
#include <math.h>
#include <string.h>


// BNO085 Register Definitions
#define BNO085_CHANNEL_COMMAND 0x00
#define BNO085_CHANNEL_REPORTS 0x01
#define BNO085_GET_FEATURE_RESPONSE 0xFC
#define BNO085_SHTP_REPORT_PRODUCT_ID 0x01
#define BNO085_SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define BNO085_SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define BNO085_SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define BNO085_SENSOR_REPORTID_GYROSCOPE 0x03
#define BNO085_SENSOR_REPORTID_ACCELEROMETER 0x01
#define BNO085_SENSOR_REPORTID_MAGNETOMETER 0x04
#define BNO085_SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Conversion factors
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f/M_PI)
#define MG_TO_MS2 (9.80665f / 1000.0f)
#define Q14_TO_FLOAT (1.0f / 16384.0f)
#define Q9_TO_FLOAT (1.0f / 64.0f)

// Global variables
static IMUData imu_data;
static bool imu_initialized = false;

// Publishers
static rcl_publisher_t imu_publisher;
static rcl_publisher_t yaw_publisher;
static rcl_publisher_t compass_publisher;
static sensor_msgs__msg__Imu imu_msg;
static std_msgs__msg__Float32 yaw_msg;
static std_msgs__msg__Float32 compass_msg;

// ... [include all your IMU-related functions here]
// init_i2c(), bno085_detect(), bno085_send_command(), 
// bno085_read_response(), read_bno085(), etc.
// Initialize I2C for BNO085
void init_i2c() {
    i2c_init(I2C_PORT, 400 * 1000); // 400kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

// Check if BNO085 is connected
bool bno085_detect() {
    uint8_t data;
    int ret = i2c_read_blocking(I2C_PORT, BNO085_ADDRESS, &data, 1, false);
    return ret == 1;
}

// Function to send command to BNO085
bool bno085_send_command(uint8_t command, uint16_t data_length, uint8_t *data) {
    uint8_t packet[data_length + 4];
    packet[0] = data_length & 0xFF;
    packet[1] = (data_length >> 8) & 0xFF;
    packet[2] = BNO085_CHANNEL_COMMAND;
    packet[3] = command;
    
    if(data_length > 0) {
        memcpy(&packet[4], data, data_length);
    }
    
    return i2c_write_blocking(I2C_PORT, BNO085_ADDRESS, packet, data_length + 4, false) == data_length + 4;
}

// Initialize BNO085
bool init_bno085() {
    char status_msg[64];

    if (!bno085_detect()) {
        // printf("BNO085 not detected at address 0x%X\n", BNO085_ADDRESS);
        snprintf(status_msg, sizeof(status_msg), "BNO not detected - uptime: %d");
        publish_log(status_msg, __FILE__, __func__, 2); // 2 = ERROR level
        return false;
    }

    // Enable magnetometer (compass)
    uint8_t mag_config[] = {
        BNO085_SENSOR_REPORTID_MAGNETOMETER,  // Report ID (0x04)
        0x00,                                 // Report interval (0 = on change)
        0x00, 0x00, 0x00, 0x00               // Specific config (unused)
    };
    
    if(!bno085_send_command(BNO085_GET_FEATURE_RESPONSE, sizeof(mag_config), mag_config)) {
        // printf("Failed to enable magnetometer\n");
        snprintf(status_msg, sizeof(status_msg), "BNO Failed to enable magnetometer - uptime: %d");
        publish_log(status_msg, __FILE__, __func__, 2); // 2 = ERROR level
        return false;
    }

    publish_log("BNO085 initialized successfully", __FILE__, __func__, 1); // 1 = INFO level

    return true;
}

// Function to read response from BNO085
bool bno085_read_response(uint8_t *buffer, uint16_t length) {
    uint8_t header[4];
    if(i2c_read_blocking(I2C_PORT, BNO085_ADDRESS, header, 4, false) != 4) {
        return false;
    }
    
    uint16_t data_length = (header[1] << 8) | header[0];
    if(data_length > length) {
        return false;
    }
    
    return i2c_read_blocking(I2C_PORT, BNO085_ADDRESS, buffer, data_length, false) == data_length;
}

// Actual BNO085 reading implementation
void read_bno085() {
    uint8_t response_buffer[32];
    
    // 1. Read Rotation Vector (Quaternion)
    uint8_t rotation_cmd[] = {BNO085_SENSOR_REPORTID_ROTATION_VECTOR, 0};
    if(bno085_send_command(BNO085_GET_FEATURE_RESPONSE, sizeof(rotation_cmd), rotation_cmd) &&
       bno085_read_response(response_buffer, sizeof(response_buffer))) {
        
        if(response_buffer[0] == BNO085_SENSOR_REPORTID_ROTATION_VECTOR && 
           response_buffer[1] >= 16) {  // Check minimum data length
            imu_data.quat_w = (int16_t)((response_buffer[8] << 8) | response_buffer[7]) * Q14_TO_FLOAT;
            imu_data.quat_x = (int16_t)((response_buffer[2] << 8) | response_buffer[1]) * Q14_TO_FLOAT;
            imu_data.quat_y = (int16_t)((response_buffer[4] << 8) | response_buffer[3]) * Q14_TO_FLOAT;
            imu_data.quat_z = (int16_t)((response_buffer[6] << 8) | response_buffer[5]) * Q14_TO_FLOAT;
        }
    }

    // 2. Read Gyroscope
    uint8_t gyro_cmd[] = {BNO085_SENSOR_REPORTID_GYROSCOPE, 0};
    if(bno085_send_command(BNO085_GET_FEATURE_RESPONSE, sizeof(gyro_cmd), gyro_cmd) &&
       bno085_read_response(response_buffer, sizeof(response_buffer))) {
        
        if(response_buffer[0] == BNO085_SENSOR_REPORTID_GYROSCOPE && 
           response_buffer[1] >= 6) {
            imu_data.gyro_x = (int16_t)((response_buffer[2] << 8) | response_buffer[1]) * Q9_TO_FLOAT * DEG_TO_RAD;
            imu_data.gyro_y = (int16_t)((response_buffer[4] << 8) | response_buffer[3]) * Q9_TO_FLOAT * DEG_TO_RAD;
            imu_data.gyro_z = (int16_t)((response_buffer[6] << 8) | response_buffer[5]) * Q9_TO_FLOAT * DEG_TO_RAD;
        }
    }

    // 3. Read Accelerometer
    uint8_t accel_cmd[] = {BNO085_SENSOR_REPORTID_ACCELEROMETER, 0};
    if(bno085_send_command(BNO085_GET_FEATURE_RESPONSE, sizeof(accel_cmd), accel_cmd) &&
       bno085_read_response(response_buffer, sizeof(response_buffer))) {
        
        if(response_buffer[0] == BNO085_SENSOR_REPORTID_ACCELEROMETER && 
           response_buffer[1] >= 6) {
            imu_data.accel_x = (int16_t)((response_buffer[2] << 8) | response_buffer[1]) * MG_TO_MS2;
            imu_data.accel_y = (int16_t)((response_buffer[4] << 8) | response_buffer[3]) * MG_TO_MS2;
            imu_data.accel_z = (int16_t)((response_buffer[6] << 8) | response_buffer[5]) * MG_TO_MS2;
        }
    }

     // 4. Read Magnetometer (Compass)
     uint8_t mag_cmd[] = {BNO085_SENSOR_REPORTID_MAGNETOMETER, 0};
     if(bno085_send_command(BNO085_GET_FEATURE_RESPONSE, sizeof(mag_cmd), mag_cmd) &&
        bno085_read_response(response_buffer, sizeof(response_buffer))) {
         
         if(response_buffer[0] == BNO085_SENSOR_REPORTID_MAGNETOMETER && 
            response_buffer[1] >= 6) {
             // Magnetometer data in μT (Q4 format)
             imu_data.mag_x = (int16_t)((response_buffer[2] << 8) | response_buffer[1]) / 16.0f;
             imu_data.mag_y = (int16_t)((response_buffer[4] << 8) | response_buffer[3]) / 16.0f;
             imu_data.mag_z = (int16_t)((response_buffer[6] << 8) | response_buffer[5]) / 16.0f;
             
             // Calculate compass heading (2D, ignoring Z-axis)
             imu_data.heading = atan2f(imu_data.mag_y, imu_data.mag_x);
             if(imu_data.heading < 0) {
                 imu_data.heading += 2*M_PI;  // Normalize to 0-2π
             }
         }
     }
     
     // 5. Read Game Rotation Vector (for yaw)
     uint8_t grv_cmd[] = {BNO085_SENSOR_REPORTID_GAME_ROTATION_VECTOR, 0};
     if(bno085_send_command(BNO085_GET_FEATURE_RESPONSE, sizeof(grv_cmd), grv_cmd) &&
        bno085_read_response(response_buffer, sizeof(response_buffer))) {
         
         if(response_buffer[0] == BNO085_SENSOR_REPORTID_GAME_ROTATION_VECTOR && 
            response_buffer[1] >= 8) {
             // Extract quaternion (Q14 format)
             float q1 = (int16_t)((response_buffer[2] << 8) | response_buffer[1]) * Q14_TO_FLOAT;
             float q2 = (int16_t)((response_buffer[4] << 8) | response_buffer[3]) * Q14_TO_FLOAT;
             float q3 = (int16_t)((response_buffer[6] << 8) | response_buffer[5]) * Q14_TO_FLOAT;
             float q0 = (int16_t)((response_buffer[8] << 8) | response_buffer[7]) * Q14_TO_FLOAT;
             
             // Convert quaternion to yaw (radians)
             imu_data.yaw = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
         }
     }
}


// Initialize IMU message
void init_imu_msg() {
    // Initialize the frame_id string properly
    imu_msg.header.frame_id.capacity = 10;  // Enough for "imu_link" + null terminator
    imu_msg.header.frame_id.size = strlen("imu_link");
    imu_msg.header.frame_id.data = (char*)malloc(imu_msg.header.frame_id.capacity);
    strcpy(imu_msg.header.frame_id.data, "imu_link");
    
    // Initialize covariance matrices (example values)
    for (int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i] = 0.0;
        imu_msg.angular_velocity_covariance[i] = 0.0;
        imu_msg.linear_acceleration_covariance[i] = 0.0;
    }
    // Set diagonal elements
    imu_msg.orientation_covariance[0] = 0.01;  // x
    imu_msg.orientation_covariance[4] = 0.01;  // y
    imu_msg.orientation_covariance[8] = 0.01;  // z
    
    imu_msg.angular_velocity_covariance[0] = 0.02;
    imu_msg.angular_velocity_covariance[4] = 0.02;
    imu_msg.angular_velocity_covariance[8] = 0.02;
    
    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[8] = 0.04;

    yaw_msg.data = 0.0f;
    compass_msg.data = 0.0f;
}



bool imu_init(rcl_node_t* node, rclc_support_t* support) {
    // Initialize I2C and BNO085
    init_i2c();
    imu_initialized = init_bno085();
    
    // Initialize publishers
    rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data_raw");
    
    // IMU YAW publish
    rclc_publisher_init_default(
        &yaw_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/imu/yaw");
    
    // IMU COMPASS publish
    rclc_publisher_init_default(
        &compass_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/imu/compass");
    
    return imu_initialized;
}

void imu_read() {
    if (imu_initialized) {
        read_bno085();
    }
}

void imu_publish() {
    if (!imu_initialized) return;
    
        // Fill IMU message
        imu_msg.orientation.w = imu_data.quat_w;
        imu_msg.orientation.x = imu_data.quat_x;
        imu_msg.orientation.y = imu_data.quat_y;
        imu_msg.orientation.z = imu_data.quat_z;
        
        imu_msg.angular_velocity.x = imu_data.gyro_x;
        imu_msg.angular_velocity.y = imu_data.gyro_y;
        imu_msg.angular_velocity.z = imu_data.gyro_z;
        
        imu_msg.linear_acceleration.x = imu_data.accel_x;
        imu_msg.linear_acceleration.y = imu_data.accel_y;
        imu_msg.linear_acceleration.z = imu_data.accel_z;
        
        // // Set header timestamp
        int64_t current_time = rmw_uros_epoch_nanos();
        imu_msg.header.stamp.sec = current_time / 1000000000;
        imu_msg.header.stamp.nanosec = current_time % 1000000000;
        
        // Publish IMU data
        rcl_publish(&imu_publisher, &imu_msg, NULL);

        compass_msg.data = imu_data.heading * RAD_TO_DEG;  // Now in degrees
        yaw_msg.data = imu_data.yaw * RAD_TO_DEG;
        // Publish yaw (deg)
        // yaw_msg.data = imu_data.yaw; // for radians
        rcl_publish(&yaw_publisher, &yaw_msg, NULL);
        
        // Publish compass heading (radians)
        // compass_msg.data = imu_data.heading; //for radians
        rcl_publish(&compass_publisher, &compass_msg, NULL);
}

void imu_cleanup() {
    // Clean up resources if needed
}