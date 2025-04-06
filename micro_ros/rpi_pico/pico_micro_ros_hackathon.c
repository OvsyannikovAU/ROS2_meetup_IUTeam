#include <stdio.h>
#include <math.h>
#include <time.h>    // For clock() if using standard C

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl_interfaces/msg/log.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/float32.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/rmw_microros.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#define USE_IMU false  // Set to false to disable IMU

// Физические параметры робота
#define WHEEL_RADIUS 0.067/2
const float RPM_TO_MPS = 2 * M_PI * WHEEL_RADIUS / 60.0f; 
#define WHEEL_BASE 0.200
#define M_PI 3.14159265358979323846

#if USE_IMU
#include "bno085_imu.h"
#endif

#include "motor_controller.h"

// Log severity levels (from rcl_interfaces/msg/Log.h)
#define LOG_UNSET 0
#define LOG_DEBUG 10
#define LOG_INFO 20
#define LOG_WARN 30
#define LOG_ERROR 40
#define LOG_FATAL 50

const uint LED_PIN = 25;

const uint LEFT_MOTOR_PWM_PIN = 22;
const uint LEFT_MOTOR_DIR1_PIN = 20;
const uint LEFT_MOTOR_DIR2_PIN = 21;
const uint LEFT_ENC_A_PIN = 12;
const uint LEFT_ENC_B_PIN = 13;

const uint RIGHT_MOTOR_PWM_PIN = 16;
const uint RIGHT_MOTOR_DIR1_PIN = 18;
const uint RIGHT_MOTOR_DIR2_PIN = 17;
const uint RIGHT_ENC_A_PIN = 14;
const uint RIGHT_ENC_B_PIN = 15;

const uint MOTORS_STBY_PIN = 19;


// Новый объект для хранения текущей позиции для одометрии
float x = 0.0f;		// Позиция по X (м)
float y = 0.0f;		// Позиция по Y (м)
float theta = 0.0f; // Ориентация (рад)

// Ориентация по умолчанию (в 3D)
const signed char orientationDefault[9] = {0, 1, 0, 0, 0, 1, 1, 0, 0};

// Global motor instances
Motor left_motor;
Motor right_motor;

// Global variables for motor control
PIDParams pid_params = {2.0, 0.0, 0.06, 0.0}; // Default values similar to turtle example
float target_linear_speed= 0.0f;
float target_angular_speed= 0.0f;

// Publishers and messages
rcl_publisher_t log_publisher;
rcl_interfaces__msg__Log log_msg;

// Subscribers and their messages
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_subscription_t pid_params_subscriber;
std_msgs__msg__Float32MultiArray pid_params_msg;

rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;

void publish_log(const char* message, const char* file, const char* function, uint8_t level) {
    // Set message fields
    log_msg.msg.data = (char*)message;
    log_msg.msg.size = strlen(message);
    log_msg.file.data = (char*)file;
    log_msg.file.size = strlen(file);
    log_msg.function.data = (char*)function;
    log_msg.function.size = strlen(function);
    log_msg.level = level;
    
    // Set timestamp
    int64_t current_time = rmw_uros_epoch_nanos();
    log_msg.stamp.sec = current_time / 1000000000;
    log_msg.stamp.nanosec = current_time % 1000000000;
    
    // Publish
    rcl_publish(&log_publisher, &log_msg, NULL);
}

void status_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    static int count = 0;
    char status_msg[64];
    snprintf(status_msg, sizeof(status_msg), "System status OK - uptime: %d", count++);
    publish_log(status_msg, __FILE__, __func__, LOG_DEBUG); // DEBUG level
}

void init_motors() {
    // Initialize left motor with correct pin numbers
    Motor_Init(&left_motor, 
        LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR1_PIN, LEFT_MOTOR_DIR2_PIN,
        LEFT_ENC_A_PIN, LEFT_ENC_B_PIN);

    // Initialize right motor with correct pin numbers
    Motor_Init(&right_motor,
            RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR1_PIN, RIGHT_MOTOR_DIR2_PIN,
            RIGHT_ENC_A_PIN, RIGHT_ENC_B_PIN);

    // Set PID parameters
    Motor_SetPID(&left_motor, &pid_params);
    Motor_SetPID(&right_motor, &pid_params);

    // Enable motor driver
    gpio_init(MOTORS_STBY_PIN);
    gpio_set_dir(MOTORS_STBY_PIN, GPIO_OUT);
    gpio_put(MOTORS_STBY_PIN, 1);
}

// Velocity command callback
void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    
    target_linear_speed = msg->linear.x; // Линейная скорость (м/с)
    target_angular_speed = msg->angular.z; // Угловая скорость (рад/с)
    
    // Вычисляем целевые скорости колес
    float target_left_rpm = (target_linear_speed - target_angular_speed * WHEEL_BASE / 2.0) / RPM_TO_MPS;
    float target_right_rpm = (target_linear_speed + target_angular_speed * WHEEL_BASE / 2.0) / RPM_TO_MPS;

    Motor_SetTargetRPM(&left_motor, target_left_rpm);
    Motor_SetTargetRPM(&right_motor, target_right_rpm);

	char str[256];
	sprintf(str, "cmd_vel_callback(), target linear: %.2f, target angular: %.2f, target left RPM: %.2f, target right RPM: %.2f", target_linear_speed, target_angular_speed, target_left_rpm, target_right_rpm);
	publish_log(str, __FILE__, __func__, LOG_INFO); // INFO level
}

// Преобразование угла в кватернион
void set_orientation(geometry_msgs__msg__Quaternion orientation, double theta)
{
	// Вычисляем компоненты кватерниона для поворота вокруг оси Z (в 2D)
	double qx = 0.0;
	double qy = 0.0;
	double qz = sin(theta / 2.0);
	double qw = cos(theta / 2.0);

	// Устанавливаем значения в сообщение
	orientation.x = qx;
	orientation.y = qy;
	orientation.z = qz;
	orientation.w = qw;
}

volatile float last_time = 0.0f;

void odometry_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    int64_t current_time = rmw_uros_epoch_nanos();
    float sec = (float) current_time / 1000000000.0;

    float dt = (sec - last_time);  // seconds
    last_time = sec;
    
    // Get current speeds
    float left_rpm = Motor_GetCurrentRPM(&left_motor);
    float right_rpm = Motor_GetCurrentRPM(&right_motor);

    // Get current encs
    float left_enc = Motor_GetCurrentENC(&left_motor);
    float right_enc= Motor_GetCurrentENC(&right_motor);
    
    // Calculate correction (if using speed matching)
    float correction =0.0;
    // float correction = speedMatchingController.compute(left_rpm, right_rpm, 
    //                  Motor_GetTargetRPM(&left_motor), Motor_GetTargetRPM(&right_motor));
    
    // // Update motors with correction
    Motor_Update(&left_motor, dt, -correction);
    Motor_Update(&right_motor, dt, correction);
    
	if (timer != NULL)
	{
		// Конвертируем RPM в линейные скорости
		float left_velocity = RPM_TO_MPS * left_rpm;   // Левое колесо (м/с)
		float right_velocity = RPM_TO_MPS * right_rpm; // Правое колесо (м/с)

		// Вычисляем линейную и угловую скорость робота
		float linear_velocity = (left_velocity + right_velocity) / 2.0f;
		float angular_velocity = (right_velocity - left_velocity) / WHEEL_BASE;

		// Интегрируем для вычисления новой позиции
		float delta_theta = angular_velocity * (dt);
		theta += delta_theta;
		theta = fmod(theta + 2 * M_PI, 2 * M_PI); // Нормализация угла

		float delta_x = linear_velocity * cos(theta) * (dt);
		float delta_y = linear_velocity * sin(theta) * (dt);

		x += delta_x;
		y += delta_y;

		// Позиция
		odom_msg.pose.pose.position.x = x;
		odom_msg.pose.pose.position.y = y;
		odom_msg.pose.pose.position.z = 0.0f;

		// Ориентация (в формате кватерниона)
		set_orientation(odom_msg.pose.pose.orientation, theta);

		// Скорости
		odom_msg.twist.twist.linear.x = linear_velocity;
		odom_msg.twist.twist.linear.y = 0.0f; // В нашем случае скорости в Y нет
		odom_msg.twist.twist.angular.z = angular_velocity;

		char str[256];
		sprintf(str, "left RPM: %.2f, right RPM: %.2f, position (x: %.2f, y: %.2f, theta: %.2f), linear velocity: %.2f, angular velocity: %.2f",
					left_rpm, right_rpm, x, y, theta, linear_velocity, angular_velocity);
        publish_log(str, __FILE__, __func__, LOG_INFO); // INFO level

        sprintf(str, "left ENC: %.2f, right ENC: %.2f",
            left_enc, right_enc);
        publish_log(str, __FILE__, __func__, LOG_INFO); // INFO level

        rcl_publish(&odom_publisher, &odom_msg, NULL);
	}
}


void init_msgs_pid_params() {
    pid_params_msg.data.capacity = 9;
    pid_params_msg.data.size = 9;
    
    // Allocate memory for the float array (replaces 'new float[...]')
    pid_params_msg.data.data = (float*)malloc(pid_params_msg.data.capacity * sizeof(float));
    
    // Check if allocation succeeded (good practice in C)
    if (pid_params_msg.data.data == NULL) {
        // Handle allocation failure (e.g., log an error, set size/capacity to 0)
        pid_params_msg.data.size = 0;
        pid_params_msg.data.capacity = 0;
        return;
    }
}

// PID parameters callback (similar to turtle example)
void pid_params_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;

    if (msg->data.size == 4) {
        pid_params.kp = msg->data.data[0];
        pid_params.ki = msg->data.data[1];
        pid_params.kd = msg->data.data[2];
        pid_params.kff = msg->data.data[3];
        
        // Set PID parameters
        Motor_SetPID(&left_motor, &pid_params);
        Motor_SetPID(&right_motor, &pid_params);
        
        // Blink LED twice to indicate PID update
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);

        char str[256];
		sprintf(str, "Pid update: MotorA [Kp=%.2f, Ki=%.2f, Kd=%.2f, Kff=%.2f]", 
            pid_params.kp , pid_params.ki, pid_params.kd, pid_params.kff);
        publish_log(str, __FILE__, __func__, LOG_INFO); // INFO level 
        }
    else {
        char str[256];
        sprintf(str, "Invalid PID parameters received: size=%zu", msg->data.size);
        publish_log(str, __FILE__, __func__, LOG_INFO); // INFO level 
    }
    
}

// Initialize PID parameters message
#include <stdlib.h>  // For malloc/free
#include <string.h>  // For memcpy/strlen

void init_msgs_odometry() {
    // Initialize the string (replacing 'new char[]' with malloc)
    odom_msg.header.frame_id.capacity = 12;
    odom_msg.header.frame_id.size = 11;
    odom_msg.header.frame_id.data = (char*)malloc(odom_msg.header.frame_id.capacity);
    if (odom_msg.header.frame_id.data == NULL) {
        // Handle allocation failure (optional)
        return;
    }
    
    // Copy the string "odom" into the allocated buffer
    const char* odom_str = "odom";
    strncpy(odom_msg.header.frame_id.data, odom_str, odom_msg.header.frame_id.capacity);
    odom_msg.header.frame_id.size = strlen(odom_str); // Update size to actual length

    // Set covariance values (unchanged from C++)
    odom_msg.pose.covariance[0] = 0.1;
    odom_msg.pose.covariance[7] = 0.1;
    odom_msg.pose.covariance[14] = 1e-3;
    odom_msg.pose.covariance[21] = 1e-3;
    odom_msg.pose.covariance[28] = 1e-3;
    odom_msg.pose.covariance[35] = 0.1;

    odom_msg.twist.covariance[0] = 0.1;
    odom_msg.twist.covariance[7] = 0.1;
    odom_msg.twist.covariance[14] = 1e-3;
    odom_msg.twist.covariance[21] = 1e-3;
    odom_msg.twist.covariance[28] = 1e-3;
    odom_msg.twist.covariance[35] = 0.1;
}


int main()
{
    // Micro-ROS initialization
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // Initialize hardware
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize log message
    log_msg.msg.capacity = 256;
    log_msg.msg.size = 0;
    log_msg.msg.data = (char*)malloc(log_msg.msg.capacity);
    log_msg.file.capacity = 64;
    log_msg.file.size = 0;
    log_msg.file.data = (char*)malloc(log_msg.file.capacity);
    log_msg.function.capacity = 64;
    log_msg.function.size = 0;
    log_msg.function.data = (char*)malloc(log_msg.function.capacity);


    // Timer handles
    rcl_timer_t timer;
    rcl_timer_t odom_timer;

    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    
    rmw_qos_profile_t rosout_qos = rmw_qos_profile_services_default;
	rosout_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
	rosout_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

    // Initialize publisher
    rclc_publisher_init(
        &log_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
        "/rosout", &rosout_qos);

    // Initialize timer
    rclc_timer_init_default(
        &odom_timer,
        &support,
        RCL_MS_TO_NS(200), // 0.2 second interval
        odometry_timer_callback);
    
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(5000), // 5 second interval
        status_timer_callback);

    // Initialize cmd_vel subscriber
    rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");

    rmw_qos_profile_t pid_params_qos = rmw_qos_profile_default;
    pid_params_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    pid_params_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    // Initialize PID params subscriber
    init_msgs_pid_params();

    rclc_subscription_init(
        &pid_params_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/pid_params", &pid_params_qos);
    
    
    init_msgs_odometry();
    rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom"
    );

    // IMU publisher
    #if USE_IMU
        if (!imu_init(&node, &support)) {
            publish_log("IMU initialization failed", __FILE__, __func__, LOG_ERROR);
            rcl_timer_t imu_timer;
            rclc_executor_init(&executor, &support.context, 10, &allocator);
        }
    #else
        rclc_executor_init(&executor, &support.context, 6, &allocator);
    #endif


    // Create executor with 3 handles (timer + subscribers + publishers)
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_timer(&executor, &odom_timer);
    rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &pid_params_subscriber, &pid_params_msg, &pid_params_callback, ON_NEW_DATA);

    gpio_put(LED_PIN, 1);
    
    // main part of work
    init_motors();

    // Timing variables for control loop
    uint64_t last_control_time = time_us_64();
    const uint32_t control_interval_us = 10000; // 10ms control loop (100Hz)

    while (true) {
        // Get current time
        uint64_t current_time = time_us_64();
        
        // Motor control update at fixed intervals
        if (current_time - last_control_time >= control_interval_us) {
            float dt = (current_time - last_control_time) / 1e6f; // Convert to seconds
            last_control_time = current_time;

            // Update motors without correction (0.0f) since odometry isn't ready
            Motor_Update(&left_motor, dt, 0.0f);
            Motor_Update(&right_motor, dt, 0.0f);
        }

        // Handle ROS communication
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        
        // Optional: Add a small delay to prevent busy waiting
        sleep_us(1000);
    }
    
    // Cleanup (though we'll never get here in this example)
    #if USE_IMU
        imu_cleanup();
    #endif
    return 0;
}