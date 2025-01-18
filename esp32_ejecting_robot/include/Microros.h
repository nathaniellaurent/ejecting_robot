#include "Error.h"
#include "Motor.h"
#include <micro_ros_platformio.h>

#include <MPU6500_WE.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/vector3.h>

#define MPU6500_ADDR 0x68

#define AIN1 12
#define AIN2 13
#define BIN1 16
#define BIN2 17

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            error_loop();            \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }

// Error handle loop


const IPAddress agent_ip(192, 168, 193, 195);
const size_t agent_port = 8888;

const char ssid[] = "Pixel_5317";
const char psk[] = "ce53432524";

class Microros
{

public:
    static void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
    static void axes_callback(const void *msgin);
    static void buttons_callback(const void *msgin);
    static void setup();
    static void spin_nodes();
    static bool ping();

    static std::shared_ptr<MPU6500_WE> myMPU6500;

private:
    static void error_loop();
    static rcl_publisher_t resultG_publisher;
    static rcl_publisher_t accel_publisher;
    static rcl_publisher_t gyro_publisher;
    static rcl_publisher_t out_publisher;

    static rcl_subscription_t buttons_subscription;
    static rcl_subscription_t axes_subscription;

    static std_msgs__msg__Float32 resultG_msg_float;
    static geometry_msgs__msg__Vector3 accel_msg;
    static geometry_msgs__msg__Vector3 gyro_msg;
    static std_msgs__msg__Int32 out_msg;

    static std_msgs__msg__Int32MultiArray buttons_msg;
    static std_msgs__msg__Float32MultiArray axes_msg;
    static rclc_executor_t timer_executor;

    static rclc_executor_t buttons_executor;
    static rclc_executor_t axes_executor;
    static rclc_support_t support;
    static rcl_allocator_t allocator;
    static rcl_node_t node;
    static rcl_timer_t timer;

    static Motor motor1;
    static Motor motor2;
};