#include "Microros.h"

rcl_publisher_t Microros::resultG_publisher;
rcl_publisher_t Microros::accel_publisher;
rcl_publisher_t Microros::gyro_publisher;
rcl_publisher_t Microros::out_publisher;
rcl_subscription_t Microros::buttons_subscription;
rcl_subscription_t Microros::axes_subscription;

std_msgs__msg__Float32 Microros::resultG_msg_float;
geometry_msgs__msg__Vector3 Microros::accel_msg;
geometry_msgs__msg__Vector3 Microros::gyro_msg;
std_msgs__msg__Int32 Microros::out_msg;

std_msgs__msg__Int32MultiArray Microros::buttons_msg;
std_msgs__msg__Float32MultiArray Microros::axes_msg;
rclc_executor_t Microros::timer_executor;

rclc_executor_t Microros::buttons_executor;
rclc_executor_t Microros::axes_executor;
rclc_support_t Microros::support;
rcl_allocator_t Microros::allocator;
rcl_node_t Microros::node;
rcl_timer_t Microros::timer;

std::shared_ptr<MPU6500_WE> Microros::myMPU6500;

std::shared_ptr<Motor> Microros::motor1;
std::shared_ptr<Motor> Microros::motor2;

void Microros::setup(std::shared_ptr<MPU6500_WE> passedMPU, std::shared_ptr<Motor> passedMotor1, std::shared_ptr<Motor>  passedMotor2)
{
    Error::display_error(1);

    motor1 = passedMotor1;
    motor2 = passedMotor2;

    myMPU6500 = passedMPU;

    buttons_msg.data.capacity = 17;
    buttons_msg.data.size = 0;
    buttons_msg.data.data = (int32_t *)malloc(buttons_msg.data.capacity * sizeof(int32_t));

    buttons_msg.layout.dim.capacity = 1;
    buttons_msg.layout.dim.size = 0;
    buttons_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension *)malloc(buttons_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

    buttons_msg.layout.dim.data[0].label.capacity = 10;
    buttons_msg.layout.dim.data[0].label.size = 0;
    buttons_msg.layout.dim.data[0].label.data = (char *)malloc(buttons_msg.layout.dim.data[0].label.capacity * sizeof(char));

    axes_msg.data.capacity = 8;
    axes_msg.data.size = 0;
    axes_msg.data.data = (float *)malloc(buttons_msg.data.capacity * sizeof(float));

    axes_msg.layout.dim.capacity = 1;
    axes_msg.layout.dim.size = 0;
    axes_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension *)malloc(buttons_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

    axes_msg.layout.dim.data[0].label.capacity = 10;
    axes_msg.layout.dim.data[0].label.size = 0;
    axes_msg.layout.dim.data[0].label.data = (char *)malloc(axes_msg.layout.dim.data[0].label.capacity * sizeof(char));

    set_microros_wifi_transports((char *)ssid, (char *)psk, agent_ip, agent_port);

    delay(250);
    allocator = rcl_get_default_allocator();
    Serial.println("Before support init");
    // create init_options
    while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
    {
        delay(1000);
    }
    Serial.println("After support init");

    Error::display_error(2);

    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));
    Error::display_error(3);

    // create publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &resultG_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "MPU_6500/Resultant_G"));

    RCCHECK(rclc_publisher_init_best_effort(
        &accel_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "MPU_6500/Acceleration"));

    RCCHECK(rclc_publisher_init_best_effort(
        &gyro_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "MPU_6500/Gyroscope"));

    RCCHECK(rclc_publisher_init_best_effort(
        &out_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "out_topic"));

    Serial.println("Starting subscriber...");

    RCCHECK(rclc_subscription_init_best_effort(
        &buttons_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "buttons_in"));

    RCCHECK(rclc_subscription_init_best_effort(
        &axes_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "axes_in"));

    // create timer,
    const unsigned int timer_timeout = 10;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    Error::display_error(4);
    // create executor
    RCCHECK(rclc_executor_init(&timer_executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_init(&buttons_executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_init(&axes_executor, &support.context, 1, &allocator));
    Error::display_error(5);

    rcl_ret_t buttons_rc = rclc_executor_add_subscription(&buttons_executor, &buttons_subscription, &buttons_msg, &buttons_callback, ALWAYS);
    rcl_ret_t axes_rc = rclc_executor_add_subscription(&axes_executor, &axes_subscription, &buttons_msg, &axes_callback, ALWAYS);

    rcl_ret_t rc2 = rclc_executor_add_timer(&timer_executor, &timer);
    Error::display_error(6);

    if (RCL_RET_OK != buttons_rc)
    {
        Serial.println("Error adding buttons subscriber to executor.");
    }
    else
        Serial.println("Successfully added buttons subscriber to executor.");

    if (RCL_RET_OK != axes_rc)
    {
        Serial.println("Error adding axes subscriber to executor.");
    }
    else
        Serial.println("Successfully added axes subscriber to executor.");

    if (RCL_RET_OK != rc2)
    {
        Serial.println("Error adding timer to executor.");
    }
    else
        Serial.println("Successfully added timer to executor.");

    if (!myMPU6500->init())
    {
        Serial.println("MPU6500 does not respond");
        Error::display_error(10);
    }
    else
    {
        Serial.println("MPU6500 is connected");
    }

    /* The slope of the curve of acceleration vs measured values fits quite well to the theoretical
     * values, e.g. 16384 units/g in the +/- 2g range. But the starting point, if you position the
     * MPU6500 flat, is not necessarily 0g/0g/1g for x/y/z. The autoOffset function measures offset
     * values. It assumes your MPU6500 is positioned flat with its x,y-plane. The more you deviate
     * from this, the less accurate will be your results.
     * The function also measures the offset of the gyroscope data. The gyroscope offset does not
     * depend on the positioning.
     * This function needs to be called at the beginning since it can overwrite your settings!
     */
    //   Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
    delay(100);
    myMPU6500->autoOffsets();
    //   Serial.println("Done!");

    /*  This is a more accurate method for calibration. You have to determine the minimum and maximum
     *  raw acceleration values of the axes determined in the range +/- 2 g.
     *  You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
     *  Use either autoOffset or setAccOffsets, not both.
     */
    // myMPU6500.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);

    /*  The gyroscope data is not zero, even if you don't move the MPU6500.
     *  To start at zero, you can apply offset values. These are the gyroscope raw values you obtain
     *  using the +/- 250 degrees/s range.
     *  Use either autoOffset or setGyrOffsets, not both.
     */
    // myMPU6500.setGyrOffsets(45.0, 145.0, -105.0);

    /*  You can enable or disable the digital low pass filter (DLPF). If you disable the DLPF, you
     *  need to select the bandwdith, which can be either 8800 or 3600 Hz. 8800 Hz has a shorter delay,
     *  but higher noise level. If DLPF is disabled, the output rate is 32 kHz.
     *  MPU6500_BW_WO_DLPF_3600
     *  MPU6500_BW_WO_DLPF_8800
     */
    myMPU6500->enableGyrDLPF();
    // myMPU6500.disableGyrDLPF(MPU6500_BW_WO_DLPF_8800); // bandwdith without DLPF

    /*  Digital Low Pass Filter for the gyroscope must be enabled to choose the level.
     *  MPU6500_DPLF_0, MPU6500_DPLF_2, ...... MPU6500_DPLF_7
     *
     *  DLPF    Bandwidth [Hz]   Delay [ms]   Output Rate [kHz]
     *    0         250            0.97             8
     *    1         184            2.9              1
     *    2          92            3.9              1
     *    3          41            5.9              1
     *    4          20            9.9              1
     *    5          10           17.85             1
     *    6           5           33.48             1
     *    7        3600            0.17             8
     *
     *    You achieve lowest noise using level 6
     */
    myMPU6500->setGyrDLPF(MPU6500_DLPF_6);

    /*  Sample rate divider divides the output rate of the gyroscope and accelerometer.
     *  Sample rate = Internal sample rate / (1 + divider)
     *  It can only be applied if the corresponding DLPF is enabled and 0<DLPF<7!
     *  Divider is a number 0...255
     */
    myMPU6500->setSampleRateDivider(5);

    /*  MPU6500_GYRO_RANGE_250       250 degrees per second (default)
     *  MPU6500_GYRO_RANGE_500       500 degrees per second
     *  MPU6500_GYRO_RANGE_1000     1000 degrees per second
     *  MPU6500_GYRO_RANGE_2000     2000 degrees per second
     */
    myMPU6500->setGyrRange(MPU6500_GYRO_RANGE_250);

    /*  MPU6500_ACC_RANGE_2G      2 g   (default)
     *  MPU6500_ACC_RANGE_4G      4 g
     *  MPU6500_ACC_RANGE_8G      8 g
     *  MPU6500_ACC_RANGE_16G    16 g
     */
    myMPU6500->setAccRange(MPU6500_ACC_RANGE_2G);

    /*  Enable/disable the digital low pass filter for the accelerometer
     *  If disabled the bandwidth is 1.13 kHz, delay is 0.75 ms, output rate is 4 kHz
     */
    myMPU6500->enableAccDLPF(true);

    /*  Digital low pass filter (DLPF) for the accelerometer, if enabled
     *  MPU6500_DPLF_0, MPU6500_DPLF_2, ...... MPU6500_DPLF_7
     *   DLPF     Bandwidth [Hz]      Delay [ms]    Output rate [kHz]
     *     0           460               1.94           1
     *     1           184               5.80           1
     *     2            92               7.80           1
     *     3            41              11.80           1
     *     4            20              19.80           1
     *     5            10              35.70           1
     *     6             5              66.96           1
     *     7           460               1.94           1
     */
    myMPU6500->setAccDLPF(MPU6500_DLPF_6);

    /* You can enable or disable the axes for gyroscope and/or accelerometer measurements.
     * By default all axes are enabled. Parameters are:
     * MPU6500_ENABLE_XYZ  //all axes are enabled (default)
     * MPU6500_ENABLE_XY0  // X, Y enabled, Z disabled
     * MPU6500_ENABLE_X0Z
     * MPU6500_ENABLE_X00
     * MPU6500_ENABLE_0YZ
     * MPU6500_ENABLE_0Y0
     * MPU6500_ENABLE_00Z
     * MPU6500_ENABLE_000  // all axes disabled
     */
    // myMPU6500.enableAccAxes(MPU6500_ENABLE_XYZ);
    // myMPU6500.enableGyrAxes(MPU6500_ENABLE_XYZ);
    Error::display_error(0);

    delay(200);
}

void Microros::timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    // Serial.println("Timer callback called");
    if (timer != NULL)
    {
        xyzFloat gValue = myMPU6500->getGValues();
        xyzFloat gyr = myMPU6500->getGyrValues();
        float temp = myMPU6500->getTemperature();
        float resultantG = myMPU6500->getResultantG(gValue);

        // if(resultantG > 1.1){
        //     digitalWrite(ledPin,HIGH);
        // }
        // else{
        //     digitalWrite(ledPin,LOW);
        // }

        resultG_msg_float.data = int(resultantG * 100);

        // imu_msg.linear_acceleration.x = gValue.x;
        // imu_msg.linear_acceleration.y = gValue.y;
        // imu_msg.linear_acceleration.z = gValue.z;

        // imu_msg.angular_velocity.x = gyr.x;
        // imu_msg.angular_velocity.y = gyr.y;
        // imu_msg.angular_velocity.z = gyr.z;

        accel_msg.x = 9.81 * gValue.x;
        accel_msg.y = 9.81 * gValue.y;
        accel_msg.z = 9.81 * gValue.z;

        // if (accel_msg.z < 5)
        // {
        //     motor1.setSpeed(1);
        //     motor2.setSpeed(1);
        // }

        gyro_msg.x = gyr.x;
        gyro_msg.y = gyr.y;
        gyro_msg.z = gyr.z;

        RCSOFTCHECK(rcl_publish(&resultG_publisher, &resultG_msg_float, NULL));
        RCSOFTCHECK(rcl_publish(&accel_publisher, &accel_msg, NULL));
        RCSOFTCHECK(rcl_publish(&gyro_publisher, &gyro_msg, NULL));
        RCSOFTCHECK(rcl_publish(&out_publisher, &Microros::out_msg, NULL));
    }
}

void Microros::axes_callback(const void *msgin)
{
    // Serial.println("Callback axes called");
    const std_msgs__msg__Float32MultiArray *axes_msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    float *axes = axes_msg->data.data;

    float leftStick = axes[1] / 0.7;
    float rightStick = axes[3] / 0.72;

    std::string printString1 = "leftStick: " + std::to_string(leftStick);
    // Serial.println(leftStick);
    // Serial.println(rightStick);

    if (leftStick > 1)
    {
        leftStick = 1;
    }
    else if (axes[1] < -1)
    {
        leftStick = -1;
    }

    if (rightStick > 1)
    {
        rightStick = 1;
    }
    else if (rightStick < -1)
    {
        rightStick = -1;
    }

    motor1->setSpeed(leftStick);
    motor2->setSpeed(rightStick);

    // myservo.write(90*axes[0]/0.75+ 90);
}

void Microros::buttons_callback(const void *msgin)
{
    // Serial.println("Callback buttons called");
    const std_msgs__msg__Int32MultiArray *buttons_msg = (const std_msgs__msg__Int32MultiArray *)msgin;
    int32_t *buttons = buttons_msg->data.data;
    // Serial.println("Publishing: " + buttons[0]);

    if (buttons[0] == 1)
    {
        Error::ledOn();
    }
    else
    {
        Error::ledOff();
    }

    out_msg.data = int(buttons[0]);

    RCSOFTCHECK(rcl_publish(&out_publisher, &out_msg, NULL));
}

void Microros::spin_nodes()
{
    RCSOFTCHECK(rclc_executor_spin_some(&timer_executor, RCL_MS_TO_NS(1)));
    RCSOFTCHECK(rclc_executor_spin_some(&buttons_executor, RCL_MS_TO_NS(1)));
    RCSOFTCHECK(rclc_executor_spin_some(&axes_executor, RCL_MS_TO_NS(1)));
}

void Microros::error_loop()
{

    while (1)
    {
        delay(100);
    }
}

bool Microros::ping()
{
    rmw_ret_t pingResult = rmw_uros_ping_agent(50, 10);
    if (pingResult != RMW_RET_OK)
    {
        Serial.println("Ping failed");
        Error::display_error(15);
        return false;
    }
    else
    {
        Serial.println("Ping successful");
        Error::display_error(0);

        return true;
    }
}

void Microros::shutdown()
{

    // rcl_publisher_t Microros::resultG_publisher;
    // rcl_publisher_t Microros::accel_publisher;
    // rcl_publisher_t Microros::gyro_publisher;
    // rcl_publisher_t Microros::out_publisher;
    // rcl_subscription_t Microros::buttons_subscription;
    // rcl_subscription_t Microros::axes_subscription;

    // std_msgs__msg__Int32MultiArray Microros::buttons_msg;
    // std_msgs__msg__Float32MultiArray Microros::axes_msg;
    // rclc_executor_t Microros::timer_executor;

    // rclc_executor_t Microros::buttons_executor;
    // rclc_executor_t Microros::axes_executor;
    // rclc_support_t Microros::support;
    // rcl_allocator_t Microros::allocator;
    // rcl_node_t Microros::node;
    // rcl_timer_t Microros::timer;

    RCSOFTCHECK(rclc_executor_fini(&timer_executor));
    RCSOFTCHECK(rclc_executor_fini(&buttons_executor));
    RCSOFTCHECK(rclc_executor_fini(&axes_executor));

    RCSOFTCHECK(rcl_timer_fini(&timer));

    Error::display_error(1);

    RCSOFTCHECK(rcl_publisher_fini(&resultG_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&accel_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&gyro_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&out_publisher, &node));
    Error::display_error(2);

    RCSOFTCHECK(rcl_subscription_fini(&buttons_subscription, &node));
    RCSOFTCHECK(rcl_subscription_fini(&axes_subscription, &node));
    Error::display_error(3);

    RCSOFTCHECK(rcl_node_fini(&node));

    RCSOFTCHECK(rclc_support_fini(&support));

    std_msgs__msg__Int32MultiArray__fini(&buttons_msg);
    std_msgs__msg__Float32MultiArray__fini(&axes_msg);
    std_msgs__msg__Float32__fini(&resultG_msg_float);
    geometry_msgs__msg__Vector3__fini(&accel_msg);
    geometry_msgs__msg__Vector3__fini(&gyro_msg);
    std_msgs__msg__Int32__fini(&out_msg);

    Error::display_error(4);

    // free(buttons_msg.data.data);
    // free(buttons_msg.layout.dim.data[0].label.data);
    // free(buttons_msg.layout.dim.data);

    // free(axes_msg.data.data);
    // free(axes_msg.layout.dim.data[0].label.data);
    // free(axes_msg.layout.dim.data);
}