#include "uros_pub.h"

#include <math.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32.h>

#define UROS_SOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

static rcl_node_t node;
static rcl_publisher_t imu_publisher;
static rcl_publisher_t filtered_imu_publisher;
static rcl_publisher_t state_publisher;
static sensor_msgs__msg__Imu imu_msg;
static sensor_msgs__msg__Imu filtered_imu_msg;
static std_msgs__msg__Int32 state_msg;
static rclc_support_t support;
static rcl_allocator_t allocator;

static bool connected = false;
static bool filter_initialized = false;
static float filter_alpha = 0.2f;
static float filtered_ax = 0.0f;
static float filtered_ay = 0.0f;
static float filtered_az = 0.0f;
static float filtered_gx = 0.0f;
static float filtered_gy = 0.0f;
static float filtered_gz = 0.0f;

static void printRclError(Print& log, const char* prefix) {
    log.print(prefix);
    log.print(": ");
    log.println(rcl_get_error_string().str);
    rcl_reset_error();
}

static bool checkRcl(Print& log, rcl_ret_t ret, const char* message) {
    if (ret != RCL_RET_OK) {
        printRclError(log, message);
        return false;
    }
    return true;
}

static void applyLowPassFilter(sensor_msgs__msg__Imu& filtered,
                               const sensor_msgs__msg__Imu& raw) {
    if (!filter_initialized) {
        filtered_ax = raw.linear_acceleration.x;
        filtered_ay = raw.linear_acceleration.y;
        filtered_az = raw.linear_acceleration.z;
        filtered_gx = raw.angular_velocity.x;
        filtered_gy = raw.angular_velocity.y;
        filtered_gz = raw.angular_velocity.z;
        filter_initialized = true;
    } else {
        filtered_ax = filter_alpha * raw.linear_acceleration.x + (1.0f - filter_alpha) * filtered_ax;
        filtered_ay = filter_alpha * raw.linear_acceleration.y + (1.0f - filter_alpha) * filtered_ay;
        filtered_az = filter_alpha * raw.linear_acceleration.z + (1.0f - filter_alpha) * filtered_az;
        filtered_gx = filter_alpha * raw.angular_velocity.x + (1.0f - filter_alpha) * filtered_gx;
        filtered_gy = filter_alpha * raw.angular_velocity.y + (1.0f - filter_alpha) * filtered_gy;
        filtered_gz = filter_alpha * raw.angular_velocity.z + (1.0f - filter_alpha) * filtered_gz;
    }

    filtered = raw;
    filtered.linear_acceleration.x = filtered_ax;
    filtered.linear_acceleration.y = filtered_ay;
    filtered.linear_acceleration.z = filtered_az;
    filtered.angular_velocity.x = filtered_gx;
    filtered.angular_velocity.y = filtered_gy;
    filtered.angular_velocity.z = filtered_gz;
}

bool urosPubInit(Print& log, const char* node_name, float imu_filter_alpha) {
    connected = false;
    filter_initialized = false;
    filter_alpha = imu_filter_alpha;

    log.println("Initializing rclc support...");
    allocator = rcl_get_default_allocator();
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (!checkRcl(log, ret, "Failed to initialize rclc support")) {
        return false;
    }
    log.println("rclc support initialized");

    log.println("Creating ROS node...");
    ret = rclc_node_init_default(&node, node_name, "", &support);
    if (!checkRcl(log, ret, "Failed to create ROS node")) {
        return false;
    }
    log.println("ROS node created");

    log.println("Creating IMU publisher (best effort QoS)...");
    ret = rclc_publisher_init_best_effort(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data"
    );
    if (ret != RCL_RET_OK) {
        printRclError(log, "Failed to create IMU publisher");
    }
    log.println("IMU publisher created");

    log.println("Creating filtered IMU publisher (best effort QoS)...");
    ret = rclc_publisher_init_best_effort(
        &filtered_imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/filtered"
    );
    if (ret != RCL_RET_OK) {
        printRclError(log, "Failed to create filtered IMU publisher");
    }
    log.println("Filtered IMU publisher created");

    log.println("Creating state publisher (best effort QoS)...");
    ret = rclc_publisher_init_best_effort(
        &state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/robot/state"
    );
    if (ret != RCL_RET_OK) {
        printRclError(log, "Failed to create state publisher");
    }
    log.println("State publisher created");

    connected = true;
    return true;
}

bool urosPubIsConnected() {
    return connected;
}

void urosPubPublishImu(float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float qx, float qy, float qz, float qw) {
    if (!connected) {
        return;
    }

    imu_msg.header.stamp.sec = millis() / 1000;
    imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
    imu_msg.orientation.x = qx;
    imu_msg.orientation.y = qy;
    imu_msg.orientation.z = qz;
    imu_msg.orientation.w = qw;

    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    imu_msg.angular_velocity.x = gx * M_PI / 180.0;
    imu_msg.angular_velocity.y = gy * M_PI / 180.0;
    imu_msg.angular_velocity.z = gz * M_PI / 180.0;

    UROS_SOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

    applyLowPassFilter(filtered_imu_msg, imu_msg);

    UROS_SOFTCHECK(rcl_publish(&filtered_imu_publisher, &filtered_imu_msg, NULL));
}

void urosPubPublishState(int32_t state) {
    if (!connected) {
        return;
    }

    state_msg.data = state;
    UROS_SOFTCHECK(rcl_publish(&state_publisher, &state_msg, NULL));
}
