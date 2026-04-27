#include "uros_pub.h"

#include "uros_core.h"

#include <math.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32.h>

static rcl_publisher_t imu_publisher = rcl_get_zero_initialized_publisher();
static rcl_publisher_t filtered_imu_publisher = rcl_get_zero_initialized_publisher();
static rcl_publisher_t state_publisher = rcl_get_zero_initialized_publisher();
static sensor_msgs__msg__Imu imu_msg;
static sensor_msgs__msg__Imu filtered_imu_msg;
static std_msgs__msg__Int32 state_msg;

static bool initialized = false;
static bool imu_publisher_initialized = false;
static bool filtered_imu_publisher_initialized = false;
static bool state_publisher_initialized = false;
static bool filter_initialized = false;
static float filter_alpha = 0.2f;
static float filtered_ax = 0.0f;
static float filtered_ay = 0.0f;
static float filtered_az = 0.0f;
static float filtered_gx = 0.0f;
static float filtered_gy = 0.0f;
static float filtered_gz = 0.0f;
static unsigned long last_publish_error_ms = 0;

static void logPublishError(const char* topic, rcl_ret_t ret) {
    if (ret == RCL_RET_OK) {
        return;
    }

    const unsigned long now = millis();
    if ((now - last_publish_error_ms) >= 1000UL) {
        Serial.print("[micro-ROS] publish failed topic=");
        Serial.print(topic);
        Serial.print(" ret=");
        Serial.print((int)ret);
        if (rcl_error_is_set()) {
            Serial.print(" err=");
            Serial.print(rcl_get_error_string().str);
        }
        Serial.println();
        last_publish_error_ms = now;
    }

    if (rcl_error_is_set()) {
        rcl_reset_error();
    }
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

bool urosPubInit(Print& log, float imu_filter_alpha) {
    if (initialized) {
        urosPubFini(log);
    }

    filter_initialized = false;
    filter_alpha = imu_filter_alpha;

    if (!urosCoreIsConnected()) {
        return false;
    }

    rcl_ret_t ret;

    log.println("Creating IMU publisher (best effort QoS)...");
    ret = rclc_publisher_init_best_effort(
        &imu_publisher,
        urosCoreNode(),
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data"
    );
    if (ret != RCL_RET_OK) {
        log.print("Failed to create IMU publisher: ");
        log.println(rcl_get_error_string().str);
        rcl_reset_error();
        urosPubFini(log);
        return false;
    }
    imu_publisher_initialized = true;
    log.println("IMU publisher created");

    log.println("Creating filtered IMU publisher (best effort QoS)...");
    ret = rclc_publisher_init_best_effort(
        &filtered_imu_publisher,
        urosCoreNode(),
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/filtered"
    );
    if (ret != RCL_RET_OK) {
        log.print("Failed to create filtered IMU publisher: ");
        log.println(rcl_get_error_string().str);
        rcl_reset_error();
        urosPubFini(log);
        return false;
    }
    filtered_imu_publisher_initialized = true;
    log.println("Filtered IMU publisher created");

    log.println("Creating state publisher (best effort QoS)...");
    ret = rclc_publisher_init_best_effort(
        &state_publisher,
        urosCoreNode(),
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/robot/state"
    );
    if (ret != RCL_RET_OK) {
        log.print("Failed to create state publisher: ");
        log.println(rcl_get_error_string().str);
        rcl_reset_error();
        urosPubFini(log);
        return false;
    }
    state_publisher_initialized = true;
    log.println("State publisher created");
    initialized = true;
    return true;
}

void urosPubFini(Print& log) {
    if (!initialized && !imu_publisher_initialized &&
        !filtered_imu_publisher_initialized && !state_publisher_initialized) {
        return;
    }

    if (state_publisher_initialized) {
        rcl_ret_t ret = rcl_publisher_fini(&state_publisher, urosCoreNode());
        if (ret != RCL_RET_OK) {
            log.print("Failed to destroy state publisher: ");
            log.println(rcl_get_error_string().str);
            rcl_reset_error();
        }
        state_publisher = rcl_get_zero_initialized_publisher();
        state_publisher_initialized = false;
    }

    if (filtered_imu_publisher_initialized) {
        rcl_ret_t ret = rcl_publisher_fini(&filtered_imu_publisher, urosCoreNode());
        if (ret != RCL_RET_OK) {
            log.print("Failed to destroy filtered IMU publisher: ");
            log.println(rcl_get_error_string().str);
            rcl_reset_error();
        }
        filtered_imu_publisher = rcl_get_zero_initialized_publisher();
        filtered_imu_publisher_initialized = false;
    }

    if (imu_publisher_initialized) {
        rcl_ret_t ret = rcl_publisher_fini(&imu_publisher, urosCoreNode());
        if (ret != RCL_RET_OK) {
            log.print("Failed to destroy IMU publisher: ");
            log.println(rcl_get_error_string().str);
            rcl_reset_error();
        }
        imu_publisher = rcl_get_zero_initialized_publisher();
        imu_publisher_initialized = false;
    }

    initialized = false;
    filter_initialized = false;
}

void urosPubPublishImu(float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float qx, float qy, float qz, float qw) {
    if (!initialized || !urosCoreIsConnected()) {
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

    logPublishError("/imu/data", rcl_publish(&imu_publisher, &imu_msg, NULL));

    applyLowPassFilter(filtered_imu_msg, imu_msg);

    logPublishError("/imu/filtered", rcl_publish(&filtered_imu_publisher, &filtered_imu_msg, NULL));
}

void urosPubPublishState(int32_t state) {
    if (!initialized || !urosCoreIsConnected()) {
        return;
    }

    state_msg.data = state;
    logPublishError("/robot/state", rcl_publish(&state_publisher, &state_msg, NULL));
}
