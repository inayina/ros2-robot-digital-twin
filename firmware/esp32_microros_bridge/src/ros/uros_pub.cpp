#include "ros/uros_pub.h"

#include "ros/uros_core.h"

#include <math.h>
#include <stdio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

static rcl_publisher_t imu_publisher = rcl_get_zero_initialized_publisher();
static rcl_publisher_t filtered_imu_publisher = rcl_get_zero_initialized_publisher();
static rcl_publisher_t state_publisher = rcl_get_zero_initialized_publisher();
static rcl_publisher_t motor_actual_rpm_publisher = rcl_get_zero_initialized_publisher();
static rcl_publisher_t motor_state_publisher = rcl_get_zero_initialized_publisher();
static rcl_publisher_t motor_status_publisher = rcl_get_zero_initialized_publisher();
static sensor_msgs__msg__Imu imu_msg;
static sensor_msgs__msg__Imu filtered_imu_msg;
static std_msgs__msg__Int32 state_msg;
static std_msgs__msg__Float32 motor_actual_rpm_msg;
static std_msgs__msg__String motor_state_msg;
static char motor_state_buffer[384];

static bool initialized = false;
static bool imu_publisher_initialized = false;
static bool filtered_imu_publisher_initialized = false;
static bool state_publisher_initialized = false;
static bool motor_actual_rpm_publisher_initialized = false;
static bool motor_state_publisher_initialized = false;
static bool motor_status_publisher_initialized = false;
static bool filter_initialized = false;
static float filter_alpha = 0.2f;
static float filtered_ax = 0.0f;
static float filtered_ay = 0.0f;
static float filtered_az = 0.0f;
static float filtered_gx = 0.0f;
static float filtered_gy = 0.0f;
static float filtered_gz = 0.0f;
static unsigned long last_publish_error_ms = 0;
static UrosPubStats pub_stats = {};

static void recordPublishResult(const char* topic,
                                rcl_ret_t ret,
                                uint32_t& publish_count,
                                uint32_t& error_count) {
    if (ret == RCL_RET_OK) {
        publish_count++;
        return;
    }

    error_count++;
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

static void fillImuMessage(sensor_msgs__msg__Imu& msg,
                           float ax, float ay, float az,
                           float gx, float gy, float gz,
                           float qx, float qy, float qz, float qw) {
    msg.header.stamp.sec = millis() / 1000;
    msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
    msg.orientation.x = qx;
    msg.orientation.y = qy;
    msg.orientation.z = qz;
    msg.orientation.w = qw;

    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;

    msg.angular_velocity.x = gx * M_PI / 180.0;
    msg.angular_velocity.y = gy * M_PI / 180.0;
    msg.angular_velocity.z = gz * M_PI / 180.0;
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

    log.println("Creating motor actual_rpm publisher (best effort QoS)...");
    ret = rclc_publisher_init_best_effort(
        &motor_actual_rpm_publisher,
        urosCoreNode(),
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/motor/actual_rpm"
    );
    if (ret != RCL_RET_OK) {
        log.print("Failed to create motor actual_rpm publisher: ");
        log.println(rcl_get_error_string().str);
        rcl_reset_error();
        urosPubFini(log);
        return false;
    }
    motor_actual_rpm_publisher_initialized = true;
    log.println("Motor actual_rpm publisher created");

    motor_state_msg.data.data = motor_state_buffer;
    motor_state_msg.data.size = 0;
    motor_state_msg.data.capacity = sizeof(motor_state_buffer);

    log.println("Creating motor state publisher (best effort QoS)...");
    ret = rclc_publisher_init_best_effort(
        &motor_state_publisher,
        urosCoreNode(),
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/motor/state"
    );
    if (ret != RCL_RET_OK) {
        log.print("Failed to create motor state publisher: ");
        log.println(rcl_get_error_string().str);
        rcl_reset_error();
        urosPubFini(log);
        return false;
    }
    motor_state_publisher_initialized = true;
    log.println("Motor state publisher created");

    log.println("Creating motor status publisher (best effort QoS)...");
    ret = rclc_publisher_init_best_effort(
        &motor_status_publisher,
        urosCoreNode(),
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/motor/status"
    );
    if (ret != RCL_RET_OK) {
        log.print("Failed to create motor status publisher: ");
        log.println(rcl_get_error_string().str);
        rcl_reset_error();
        urosPubFini(log);
        return false;
    }
    motor_status_publisher_initialized = true;
    log.println("Motor status publisher created");

    initialized = true;
    return true;
}

void urosPubFini(Print& log) {
    if (!initialized && !imu_publisher_initialized &&
        !filtered_imu_publisher_initialized && !state_publisher_initialized &&
        !motor_actual_rpm_publisher_initialized && !motor_state_publisher_initialized &&
        !motor_status_publisher_initialized) {
        return;
    }

    if (motor_status_publisher_initialized) {
        rcl_ret_t ret = rcl_publisher_fini(&motor_status_publisher, urosCoreNode());
        if (ret != RCL_RET_OK) {
            log.print("Failed to destroy motor status publisher: ");
            log.println(rcl_get_error_string().str);
            rcl_reset_error();
        }
        motor_status_publisher = rcl_get_zero_initialized_publisher();
        motor_status_publisher_initialized = false;
    }

    if (motor_state_publisher_initialized) {
        rcl_ret_t ret = rcl_publisher_fini(&motor_state_publisher, urosCoreNode());
        if (ret != RCL_RET_OK) {
            log.print("Failed to destroy motor state publisher: ");
            log.println(rcl_get_error_string().str);
            rcl_reset_error();
        }
        motor_state_publisher = rcl_get_zero_initialized_publisher();
        motor_state_publisher_initialized = false;
        motor_state_msg.data.data = nullptr;
        motor_state_msg.data.size = 0;
        motor_state_msg.data.capacity = 0;
    }

    if (motor_actual_rpm_publisher_initialized) {
        rcl_ret_t ret = rcl_publisher_fini(&motor_actual_rpm_publisher, urosCoreNode());
        if (ret != RCL_RET_OK) {
            log.print("Failed to destroy motor actual_rpm publisher: ");
            log.println(rcl_get_error_string().str);
            rcl_reset_error();
        }
        motor_actual_rpm_publisher = rcl_get_zero_initialized_publisher();
        motor_actual_rpm_publisher_initialized = false;
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

void urosPubPublishImuRaw(float ax, float ay, float az,
                          float gx, float gy, float gz,
                          float qx, float qy, float qz, float qw) {
    if (!initialized || !urosCoreIsConnected() || !imu_publisher_initialized) {
        return;
    }

    fillImuMessage(imu_msg, ax, ay, az, gx, gy, gz, qx, qy, qz, qw);
    recordPublishResult(
        "/imu/data",
        rcl_publish(&imu_publisher, &imu_msg, NULL),
        pub_stats.imu_data_publish_count,
        pub_stats.imu_data_publish_error_count);
}

void urosPubPublishFilteredImu(float ax, float ay, float az,
                               float gx, float gy, float gz,
                               float qx, float qy, float qz, float qw) {
    if (!initialized || !urosCoreIsConnected() || !filtered_imu_publisher_initialized) {
        return;
    }

    fillImuMessage(imu_msg, ax, ay, az, gx, gy, gz, qx, qy, qz, qw);
    applyLowPassFilter(filtered_imu_msg, imu_msg);

    recordPublishResult(
        "/imu/filtered",
        rcl_publish(&filtered_imu_publisher, &filtered_imu_msg, NULL),
        pub_stats.filtered_imu_publish_count,
        pub_stats.filtered_imu_publish_error_count);
}

void urosPubPublishState(int32_t state) {
    if (!initialized || !urosCoreIsConnected() || !state_publisher_initialized) {
        return;
    }

    state_msg.data = state;
    recordPublishResult(
        "/robot/state",
        rcl_publish(&state_publisher, &state_msg, NULL),
        pub_stats.robot_state_publish_count,
        pub_stats.robot_state_publish_error_count);
}

void urosPubPublishMotorActualRpm(float actual_rpm) {
    if (!initialized || !urosCoreIsConnected() || !motor_actual_rpm_publisher_initialized) {
        return;
    }

    motor_actual_rpm_msg.data = actual_rpm;
    recordPublishResult(
        "/motor/actual_rpm",
        rcl_publish(&motor_actual_rpm_publisher, &motor_actual_rpm_msg, NULL),
        pub_stats.motor_actual_rpm_publish_count,
        pub_stats.motor_actual_rpm_publish_error_count);
}

void urosPubPublishMotorState(const MotorControlStateSnapshot& state) {
    if (!initialized || !urosCoreIsConnected() ||
        (!motor_state_publisher_initialized && !motor_status_publisher_initialized)) {
        return;
    }

    const int enabled = state.control_enabled ? 1 : 0;
    const int enabled_request = state.enabled ? 1 : 0;
    const int timeout = state.timeout_active ? 1 : 0;
    const int estop = state.estop_active ? 1 : 0;
    const int fault = state.fault_active ? 1 : 0;
    const int closed_loop = state.closed_loop ? 1 : 0;
    const int hardware_outputs_enabled = state.hardware_outputs_enabled ? 1 : 0;
    const char* status = fault ? "fault" : estop ? "stopped" : timeout ? "stale" : "ok";

    const int len = snprintf(
        motor_state_buffer,
        sizeof(motor_state_buffer),
        "{\"status\":\"%s\",\"target_rpm\":%.3f,\"actual_rpm\":%.3f,\"error_rpm\":%.3f,"
        "\"measured_rpm\":%.3f,\"pwm_duty\":%.3f,\"pwm\":%.3f,"
        "\"max_pwm\":%.3f,\"command_timeout_ms\":%lu,\"direction\":%d,"
        "\"control_enabled\":%d,\"enabled\":%d,\"hardware_outputs_enabled\":%d,"
        "\"closed_loop\":%d,\"saturated\":%d,\"timeout\":%d,"
        "\"stop\":%d,\"estop\":%d,\"fault\":%d,\"source\":\"%s\",\"loop\":%lu}",
        status,
        state.target_rpm,
        state.actual_rpm,
        state.error_rpm,
        state.actual_rpm,
        state.pwm_duty,
        state.pwm_duty,
        state.max_pwm_limit,
        (unsigned long)state.command_timeout_ms,
        (int)state.direction,
        enabled,
        enabled_request,
        hardware_outputs_enabled,
        closed_loop,
        state.saturated ? 1 : 0,
        timeout,
        estop,
        estop,
        fault,
        motorCommandSourceName(state.active_source),
        (unsigned long)state.loop_count);

    if (len <= 0) {
        return;
    }

    motor_state_msg.data.size =
        (len < (int)sizeof(motor_state_buffer)) ? (size_t)len : sizeof(motor_state_buffer) - 1U;
    if (motor_state_publisher_initialized) {
        recordPublishResult(
            "/motor/state",
            rcl_publish(&motor_state_publisher, &motor_state_msg, NULL),
            pub_stats.motor_state_publish_count,
            pub_stats.motor_state_publish_error_count);
    }

    if (motor_status_publisher_initialized) {
        recordPublishResult(
            "/motor/status",
            rcl_publish(&motor_status_publisher, &motor_state_msg, NULL),
            pub_stats.motor_status_publish_count,
            pub_stats.motor_status_publish_error_count);
    }
}

UrosPubStats urosPubGetStats() {
    return pub_stats;
}
