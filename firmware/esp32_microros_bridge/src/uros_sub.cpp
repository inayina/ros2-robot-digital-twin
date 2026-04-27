#include "uros_sub.h"

#include "uros_core.h"

#include <geometry_msgs/msg/twist.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/subscription.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

static rcl_subscription_t cmd_vel_subscriber = rcl_get_zero_initialized_subscription();
static geometry_msgs__msg__Twist cmd_vel_msg;
static rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
static UrosCmdVelCallback cmd_vel_callback = nullptr;
static bool initialized = false;
static bool subscriber_initialized = false;
static bool executor_initialized = false;
static unsigned long last_spin_error_ms = 0;

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

static void handleCmdVelMessage(const void* msg_in) {
    if (msg_in == nullptr || cmd_vel_callback == nullptr) {
        return;
    }

    const geometry_msgs__msg__Twist* msg =
        static_cast<const geometry_msgs__msg__Twist*>(msg_in);
    cmd_vel_callback((float)msg->linear.x, (float)msg->angular.z);
}

bool urosSubInit(Print& log) {
    if (initialized) {
        urosSubFini(log);
    }

    if (!urosCoreIsConnected()) {
        return false;
    }

    log.println("Creating cmd_vel subscriber (reliable QoS)...");
    rcl_ret_t ret = rclc_subscription_init_default(
        &cmd_vel_subscriber,
        urosCoreNode(),
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"
    );
    if (!checkRcl(log, ret, "Failed to create cmd_vel subscriber")) {
        return false;
    }
    subscriber_initialized = true;
    log.println("cmd_vel subscriber created");

    log.println("Creating executor...");
    ret = rclc_executor_init(&executor, &urosCoreSupport()->context, 1, urosCoreAllocator());
    if (!checkRcl(log, ret, "Failed to initialize executor")) {
        urosSubFini(log);
        return false;
    }
    executor_initialized = true;

    ret = rclc_executor_add_subscription(
        &executor,
        &cmd_vel_subscriber,
        &cmd_vel_msg,
        &handleCmdVelMessage,
        ON_NEW_DATA
    );
    if (!checkRcl(log, ret, "Failed to add cmd_vel subscription to executor")) {
        urosSubFini(log);
        return false;
    }
    log.println("Executor ready");

    initialized = true;
    return true;
}

void urosSubFini(Print& log) {
    if (executor_initialized) {
        rcl_ret_t ret = rclc_executor_fini(&executor);
        if (ret != RCL_RET_OK) {
            printRclError(log, "Failed to finalize executor");
        }
        executor = rclc_executor_get_zero_initialized_executor();
        executor_initialized = false;
    }

    if (subscriber_initialized) {
        rcl_ret_t ret = rcl_subscription_fini(&cmd_vel_subscriber, urosCoreNode());
        if (ret != RCL_RET_OK) {
            printRclError(log, "Failed to destroy cmd_vel subscriber");
        }
        cmd_vel_subscriber = rcl_get_zero_initialized_subscription();
        subscriber_initialized = false;
    }

    initialized = false;
}

void urosSubSetCmdVelCallback(UrosCmdVelCallback callback) {
    cmd_vel_callback = callback;
}

void urosSubSpinSome(uint32_t timeout_ms) {
    if (!initialized || !urosCoreIsConnected()) {
        return;
    }

    rcl_ret_t ret = rclc_executor_spin_some(&executor, (uint64_t)timeout_ms * 1000000ULL);
    if (ret == RCL_RET_OK) {
        return;
    }

    const unsigned long now = millis();
    if ((now - last_spin_error_ms) >= 1000UL) {
        Serial.print("[micro-ROS] executor spin failed ret=");
        Serial.print((int)ret);
        if (rcl_error_is_set()) {
            Serial.print(" err=");
            Serial.print(rcl_get_error_string().str);
        }
        Serial.println();
        last_spin_error_ms = now;
    }

    if (rcl_error_is_set()) {
        rcl_reset_error();
    }
}
