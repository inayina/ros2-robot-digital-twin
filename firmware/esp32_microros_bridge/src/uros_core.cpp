#include "uros_core.h"

#include <rcl/error_handling.h>
#include <rcl/context.h>
#include <rmw_microros/timing.h>

static rcl_node_t node = rcl_get_zero_initialized_node();
static rclc_support_t support = rclc_support_t{};
static rcl_allocator_t allocator;
static bool connected = false;
static bool support_initialized = false;
static bool node_initialized = false;

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

bool urosCoreInit(Print& log, const char* node_name) {
    if (support_initialized || node_initialized) {
        urosCoreFini(log);
    }

    connected = false;

    log.println("Initializing rclc support...");
    allocator = rcl_get_default_allocator();
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (!checkRcl(log, ret, "Failed to initialize rclc support")) {
        return false;
    }
    support_initialized = true;
    log.println("rclc support initialized");

    log.println("Creating ROS node...");
    ret = rclc_node_init_default(&node, node_name, "", &support);
    if (!checkRcl(log, ret, "Failed to create ROS node")) {
        urosCoreFini(log);
        return false;
    }
    node_initialized = true;
    log.println("ROS node created");

    connected = true;
    return true;
}

void urosCoreFini(Print& log) {
    connected = false;

    if (support_initialized) {
        rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
        if (rmw_context != nullptr) {
            rmw_ret_t ret = rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
            if (ret != RMW_RET_OK) {
                log.print("Failed to set entity destroy timeout: ");
                log.println((int)ret);
            }
        }
    }

    if (node_initialized) {
        rcl_ret_t ret = rcl_node_fini(&node);
        if (ret != RCL_RET_OK) {
            printRclError(log, "Failed to destroy ROS node");
        }
        node = rcl_get_zero_initialized_node();
        node_initialized = false;
    }

    if (support_initialized) {
        rcl_ret_t ret = rclc_support_fini(&support);
        if (ret != RCL_RET_OK) {
            printRclError(log, "Failed to finalize rclc support");
        }
        support = rclc_support_t{};
        support_initialized = false;
    }
}

bool urosCoreIsConnected() {
    return connected;
}

rcl_node_t* urosCoreNode() {
    return &node;
}

rclc_support_t* urosCoreSupport() {
    return &support;
}

rcl_allocator_t* urosCoreAllocator() {
    return &allocator;
}
