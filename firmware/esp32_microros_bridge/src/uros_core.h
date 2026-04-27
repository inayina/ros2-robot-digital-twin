#ifndef UROS_CORE_H
#define UROS_CORE_H

#include <Arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>

bool urosCoreInit(Print& log, const char* node_name);
void urosCoreFini(Print& log);
bool urosCoreIsConnected();

rcl_node_t* urosCoreNode();
rclc_support_t* urosCoreSupport();
rcl_allocator_t* urosCoreAllocator();

#endif
