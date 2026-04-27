#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

#include "wifi_config.h"

#ifndef WIFI_SSID
#error "WIFI_SSID is not defined. Copy include/wifi_config.example.h to include/wifi_config.h and fill in your local values."
#endif

#ifndef WIFI_PASS
#error "WIFI_PASS is not defined. Copy include/wifi_config.example.h to include/wifi_config.h and fill in your local values."
#endif

#ifndef AGENT_IP
#error "AGENT_IP is not defined. Copy include/wifi_config.example.h to include/wifi_config.h and fill in your local values."
#endif

#ifndef AGENT_PORT
#error "AGENT_PORT is not defined. Copy include/wifi_config.example.h to include/wifi_config.h and fill in your local values."
#endif

namespace app_config {

constexpr unsigned long kUsbSerialBaud = 921600UL;
constexpr int kUartRx = 16;
constexpr int kUartTx = 17;
constexpr unsigned long kUartBaud = 921600UL;

constexpr char kWifiSsid[] = WIFI_SSID;
constexpr char kWifiPass[] = WIFI_PASS;
constexpr char kAgentIp[] = AGENT_IP;
constexpr uint16_t kAgentPort = AGENT_PORT;
constexpr char kRosNodeName[] = "stm32_bridge";

constexpr float kImuFilterAlpha = 0.2f;
constexpr bool kAcceptTrainCsvAsImu = false;

constexpr bool kEnableImuDebug = true;
constexpr bool kEnableCmdVelDebug = true;
constexpr bool kEnableBridgeRuntimeDebug = true;
constexpr bool kEnableStm32DebugLinePrint = false;

constexpr unsigned long kImuDebugIntervalMs = 1000UL;
constexpr unsigned long kCmdVelDebugIntervalMs = 500UL;
constexpr unsigned long kBridgeRuntimeDebugIntervalMs = 10000UL;

constexpr unsigned long kWifiRetryIntervalMs = 5000UL;
constexpr unsigned long kMicroRosInitRetryIntervalMs = 2000UL;
constexpr uint32_t kExecutorSpinTimeoutMs = 5U;
constexpr unsigned long kTransportSettleDelayMs = 250UL;

}  // namespace app_config

#endif
