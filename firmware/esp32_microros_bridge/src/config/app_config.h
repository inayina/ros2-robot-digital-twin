#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <freertos/FreeRTOS.h>
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
constexpr bool kEnableTargetRpmDebug = true;
constexpr bool kEnableBridgeRuntimeDebug = true;
constexpr bool kEnableStm32DebugLinePrint = false;

constexpr unsigned long kImuDebugIntervalMs = 1000UL;
constexpr unsigned long kCmdVelDebugIntervalMs = 500UL;
constexpr unsigned long kTargetRpmDebugIntervalMs = 500UL;
constexpr unsigned long kRuntimeStatsPrintIntervalMs = 10000UL;

constexpr unsigned long kWifiRetryIntervalMs = 5000UL;
constexpr unsigned long kMicroRosInitRetryIntervalMs = 2000UL;
constexpr uint32_t kExecutorSpinTimeoutMs = 5U;
constexpr unsigned long kTransportSettleDelayMs = 250UL;
constexpr unsigned long kRosCommTaskDelayMs = 1UL;
constexpr unsigned long kMotorControlTaskPeriodMs = 10UL;
constexpr unsigned long kMotorControlCommandTimeoutMs = 500UL;
constexpr unsigned long kMotorControlCommandTimeoutMinMs = 100UL;
constexpr unsigned long kMotorControlCommandTimeoutMaxMs = 5000UL;
constexpr unsigned long kMotorControlDebugIntervalMs = 2000UL;

constexpr unsigned long kImuPublishIntervalMs = 20UL;             // 50 Hz
constexpr unsigned long kFilteredImuPublishIntervalMs = 40UL;     // 25 Hz
constexpr unsigned long kRobotStatePublishIntervalMs = 100UL;     // 10 Hz
constexpr unsigned long kMotorActualRpmPublishIntervalMs = 50UL;  // 20 Hz
constexpr unsigned long kMotorStateJsonPublishIntervalMs = 200UL; // 5 Hz

constexpr float kMockMotorResponseAlpha = 0.08f;
constexpr float kMockMotorZeroEpsilonRpm = 0.02f;
constexpr float kMockMotorMaxAbsRpm = 300.0f;
constexpr float kMotorControlDefaultMaxPwm = 0.25f;
constexpr float kMotorControlCommandMaxPwm = 0.50f;

constexpr bool kEnableMotorHardwareOutputs = true;
constexpr bool kEnableTb6612ChannelABenchTest = false;
constexpr bool kEnableTb6612ChannelBBenchTest = false;
constexpr bool kEnableN20ClosedLoopBench = false;
constexpr int8_t kTb6612BenchChannelAPwmPin = 4;
constexpr int8_t kTb6612BenchChannelADirIn1Pin = 5;
constexpr int8_t kTb6612BenchChannelADirIn2Pin = 6;
constexpr int8_t kTb6612BenchChannelBPwmPin = 7;
constexpr int8_t kTb6612BenchChannelBDirIn1Pin = 8;
constexpr int8_t kTb6612BenchChannelBDirIn2Pin = 9;
constexpr int8_t kTb6612BenchStandbyPin = 18;
constexpr int8_t kN20BenchEncoderAPin = 10;
constexpr int8_t kN20BenchEncoderBPin = 11;
constexpr uint8_t kTb6612BenchChannelAPwmChannel = 0U;
constexpr uint8_t kTb6612BenchChannelBPwmChannel = 1U;
constexpr uint32_t kTb6612BenchPwmFrequencyHz = 1000U;
constexpr uint8_t kTb6612BenchPwmResolutionBits = 8U;
constexpr float kTb6612BenchTestDuty = 0.18f;
constexpr unsigned long kTb6612BenchPulseMs = 300UL;
constexpr unsigned long kTb6612BenchCoastMs = 700UL;

constexpr unsigned long kN20ClosedLoopBenchControlPeriodMs = 50UL;
constexpr unsigned long kN20ClosedLoopBenchPrintIntervalMs = 100UL;
constexpr bool kN20ClosedLoopBenchInvertMeasuredTicks = true;
constexpr float kN20ClosedLoopBenchMaxPwm = 0.25f;
constexpr unsigned long kN20ClosedLoopBenchProfileStep1StartMs = 2000UL;
constexpr unsigned long kN20ClosedLoopBenchProfileStep2StartMs = 6000UL;
constexpr unsigned long kN20ClosedLoopBenchProfileStep3StartMs = 10000UL;
constexpr unsigned long kN20ClosedLoopBenchProfileStep4StartMs = 14000UL;
constexpr unsigned long kN20ClosedLoopBenchProfileStep5StartMs = 18000UL;
constexpr float kN20ClosedLoopBenchProfileStep0TargetTicksPerSec = 0.0f;
constexpr float kN20ClosedLoopBenchProfileStep1TargetTicksPerSec = 500.0f;
constexpr float kN20ClosedLoopBenchProfileStep2TargetTicksPerSec = 800.0f;
constexpr float kN20ClosedLoopBenchProfileStep3TargetTicksPerSec = 1000.0f;
constexpr float kN20ClosedLoopBenchProfileStep4TargetTicksPerSec = 500.0f;
constexpr float kN20ClosedLoopBenchProfileStep5TargetTicksPerSec = 0.0f;
constexpr float kN20ClosedLoopBenchKp = 0.00018f;
constexpr float kN20ClosedLoopBenchKi = 0.00005f;
constexpr float kN20ClosedLoopBenchKd = 0.0f;
constexpr float kN20ClosedLoopBenchIntegralMin = -400.0f;
constexpr float kN20ClosedLoopBenchIntegralMax = 400.0f;

constexpr uint32_t kRosCommTaskStackBytes = 8192U;
constexpr uint32_t kMotorControlTaskStackBytes = 6144U;
constexpr UBaseType_t kRosCommTaskPriority = 2U;
constexpr UBaseType_t kMotorControlTaskPriority = 2U;
constexpr BaseType_t kRosCommTaskCore = 0;
constexpr BaseType_t kMotorControlTaskCore = 1;

}  // namespace app_config

#endif
