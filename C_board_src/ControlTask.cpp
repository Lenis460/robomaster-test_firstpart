// ControlTask.cpp — 电控第一部分考核: GM6020 正弦跟随
//
// 任务: 让 GM6020 分别跟随自定义的正弦速度曲线与正弦位置曲线,
//       1kHz 控制周期, IWDG 看门狗保护 + CAN 反馈超时保护。
//
// 控制结构:
//   速度模式(单环):  正弦速度给定 ──► 速度环 PID ──► 电压
//   位置模式(串级):  正弦位置给定 ──► 角度环 PID ──► 速度给定
//                                   速度给定    ──► 速度环 PID ──► 电压
//
// 调用关系:
//   main(): ControlTaskInit()               上电一次, 启动 CAN 与 TIM6
//   TIM6 1kHz: HAL_TIM_PeriodElapsedCallback -> MainTask()
//   CAN1 收帧: HAL_CAN_RxFifo0MsgPendingCallback -> CanFeedbackCallback()

#include "ControlTask.h"
#include "main.h"    // stm32f4xx_hal.h
#include "can.h"     // hcan1
#include "tim.h"     // htim6
#include "iwdg.h"    // hiwdg
#include "Gm6020.h"
#include "Pid.h"
#include <cmath>

// ================= 模式切换 =================
#define MODE_SPEED    0
#define MODE_POSITION 1
#define CONTROL_MODE  MODE_SPEED   // 考核两种跟随, 改这一行并重新编译来切换

namespace {

constexpr float kDt = 0.001f;                       // 控制周期 1kHz
constexpr float kTwoPi = 6.28318530717958647692f;

// ---- 正弦曲线自定义参数 ----
constexpr float kSpeedAmp  = 5.0f;                  // 速度幅值 rad/s (约 48 rpm)
constexpr float kSpeedFreq = 1.0f;                  // 速度频率 Hz
constexpr float kPosAmp    = 3.14159265f;           // 位置幅值 rad (半圈)
constexpr float kPosFreq   = 0.5f;                  // 位置频率 Hz

// 角度环输出(即速度给定)限幅, 必须大于位置正弦所需峰值速度 2π·f·A ≈ 9.87 rad/s
constexpr float kVelCmdLimit = 12.0f;               // rad/s

constexpr uint8_t  kMotorId            = 1;         // 电机拨码 ID
constexpr uint32_t kFeedbackTimeoutMs  = 20;        // 反馈超时判定 (正常 1ms 一帧)

// ---- PID 初始参数(上台后在硬件上整定) ----
// 速度环输出是电压 ±25000; 电机反电动势约 0.6V/(rad/s) -> 12500 计数/rad/s 量级,
// Kp 取千级让 1 rad/s 误差产生千级电压修正, Ki 消除摩擦引起的稳态误差
constexpr float kVelKp = 2000.0f, kVelKi = 2000.0f, kVelKd = 0.0f;
// 角度环输出是速度给定 rad/s, Kp 取 10 表示 0.1rad 误差给出 1rad/s 修正
constexpr float kAngKp = 10.0f,   kAngKi = 0.0f,    kAngKd = 0.0f;

constexpr int16_t kVoltageMax = 25000;              // GM6020 电压给定满量程

} // namespace

// ================= 全局对象 =================
static Gm6020 motor(kMotorId);
static Pid    vel_pid(kVelKp, kVelKi, kVelKd, kVoltageMax, -kVoltageMax);
static Pid    ang_pid(kAngKp, kAngKi, kAngKd, kVelCmdLimit, -kVelCmdLimit);

static float    t = 0.0f;              // 控制时标, 每拍累加, 供正弦发生器使用
static uint32_t last_feedback_ms = 0;  // 最近一次收到电机反馈的时刻
static uint32_t tx_fail_cnt = 0;       // CAN 发送失败计数(调试观察用)

// ================= 供 main.c 调用的接口 =================

extern "C" void ControlTaskInit(void) {
    // CAN 过滤器: 掩码全 0 = 不筛选, 放行总线上所有标识符(反馈帧 0x205 由软件核对)
    CAN_FilterTypeDef filter = {};
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &filter);

    HAL_CAN_Start(&hcan1);
    // 使能"FIFO0 收到报文"中断, 收到反馈帧时进入 HAL_CAN_RxFifo0MsgPendingCallback
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    // 启动 TIM6 1kHz 周期中断, 每 1ms 进入 MainTask()
    HAL_TIM_Base_Start_IT(&htim6);
}

extern "C" void MainTask(void) {
    t += kDt;

    // ---- 读取反馈 ----
    // CAN 接收中断优先级高于本任务, decode 可能在读取途中打断我们,
    // 造成"整圈数与圈内角度不配套"的撕裂读数, 故关中断做一个原子快照
    __disable_irq();
    float vel = motor.vel();
    float pos = motor.angle();
    __enable_irq();
    (void)pos;   // 仅位置模式使用, 抑制 unused 警告

    // ---- 保护: 反馈超时则清零输出并复位 PID, 防止总线断线后电机带旧指令乱跑 ----
    float voltage = 0.0f;
    if (HAL_GetTick() - last_feedback_ms <= kFeedbackTimeoutMs) {
        float vel_cmd;
#if CONTROL_MODE == MODE_SPEED
        // 正弦速度给定: v(t) = A·sin(2πft)
        vel_cmd = kSpeedAmp * sinf(kTwoPi * kSpeedFreq * t);
#else
        // 正弦位置给定: θ(t) = A·sin(2πft), 角度环算出需要的速度
        float pos_ref = kPosAmp * sinf(kTwoPi * kPosFreq * t);
        vel_cmd = ang_pid.calc(pos_ref, pos);
#endif
        voltage = vel_pid.calc(vel_cmd, vel);
    } else {
        vel_pid.reset();
        ang_pid.reset();
    }

    // ---- 发送电压控制帧 (0x1FF, 本电机占 2 字节, 其余槽位为 0) ----
    motor.setVoltage(static_cast<int16_t>(voltage));
    CAN_TxHeaderTypeDef tx_header = {};
    tx_header.StdId              = motor.txId();
    tx_header.IDE                = CAN_ID_STD;
    tx_header.RTR                = CAN_RTR_DATA;
    tx_header.DLC                = 8;
    tx_header.TransmitGlobalTime = DISABLE;
    uint8_t tx_data[8] = {0};
    motor.encode(tx_data);
    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &mailbox) != HAL_OK) {
        tx_fail_cnt++;
    }

    // ---- 喂看门狗: IWDG 约 0.8s 超时, 1kHz 喂狗余量充足;
    //      若控制逻辑卡死, 狗超时复位整机, 这就是"看门狗保护" ----
    HAL_IWDG_Refresh(&hiwdg);
}

extern "C" void CanFeedbackCallback(uint32_t std_id, const uint8_t *data) {
    if (std_id == motor.rxId()) {
        motor.decode(data);
        last_feedback_ms = HAL_GetTick();
    }
}

// ================= HAL 回调对接 =================

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        MainTask();
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        CanFeedbackCallback(rx_header.StdId, rx_data);
    }
}
