// ControlTask.cpp — 控制任务实现
// 第 2 步: 空壳结构, 保证接线后可编译链接; 第 3 步填入正式逻辑
//
// 调用关系:
//   main.c: main()         -> ControlTaskInit()   上电初始化(启动 CAN/定时器)
//   TIM6 1kHz 中断          -> HAL_TIM_PeriodElapsedCallback -> MainTask()
//   CAN1 收到反馈帧         -> HAL_CAN_RxFifo0MsgPendingCallback -> CanFeedbackCallback()
//
// 注意: HAL 是 C 库, 它的回调是 C 弱符号, 在 C++ 文件里定义必须套 extern "C",
//       否则名字修饰不同, 链接器不会用我们的实现去覆盖弱符号, 回调永远不执行。

#include "ControlTask.h"
#include "main.h"   // 引入 stm32f4xx_hal.h, 提供 TIM/CAN 句柄类型

// ================= 供 main.c 调用的接口 =================

extern "C" void ControlTaskInit(void) {
    // TODO(第3步): 配置 CAN 过滤器 -> 启动 CAN -> 激活接收中断 -> 启动 TIM6
}

extern "C" void MainTask(void) {
    // TODO(第3步): 生成正弦目标 -> PID 计算 -> CAN 发送电压帧 -> 喂看门狗
}

extern "C" void CanFeedbackCallback(uint32_t std_id, const uint8_t *data) {
    // TODO(第3步): std_id 匹配电机 rxId 后调用 motor.decode(data)
}

// ================= HAL 回调对接 =================

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // TODO(第3步): 判断 htim == &htim6 后调用 MainTask()
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    // TODO(第3步): HAL_CAN_GetRxMessage 取出帧后调用 CanFeedbackCallback()
}
