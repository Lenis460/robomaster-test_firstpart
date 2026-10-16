# RM 电控第一部分考核：GM6020 正弦跟随

基于 RoboMaster C 型开发板（STM32F407IGH6），控制 GM6020 直流无刷电机分别跟随自定义的**正弦速度曲线**与**正弦位置曲线**，1kHz 控制周期，独立看门狗保护。

## 演示效果

| 模式 | 目标曲线 | 实测跟随 |
|---|---|---|
| 速度模式 | 5·sin(2π·1·t) rad/s | 幅度/相位基本重合（见 [speed_curve.png](docs/speed_curve.png)） |
| 位置模式 | π·sin(2π·0.5·t) rad（±半圈摆动） | 跟随度约 95% |

- 速度模式实拍：[docs/demo_speed.mp4](docs/demo_speed.mp4)
- 位置模式实拍：[docs/demo_position.mp4](docs/demo_position.mp4)
- Ozone 波形截图：[docs/speed_curve.png](docs/speed_curve.png)

## 硬件连接

| 连接 | 说明 |
|---|---|
| GM6020 细线 → C 板 2pin CAN1 | CANH/CANL 对应，**CANH-CANL 间并联 120Ω 终端电阻** |
| GM6020 电源线 → C 板输出 XT30 | 24V，电池经转接板接入输入 XT30 |
| 电机拨码 | **ID = 1**（反馈帧 0x205，控制帧 0x1FF） |
| DAPLink → C 板 SWD 座 | SWDIO/SWCLK/GND/3V3 四线，Ozone 下载调试 |

## 编译与烧录

工具链：`arm-none-eabi-gcc`（Arm GNU 13.2）、`CMake ≥3.22`、`Ninja`。

```bash
cmake -S . -B build/Debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug
# 产物: build/Debug/C_project.elf, 用 Ozone/CubeProgrammer 烧录
```

模式切换：`C_board_src/ControlTask.cpp` 顶部的 `CONTROL_MODE` 宏（`MODE_SPEED` / `MODE_POSITION`），改后重新编译。

## 控制架构

```
TIM6 1kHz 中断 (MainTask)
  正弦发生器 ──► [位置模式: 角度环 PID] ──► 速度环 PID ──►(+速度前馈 kff·v_ref)──► CAN 发送 0x1FF
                    ▲                          ▲
                    └──电机累计角度──┐   ┌──电机转速──┘
                                    │   │
  CAN1 收帧中断 (0x205) ──► Gm6020::decode(多圈角度/转速解码) ──┘
  每拍末尾: HAL_IWDG_Refresh() 喂狗
```

- **速度模式**：单环 PID + 速度前馈。前馈 `u_ff = kff × v_ref` 按电机增益模型直接提供主力电压（开环、不进入反馈环路、不消耗稳定裕度），反馈 PID 只修正摩擦/负载残差，因此 Kp 无需很大即可精确跟随，避免了纯 P 高增益与环路延迟冲突导致的发散（实测纯 P 在 Kp≈820 时失稳）。
- **位置模式**：角度环（输出速度给定，限幅 ±12 rad/s）+ 速度环（复用速度模式的整定结果）串级结构，速度前馈同样作用于内环。
- 电机反馈 1kHz（0x205：角度 0~8191/圈、转速 rpm、转矩电流、温度）；`Gm6020` 类通过跨零检测实现**多圈累计角度**。

## 整定参数（Ozone 在线整定后固化）

| 参数 | 值 | 说明 |
|---|---|---|
| 速度环 Kp / Ki / Kd | 730 / 100 / 0 | 输出电压 ±25000 |
| 速度前馈 kff | 670 计数/(rad/s) | 实测整定（理论估算 470，差值来自摩擦与供电内阻） |
| 角度环 Kp | 10 | 输出速度给定 ±12 rad/s |

调参方法：控制量以 `volatile` 全局变量暴露（`dbg_kp/dbg_ki/dbg_kff/dbg_akp...`），每拍同步写入控制器，在 Ozone Watch 窗口直接改值即时生效，无需反复编译烧录；波形用 Ozone Data Sampling（1kHz 采样）观察 `dbg_ref/dbg_fdb`。

## 保护机制（四层）

| 层 | 机制 | 行为 |
|---|---|---|
| 1 | **独立看门狗 IWDG**（LSI 32kHz ÷64 ÷400 ≈ 0.8s） | 每控制拍末尾喂狗；控制循环卡死 → 狗超时 → 整机复位。调试时通过 `__HAL_DBGMCU_FREEZE_IWDG()` 冻结，避免断点期间复位 |
| 2 | **CAN 反馈超时**（>20ms 无 0x205 帧） | 输出电压清零 + PID 状态复位，防止总线断线后电机带旧指令失控 |
| 3 | **CAN bus-off 自恢复**（每 100ms 检查 BOF 标志） | 总线错误导致外设锁死时自动 Stop/Start 重启，实测两次从"开机总线空转→bus-off"中自愈 |
| 4 | **位置误差卷绕**到 [-π, π] | 反馈丢帧可能使多圈累计错计一整圈（±2π 幽灵误差），卷绕后位置环只朝最近等效位置修正，不会狂转追错（实测有效） |

## 已知问题与改进方向

- 位置模式跟随度约 95%，可继续上调角度环 Kp（10 → 12~15）；
- 电压模式依赖电机反电动势模型，负载变化会引入稳态误差，可升级电流环（转矩控制，0x1FE 帧）；
- 正弦发生器基于上电零点，无相位补偿（前馈纯速度项，未加加速度前馈）。
