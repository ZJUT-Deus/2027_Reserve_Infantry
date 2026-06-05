# Chassis+Shoot文件说明

麦轮底盘 + 发射机构 (STM32H723 + FreeRTOS + DM3519 + C610/C615)

## 硬件平台

| 项目 | 型号 |
|---|---|
| MCU | STM32H723VGTx (Cortex-M7, 550MHz) |
| 底盘电机 | DM3519 ×4 (MIT 协议, FDCAN1) |
| 摩擦轮 | C615 ×2 (PWM 开环, TIM1 CH1/CH3) |
| 拨弹轮 | C610 ×1 (CAN 电流环, FDCAN2) |
| 底盘 | 麦克纳姆轮 (Mecanum) |
| 遥控器 | I6X (UART5, SBUS 协议) |
| RTOS | FreeRTOS (CMSIS-RTOS v2) |
| IDE | Vscode+Keil MDK-ARM (ARM Compiler 5) |

## 目录结构

```
Chasis/
├── Core/                       # CubeMX 生成 (HAL 初始化)
│   ├── Inc/                    #   外设头文件
│   └── Src/                    #   main.c, freertos.c, 外设驱动
├── Drivers/                    # STM32H7 HAL 库 + CMSIS
├── Middlewares/                 # FreeRTOS
├── MDK-ARM/                    # Keil 工程文件
├── User/                       # 用户代码
│   ├── Algorithm/              #   算法库 (PID)
│   ├── Application/            #   FreeRTOS 任务
│   ├── Bsp/                    #   板级驱动 (FDCAN)
│   ├── Component/              #   通用组件 (类型定义, 工具函数)
│   ├── Device/                 #   设备驱动 (DM3519, I6X)
│   ├── Moudle/                 #   业务模块 (底盘控制, 通信)
│   └── Protocol/               #   协议层 (CAN 电机协议)
├── CODING_STYLE.md             # 代码注释规范
└── README.md                   # 本文件
```

## 控制架构

```
I6X 遥控器 (UART5) / 未来视觉上位机
  │
  ▼
Communicate::handle_rc()        # 控制源选择 + 模式分发
  │
  ├─ 底盘控制
  │   ├─ FREE 模式 → chassis_set_velocity(vx, vy, 0)
  │   ├─ SPIN 模式 → chassis_set_spin(vx, vy, SPIN_WZ)
  │   └─ 离线/失能 → chassis_stop()
  │
  ├─ 射击控制 (SW_A=摩擦轮, SW_B=连发)
  │   ├─ 离线/失能 → shoot_stop()
  │   ├─ SW_A↑ + SW_B↑ → shoot_continue_bullet()  # 连发
  │   ├─ SW_A↑          → shoot_ready()            # 准备
  │   └─ 其他           → shoot_stop()             # 停止
  │
  ▼
Chassis / Shoot 模块           # 各自独立的状态机 + PID + 输出
  │
  ▼
DM3519×4 (FDCAN1)              C610 (FDCAN2) + C615×2 (TIM1 PWM)
  底盘电机执行                    发射机构执行
```

## 代码分层架构

项目采用四层分层架构, 所有业务模块 (Chassis、Shoot、未来 GImbal) 遵循统一模式:

```
Moudle (业务层)
  Communicate.h/cpp — 控制源选择 + 指令分发
  Chassis.h/cpp     — 底盘运动控制
  Shoot.h/cpp       — 射击控制
  Gimbal.h/cpp      — 未来云台控制

Protocol (协议层)
  Can_receive.h/cpp — CAN 收发 + 电机协议解析

Device (设备层)
  Motor.h/cpp — DM3519/C610/C615 电机驱动抽象
  I6X.h/cpp   — I6X 遥控器 SBUS 协议解析

Algorithm (算法层)
  pid.h/cpp — PID 控制器
  user_lib.h — 滤波/限幅/数学工具

Bsp (板级驱动)
  bsp_fdcan.h/cpp — FDCAN 驱动封装
  bsp_pwm.h/cpp   — PWM 驱动 (C615)
```

### 各模块内部统一结构

每个业务模块 (Chassis / Shoot / Gimbal) 内部遵循**相同的代码结构**:

```
Module.h / Module.cpp
  │
  ├── 枚举定义 (模式/状态枚举, typedef enum { ... } xxx_e)  ← .h
  ├── 参数宏 (运动学/PID/阈值 #define)                     ← .h
  ├── 类定义 (class Module { ... })                      ← .h
  │   ├── 设备成员 (电机对象、滤波器对象)                      ← 硬件相关
  │   ├── 状态成员 (模式、标志位、计数器)                       ← 状态机数据
  │   ├── init()            — 初始化电机 + PID + 滤波器     ← .cpp
  │   ├── feedback_update() — 从 CAN/编码器 更新测量值       ← .cpp
  │   ├── set_control()     — 根据当前模式计算目标值           ← .cpp
  │   ├── solve()           — PID 计算, 得到控制量          ← .cpp
  │   └── output()          — 通过 CAN/PWM 输出控制量       ← .cpp
  │
  ├── 全局 extern 声明 (extern Module module;)            ← .h
  └── C 接口函数声明 (extern "C" { ... })                 ← .h
      ├── module_init(void)     — FreeRTOS 任务初始化调用
      ├── module_control_loop() — 任务循环调用 (feed → set → solve → out)
      └── module_xxx() / module_stop() — Communicate 调用的模式设置接口
```

### Chassis 模块详解

| 组件 | 说明 |
|---|---|
| **枚举** | `chassis_behaviour_e` — ZERO_FORCE / FREE / SPIN |
| **类成员** | DM3519[4] (电机) + speed_t x/y/z (速度) + First_order_filter (加速度滤波) |
| **C 接口** | `chassis_set_velocity()` / `chassis_set_spin()` / `chassis_stop()` — 由 Communicate 调用 |
| **控制循环** | `init()` → `feedback_update()` → `set_control()` (模式+运动学+限幅) → `solve()` (4路PID) → `output()` (CAN MIT协议) |
| **关键算法** | 麦轮逆运动学 (`chassis_vector_to_mecanum_wheel_speed`), 一阶加速度滤波 |

### Shoot 模块详解

| 组件 | 说明 |
|---|---|
| **枚举** | `shoot_mode_e` — STOP / READY / CONTINUE_BULLET; `trigger_anti_jam_state_e` — IDLE / REVERSE / RECOVERY / LOCKED |
| **类成员** | C615[2] (摩擦轮 PWM) + C610 (拨弹轮 CAN) + First_order_filter (摩擦轮缓启动) |
| **C 接口** | `shoot_ready()` / `shoot_continue_bullet()` / `shoot_stop()` — 由 Communicate 调用 |
| **控制循环** | `init()` → `feedback_update()` (读C610反馈+开环C615) → `set_control()` (模式+防卡弹状态机) → `solve()` (拨弹PID) → `output()` (C610 CAN电流 + C615 PWM脉冲) |
| **关键算法** | 拨弹轮速度环 PID, 摩擦轮缓启动一阶滤波, 防卡弹 4 状态机 (堵转检测→反转退弹→恢复→重试/锁死) |

### 遥控器通道映射

| 通道 | 功能 |
|---|---|
| CH3 (油门) | 底盘 Vx (前进/后退) |
| CH2 (副翼) | 底盘 Vy (左移/右移) |
| SW_A 两段开关 | 摩擦轮: 上=启动, 下=停止 |
| SW_B 两段开关 | 拨弹轮: 上=连发, 下=停止 |
| SW_C 三段开关 | 底盘模式: 上=SPIN, 中=FREE, 下=ZERO_FORCE |

> **控制逻辑**: SW_A 单独上推 = 摩擦轮旋转 (READY), SW_A+SW_B 同时上推 = 连发 (CONTINUE_BULLET)。SW_C 处于 ZERO_FORCE 或遥控器离线时, 射击强制停止。


## 底盘模式

| 模式 | 枚举值 | 行为 |
|---|---|---|
| ZERO_FORCE | `CHASSIS_ZERO_FORCE` | 无力模式, 电流归零, 上电默认 |
| FREE | `CHASSIS_FREE` | 自由速度控制, 加速度滤波 + 死区抑制 |
| SPIN | `CHASSIS_SPIN` | 小陀螺, 固定 wz 旋转 + 摇杆平移, 无死区 |

## 运动学参数

| 参数 | 值 | 单位 |
|---|---|---|
| 麦轮半径 | 0.076 | m |
| 轮距中心距离 | 0.185 | m |
| X 轴最大速度 | 2.0 | m/s |
| Y 轴最大速度 | 1.5 | m/s |
| Z 轴最大角速度 | 14.0 | rad/s |
| 小陀螺旋转速度 | 5.0 | rad/s |
| 控制周期 | 2 | ms |

底盘参数，之后再做调整，参数复制的之前的步兵

关于遥控器的详细使用和代码文档参见 [README_I6X.md](README_I6X.md)

## 云台 (Gimbal) 模块构筑指南

完全复用 Chassis / Shoot 的分层模式, 按以下对应关系放置新文件:

| 文件 | 位置 | 说明 |
|---|---|---|
| `Gimbal.h` | `User/Moudle/Inc/` | 模式枚举 + 参数宏 + 类声明 + C 接口声明 |
| `Gimbal.cpp` | `User/Moudle/Src/` | init → feed → set → solve → out 五步法实现 |
| `gimbal_task.cpp` | `User/Application/Src/` | FreeRTOS 任务, 调用 `gimbal_control_loop()` |

- **如果新增电机型号**: 在 `User/Device/Inc/Motor.h` 中添加类, 参考 C615 (PWM) / C610 (CAN)
- **如果新增 CAN 总线电机协议**: 在 `User/Protocol/Inc/Can_receive.h` 中添加解析函数
- **遥控器端**: 在 `Communicate::handle_rc()` 中追加云台调度分支, 参考射击控制的 if-else 结构
- **关键原则**: 保持 init → feed → set → solve → out 五步结构; Communicate 只通过 C 接口函数设置模式, 不直接访问内部成员

## 编译与烧录

1. 用 Keil MDK 打开 `MDK-ARM/Chassis.uvprojx`
2. 确认工程包含 `User/` 目录下所有 `.cpp` 源文件
3. 编译 (F7) → 烧录 (F8)

## 推送到仓库
1. 推送前先运行kill.bat清理编译文件再上传
2. 推送前最好先参考开发规范进行代码风格检查，功能测试完成后最好删除测试用临时变量 
3. 在项目基础上修改时先推送到一个新的分支，功能测试完善后再进行合并

## 开发规范

参见 [CODING_STYLE.md](CODING_STYLE.md) — Doxygen 注释格式与代码风格约定，可以直接丢给Ai当作Skill进行代码风格统一。


