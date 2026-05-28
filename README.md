# Chassis文件说明

麦轮底盘 (STM32H723 + FreeRTOS + DM3519 电机)

## 硬件平台

| 项目 | 型号 |
|---|---|
| MCU | STM32H723VGTx (Cortex-M7, 550MHz) |
| 电机 | DM3519 ×4 (MIT 协议, FDCAN 通信(使用Classic模式)) |
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
I6X 遥控器 (UART5)
  │
  ▼
Communicate::handle_rc()        # RC 解析 + 模式分发
  │
  ├─ FREE 模式
  │    └─ chassis_set_velocity(vx, vy, 0)
  ├─ SPIN 模式
  │    └─ chassis_set_spin(vx, vy, SPIN_WZ)
  └─ 离线/失能
       └─ chassis_stop()
  │
  ▼
Chassis::set_control()          # 速度限幅 + 行为模式
  │
  ▼
chassis_vector_to_mecanum...() # 麦轮逆运动学
  │
  ▼
Chassis::solve()                # 4 路速度环 PID
  │
  ▼
Chassis::output()               # CAN 总线发送 MIT 指令
  │
  ▼
DM3519 ×4 (FDCAN)               # 电机执行
```

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

## 遥控器通道映射

| 通道 | 功能 |
|---|---|
| CH3 (油门) | 底盘 Vx (前进/后退) |
| CH2 (副翼) | 底盘 Vy (左移/右移) |
| SW_C 三段开关 | 模式切换: 上=SPIN, 中=FREE, 下=ZERO_FORCE |

关于遥控器的详细使用和代码文档参见 [README_I6X.md](README_I6X.md)

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


