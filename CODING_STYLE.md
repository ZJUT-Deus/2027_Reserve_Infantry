# 代码风格与注释规范

底盘运动控制项目 (STM32H723 + FreeRTOS + C/C++ 混合) 的编码规范。

---

## 1. 文件头注释

每个源文件和头文件**必须**在顶部包含 Doxygen 格式的文件头注释：

```cpp
/**
 * @file    Chassis.h
 * @brief   底盘运动控制模块 (DM3519 电机 + 麦轮运动学)
 * @author  kk
 * @date    2026-05-25
 */
```

- `@file` — 文件名，与磁盘上的实际文件名一致
- `@brief` — 一行中文简述模块功能
- `@author` — 作者
- `@date` — 创建日期，格式 `YYYY-MM-DD`

> CubeMX 自动生成的 `Core/` 目录文件保留 STM 官方头注释格式，无需修改。

---

## 2. 函数注释

每个函数**必须**在其定义前添加 Doxygen 块注释。至少包含 `@brief`，有参数和返回值时补充 `@param` 和 `@return`/`@retval`。

```cpp
/**
 * @brief  底盘初始化, 配置 4 个 DM3519 电机并初始化速度环 PID
 */
void Chassis::init()
{
    ...
}

/**
 * @brief  设置底盘自由运动速度 (C 接口)
 * @param  vx 前进速度 (m/s), 正值前进
 * @param  vy 横移速度 (m/s), 正值左移
 * @param  wz 旋转角速度 (rad/s), 正值逆时针
 */
void chassis_set_velocity(fp32 vx, fp32 vy, fp32 wz)
{
    ...
}

/**
 * @brief  执行一次 PID 计算
 * @return PID 输出值
 */
fp32 Pid::pid_calc()
{
    ...
}
```

- `@brief` — 一行中文简述，动词开头（"初始化"、"设置"、"执行"）
- `@param` — 参数名 + 中文说明。有多行时对齐参数名
- `@return` — 返回值说明（有返回值时）
- `@retval none` — 无返回值时标注（可选）
- 静态辅助函数可省略注释，但对外接口函数必须有

---

## 3. 结构体 / 类成员注释

所有结构体成员、类成员变量使用**行内** Doxygen 格式 `/**< */`：

```cpp
typedef struct
{
    fp32 Kp;            /**< 比例系数 */
    fp32 Ki;            /**< 积分系数 */
    fp32 Kd;            /**< 微分系数 */
    fp32 max_iout;      /**< 积分输出限幅 */
    fp32 max_out;       /**< 总输出限幅 */
    fp32 *set;          /**< 设定值指针 */
    fp32 *ref;          /**< 反馈值指针 */
} pid_data_t;

class Chassis
{
public:
    chassis_behaviour_e chassis_behaviour_mode;     /**< 当前行为模式 */
    chassis_behaviour_e last_chassis_behaviour_mode; /**< 上一次行为模式 */
    speed_t x;  /**< X 轴 (前进) 速度 */
    speed_t y;  /**< Y 轴 (横移) 速度 */
    speed_t z;  /**< Z 轴 (旋转) 角速度 */
};
```

- 注释对齐到同一列（使用空格）
- 中文、英文、单位混用时加括号说明，如 `前进速度 (m/s)`

---

## 4. 宏定义注释

`#define` 宏使用 `/** @brief */` 注释，放在宏定义的**上一行**：

```cpp
/** @brief 麦轮半径 (m) */
#define MOTOR_WHEEL_RADIUS         0.076f
/** @brief 轮距中心距离 (m) */
#define MOTOR_DISTANCE_TO_CENTER   0.185f
/** @brief 速度环 PID Kp */
#define CHASSIS_SPEED_PID_KP       2.0f
/** @brief 速度控制模式 */
#define SPEED 0
```

---

## 5. 枚举注释

枚举类型和每个枚举值都需要注释：

```cpp
/** @brief 底盘行为状态机 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0, /**< 无力模式 */
    CHASSIS_FREE,           /**< 自由速度模式 */
    CHASSIS_SPIN,           /**< 小陀螺模式 */
} chassis_behaviour_e;
```

---

## 6. 命名规范

| 类型 | 风格 | 示例 |
|---|---|---|
| **类名** | PascalCase | `Chassis`, `DM3519`, `Can_receive`, `First_order_filter` |
| **函数 / 方法** | snake_case | `chassis_init()`, `feedback_update()`, `pid_calc()`, `fp32_constrain()` |
| **变量 / 成员** | snake_case | `chassis_behaviour_mode`, `user_vx_set`, `speed_set` |
| **宏 / 常量** | UPPER_SNAKE_CASE | `CHASSIS_FR_CAN_ID`, `MOTOR_WHEEL_RADIUS`, `PID_SPEED` |
| **typedef 结构体** | snake_case + `_t` | `pid_data_t`, `esc_inf_t`, `motor_ctrl_t` |
| **typedef 枚举** | snake_case + `_e` | `chassis_behaviour_e`, `i6x_chassis_mode_e` |
| **文件名** | PascalCase | `Chassis.h`, `Motor.h`, `Can_receive.h` |

> 文件名统一使用 PascalCase，与类名保持一致。已有文件逐步统一。

---

## 7. 代码格式

格式由 `.clang-format` (在 `MDK-ARM/` 目录下) 定义，核心规则：

| 规则 | 设置 |
|---|---|
| 基础风格 | Microsoft |
| 缩进宽度 | 4 空格 |
| Tab | 不使用 |
| 大括号 | Linux 风格 (左括号在同行) |
| 访问修饰符偏移 | -4 |
| case 缩进 | 缩进 |
| `#include` 排序 | 不自动排序 |
| 短 if/loop/block | 允许单行 |
| 宏对齐 | 跨空行对齐 |
| 赋值对齐 | 连续赋值对齐 |
| 列宽限制 | 无限制 |

---

## 8. 头文件结构

头文件按以下顺序组织：

```cpp
// 1. 文件头注释
/**
 * @file    Example.h
 * @brief   示例模块
 * @author  kk
 * @date    2026-05-27
 */

// 2. 头文件保护
#ifndef EXAMPLE_H
#define EXAMPLE_H

// 3. 依赖包含
#include "struct_typedef.h"
#include "main.h"

// 4. extern "C" 起始 (如果有 C 接口)
#ifdef __cplusplus
extern "C" {
#endif

// 5. C 语言定义 (宏, 枚举, 结构体)
/** @brief ... */
#define EXAMPLE_VALUE  10

// 6. extern "C" 结束
#ifdef __cplusplus
}
#endif

// 7. C++ 类定义 (仅 .h 文件)
class Example
{
public:
    void init();
};

// 8. 全局 extern 声明
extern Example example;

// 9. C 接口函数声明 (extern "C" 区)
#ifdef __cplusplus
extern "C" {
#endif

void example_init(void);

#ifdef __cplusplus
}
#endif

// 10. 文件尾
#endif
```

> 头文件保护名称：`文件名全大写_H`，如 `CHASSIS_H`、`PID_H`。不使用双下划线前缀。

---

## 9. 源文件结构

```cpp
// 1. 文件头注释
/**
 * @file    Example.cpp
 * @brief   示例模块实现
 * @author  kk
 * @date    2026-05-27
 */

// 2. 自身头文件
#include "Example.h"

// 3. 其他依赖
#include "bsp_fdcan.h"

// 4. 静态变量 / 全局对象定义
Example example;

// 5. 函数实现 (按头文件声明顺序)
/**
 * @brief  初始化
 */
void Example::init()
{
    ...
}
```

---

## 10. 其他约定

- **注释语言**：使用中文，术语可用英文（如 "PID"、"CAN"、"FDCAN"、"MIT 模式"）
- **单位标注**：物理量注释必须包含单位，如 `前进速度 (m/s)`、`滤波时间间隔 (s)`
- **空行**：逻辑段落间用空行分隔，类方法间用空行分隔
- **const 正确性**：只读指针参数使用 `const`，如 `const dm_motor_measure_t *measure`
- **static 限定**：仅在单个 `.cpp` 内使用的函数和变量必须标记为 `static`
