# VT13 风格 I6X 移植版本

这个文件夹里的 `I6X.h/.cpp` 按原项目 `VT13.h/.cpp` 的风格组织，适合直接放进新工程的 `user_code/device`。当前版本只负责麦轮底盘遥控输入。
I6X没有CRC校验

- 使用 `#ifndef I6X_H` 头文件保护。
- 使用 `class I6X`。
- 内部保存 `i6x_rc_ctrl` 和 `last_i6x_rc_ctrl`。
- 提供 `init()`、`unpack()`、`get_i6x_remote_control_point()`、`get_last_i6x_remote_control_point()`。
- 使用全局对象 `extern I6X i6x;`，和原来的 `extern VT13 vt13;` 一样。
- 宏定义风格也贴近 `VT13.h`。

## 用了 PDF 里的内容

- SBUS 帧长度：25 字节。
- 帧头：`0x0F`。
- 帧尾：`0x00`。
- 10 个通道的位运算解包方式。
- 前 6 个通道映射到 `-660 ~ 660`。
- 后 4 个通道归一化为拨杆三态：上 `1`、中 `0`、下 `-1`。
- 使用 `frame_lost` 和 `failsafe` 标志。

## 接入方式

在通信初始化里：

```cpp
i6x.init(I6X_UART, i6x.Rx_Buffer, I6X_RX_BUFFER_SIZE);
```

在 UART 回调里：

```cpp
if (huart->Instance == UART_I6X)
{
    i6x.unpack(HAL_GetTick());
    HAL_UARTEx_ReceiveToIdle_DMA(I6X_UART, i6x.Rx_Buffer, i6x.Rx_Buffer_Size);
}
```

在底盘初始化里：

```cpp
const I6X_RC_ctrl_t *rc = i6x.get_i6x_remote_control_point();
```

检查是否在线
```cpp
i6x.update_online(HAL_GetTick());
```

## 命令映射层

这一版已经内置命令映射层，不建议业务模块到处直接写 `rc->rc.ch[]` 下标。

底盘任务里优先读取：

```cpp
i6x.update_online(HAL_GetTick());

if (i6x.chassis_cmd.enable)
{
    vx_set = i6x.chassis_cmd.vx;
    vy_set = i6x.chassis_cmd.vy;
    wz_set = i6x.chassis_cmd.wz;
}
else
{
    vx_set = 0.0f;
    vy_set = 0.0f;
    wz_set = 0.0f;
}
```

通道编号、拨杆功能、速度上限集中在 `I6X.h` 顶部的映射宏里改，不要散落到底盘业务代码里。

当前默认物理操作：

- `ch[0]`：底盘前后速度 `vx`
- `ch[1]`：底盘左右平移 `vy`
- `ch[2]`：底盘旋转速度 `wz`
- `sw[0]`：底盘三段模式，上拨为小陀螺模式，中位为自由运动模式，下拨为无力模式
