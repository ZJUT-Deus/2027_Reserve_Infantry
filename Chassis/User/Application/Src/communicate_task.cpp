/**
 * @file    communicate_task.cpp
 * @brief   遥控器通信与控制任务, 由 Communicate 模块驱动
 * @author  kk
 * @date    2026-05-26
 */

#include "communicate_task.h"
#include "Communicate.h"

/**
 * @brief  初始化 I6X 遥控器 (C 接口, 供 main.c 调用)
 */
extern "C" void i6x_remote_init(void)
{
    i6x.init(&huart5, NULL, I6X_RX_BUFFER_SIZE);
}

/**
 * @brief  遥控器通信与控制任务入口
 * @param  argument 未使用
 */
void Communicate_Task(void *argument)
{
    vTaskDelay(COMMUNICATE_TASK_INIT_TIME);

    for (;;)
    {
        communicate.handle_rc();

        osDelay(COMMUNICATE_TASK_PERIOD_MS);
    }
}
