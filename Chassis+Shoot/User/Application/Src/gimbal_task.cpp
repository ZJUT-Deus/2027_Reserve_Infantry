/**
 * @file    gimbal_task.cpp
 * @brief   云台 FreeRTOS 控制任务实现
 * @author  kk
 * @date    2026-06-03
 */

#include "gimbal_task.h"
#include "Gimbal.h"

/**
 * @brief  云台控制任务入口
 * @param  argument 未使用
 */
void Gimbal_Task(void *argument)
{
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    gimbal_init();

    for (;;)
    {
        gimbal_control_loop();

        osDelay(GIMBAL_TASK_PERIOD_MS);
    }
}
