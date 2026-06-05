/**
 * @file    shoot_task.cpp
 * @brief   射击 FreeRTOS 控制任务实现
 * @author  kk
 * @date    2026-06-04
 */

#include "shoot_task.h"
#include "Shoot.h"

/**
 * @brief  射击控制任务入口
 * @param  argument 未使用
 */
void Shoot_Task(void *argument)
{
    vTaskDelay(SHOOT_TASK_INIT_TIME);

    shoot_init();

    for (;;)
    {
        shoot_control_loop();

        osDelay(SHOOT_CONTROL_TIME_MS);
    }
}
