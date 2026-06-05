/**
 * @file    chassis_task.cpp
 * @brief   底盘 FreeRTOS 控制任务实现
 * @author  kk
 * @date    2026-05-25
 */

#include "chassis_task.h"
#include "Chassis.h"

/**
 * @brief  底盘控制任务入口
 * @param  argument 未使用
 * @retval none
 */
void Chassis_Task(void *argument)
{
    vTaskDelay(CHASSIS_TASK_INIT_TIME);

    chassis_init();

    for (;;)
    {
        chassis_control_loop();

        osDelay(CHASSIS_CONTROL_TIME_MS);
    }
}
