/**
 * @file    chassis_task.h
 * @brief   底盘控制任务声明 (CMSIS-RTOS v2)
 * @author  kk
 * @date    2026-05-25
 */

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

/** @brief 底盘任务初始化延时 (ms) */
#define CHASSIS_TASK_INIT_TIME 30
/** @brief 底盘控制周期 (ms) */
#define CHASSIS_CONTROL_TIME_MS 2

#ifdef __cplusplus
extern "C" {
#endif

void Chassis_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
