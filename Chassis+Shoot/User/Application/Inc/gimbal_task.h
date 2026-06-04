/**
 * @file    gimbal_task.h
 * @brief   云台 FreeRTOS 任务声明 (CMSIS-RTOS v2)
 * @author  kk
 * @date    2026-06-03
 */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

/** @brief 云台任务初始化延时 (ms) */
#define GIMBAL_TASK_INIT_TIME 80
/** @brief 云台任务执行周期 (ms) */
#define GIMBAL_TASK_PERIOD_MS 2

#ifdef __cplusplus
extern "C" {
#endif

void Gimbal_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
