/**
 * @file    shoot_task.h
 * @brief   射击控制任务声明 (CMSIS-RTOS v2)
 * @author  kk
 * @date    2026-06-04
 */

#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

/** @brief 射击任务初始化延时 (ms) */
#define SHOOT_TASK_INIT_TIME  40
/** @brief 射击控制周期 (ms) */
#define SHOOT_CONTROL_TIME_MS 2

#ifdef __cplusplus
extern "C" {
#endif

void Shoot_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
