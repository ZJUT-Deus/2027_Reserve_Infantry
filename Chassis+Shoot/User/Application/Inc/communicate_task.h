/**
 * @file    communicate_task.h
 * @brief   CAN/UART 通信 FreeRTOS 任务声明 (CMSIS-RTOS v2)
 * @author  kk
 * @date    2026-05-25
 */

#ifndef COMMUNICATE_TASK_H
#define COMMUNICATE_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

/** @brief 通信任务初始化延时 (ms) */
#define COMMUNICATE_TASK_INIT_TIME 50
/** @brief 通信任务执行周期 (ms) */
#define COMMUNICATE_TASK_PERIOD_MS 5

#ifdef __cplusplus
extern "C" {
#endif

void Communicate_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
