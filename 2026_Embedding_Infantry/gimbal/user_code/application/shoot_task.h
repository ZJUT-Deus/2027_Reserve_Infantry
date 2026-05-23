#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H
#include "cmsis_os.h"
#include "Shoot.h"

//任务初始化 空闲一段时间
#define SHOOT_TASK_INIT_TIME 201
#define SHOOT_TASK_CONTROL_DELAY_TICK 1 // 任务循环延时 tick

#ifdef __cplusplus
extern "C" {

extern void shoot_task(void *pvParameters);

}
#endif


#endif 
