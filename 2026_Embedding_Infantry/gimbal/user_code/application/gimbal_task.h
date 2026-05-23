#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "gimbal.h"


#define GIMBAL_TASK_INIT_TIME 201

#define GIMBAL_CONTROL_TIME_MS 2



#ifdef __cplusplus
extern "C" {

extern void gimbal_task(void *pvParameters);


}
#endif


#endif 
