#ifndef TEST_TASK_H
#define TEST_TASK_H


#include "cmsis_os.h"
#include "main.h"
#include "bsp_buzzer.h"
//任务开始空闲一段时间
#define TEST_TASK_INIT_TIME 30

//底盘任务控制间隔 2ms
#define TEST_CONTROL_TIME_MS 2

/**
  * @brief          test_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */

#ifdef __cplusplus
extern "C" {


extern void test_task(void const * pvParameters);


}
#endif




#endif

