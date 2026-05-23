#include "test_task.h"



int a=0;

/**
* @brief          communucat_task
* @param[in]      pvParameters: NULL
* @retval         none
*/


void test_task(void const * pvParameters)
{
    vTaskDelay(TEST_TASK_INIT_TIME);

    BSP_Buzzer_Init();

    while (1){
        a++;
        //LED灯
        //WS2812_Ctrl(a%255, (a*2)%255, (a*3)%255);
        //蜂鸣器
        //BSP_Buzzer_On();

        vTaskDelay(TEST_CONTROL_TIME_MS);
    }
}






