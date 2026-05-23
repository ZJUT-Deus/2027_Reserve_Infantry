#include "communicate_task.h"

#include "communicate.h"




/**
* @brief          communucat_task
* @param[in]      pvParameters: NULL
* @retval         none
*/





void communicate_task(void const * pvParameters)
{
  vTaskDelay(COMMUNICATE_TASK_INIT_TIME);

  communicate.init();

  while (1)
  {
    
  
    communicate.run();

    //œµÕ≥—” ±
    vTaskDelay(COMMUNICATE_CONTROL_TIME_MS);
  }
}
