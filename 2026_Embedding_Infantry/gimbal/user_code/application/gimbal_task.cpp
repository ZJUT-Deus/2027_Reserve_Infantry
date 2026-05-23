#include "gimbal_task.h"

void gimbal_task(void *pvParameters)
{
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //云台初始化
    gimbal.init();

    while (1)
    {
     //设置云台状态机
     gimbal.set_mode();
     //云台数据反馈
     gimbal.feedback_update();
     //设置云台控制量
     gimbal.set_control();
     //设置PID计算
     gimbal.solve();
     //输出电流
     gimbal.output();
      //系统延时
      vTaskDelay(GIMBAL_CONTROL_TIME_MS);
    }

}
