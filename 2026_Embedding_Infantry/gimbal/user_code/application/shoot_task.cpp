#include "shoot_task.h"

#include "simple_ref_test.h"

void shoot_task(void *pvParameters)
{
    vTaskDelay(SHOOT_TASK_INIT_TIME);
    //发射机构初始化
    shoot.init();
    simple_ref_test_init();
    while (1)
    {
       //设置发射机构状态机
       shoot.set_mode();
       //发射机构数据反馈
       shoot.feedback_update();
       simple_ref_test_periodic();
       //设置发射机构控制量
       shoot.set_control();
       //设置PID计算
       shoot.solve();
       //输出电流
       shoot.output();
        //系统延时
        vTaskDelay(SHOOT_TASK_CONTROL_DELAY_TICK);
    }
}
