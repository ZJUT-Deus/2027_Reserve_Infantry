#include "chassis_task.h"
#include "chassis.h"





/**
  * @brief          chassis_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void chassis_task(void *pvParameters) {
  vTaskDelay(CHASSIS_TASK_INIT_TIME);
  chassis.init();

  while (true) {
    // 设置模式
    chassis.set_mode();
    // 反馈数据
    chassis.feedback_update();
    // 设置控制量
    chassis.set_contorl();
    // 解算
    chassis.solve();
    // 功率控制
    chassis.power_ctrl();
    // 电流输出
    chassis.output();

    // 系统延时
    vTaskDelay(CHASSIS_CONTROL_TIME_MS);
  }
}
