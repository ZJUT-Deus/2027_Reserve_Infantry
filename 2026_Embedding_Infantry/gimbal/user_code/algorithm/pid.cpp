
#include "Pid.h"

#include "user_lib.h"
fp32 pyh=0;
fp32 iyh=0;
fp32 dyh=0;
fp32 fyh=0;
#define NOW 0
#define OLD 1
#define NEW 2

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void Pid::init(uint8_t mode_, const fp32 *pid_parm, fp32 *ref_, fp32 *set_, fp32 erro_delta_)
{
    mode = mode_;
    data.Kp = pid_parm[0];
    data.Ki = pid_parm[1];
    data.Kd = pid_parm[2];
    data.kf = pid_parm[3];
    data.max_iout = pid_parm[4];
    data.max_out = pid_parm[5];
    data.set = set_;
    data.set_last = *set_;
    data.ref = ref_;
    data.ref_last = *ref_;
    data.error = *set_ - *ref_;

    
    if (data.mode == PID_ANGLE)
         data.error_delta = erro_delta_;
}


Pid::Pid(uint8_t mode_, const fp32 *pid_parm, fp32 *ref_, fp32 *set_)
{
    mode = mode_;
    data.Kp = pid_parm[0];
    data.Ki = pid_parm[1];
    data.Kd = pid_parm[2];
    data.kf = pid_parm[3];
    data.max_iout = pid_parm[4];
    data.max_out = pid_parm[5];
    
    data.ref_last = *ref_;
    data.set = set_;
    data.ref = ref_;
    data.error = *set_ - *ref_;

}




/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */


 fp32 Pid::pid_calc()
 {
     data.last_error = data.error;
     data.error = *data.set - *data.ref;
     if (mode == PID_SPEED)
         data.error_delta = data.error - data.last_error;

     if (mode == PID_ANGLE){
        data.error = rad_format(data.error);
        data.error_delta = data.error - data.last_error;       
         }

     data.Pout = data.Kp * data.error;
     data.Iout += data.Ki * data.error;
     data.Dout = data.Kd * (data.error_delta);
     data.Fout = data.kf * (*data.set - data.set_last);

     pyh = data.Pout;
     iyh = data.Iout;
     dyh = data.Dout;
     fyh = data.Fout;
     
     LimitMax(data.Iout, data.max_iout);
     
     data.ref_last = *data.ref;
     data.set_last = *data.set;
     data.out = data.Pout + data.Iout + data.Dout + data.Fout;
     LimitMax(data.out, data.max_out);

     return data.out;
}


/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void Pid::pid_clear()
{
    data.last_error = 0;
    data.error = 0;
    *data.set = 0;
    *data.ref = 0;
    data.out =  0;
    data.Pout = 0;
    data.Iout = 0;
    data.Dout = 0;
    data.Fout = 0;
}

void Pid::Clear()
{
    data.last_error = 0;
    data.Iout = 0;
    data.Dout = 0;
    data.out = 0;
    data.Fout = 0;
}

void Pid::align_state_to_current()
{
    fp32 current_set = 0.0f;
    fp32 current_ref = 0.0f;
    fp32 current_error = 0.0f;

    if (data.set)
    {
        current_set = *data.set;
    }

    if (data.ref)
    {
        current_ref = *data.ref;
    }

    current_error = current_set - current_ref;
    if (mode == PID_ANGLE)
    {
        current_error = rad_format(current_error);
    }

    data.set_last = current_set;
    data.ref_last = current_ref;
    data.error = current_error;
    data.last_error = current_error;
    data.error_delta = 0.0f;
    data.out = 0.0f;
    data.Pout = 0.0f;
    data.Iout = 0.0f;
    data.Dout = 0.0f;
    data.Fout = 0.0f;
}
