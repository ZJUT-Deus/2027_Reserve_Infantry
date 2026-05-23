#ifndef COMMUNICAT_H
#define COMMUNICAT_H

#include "Imu.h"
#include "main.h"
#include "VT13.h"
#include "Vision.h"
#include "Serialplot.h"
#include "cmsis_os.h"
#include "Can_receive.h"
#include "bsp_buzzer.h"
#include "WS2812.h"
#include "Referee.h"

#define TOP_SWITCH_DEFAULT 1

//extern uint32_t  bus_off_recovery_count = 0;

class Communicate
{
public:
    void init();

    void run();

    
};


extern IMU imu;
extern VT13 vt13;
extern Vision vision;
extern Communicate communicate;
extern Can_receive can_receive;
extern Referee referee;
extern Serialplot serialplot;
extern bool_t top_switch;
extern bool_t is_sweeping_360;
extern bool_t auto_switch;

#endif

