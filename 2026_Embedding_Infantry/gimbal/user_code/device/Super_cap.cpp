// #include "Super_cap.h"
// #include "Communicate.h""
// #include "cmsis_os.h"
// #include "main.h"
// #include "can_receive.h"

// #include "cmsis_os.h"
// #include "main.h"

// #include "bsp_fdcan.h"
// #include "fdcan.h"

// #include "fifo.h"
// #include "struct_typedef.h"

// void Super_Cap::init() {
// 	can_receive.set_super_cap_init(0x00);
// 	super_cap_trigger=0;
// }


// void Super_Cap::run() {
// 	uint8_t data[3]={Expected_Power,0x00,0x01};
// 	can_receive.set_super_cap_crtl(data);
// }

// void Super_Cap::ready(uint8_t is_ready)
// {
// 	super_cap_is_ready=is_ready;
// }

// void Super_Cap::feedback(super_cap_t super_cap_receive)
// {
// 	  vcap=super_cap_receive.vcap;
// 		WorkIntensity1=super_cap_receive.WorkIntensity1;
// 		WorkIntensity2=super_cap_receive.WorkIntensity2;
// 		Pchassis=super_cap_receive.Pchassis;
// }






