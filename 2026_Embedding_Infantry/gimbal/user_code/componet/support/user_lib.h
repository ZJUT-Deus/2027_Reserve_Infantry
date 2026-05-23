#ifndef USER_LIB_H
#define USER_LIB_H

#include "struct_typedef.h"

fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);

#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
