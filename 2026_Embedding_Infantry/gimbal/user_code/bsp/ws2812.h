#ifndef WS2812_H
#define WS2812_H

#include "main.h"
#include "tim.h"

#define WS2812_NUM_LEDS 8  // 支持的LED数量，可根据实际情况修改

// WS2812驱动类（结构体封装）
typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    uint8_t buffer[WS2812_NUM_LEDS * 3]; // GRB缓存
} WS2812_HandleTypeDef;

extern WS2812_HandleTypeDef ws2812;

// 方法声明
void WS2812_Init(WS2812_HandleTypeDef* ws, TIM_HandleTypeDef* htim, uint32_t channel);
void WS2812_SetRGB(WS2812_HandleTypeDef* ws, uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void WS2812_Show(WS2812_HandleTypeDef* ws);

#endif // WS2812_H
