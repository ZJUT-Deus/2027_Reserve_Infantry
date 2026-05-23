#include "WS2812.h"

WS2812_HandleTypeDef ws2812;
// 获取外设时钟的核心频率，我们使用定时器所在的总线时钟
// 为了简便，目前假设ARR已通过本函数根据频率自动推导
// H7系列大部分情况下，如果SystemClock_Config是按照例程的PLL结构
// 定时器时钟(TIM_CLK)一般等于240MHz或者更高。
// 只需要 周期时间 = 1.25us (即800kHz)，则 ARR = TIM_CLK / 800kHz - 1
void WS2812_Init(WS2812_HandleTypeDef* ws, TIM_HandleTypeDef* htim, uint32_t channel) {
    ws->htim = htim;
    ws->channel = channel;
    
    // 清空缓存
    for (int i = 0; i < WS2812_NUM_LEDS * 3; i++) {
        ws->buffer[i] = 0;
    }

    // 假设系统时钟240MHz下定时器时钟也是240MHz；
    // 只需要提供800KHz的PWM信号
    __HAL_TIM_SET_AUTORELOAD(htim, 300 - 1);

    // 初始输出占空比为0（保持低电平输出）
    switch(channel) {
        case TIM_CHANNEL_1: htim->Instance->CCR1 = 0; break;
        case TIM_CHANNEL_2: htim->Instance->CCR2 = 0; break;
        case TIM_CHANNEL_3: htim->Instance->CCR3 = 0; break;
        case TIM_CHANNEL_4: htim->Instance->CCR4 = 0; break;
    }

    // 开启常规PWM输出（非DMA）
    HAL_TIM_PWM_Start(htim, channel);
    
    // 强制更新并使能计数器（防止因HAL库版本差异导致定时器未正常计数）
    htim->Instance->EGR = TIM_EGR_UG;
    htim->Instance->CR1 |= TIM_CR1_CEN;
}

void WS2812_SetRGB(WS2812_HandleTypeDef* ws, uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index < WS2812_NUM_LEDS) {
        // WS2812的数据格式为 GRB，高位先发
        ws->buffer[index * 3 + 0] = g;
        ws->buffer[index * 3 + 1] = r;
        ws->buffer[index * 3 + 2] = b;
    }
}

// 核心发送函数（阻塞轮询方式），极小开销且不依赖DMA
void WS2812_Show(WS2812_HandleTypeDef* ws) {
    uint32_t ccr_addr;
    switch(ws->channel) {
        case TIM_CHANNEL_1: ccr_addr = (uint32_t)&ws->htim->Instance->CCR1; break;
        case TIM_CHANNEL_2: ccr_addr = (uint32_t)&ws->htim->Instance->CCR2; break;
        case TIM_CHANNEL_3: ccr_addr = (uint32_t)&ws->htim->Instance->CCR3; break;
        case TIM_CHANNEL_4: ccr_addr = (uint32_t)&ws->htim->Instance->CCR4; break;
        default: return;
    }
    
    volatile uint32_t *ccr_reg = (volatile uint32_t *)ccr_addr;
    uint32_t arr_val = ws->htim->Instance->ARR + 1; // 当前周期计数值（理论为300）
    
    // T0H占用总周期的约32% (0.4us / 1.25us)
    // T1H占用总周期的约64% (0.8us / 1.25us)
    uint32_t val_0 = (arr_val * 32) / 100;
    uint32_t val_1 = (arr_val * 64) / 100;

    // 为了防止定时器未启动或者意外导致的死循环卡死，添加超时上限
    uint32_t timeout_max = 50000;

    // 定义内联辅助宏：等待CNT寄存器回卷（等待一个周期结束）
    // 这种比读写SR(Update标志)更安全，能够绝对抗干扰和防止在各种优化级别卡死。
#define WAIT_FOR_CYCLE() do { \
        uint32_t current_cnt = ws->htim->Instance->CNT; \
        uint32_t timeout = timeout_max; \
        while (timeout--) { \
            uint32_t next_cnt = ws->htim->Instance->CNT; \
            if (next_cnt < current_cnt) break; \
            current_cnt = next_cnt; \
        } \
    } while(0)

    // 屏蔽全局中断，防止OS或者系统滴答中断打断由于软件阻塞带来的严格时序
    __disable_irq();

    // 1. 发送Reset信号（保持占空比为0至少50us）
    // 50us / 1.25us = 40 次周期
    *ccr_reg = 0;
    // 等待当前所在的杂游周期结束，以校准相位
    WAIT_FOR_CYCLE();
    
    // 等待 45 个周期长复位
    for (int i = 0; i < 45; i++) {
        WAIT_FOR_CYCLE();
    }

    // 2. 依次发送每个字节的所有位数据
    uint32_t total_bytes = WS2812_NUM_LEDS * 3;
    uint8_t *ptr = ws->buffer;
    
    for (uint32_t i = 0; i < total_bytes; i++) {
        uint8_t data = ptr[i];
        for (int b = 7; b >= 0; b--) {
            // 将此位的占空比写进预装载寄存器 (等效为改变下个周期的脉宽)
            *ccr_reg = (data & (1 << b)) ? val_1 : val_0;
            
            // 等待当前周期结束，此时上述写入将会被真正锁存并生效给新的周期
            WAIT_FOR_CYCLE();
        }
    }

    // 3. 结束后保持0占空比输出，防止LED进入乱码态
    *ccr_reg = 0;
    WAIT_FOR_CYCLE();

    // 恢复全局中断
    __enable_irq();
}
