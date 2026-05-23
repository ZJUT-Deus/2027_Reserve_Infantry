#include "WS2812.h"
#include <stdlib.h> 



// 初始化定时器与PWM，并动态分配内存
void WS2812::Init(TIM_HandleTypeDef* timerHandle, uint32_t timerChannel, uint16_t ledsCount) {
    htim = timerHandle;
    channel = timerChannel;
    numLeds = ledsCount;
    
    // 如果发生了再次Init的情况，避免内存泄露（在C++11的初始化帮助下此变量默认为nullptr）
    if (buffer != nullptr) {
        delete[] buffer;
    }
    
    // 使用new进行动态分配，保证灵活性。在嵌入式中，由于只在初始化时分配一次，通常不会造成内存碎片。
    buffer = new uint8_t[numLeds * 3](); 
    
    Clear();

    if (htim == nullptr) return;

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

// 设置特定LED的颜色
void WS2812::SetRGB(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index < numLeds && buffer != nullptr) {
        // WS2812的数据格式为 GRB，高位先发
        buffer[index * 3 + 0] = g;
        buffer[index * 3 + 1] = r;
        buffer[index * 3 + 2] = b;
    }
}

// 设置所有LED的颜色
void WS2812::SetAllRGB(uint8_t r, uint8_t g, uint8_t b) {
    for (uint16_t i = 0; i < numLeds; i++) {
        SetRGB(i, r, g, b);
    }
}

// 清除所有颜色
void WS2812::Clear() {
    if (buffer != nullptr) {
        for (int i = 0; i < numLeds * 3; i++) {
            buffer[i] = 0;
        }
    }
}

// 定义内联辅助宏：等待CNT寄存器回卷（等待一个周期结束）
#define WAIT_FOR_CYCLE() do { \
        uint32_t current_cnt = htim->Instance->CNT; \
        uint32_t timeout = timeout_max; \
        while (timeout--) { \
            uint32_t next_cnt = htim->Instance->CNT; \
            if (next_cnt < current_cnt) break; \
            current_cnt = next_cnt; \
        } \
    } while(0)

// 核心发送函数（阻塞轮询方式），极小开销且不依赖DMA
void WS2812::Show() {
    if (htim == nullptr || buffer == nullptr) return;

    uint32_t ccr_addr;
    switch(channel) {
        case TIM_CHANNEL_1: ccr_addr = (uint32_t)&htim->Instance->CCR1; break;
        case TIM_CHANNEL_2: ccr_addr = (uint32_t)&htim->Instance->CCR2; break;
        case TIM_CHANNEL_3: ccr_addr = (uint32_t)&htim->Instance->CCR3; break;
        case TIM_CHANNEL_4: ccr_addr = (uint32_t)&htim->Instance->CCR4; break;
        default: return;
    }
    
    volatile uint32_t *ccr_reg = (volatile uint32_t *)ccr_addr;
    uint32_t arr_val = htim->Instance->ARR + 1; // 当前周期计数值（理论为300）
    
    // T0H占用总周期的约32% (0.4us / 1.25us)
    // T1H占用总周期的约64% (0.8us / 1.25us)
    uint32_t val_0 = (arr_val * 32) / 100;
    uint32_t val_1 = (arr_val * 64) / 100;

    // 为了防止定时器未启动或者意外导致的死循环卡死，添加超时上限
    uint32_t timeout_max = 50000;

    // 屏蔽全局中断，防止OS或者系统滴答中断打断由于软件阻塞带来的严格时序
    __disable_irq();

    // 1. 发送Reset信号（保持占空比为0至少50us）
    *ccr_reg = 0;
    
    // 等待当前所在的杂游周期结束，以校准相位
    WAIT_FOR_CYCLE();
    
    // 等待 45 个周期长复位
    for (int i = 0; i < 45; i++) {
        WAIT_FOR_CYCLE();
    }

    // 2. 依次发送每个字节的所有位数据
    uint32_t total_bytes = numLeds * 3;
    uint8_t *ptr = buffer;
    
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
