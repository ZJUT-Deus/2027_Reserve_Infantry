#ifndef __LCD_H
#define __LCD_H

#include "main.h"
#include "spi.h"
#include "lcdfont.h"

// 屏幕方向定义
typedef enum {
    LCD_PORTRAIT = 0,           // 竖屏
    LCD_PORTRAIT_REVERSE = 1,   // 竖屏反转
    LCD_LANDSCAPE = 2,          // 横屏
    LCD_LANDSCAPE_REVERSE = 3   // 横屏反转
} LCDOrientation;

// 字体大小定义
typedef enum {
    FONT_12 = 12,
    FONT_16 = 16,
    FONT_24 = 24,
    FONT_32 = 32
} FontSize;

// 颜色定义
#define WHITE         0xFFFF
#define BLACK         0x0000  
#define BLUE          0x001F  
#define BRED          0XF81F
#define GRED          0XFFE0
#define GBLUE         0X07FF
#define RED           0xF800
#define MAGENTA       0xF81F
#define GREEN         0x07E0
#define CYAN          0x7FFF
#define YELLOW        0xFFE0
#define BROWN         0XBC40 //棕色
#define BRRED         0XFC07 //棕红色
#define GRAY          0X8430 //灰色
#define DARKBLUE      0X01CF //深蓝色
#define LIGHTBLUE     0X7D7C //浅蓝色
#define GRAYBLUE      0X5458 //灰蓝色
#define LIGHTGREEN    0X841F //浅绿色
#define LGRAY         0XC618 //浅灰色(PANNEL),窗体背景色
#define LGRAYBLUE     0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE        0X2B12 //浅棕蓝色(选择条目的反色)

#if useAnalogSPI
#define LCD_SCLK_Clr() HAL_GPIO_WritePin(LCD_SCK_GPIO_Port,LCD_SCK_Pin, GPIO_PIN_RESET)//SCL=SCLK
#define LCD_SCLK_Set() HAL_GPIO_WritePin(LCD_SCK_GPIO_Port,LCD_SCK_Pin, GPIO_PIN_SET)

#define LCD_MOSI_Clr() HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_SDA_Pin, GPIO_PIN_RESET)//SDA=MOSI
#define LCD_MOSI_Set() HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_SDA_Pin, GPIO_PIN_SET)
#endif
#define LCD_RES_Clr()  HAL_GPIO_WritePin(LCD_RES_GPIO_Port,LCD_RES_Pin, GPIO_PIN_RESET)//RES
#define LCD_RES_Set()  HAL_GPIO_WritePin(LCD_RES_GPIO_Port,LCD_RES_Pin, GPIO_PIN_SET)

#define LCD_DC_Clr()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin, GPIO_PIN_RESET)//DC
#define LCD_DC_Set()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin, GPIO_PIN_SET)
 		     
#define LCD_CS_Clr()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin, GPIO_PIN_RESET)//CS
#define LCD_CS_Set()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin, GPIO_PIN_SET)

#define LCD_BLK_Clr(x)  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port,LCD_BLK_Pin, GPIO_PIN_RESET)//BLK TIM1->CCR1=x//
#define LCD_BLK_Set(x)  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port,LCD_BLK_Pin, GPIO_PIN_SET)//TIM1->CCR1=x//



class LCD {
public:
    // 构造函数
    LCD(SPI_HandleTypeDef* spi, LCDOrientation orient, bool analogSPI) 
    :hspi(spi), orientation(orient) ,useAnalogSPI(analogSPI){
    if(orientation == LCD_PORTRAIT || orientation == LCD_PORTRAIT_REVERSE) {
        lcdWidth = 240;
        lcdHeight = 280;
    } else {
        lcdWidth = 280;
        lcdHeight = 240;
    }
}
    
    // 初始化函数
    void init();
    
    // 基础绘图函数
    void fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color);
    // 文本显示函数
    void drawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, FontSize size, bool mode = false);
    void drawString(uint16_t x, uint16_t y, const char* str, uint16_t fg, uint16_t bg, FontSize size, bool mode = false);
    
    // 数字显示函数
    void drawNumber(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fg, uint16_t bg, FontSize size=FONT_24);
    void drawFloat(uint16_t x, uint16_t y, float num, uint8_t intLen, uint8_t fracLen, uint16_t fg, uint16_t bg, FontSize size=FONT_24);
    
    // 高级显示函数
    void showValue(uint16_t x, uint16_t y, const char* name, uint16_t value, uint8_t len,FontSize size=FONT_24);
    void showFloat(uint16_t x, uint16_t y, const char* name, float value, uint8_t intLen, uint8_t fracLen,FontSize size=FONT_24);
    
    // 设置屏幕方向
    void setOrientation(LCDOrientation orientation);
    // 获取屏幕尺寸
    uint16_t width() const;
    uint16_t height() const;

private:
    // 底层通信函数
    void writeBus(uint8_t dat);
    void writeData8(uint8_t dat);
    void writeData16(uint16_t dat);
    void writeReg(uint8_t dat);
    void setAddress(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
    
    // 辅助函数
    uint32_t power(uint8_t m, uint8_t n) const;
    
    // 配置参数
    bool useAnalogSPI;
    SPI_HandleTypeDef* hspi;
    LCDOrientation orientation;
    
    // 屏幕尺寸
    uint16_t lcdWidth;
    uint16_t lcdHeight;
};

#endif // __LCD_H