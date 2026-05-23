#include "lcd.h"
/*使用例程：


	LCD lcd(&hspi1, LCD_LANDSCAPE, false);        
                   //创建一个LCD类，参数： LCD(SPI_HandleTypeDef* spi, LCDOrientation orient, bool analogSPI) 
                                                    spi句柄             屏幕显示方向            软/硬件spi
                                                                      LCD_PORTRAIT 竖屏        0: 硬件SPI  1: 软件SPI
                                                                      LCD_PORTRAIT_REVERSE 竖屏反转
                                                                      LCD_LANDSCAPE 横屏
                                                                      LCD_LANDSCAPE_REVERSE 横屏反转
    lcd.init();                                  
    lcd.fill(0, 0, lcd.width(), lcd.height(), BLACK);                                               
                    //清屏，参数：fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
                                       起始坐标 x  起始坐标 y      宽度        高度        覆盖颜色                                                              
    lcd.drawString(10, 10, "Hello World!", WHITE, BLACK, FONT_24);                                                                                                 
                    //显示字符串 ，参数 ：drawString(uint16_t x, uint16_t y, const char* str, uint16_t fg, uint16_t bg, FontSize size, bool mode = false);                                                                                
                                                    开始坐标 x  开始坐标 y   显示内容           字体颜色       背景颜色       字体大小        显示模式
                                                                                                                        FONT_12 ：12*6     false 非叠加模式
                                                                                                                        FONT_16 ：16*8     true  叠加模式
                                                                                                                        FONT_24 ：24*12
                                                                                                                        FONT_32 ：32*16
    lcd.showValue(10, 40, "Temp", 25, 3);
                    //显示整型数据，参数：showValue(uint16_t x, uint16_t y, const char* name, uint16_t value, uint8_t len,FontSize size)
                                                      起始坐标x  起始坐标y     显示标签           整型数据       保留位数    字体大小（默认24）
    lcd.showFloat(10, 70, "Voltage", 3.14f, 1, 2,FONT_16); 
                    //显示浮点数据，参数：showFloat(uint16_t x, uint16_t y, const char* name, float value, uint8_t intLen, uint8_t fracLen,FontSize size)  
                                                      起始坐标x  起始坐标y     显示标签         浮点型数据     整数部分保留位数  小数部分保留位数  字体大小（默认24）                                
*/                                    
void LCD::init() {
 
    LCD_RES_Clr();
    HAL_Delay(100);
    LCD_RES_Set();
    HAL_Delay(100);
    
    LCD_BLK_Set(); 
    HAL_Delay(100);
    
    writeReg(0x11); 
    HAL_Delay(120);
    
    writeReg(0x36);
    switch(orientation) {
        case LCD_PORTRAIT:
            writeData8(0x00);
            break;
        case LCD_PORTRAIT_REVERSE:
            writeData8(0xC0);
            break;
        case LCD_LANDSCAPE:
            writeData8(0x70);
            break;
        case LCD_LANDSCAPE_REVERSE:
            writeData8(0xA0);
            break;
    }
    
    writeReg(0x3A);
    writeData8(0x05);
    
    writeReg(0xB2);
    writeData8(0x0C);
    writeData8(0x0C);
    writeData8(0x00);
    writeData8(0x33);
    writeData8(0x33);
    
    writeReg(0xB7);
    writeData8(0x35);
    
    writeReg(0xBB);
    writeData8(0x32); // Vcom=1.35V
    
    writeReg(0xC2);
    writeData8(0x01);
    
    writeReg(0xC3);
    writeData8(0x15); // GVDD=4.8V
    
    writeReg(0xC4);
    writeData8(0x20); // VDV, 0x20:0v
    
    writeReg(0xC6);
    writeData8(0x0F); // 0x0F:60Hz
    
    writeReg(0xD0);
    writeData8(0xA4);
    writeData8(0xA1);
    
    writeReg(0xE0);
    writeData8(0xD0);
    writeData8(0x08);
    writeData8(0x0E);
    writeData8(0x09);
    writeData8(0x09);
    writeData8(0x05);
    writeData8(0x31);
    writeData8(0x33);
    writeData8(0x48);
    writeData8(0x17);
    writeData8(0x14);
    writeData8(0x15);
    writeData8(0x31);
    writeData8(0x34);
    
    writeReg(0xE1);
    writeData8(0xD0);
    writeData8(0x08);
    writeData8(0x0E);
    writeData8(0x09);
    writeData8(0x09);
    writeData8(0x15);
    writeData8(0x31);
    writeData8(0x33);
    writeData8(0x48);
    writeData8(0x17);
    writeData8(0x14);
    writeData8(0x15);
    writeData8(0x31);
    writeData8(0x34);
    
    writeReg(0x21);
    writeReg(0x29);
}

void LCD::fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    setAddress(x, y, x + w - 1, y + h - 1);
    
    for(uint16_t i = y; i < y + h; i++) {
        for(uint16_t j = x; j < x + w; j++) {
            writeData16(color);
        }
    }
}


void LCD::LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	setAddress(x,y,x,y);
	writeData16(color);
} 
void LCD::drawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, FontSize size, bool mode) {
    uint8_t temp, sizex, t, m = 0;
    uint16_t i, TypefaceNum;
    uint16_t x0 = x;
    
    sizex = size / 2;
    TypefaceNum = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * size;
    c = c - ' ';
    
    setAddress(x, y, x + sizex - 1, y + size - 1);
    
    for(i = 0; i < TypefaceNum; i++) {
        switch(size) {
            case FONT_12: temp = ascii_1206[c][i]; break;
            case FONT_16: temp = ascii_1608[c][i]; break;
            case FONT_24: temp = ascii_2412[c][i]; break;
            case FONT_32: temp = ascii_3216[c][i]; break;
            default: return;
        }
        
        for(t = 0; t < 8; t++) {
            if(!mode) {
                if(temp & (0x01 << t)) writeData16(fg);
                else writeData16(bg);
                m++;
                if(m % sizex == 0) {
                    m = 0;
                    break;
                }
            } else {
                if(temp & (0x01 << t)) LCD_DrawPoint(x, y, fg);
                x++;
                if((x - x0) == sizex) {
                    x = x0;
                    y++;
                    break;
                }
            }
        }
    }
}

void LCD::drawString(uint16_t x, uint16_t y, const char* str, uint16_t fg, uint16_t bg, FontSize size, bool mode) {
    while(*str != '\0') {
        drawChar(x, y, *str, fg, bg, size, mode);
        x += size / 2;
        str++;
    }
}

void LCD::drawNumber(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fg, uint16_t bg, FontSize size) {
    uint8_t t, temp;
    uint8_t enshow = 0;
    uint8_t sizex = size / 2;
    
    for(t = 0; t < len; t++) {
        temp = (num / power(10, len - t - 1)) % 10;
        if(enshow == 0 && t < (len - 1)) {
            if(temp == 0) {
                drawChar(x + t * sizex, y, ' ', fg, bg, size, 0);
                continue;
            } else {
                enshow = 1;
            }
        }
        drawChar(x + t * sizex, y, temp + '0', fg, bg, size, 0);
    }
}

void LCD::drawFloat(uint16_t x, uint16_t y, float num, uint8_t intLen, uint8_t fracLen, uint16_t fg, uint16_t bg, FontSize size) {
    int16_t num_int;
    uint8_t t, temp, sizex;
    sizex = size / 2;
    num_int = num * power(10, fracLen);

    if(num < 0) {
        drawChar(x, y, '-', fg, bg, size, 0);
        num_int = -num_int;
        x += sizex;
        intLen++;
    } else {
        drawChar(x, y, ' ', fg, bg, size, 0);
        x += sizex;
        intLen++;
    }

    fill(x, y, (intLen + fracLen + 1) * sizex, size, bg);

    for(t = 0; t < intLen + fracLen; t++) {
        if(t == intLen) {
            drawChar(x + intLen * sizex, y, '.', fg, bg, size, 0);
            t++;
            intLen++;
        }
        temp = ((num_int / power(10, intLen + fracLen - t - 1)) % 10) + '0';
        drawChar(x + t * sizex, y, temp, fg, bg, size, 0);
    }
}

void LCD::showValue(uint16_t x, uint16_t y, const char* name, uint16_t value, uint8_t len,FontSize size) {
    drawString(x, y, name, WHITE, BLACK, size, 0);
    int n = 0;
    while(*name != '\0') { n++; name++; }
    x += n * 12;
    drawChar(x, y, ':', WHITE, BLACK, size, 0);
    x += 12;
    drawNumber(x, y, value, len, WHITE, BLACK, size);
}

void LCD::showFloat(uint16_t x, uint16_t y, const char* name, float value, uint8_t intLen, uint8_t fracLen,FontSize size) {
    drawString(x, y, name, WHITE, BLACK, size, 0);
    int n = 0;
    while(*name != '\0') { n++; name++; }
    x += n * 12;
    drawChar(x, y, ':', WHITE, BLACK, size, 0);
    x += 12;
    drawFloat(x, y, value, intLen + 1, fracLen, WHITE, BLACK, size);
}

void LCD::setOrientation(LCDOrientation orient) {
    orientation = orient;
    if(orientation == LCD_PORTRAIT || orientation == LCD_PORTRAIT_REVERSE) {
        lcdWidth = 240;
        lcdHeight = 280;
    } else {
        lcdWidth = 280;
        lcdHeight = 240;
    }
}

uint16_t LCD::width() const {
    return lcdWidth;
}

uint16_t LCD::height() const {
    return lcdHeight;
}

// ??????
void LCD::writeBus(uint8_t dat) {
    LCD_CS_Clr();
#if useAnalogSPI
        for(uint8_t i = 0; i < 8; i++) {
            LCD_SCLK_Clr();
            if(dat & 0x80) {
                LCD_MOSI_Set();
            } else {
                LCD_MOSI_Clr();
            }
            LCD_SCLK_Set();
            dat <<= 1;
        }
#else
   HAL_SPI_Transmit(hspi, &dat, 1, 0xffff);
#endif
    LCD_CS_Set();
}

void LCD::writeData8(uint8_t dat) {
    writeBus(dat);
}

void LCD::writeData16(uint16_t dat) {
    writeBus(dat >> 8);
    writeBus(dat);
}

void LCD::writeReg(uint8_t dat) {
    LCD_DC_Clr();
    writeBus(dat);
    LCD_DC_Set();
}

void LCD::setAddress(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    if(orientation == LCD_PORTRAIT || orientation == LCD_PORTRAIT_REVERSE) {
        writeReg(0x2A); 
        writeData16(x1);
        writeData16(x2);
        writeReg(0x2B);
        writeData16(y1 + 20);
        writeData16(y2 + 20);
        writeReg(0x2C);
    } else {
        writeReg(0x2A); 
        writeData16(x1 + 20);
        writeData16(x2 + 20);
        writeReg(0x2B); 
        writeData16(y1);
        writeData16(y2);
        writeReg(0x2C); 
    }
}

uint32_t LCD::power(uint8_t m, uint8_t n) const {
    uint32_t result = 1;
    while(n--) result *= m;
    return result;
}