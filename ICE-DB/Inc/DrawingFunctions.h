#ifndef _DRFUNC_
#define _DRFUNC_

#include "fonts.h"
#include "functions.h"
#include "tft.h"
#include "stdint.h"
#include "math.h"

extern volatile uint16_t RPM ;
extern volatile uint16_t Old_RPM ;
extern volatile uint8_t Old_Gear_Number ;

void Draw_Circle(uint16_t x0, uint16_t y0, uint16_t r, uint32_t Color);

void LCD_Showthrmo(uint16_t color);

void ShowIntroScreen(uint16_t fc);

void drawPointer(uint16_t x0, uint16_t y0, uint16_t r, uint16_t theta,uint8_t thickness, uint16_t color);

void Draw_ARC(uint16_t x0, uint16_t y0, uint16_t r, uint16_t thetaStart,uint16_t thetaFinish, uint32_t Color);

void Draw_ARC_with_thickness(uint16_t x0, uint16_t y0, uint16_t r,uint16_t thetaStart, uint16_t thetaFinish, int32_t thickness, uint32_t Color);

void Draw_Travelling_Pointer(uint16_t x0, uint16_t y0, uint16_t r,uint16_t theta, uint8_t thickness, uint16_t Color);

void LCD_ShowScale(uint16_t fc);

void Draw_line_with_thickness(uint16_t x1, uint16_t y1, uint16_t x2,uint16_t y2, uint32_t Color, uint16_t thickness);

void drawArcPointer(uint16_t Arc_Pointer_Raduis, uint16_t Arc_Raduis,uint8_t Theta, uint16_t x0,
		uint16_t y0, uint8_t Thickness,uint16_t Color, uint16_t Bg_Color);

void LCD_Showbatt(uint16_t volt);

void Draw_ARC_with_thickness_2(uint16_t x0, uint16_t y0, uint16_t r,uint16_t thetaStart, uint16_t thetaFinish, int32_t thickness, uint32_t Color);

void drawBattery(uint16_t Level, uint16_t Color) ;

void drawRPMSlider(uint16_t Back_Ground_Color);

void drawRPMValue(void);

void drawLap(void);

void drawGear(uint8_t Gear_Number, uint8_t Size, uint16_t Back_Ground_Color);


#endif
