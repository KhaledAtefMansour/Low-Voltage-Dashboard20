/*
 * DrawingFunctions.c
 *
 *  Created on: Mar 19, 2020
 *      Author: Mostafa Gamal
 */
#include "DrawingFunctions.h"

volatile uint16_t RPM = 1000;
volatile uint16_t Old_RPM = 0;
volatile uint8_t Old_Gear_Number = 6;


void Draw_Circle(uint16_t x0, uint16_t y0, uint16_t r, uint32_t Color) {
	int d = 3 - 2 * r;
	int x = 0;
	int y = r;
	while (x <= y) {
		drawPixel((x + x0), (y + y0), Color); // first quad
		drawPixel((-x + x0), (y + y0), Color); // second quad
		drawPixel((-x + x0), (-y + y0), Color); // third quad
		drawPixel((x + x0), (-y + y0), Color); // forth quad
		drawPixel((y + x0), (x + y0), Color); // first quad
		drawPixel((-y + x0), (x + y0), Color); // second quad
		drawPixel((-y + x0), (-x + y0), Color); // third quad
		drawPixel((y + x0), (-x + y0), Color); // forth quad
		x++;
		if (d < 0) {
			d = d + 4 * x + 6;
		} else {
			d = d + 4 * (x - y) + 10;
			y--;
		}
	}
}
void LCD_Showthrmo(uint16_t color) {
	uint8_t temp1;
	uint8_t counter = 0;
	uint16_t index = 0;
	temp1 = thermo[index];
	for (uint16_t i = 0; i < 30; i++) {
		for (uint16_t j = 0; j < 80; j++) {
			if (temp1 & 0x01) {
				drawPixel(10 + j, 220 + i, color);
			}
			temp1 >>= 1;
			counter++;
			if (counter == 8) {
				index++;
				temp1 = thermo[index];
				counter = 0;
			}
		}
	}
}
void ShowIntroScreen(uint16_t fc) {
	fillScreen(BLACK);
	uint8_t temp1;
	uint8_t counter = 0;
	uint16_t index = 0;
	temp1 = IntroScreenBitmap[index];
	for (uint16_t i = 60; i < 260; i++) {
		for (uint16_t j = 0; j < 472; j++) {
			if ((temp1 & 0x01)) /* The condition here is inverted because the array of the picture was inverted */
			{
				drawPixel(j, i, fc);
			}
			temp1 >>= 1;
			counter++;
			if (counter == 8) {
				index++;
				temp1 = IntroScreenBitmap[index];
				counter = 0;
			}
		}
	}
	HAL_Delay(2000);
}
void drawPointer(uint16_t x0, uint16_t y0, uint16_t r, uint16_t theta,
		uint8_t thickness, uint16_t color) {
	int16_t x = x0 - r * cos(theta * 3.14 / 180);
	int16_t y = y0 - r * sin(theta * 3.14 / 180);

	fillTriangle((x0 - ((y - y0) * thickness / r)),
			(y0 - ((x0 - x) * thickness / r)),
			(x0 + ((y - y0) * thickness / r)),
			(y0 + ((x0 - x) * thickness / r)), x, y, color);
}
void Draw_ARC(uint16_t x0, uint16_t y0, uint16_t r, uint16_t thetaStart,
		uint16_t thetaFinish, uint32_t Color) {
	int16_t xFinish = r * cos(thetaFinish * 3.14 / 180);
	int16_t yFinish = r * sin(thetaFinish * 3.14 / 180);
	int16_t xStart = r * cos(thetaStart * 3.14 / 180);

	int d = 3 - 2 * r;
	int x = 0;
	int y = r;
	while (x <= y) {
		if (x <= xFinish && x >= xStart) {
			drawPixel((x + x0), (-y + y0), Color);
		} // first quad
		if (-x <= xFinish && -x >= xStart) {
			drawPixel((-x + x0), (-y + y0), Color); // second quad
		}
		if (y <= xFinish && y >= xStart) {
			drawPixel((y + x0), (-x + y0), Color); // third quad
		}
		if (-y <= xFinish && -y >= xStart) {
			drawPixel((-y + x0), (-x + y0), Color); // forth quad
		}
		if (d < 0) {
			d = d + 4 * x + 6;
		} else {
			d = d + 4 * (x - y) + 10;
			y--;
		}
		x++;
	}

}
void Draw_ARC_with_thickness(uint16_t x0, uint16_t y0, uint16_t r,
		uint16_t thetaStart, uint16_t thetaFinish, int32_t thickness, uint32_t Color) {
	for (int i = 0; i <= (thickness / 2); i += 1) {
		Draw_ARC(x0, y0, r + i, thetaStart, thetaFinish, Color);
		Draw_ARC(x0, y0, r - i, thetaStart, thetaFinish, Color);
	}

}
void Draw_Travelling_Pointer(uint16_t x0, uint16_t y0, uint16_t r,
		uint16_t theta, uint8_t thickness, uint16_t Color) {
	drawPointer(r * cos(theta * 3.14 / 180) + x0,
			y0 - r * sin(theta * 3.14 / 180), r / 5, 180 - theta, thickness,
			Color);
}
void LCD_ShowScale(uint16_t fc) {
	uint8_t temp1;
	uint8_t counter = 0;
	uint16_t index = 0;
	temp1 = scale[index];
	for (uint16_t i = 104; i < 320; i++) {
		for (uint16_t j = 224; j < 480; j++) {
			if (!(temp1 & 0x01)) /* The condition here is inverted because the array of the picture was inverted */
			{
				drawPixel(j, i, fc);
			}
			temp1 >>= 1;
			counter++;
			if (counter == 8) {
				index++;
				temp1 = scale[index];
				counter = 0;
			}
		}
	}
}
void Draw_line_with_thickness(uint16_t x1, uint16_t y1, uint16_t x2,
		uint16_t y2, uint32_t Color, uint16_t thickness) {
	for (int i = 0; i <= (thickness / 2); i++) {
		drawLine(x1, y1 + i, x2, y2 + i, Color);
		drawLine(x1, y1 - i, x2, y2 - i, Color);
	}
}

void drawArcPointer(uint16_t Arc_Pointer_Raduis, uint16_t Arc_Raduis,
		uint8_t Theta, uint16_t x0, uint16_t y0, uint8_t Thickness,
		uint16_t Color, uint16_t Bg_Color) {
	drawPointer(x0 + Arc_Raduis * cos(Theta * 22 / (180 * 7)),
			y0 + Arc_Raduis * sin(Theta * 22 / (180 * 7)), Arc_Pointer_Raduis,
			Theta, Thickness, Color);
}
void LCD_Showbatt(uint16_t volt) {
	uint16_t color;
	if (volt >= 11) {
		color = GREEN;
	} else if (volt < 11 && volt >= 10) {
		color = YELLOW;
	} else if (volt < 10) {
		color = RED;
	}
	uint8_t temp1;
	uint8_t counter = 0;
	uint16_t index = 0;
	temp1 = battBitmaps[index];

	for (uint16_t i = 0; i < 30; i++) {
		for (uint16_t j = 0; j < 48; j++) {
			if (temp1 & 0x01) {
				drawPixel(10 + j, 270 + i, color);
			}
			temp1 >>= 1;
			counter++;
			if (counter == 8) {
				index++;
				temp1 = battBitmaps[index];
				counter = 0;
			}
		}
	}
}
void Draw_ARC_with_thickness_2(uint16_t x0, uint16_t y0, uint16_t r,
		uint16_t thetaStart, uint16_t thetaFinish, int32_t thickness, uint32_t Color)
{
	Draw_line_with_thickness(x0, y0, r * sin(thetaFinish), r * cos(thetaFinish),
			Color, 6);
	for (int i = 0; i <= (thickness / 2); i += 1) {
		Draw_ARC(x0, y0, r + i, thetaStart, thetaFinish, Color);
		Draw_ARC(x0, y0, r - i, thetaStart, thetaFinish, Color);
	}
	Draw_line_with_thickness(x0, y0, r * sin(thetaFinish), r * cos(thetaFinish),
			BLACK, 6);
}
void drawBattery(uint16_t Level, uint16_t Color) {
	fillTriangle(320, 480, 320, 480 - Level, 320 - (float) Level / 1.732,
			480 - Level, Color);
}


void drawRPMSlider(uint16_t Back_Ground_Color) {
	uint16_t Level = RPM / 29;
	if (Old_RPM >= RPM)
		fillRect(Level, 0, Old_RPM / 29 - Level, 45, Back_Ground_Color);
	if (Level <= 300)
		fillRect(0, 0, Level, 45, WHITE);
	else if (Level <= 400 && Level > 300)
		fillRect(0, 0, Level, 45, GREEN);
	else
		fillRect(0, 0, Level, 45, RED);
}
drawRPMValue() {
	char rpm[5];
	itoa(Old_RPM, rpm, 10);
	printnewtstr(100, 110, BLACK, &mono12x7bold, 2, rpm);
	Old_RPM = RPM;
	itoa(RPM, rpm, 10);
	printnewtstr(100, 110, RED, &mono12x7bold, 2, rpm);

}
void drawLap() {
	Draw_line_with_thickness(10, 130, 80, 100, WHITE, 2);
	printnewtstr(130, 50, WHITE, &mono12x7, 1, "22");
	printnewtstr(160, 40, WHITE, &mono9x7, 1, "Lap");
}
void drawGear(uint8_t Gear_Number, uint8_t Size, uint16_t Back_Ground_Color) {

	char gear[2];
	if (Old_Gear_Number == 6)
		printnewtstr(290, 110, Back_Ground_Color, &mono18x7bold, Size, "N");
	else {
		itoa(Old_Gear_Number, gear, 10);
		printnewtstr(290, 110, Back_Ground_Color, &mono18x7bold, Size, gear);
	}
	Old_Gear_Number = Gear_Number;
	if (Gear_Number == 6)
		printnewtstr(290, 110, RED, &mono18x7bold, Size, "N");
	else {
		itoa(Gear_Number, gear, 10);
		printnewtstr(290, 110, RED, &mono18x7bold, Size, gear);
	}
}


