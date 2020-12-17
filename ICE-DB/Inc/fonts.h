#ifndef __FONTS_H
#define __FONTS_H


#define NULL ((void *)0)

#include "stdint.h"

typedef struct {
	uint16_t bitmapOffset;     ///< Pointer into GFXfont->bitmap
	uint16_t  width;            ///< Bitmap dimensions in pixels
    uint16_t  height;           ///< Bitmap dimensions in pixels
	uint8_t  xAdvance;         ///< Distance to advance cursor (x axis)
	int8_t   xOffset;          ///< X dist from cursor pos to UL corner
    int8_t   yOffset;          ///< Y dist from cursor pos to UL corner
} GFXglyph;

/// Data stored for FONT AS A WHOLE
typedef struct {
	uint8_t  *bitmap;      ///< Glyph bitmaps, concatenated
	GFXglyph *glyph;       ///< Glyph array
	uint8_t   first;       ///< ASCII extents (first char)
    uint8_t   last;        ///< ASCII extents (last char)
	uint8_t   yAdvance;    ///< Newline distance (y axis)
} GFXfont;

extern GFXfont *gfxFont;
extern const GFXfont speedometer;
extern const GFXfont mono9x7;
extern const GFXfont mono9x7bold;
extern const GFXfont mono12x7;
extern const GFXfont mono12x7bold;
extern const GFXfont mono18x7;
extern const GFXfont mono18x7bold;
extern const unsigned char thermo[];
extern const unsigned char battBitmaps[];
extern const unsigned char scale[];
extern const uint8_t IntroScreenBitmap[];

#endif

