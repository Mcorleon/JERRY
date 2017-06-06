#ifndef	_OLED_H_
#define _OLED_H_

#define RESET  PTC18_OUT
#define DC     PTC19_OUT
#define D1     PTC17_OUT
#define D0     PTC16_OUT

 typedef unsigned char byte;
 typedef unsigned int word;
 extern byte beyond96x64[512];
 void LCD_WrDat(uint8 data);
 void LCD_Set_Pos(byte x, byte y);
 void LCD_Init(void);
 void LCD_CLS(void);
 void LCD_P6x8Str(byte x,byte y,byte ch[]);
 void LED_P6x8Char(byte x,byte y,byte ch);
 void LCD_P8x16Str(byte x,byte y,byte ch[]);
 void LCD_P14x16Str(byte x,byte y,byte ch[]);
 void LCD_Print(byte x, byte y, byte ch[]);
 void LCD_PutPixel(byte x,byte y);
 void LCD_Rectangle(byte x1,byte y1,byte x2,byte y2,byte gif);
 void Draw_BMP(byte x0,byte y0,byte x1,byte y1,byte bmp[]); 
 void LCD_Fill(byte dat);
 void Display(int16 i,uint8 x,uint8 y);
 void DisplayFloat(int16 i,uint8 x,uint8 y);
 void OLED_CCD_image_binaryzation_draw(uint8 x, uint8 y, uint8* s);
 uint8 PixelAverage(uint8 len, uint8 *data);
 uint8 CCD_Threshold_Find(uint8* s);
 //extern void CCD_display(uint8 num);
  void LED_PrintValueFP(uint8 x, uint8 y, unsigned int data, uint8 num);
 void LED_PrintValueF(unsigned char x, unsigned char y, float data, unsigned char num);
#endif


