#ifndef OLED_DRAW_H
#define OLED_DRAW_H
#include "oled_basic.h"
#include "oled_color.h"
#include "stdint.h"
void DrawPixel(int x,int y);
void DrawLine(int x1,int y1,int x2,int y2);
void DrawFastHLine(int x, int y, unsigned char w);
void DrawFastVLine(int x, int y, unsigned char h);
void DrawPolyLineTo(const TypeXY *points,int num);
void DrawFillRect1(int left,int top,int right,int bottom);
void DrawRect1(int left,int top,int right,int bottom);
void DrawFillRect2(int left,int top,int width,int height);
void DrawRect2(int left,int top,int width,int height);
void DrawCircle( int usX_Center, int usY_Center, int usRadius);
void DrawFillCircle( int usX_Center, int usY_Center, int usRadius);
void DrawCircleHelper(int x0, int y0, unsigned char r, unsigned char cornername);
void DrawFillCircleHelper(int x0, int y0, unsigned char r, unsigned char cornername, int delta);
void DrawArc(int x,int y,unsigned char r,int angle_start,int angle_end);
void DrawFillArc(int x,int y,unsigned char r,int angle_start,int angle_end);
void DrawRoundRect(int x, int y, unsigned char w, unsigned char h, unsigned char r);
void DrawfillRoundRect(int x, int y, unsigned char w, unsigned char h, unsigned char r);
void DrawEllipse(int x0,int y0,int a,int b);
void DrawFillEllipse(int x0,int y0,int a,int b);
void DrawEllipseRect( int x0, int y0, int x1, int y1);
void DrawTriangle(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2);
void DrawFillTriangle(int x0, int y0, int x1, int y1, int x2, int y2);
void DrawBitmap(int x, int y, const unsigned char *bitmap, unsigned char w, unsigned char h);
void DrawXBitmap(int x, int y, const unsigned char *bitmap, unsigned char w, unsigned char h);//此函数具体注释请参照'draw_api.h'内
void DrawSlowBitmap(int x, int y, const unsigned char *bitmap, unsigned char w, unsigned char h);
void DrawChar(int x, int y, unsigned char c);
void DrawString(int x, int y,char *str);
void DrawNum(unsigned char x,unsigned char y,unsigned int num,unsigned char len);
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t *cn);
void OLED_LOGO(void);
void OLED_Showdecimal(uint8_t x,uint8_t y,float num,uint8_t z_len,uint8_t f_len);

Type_color GetPixel(int x,int y);
int GetY(void);
int GetX(void);
TypeXY GetXY(void);
void MoveTo(int x,int y);
void LineTo(int x,int y);
#endif

