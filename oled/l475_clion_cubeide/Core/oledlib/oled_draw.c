/*
	Copyright (c) [2019] [一只程序缘 jiezhuo]
	[https://gitee.com/jiezhuonew/oledlib] is licensed under the Mulan PSL v1.
	You can use this software according to the terms and conditions of the Mulan PSL v1.
	You may obtain a copy of Mulan PSL v1 at:
		http://license.coscl.org.cn/MulanPSL
	THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND, EITHER EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT, MERCHANTABILITY OR FIT FOR A PARTICULAR
	PURPOSE.
	See the Mulan PSL v1 for more details.
	
	
	此c文件用于画图和字符操作(高级操作)
	(主要)由打点为基础
	再 线 折线
	再画和填充 矩形 三角形 圆 椭圆 圆角矩形
	然后是 图片 字符 字符串
	最后是 汉字
*/

#include "oled_draw.h"
#include "oled_buffer.h"
#include "oled_config.h"
#include "stdlib.h"
#include "math.h"
#include "oled_bmp.h"
#include "oled_font.h"


//画图光标
static int  _pointx=0;
static int 	_pointy=0;
//extern unsigned char ScreenBuffer[SCREEN_PAGE_NUM][SCREEN_COLUMN];
////////////////////////////////////////////////////////////////////
//以下4个函数是对当前光标的设置 供以下绘制函数调用 用户不直接使用
void MoveTo(int x,int y)
{
	_pointx=x;
	_pointy=y;
}

TypeXY GetXY(void)
{
	
	TypeXY m;
	m.x=_pointx;
	m.y=_pointy;
	return m;
}
int GetX(void)
{
	return _pointx;
}
int GetY(void)
{
	return _pointy;
}
void LineTo(int x,int y)
{
	DrawLine(_pointx,_pointy,x,y);
	_pointx=x;
	_pointy=y;

}
///////////////////////////////////////////////////////////////////////////////////
//绘制一个点
void DrawPixel(int x,int y)
{
	SetPointBuffer(x,y,GetDrawColor());
}
//得到某个点的颜色
Type_color GetPixel(int x,int y)
{
	if(GetPointBuffer(x,y))
		return pix_white;
	else
		return pix_black;
}

////////////////////////////////////////////////////////////////////////////////////
//划线 
//参数:起点坐标 终点坐标
void DrawLine(int x1,int y1,int x2,int y2)
{
	unsigned short us; 
	unsigned short usX_Current, usY_Current;
	
	int lError_X = 0, lError_Y = 0, lDelta_X, lDelta_Y, lDistance; 
	int lIncrease_X, lIncrease_Y; 	

	lDelta_X = x2 - x1; //计算坐标增量 
	lDelta_Y = y2 - y1; 

	usX_Current = x1; 
	usY_Current = y1; 

	if ( lDelta_X > 0 ) 
		lIncrease_X = 1; 			//设置单步正方向 
	else if ( lDelta_X == 0 ) 
		lIncrease_X = 0;			//垂直线 
	else 
	{ 
		lIncrease_X = -1;			//设置单步反方向 
		lDelta_X = - lDelta_X;
	} 

	//Y轴的处理方式与上图X轴的处理方式同理
	if ( lDelta_Y > 0 )
		lIncrease_Y = 1; 
	else if ( lDelta_Y == 0 )
		lIncrease_Y = 0;			//水平线 
	else 
	{
		lIncrease_Y = -1;
		lDelta_Y = - lDelta_Y;
	} 

	//选取不那么陡的方向依次画点
	if ( lDelta_X > lDelta_Y )
		lDistance = lDelta_X;
	else 
		lDistance = lDelta_Y; 

	//依次画点 进入缓存区 画好后再刷新缓冲区就好啦
	for ( us = 0; us <= lDistance + 1; us ++ )					//画线输出 
	{
		SetPointBuffer(usX_Current,usY_Current,GetDrawColor());	//画点 
		lError_X += lDelta_X ; 
		lError_Y += lDelta_Y ; 

		if ( lError_X > lDistance ) 
		{
			lError_X -= lDistance; 
			usX_Current += lIncrease_X; 
		}  

		if ( lError_Y > lDistance ) 
		{ 
			lError_Y -= lDistance; 
			usY_Current += lIncrease_Y; 
		} 		
	}  
}
//快速划线 专用于画横平竖直的线 提高效率
void DrawFastHLine(int x, int y, unsigned char w)
{
	int end = x+w;
	int a;
	
	Type_color color =GetDrawColor();
	for ( a = MAX(0,x); a < MIN(end,SCREEN_COLUMN); a++)
	{
		SetPointBuffer(a,y,color);
	}
}
//快速划线 专用于画横平竖直的线 提高效率
void DrawFastVLine(int x, int y, unsigned char h)
{
	int end = y+h;
	int a;
	
	Type_color color =GetDrawColor();
	for (a = MAX(0,y); a < MIN(end,SCREEN_ROW); a++)
	{
		SetPointBuffer(x,a,color);
	}
}
//绘制折线 开始和转折点的坐标 总点个数
void DrawPolyLineTo(const TypeXY *points,int num)
{
	int i=0;
	MoveTo(points[0].x,points[0].y);
	for(i=1;i<num;i++)
	{
		LineTo(points[i].x,points[i].y);
	}
}

///////////////////////////////////////////////////////////////////////////////////////
//使用对角点填充矩形
void DrawRect1(int left,int top,int right,int bottom)
{
	DrawLine ( left, top, right, top );
	DrawLine ( left, bottom , right , bottom );
	DrawLine ( left, top, left, bottom );
	DrawLine ( right , top, right , bottom );	
}
//功能:使用对角点填充矩形
void DrawFillRect1(int left,int top,int right,int bottom)
{
	DrawRect1(left,top,right,bottom);
	FillRect(left+1,top+1,right-left-1,bottom-top-1);	
}
//左上角坐标 矩形宽高
void DrawRect2(int left,int top,int width,int height)
{
		DrawLine ( left, top, left+width-1, top );
		DrawLine ( left, top+height-1 , left+width-1 , top+height-1 );
		DrawLine ( left, top, left, top+height-1);
		DrawLine ( left+width-1 , top, left+width-1 , top+height-1);	
}
//填充矩形
void DrawFillRect2(int left,int top,int width,int height)
{
	//先用上面的函数画外框
	DrawRect2(left,top,width,height);
	//然后填充实心
	FillRect(left+1,top+1,width-1,height-1);	
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//画圆
void DrawCircle ( int usX_Center, int usY_Center, int usRadius)
{
	short sCurrentX, sCurrentY;
	short sError;
	sCurrentX = 0; sCurrentY = usRadius;	  
	sError = 3 - ( usRadius << 1 );     //判断下个点位置的标志
	
	while ( sCurrentX <= sCurrentY )
	{
		//此处画圆打点的方法和画圆角矩形的四分之一圆弧的函数有点像
		SetPointBuffer ( usX_Center + sCurrentX, usY_Center + sCurrentY	,GetDrawColor());             //1，研究对象
		SetPointBuffer ( usX_Center - sCurrentX, usY_Center + sCurrentY ,GetDrawColor());             //2      
		SetPointBuffer ( usX_Center - sCurrentY, usY_Center + sCurrentX ,GetDrawColor());             //3
		SetPointBuffer ( usX_Center - sCurrentY, usY_Center - sCurrentX ,GetDrawColor());             //4
		SetPointBuffer ( usX_Center - sCurrentX, usY_Center - sCurrentY ,GetDrawColor());             //5       
		SetPointBuffer ( usX_Center + sCurrentX, usY_Center - sCurrentY ,GetDrawColor());             //6
		SetPointBuffer ( usX_Center + sCurrentY, usY_Center - sCurrentX ,GetDrawColor());             //7 
		SetPointBuffer ( usX_Center + sCurrentY, usY_Center + sCurrentX ,GetDrawColor());             //0
		sCurrentX ++;		
		if ( sError < 0 ) 
			sError += 4 * sCurrentX + 6;	  
		else
		{
			sError += 10 + 4 * ( sCurrentX - sCurrentY );   
			sCurrentY --;
		} 		
	}
}
//填充圆
void DrawFillCircle( int usX_Center, int usY_Center, int r)
{
	DrawFastVLine(usX_Center, usY_Center-r, 2*r+1);
	DrawFillCircleHelper(usX_Center,usY_Center, r, 3, 0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//画部分圆
//圆心坐标 半径 4份圆要画哪一份或哪几份
void DrawCircleHelper(int x0, int y0, unsigned char r, unsigned char cornername)
{
	int f = 1 - r;
	int ddF_x = 1;
	int ddF_y = -2 * r;
	int x = 0;
	int y = r;
	
	Type_color color=GetDrawColor();
	while (x<y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}

		x++;
		ddF_x += 2;
		f += ddF_x;

		if (cornername & 0x4)//右上
		{
			//此处画圆的方式是交替打点 从2边打到中间 最终x<y就打完点跳出循环
			SetPointBuffer(x0 + x, y0 + y,color);
			SetPointBuffer(x0 + y, y0 + x,color);
		}
		if (cornername & 0x2)//右下
		{
			SetPointBuffer(x0 + x, y0 - y, color);
			SetPointBuffer(x0 + y, y0 - x, color);
		}
		if (cornername & 0x8)//左上
		{
			SetPointBuffer(x0 - y, y0 + x, color);
			SetPointBuffer(x0 - x, y0 + y, color);
		}
		if (cornername & 0x1)//左下
		{
			SetPointBuffer(x0 - y, y0 - x, color);
			SetPointBuffer(x0 - x, y0 - y, color);
		}
	}
}
//填充2个四分之一圆和中间的矩形
//此函数专用于画圆角矩形
//右上四分之一圆或左下四分之一圆的圆心 半径 中间矩形的高
void DrawFillCircleHelper(int x0, int y0, unsigned char r, unsigned char cornername, int delta)
{
	int f = 1 - r;
	int ddF_x = 1;
	int ddF_y = -2 * r;
	int x = 0;
	int y = r;
	
	Type_color color=GetDrawColor();
	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}

		x++;
		ddF_x += 2;
		f += ddF_x;

		if (cornername & 0x1)//填充右边的2个四分之一圆和中间的矩形
		{
			DrawFastVLine(x0+x, y0-y, 2*y+1+delta);
			DrawFastVLine(x0+y, y0-x, 2*x+1+delta);
		}
		if (cornername & 0x2)//填充左边的2个四分之一圆和中间的矩形
		{
			DrawFastVLine(x0-x, y0-y, 2*y+1+delta);
			DrawFastVLine(x0-y, y0-x, 2*x+1+delta);
		}
	}
}
//绘制一个圆弧
//x,y:圆弧中心坐标
//r:圆弧的半径
//angle_start:圆弧起始角度
//angle_end:圆弧终止角度
//注意：慎用此方法，此方法还需优化。
void DrawArc(int x,int y,unsigned char r,int angle_start,int angle_end)
{
	float i=0;
	
	TypeXY m,temp;
	temp=GetXY();
	SetRotateCenter(x,y);
	SetAnggleDir(0);
	if(angle_end>360)
		angle_end=360;
	SetAngle(0);
	m=GetRotateXY(x,y+r);
	MoveTo(m.x,m.y);
	for(i=angle_start;i<angle_end;i+=5)
	{
		SetAngle(i);
		m=GetRotateXY(x,y+r);
		LineTo(m.x,m.y);
	}
	LineTo(x+r,y);
	MoveTo(temp.x,temp.y);
}
void DrawFillArc(int x,int y,unsigned char r,int angle_start,int angle_end)
{
	return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//画圆角矩形
void DrawRoundRect(int x, int y, unsigned char w, unsigned char h, unsigned char r)
{
	//画出边缘 因为边缘是直线 所以专用高效率函数
	DrawFastHLine(x+r, y, w-2*r); 		// Top
	DrawFastHLine(x+r, y+h-1, w-2*r); 	// Bottom
	DrawFastVLine(x, y+r, h-2*r); 		// Left
	DrawFastVLine(x+w-1, y+r, h-2*r); 	// Right
	//画出四个圆角
	DrawCircleHelper(x+r, y+r, r, 1);
	DrawCircleHelper(x+w-r-1, y+r, r, 2);
	DrawCircleHelper(x+w-r-1, y+h-r-1, r, 4);
	DrawCircleHelper(x+r, y+h-r-1, r, 8);
}
//画实心圆角矩形
void DrawfillRoundRect(int x, int y, unsigned char w, unsigned char h, unsigned char r)
{
  //画实心矩形
  DrawFillRect2(x+r, y, w-2*r, h);

  //再填充左右两边
  DrawFillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1);	//右上角的圆心
  DrawFillCircleHelper(x+r, y+r, r, 2, h-2*r-1);		//左下角的圆心
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//画椭圆
//圆心 2个轴长
void DrawEllipse(int xCenter,int yCenter,int Rx,int Ry)
{
	int Rx2=Rx*Rx;
	int Ry2=Ry*Ry;
	int twoRx2=2*Rx2;
	int twoRy2=2*Ry2;
	int p;
	int x=0;
	int y=Ry;
	int px = 0;
	int py = twoRx2*y;
	
	//椭圆最下面的点
	SetPointBuffer(xCenter+x,yCenter+y,GetDrawColor());//因为此时x=0 俩个点为同一个 原作这样写的 那就这样吧
	SetPointBuffer(xCenter-x,yCenter+y,GetDrawColor());
	//椭圆最上面的点
	SetPointBuffer(xCenter+x,yCenter-y,GetDrawColor());
	SetPointBuffer(xCenter-x,yCenter-y,GetDrawColor());
	
	//Region?1 画出上下的线条 说实话我也没看懂了 算法大佬
	p=(int)(Ry2-Rx2*Ry+0.25*Rx2);
	while(px<py)
	{
		x++;
		px+=twoRy2;
		if(p<0)
			p+=Ry2+px;
		else
		{
			y--;
			py-=twoRx2;
			p+=Ry2+px-py;
		}
		SetPointBuffer(xCenter+x,yCenter+y,GetDrawColor());
		SetPointBuffer(xCenter-x,yCenter+y,GetDrawColor());
		SetPointBuffer(xCenter+x,yCenter-y,GetDrawColor());
		SetPointBuffer(xCenter-x,yCenter-y,GetDrawColor());
	}
	
	//Region?2
	p=(int)(Ry2*(x+0.5)*(x+0.5)+Rx2*(y-1)*(y-1)-Rx2*Ry2);
	while(y>0)
	{
		y--;
		py-=twoRx2;
		if(p>0)
			p+=Rx2-py;
		else
		{
			x++;
			px+=twoRy2;
			p+=Rx2-py+px;
		}
		SetPointBuffer(xCenter+x,yCenter+y,GetDrawColor());
		SetPointBuffer(xCenter-x,yCenter+y,GetDrawColor());
		SetPointBuffer(xCenter+x,yCenter-y,GetDrawColor());
		SetPointBuffer(xCenter-x,yCenter-y,GetDrawColor());
	}
}
//填充一个椭圆 参数同上
void DrawFillEllipse(int x0, int y0,int rx,int ry)
{
	int x, y;
	int xchg, ychg;
	int err;
	int rxrx2;
	int ryry2;
	int stopx, stopy;

	rxrx2 = rx;
	rxrx2 *= rx;
	rxrx2 *= 2;

	ryry2 = ry;
	ryry2 *= ry;
	ryry2 *= 2;

	x = rx;
	y = 0;

	xchg = 1;
	xchg -= rx;
	xchg -= rx;
	xchg *= ry;
	xchg *= ry;

	ychg = rx;
	ychg *= rx;

	err = 0;

	stopx = ryry2;
	stopx *= rx;
	stopy = 0;

	while( stopx >= stopy )
	{
		DrawFastVLine( x0+x, y0-y, y+1);
		DrawFastVLine( x0-x, y0-y, y+1);
		DrawFastVLine( x0+x, y0, y+1);
		DrawFastVLine( x0-x, y0, y+1);
		y++;
		stopy += rxrx2;
		err += ychg;
		ychg += rxrx2;
		if ( 2*err+xchg > 0 )
		{
			x--;
			stopx -= ryry2;
			err += xchg;
			xchg += ryry2;      
		}
	}

	x = 0;
	y = ry;

	xchg = ry;
	xchg *= ry;

	ychg = 1;
	ychg -= ry;
	ychg -= ry;
	ychg *= rx;
	ychg *= rx;

	err = 0;
	stopx = 0;
	stopy = rxrx2;
	stopy *= ry;

	while( stopx <= stopy )
	{
		DrawFastVLine( x0+x, y0-y, y+1);
		DrawFastVLine( x0-x, y0-y, y+1);
		DrawFastVLine( x0+x, y0, y+1);
		DrawFastVLine( x0-x, y0, y+1);

		x++;
		stopx += ryry2;
		err += xchg;
		xchg += ryry2;
		if ( 2*err+ychg > 0 )
		{
			y--;
			stopy -= rxrx2;
			err += ychg;
			ychg += rxrx2;
		}
	}
}
//功能:绘制一个矩形内切椭圆
//x0,y0:矩形左上角坐标
//x1,y1:矩形右下角坐标
void DrawEllipseRect( int x0, int y0, int x1, int y1)
{
	int a = abs(x1 - x0);
	int b = abs(y1 - y0);	//get diameters
	int b1 = b&1;
	long dx = 4*(1-a)*b*b;
	long dy = 4*(b1+1)*a*a;
	long err = dx+dy+b1*a*a;
	long e2;
	
	if (x0 > x1) { x0 = x1; x1 += a; }
	if (y0 > y1) { y0 = y1; } 
	y0 += (b+1)/2;
	y1 = y0-b1;
	a *= 8*a;
	b1 = 8*b*b;
	
	do {
		DrawPixel( x1, y0);
		DrawPixel( x0, y0);
		DrawPixel( x0, y1);
		DrawPixel( x1, y1);
		e2 = 2*err;
		if (e2 >= dx) {
			x0++;
			x1--;
			err += dx += b1;
		}
		if (e2 <= dy) {
			y0++;
			y1--;
			err += dy += a;
		}
	} while (x0 <= x1);
	
	while (y0-y1 < b) {
		DrawPixel( x0-1, y0);
		DrawPixel( x1+1, y0++);
		DrawPixel( x0-1, y1);
		DrawPixel( x1+1, y1--);
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//画三角形
void DrawTriangle(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2)
{
	//很简单  就是画3条任意线
	DrawLine(x0, y0, x1, y1);
	DrawLine(x1, y1, x2, y2);
	DrawLine(x2, y2, x0, y0);
}
//填充三角形
void DrawFillTriangle(int x0, int y0, int x1, int y1, int x2, int y2)
{
	int a, b, y, last;
	int dx01, dy01,dx02,dy02,dx12,dy12,sa = 0,sb = 0;
	
	Type_color tmpcolor;
	tmpcolor =GetDrawColor();
	SetDrawColor(GetFillColor());
	if (y0 > y1)
	{
		SWAP(y0, y1); SWAP(x0, x1);
	}
	if (y1 > y2)
	{
		SWAP(y2, y1); SWAP(x2, x1);
	}
	if (y0 > y1)
	{
		SWAP(y0, y1); SWAP(x0, x1);
	}
	if(y0 == y2)
	{
		a = b = x0;
		if(x1 < a)
		{
			a = x1;
		}
		else if(x1 > b)
		{
			b = x1;
		}
		if(x2 < a)
		{
			a = x2;
		}
		else if(x2 > b)
		{
			b = x2;
		}
		DrawFastHLine(a, y0, b-a+1);
		return;
	}
	dx01 = x1 - x0,
	dy01 = y1 - y0,
	dx02 = x2 - x0,
	dy02 = y2 - y0,
	dx12 = x2 - x1,
	dy12 = y2 - y1,
	sa = 0,
	sb = 0;
	if (y1 == y2)
	{
		last = y1;   // Include y1 scanline
	}
	else
	{
		last = y1-1; // Skip it
	}

	for(y = y0; y <= last; y++)
	{
		a   = x0 + sa / dy01;
		b   = x0 + sb / dy02;
		sa += dx01;
		sb += dx02;

		if(a > b)
		{
			SWAP(a,b);
		}
		DrawFastHLine(a, y, b-a+1);
	}
	sa = dx12 * (y - y1);
	sb = dx02 * (y - y0);
	for(; y <= y2; y++)
	{
		a   = x1 + sa / dy12;
		b   = x0 + sb / dy02;
		sa += dx12;
		sb += dx02;
		if(a > b)
		{
			SWAP(a,b);
		}
		DrawFastHLine(a, y, b-a+1);
	}
	SetDrawColor(tmpcolor);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//画一幅画，PCtoLCD2002参数请选择[阴码 列行式 逆向]
//起点坐标x y 图像取模数组 图像宽高w h
void DrawBitmap(int x, int y, const unsigned char *bitmap, unsigned char w, unsigned char h)
{
	int iCol,a;
	int yOffset = abs(y) % 8;
	int sRow = y / 8;
	int rows = h/8;

	if(x+w < 0 || x > SCREEN_COLUMN-1 || y+h < 0 || y > SCREEN_ROW-1)
		return;
	if(y < 0)
	{
		sRow--;
		yOffset = 8 - yOffset;
	}

	if(h%8!=0) rows++;
	for(a = 0; a < rows; a++) 
	{
		int bRow = sRow + a;
		if(bRow > (SCREEN_ROW/8)-1) break;
		if(bRow > -2) 
		{
			for (iCol = 0; iCol<w; iCol++) 
			{
				if (iCol + x > (SCREEN_COLUMN-1)) break;
				if (iCol + x >= 0) 
				{
					if (bRow >= 0) 
					{
						if(GetDrawColor() == pix_white)
						{
							unsigned char temp = ReadByteBuffer(bRow,x + iCol);
							temp|=pgm_read_byte(bitmap+(a*w)+iCol) << yOffset;
							WriteByteBuffer(bRow,x + iCol,temp);
						}
						else if(GetDrawColor() == pix_black)
						{
							unsigned char temp = ReadByteBuffer(bRow,x + iCol);
							temp&=~(pgm_read_byte(bitmap+(a*w)+iCol) << yOffset);
							WriteByteBuffer(bRow,x + iCol,temp);
						}
						else
						{
							unsigned char temp = ReadByteBuffer(bRow,x + iCol);
							temp^=(pgm_read_byte(bitmap+(a*w)+iCol) << yOffset);
							WriteByteBuffer(bRow,x + iCol,temp);
						}
					}
					if (yOffset && bRow<(SCREEN_ROW/8)-1 && bRow > -2) 
					{
						if(GetDrawColor() == pix_white) 
						{
							unsigned char temp = ReadByteBuffer(bRow+1,x + iCol);
							temp|=pgm_read_byte(bitmap+(a*w)+iCol) >> (8-yOffset);
							WriteByteBuffer(bRow+1,x + iCol,temp);
						}
						else if (GetDrawColor() == pix_black)
						{
							unsigned char temp = ReadByteBuffer(bRow+1,x + iCol);
							temp&=~(pgm_read_byte(bitmap+(a*w)+iCol) >> (8-yOffset));
							WriteByteBuffer(bRow+1,x + iCol,temp);
						}
						else
						{
							unsigned char temp = ReadByteBuffer(bRow+1,x + iCol);
							temp^=pgm_read_byte(bitmap+(a*w)+iCol) >> (8-yOffset);
							WriteByteBuffer( bRow+1,x + iCol,temp);						
						}
					}
				}
			}
		}
	}
}

//画一幅画，PCtoLCD2002参数请选择[阴码 逐行式 逆向]，具体描述参见[draw_api.h]
//起点坐标x y 图像取模数组 图像宽高w h
void DrawXBitmap(int x, int y, const unsigned char *bitmap, unsigned char w, unsigned char h)
{
  // no need to dar at all of we're offscreen
  int xi, yi, byteWidth = (w + 7) / 8;
  if (x + w < 0 || x > SCREEN_COLUMN - 1 || y + h < 0 || y > SCREEN_ROW - 1)
    return;
  for (yi = 0; yi < h; yi++) {
    for (xi = 0; xi < w; xi++ ) {
      if (pgm_read_byte(bitmap + yi * byteWidth + xi / 8) & (1 << (xi & 7))) {
        SetPointBuffer(x + xi, y + yi, GetDrawColor());
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//显示一个字符
//关于字体尺寸及使用请看SetFontSize()的注释
//当size=0时 x为第几行 y为第几列
void DrawChar(int x, int y, unsigned char c)
{
	int i,j;
	unsigned char draw_background,bg,a,b,size,color;
	
	size=GetFontSize();		//字体尺寸
	color=GetDrawColor();	//字体颜色 1白0黑
	bg=GetTextBkMode();		//写字的时候字的背景的颜色 1白0黑
	draw_background= bg != color;	//这两个颜色要不一样字才看得到
	
	if(!size)//默认字符大小
	{
		
		
		unsigned char chr=0,i=0;
		chr = c - ' ';//得到偏移后的值
		if(x>128-1)
		{
			x=0;
			y = y + 2;
		}
//		if(Char_Size == 16)
//		{
			for(i=0;i<8;i++)
//			{
//				if(Color_Turn)
//					show_mem[y][x+i] = ~F8X16[c*16+i];+i
//				else
				WriteByteBuffer(y,x+i,F8X16[chr*16+i]);
//				ScreenBuffer[SCREEN_PAGE_NUM][SCREEN_COLUMN]=F8X16[c*16+i];
//					show_mem[y][x+i] = F8X16[c*16+i];
//			}
			for(i=0;i<8;i++)
			{
//				if(Color_Turn)
//					show_mem[y+1][x+i] = ~F8X16[c*16+i+8];
//				else
				WriteByteBuffer(y+1,x+i,F8X16[chr*16+i+8]);

//					show_mem[y+1][x+i] = F8X16[c*16+i+8];
			}
//		}
//		else
//		{
//			for(i=0;i<6;i++)
//			{
//				if(Color_Turn)
//					show_mem[y][x+i] = ~F6x8[c][i];
//				else
//					show_mem[y][x+i] = F6x8[c][i];
//			}
//		}

		
		
		
		
		
		
		
		
		
		
		
		
//		if ((x >= SCREEN_COLUMN) ||         // Clip right
//    (y >= SCREEN_ROW) ||        // Clip bottom
//    ((x + 5) < 0) ||   // Clip left
//    ((y + 8) < 0)    // Clip top
//		)
//  {
//    return;
//  }

//  for ( i=0; i<6; i++ )
//  {
//    int line;
//    if (i == 5)
//    {
//      line = 0x0;
//    }
//    else
//    {
//      line = pgm_read_byte(font5x7+(c*5)+i);
//    }

//    for ( j = 0; j<8; j++)
//    {
//      unsigned char draw_color = (line & 0x1) ? color : bg;

//      if (draw_color || draw_background) {
//        for ( a = 0; a < 1; a++ ) {
//          for ( b = 0; b < 1; b++ ) {
//            SetPointBuffer(x + i + a, y + j + b, draw_color);
//          }
//        }
//      }
//      line >>= 1;
//    }
//  }
	
	
	
//		if ((x >= SCREEN_COLUMN) ||         // Clip right
//		(y >= SCREEN_ROW) ||        		// Clip bottom
//		((x + 5 * size - 1) < 0) ||   		// Clip left
//		((y + 8 * size - 1) < 0)    		// Clip top
//		)
//			return;
//		c=c-' ';			//得到偏移后的位置
//		for(i=0;i<8;i++)
//			WriteByteBuffer(x,y+i,F8X16[c*16+i]);
//		for(i=0;i<8;i++)
//			WriteByteBuffer(x+1,y+i,F8X16[c*16+i+8]);
	}
	
	
	
	
	

	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	else//使用原作粗体字符
	{
		//判断一个字符的上下左右是否超出边界范围
		if ((x >= SCREEN_COLUMN) ||         // Clip right
		(y >= SCREEN_ROW) ||        		// Clip bottom
		((x + 5 * size - 1) < 0) ||   		// Clip left
		((y + 8 * size - 1) < 0)    		// Clip top
		)
			return;

		for (i=0; i<6; i++)
		{
			int line;
			//一个字符在font5x7中由一行6个char表示
			//line为这个字符的第某行内容
			if (i == 5)
				line = 0x0;
			else
				line = pgm_read_byte(font5x7+(c*5)+i);
			
			for (j=0; j<8; j++)
			{
				unsigned char draw_color = (line & 0x1) ? color : bg;//目前需要填充的颜色是0 就是背景色 1就是字体色

				//不同号大小的字体只是最基础字体的放大倍数 这点要注意
				//比如基础字是1个像素 放大后就是4个像素 再就是9个像素 达到马赛克的放大效果
				if (draw_color || draw_background) 
					for ( a = 0; a < size; a++ )
						for ( b = 0; b < size; b++ )
							SetPointBuffer(x + (i * size) + a, y + (j * size) + b, draw_color);

				line >>= 1;
			}
		}
	}
}
//显示字符串 就是显示多次显示字符
void DrawString(int x, int y,char *str)
{
	unsigned char j=0,tempx=x,tempy=y;
	unsigned char size=GetFontSize();
	
	if(!size)//默认字体
	{
		while (str[j]!='\0')
		{		
		//if(str[j]=='\n'||tempx+size*6>127)
			if(str[j]=='\n')
			{	
				tempy+=2;
				tempx=x;
				j++;
				continue;
			}
			DrawChar(tempx,tempy,str[j]);
			tempx+=8;
			if (tempx > 120) //TextSize8x16如果一行不够显示了，从下一行继续显示
			{
				tempx = 0;
				tempy+=2;
			}

			j++;
		}
		
//		while (str[j]!='\0')
//		{
//			DrawChar(tempx,tempy,str[j]);
//			tempx+=6;
////			if(tempx>120){tempx=0;tempy+=2;}
//			j++;
//		}
	}
	else//使用原作粗体字符
	{
		while (str[j]!='\0')
		{
			if(str[j]=='\n')
			{
				tempy+=8*size;
				tempx=x;
				j++;
				continue;
			}
			DrawChar(tempx,tempy,str[j]);
			tempx+=size*6;
			if (tempx > 120) //TextSize8x16如果一行不够显示了，从下一行继续显示
			{
				tempx = 0;
				tempy+=8*size;
			}
			j++;
		}
	}
}
//显示数字 就是多次显示数字的字符
void DrawNum(unsigned char x,unsigned char y,unsigned int num,unsigned char len)
{         	
	unsigned char t,temp;
	unsigned char enshow=0;			
	unsigned char size=GetFontSize();
	
	if(!size)
	{
		for(t=0;t<len;t++)
		{
			temp=(num/oled_pow(10,len-t-1))%10;
			if(enshow==0&&t<(len-1))
			{
				if(temp==0)
				{
					DrawChar(x+8*t,y,' ');
					continue;
				}
				else 
					enshow=1; 
			}
			DrawChar(x+8*t,y,temp+'0'); 
		}
	}
	else
	{
		for(t=0;t<len;t++)
		{
			temp=(num/oled_pow(10,len-t-1))%10;
			if(enshow==0&&t<(len-1))
			{
				if(temp==0)
				{
					DrawChar(x+(size*6)*t,y,'0');
					continue;
				}else enshow=1; 	 
			}
		 DrawChar(x+(size*6)*t,y,temp+'0'); 
		}
	}
} 

//////////////////////////////////////////////////////////////////////////////////////////////////
//显示汉字
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t *cn)
{      			    
	uint8_t j,wordNum;

	if((x > 7)||(y>128-16))
		return;
	
	while ( *cn != '\0')	 																	//在C语言中字符串结束以‘\0’结尾
	{
		for (wordNum=0; wordNum<NUM_OFCHINESE; wordNum++)
		{
			if ((CN16CHAR[wordNum].Index[0] == *cn)&&(CN16CHAR[wordNum].Index[1] == *(cn+1)))   //查询要写的字在字库中的位置
			{
				for (j=0; j<32; j++) 															//写一个字
				{		
					if (j == 16)	 															//由于16X16用到两个Y坐标，当大于等于16时，切换坐标
					{
						x++;
					}			
					WriteByteBuffer(x,y+(j%16),CN16CHAR[wordNum].Msk[j]);
				}
				y += 16;
				x--;
				if(y > (128-16))
				{x += 2;y = 0;}
			}
		}
		cn += 2;																																							//此处打完一个字，接下来寻找第二个字
	}
}

/**
 * @function: void OLED_Showdecimal(uint8_t x,uint8_t y,float num,uint8_t z_len,uint8_t f_len,uint8_t size2, uint8_t Color_Turn)
 * @description: 显示正负浮点数
 * @param {uint8_t} x待显示的数字起始横坐标,x:0~126
 * @param {uint8_t} y待显示的数字起始纵坐标, y:0~7，若选择字体大小为16，则两行数字之间需要间隔2，若选择字体大小为12，间隔1
 * @param {float} num:输入的浮点型数据
 * @param {uint8_t } z_ len:整数部分的位数
 * @param {uint8_t } f_len: 小数部分的位数
 * @param {uint8_t} size2:输入的数据大小，选择 16/12，16为8X16，12为6x8
 * @param {uint8_t} Color_Turn是否反相显示(1反相、0不反相)
 * @return {*}
 */
void OLED_Showdecimal(uint8_t x,uint8_t y,float num,uint8_t z_len,uint8_t f_len)
{
	uint8_t t,temp,i=0;//i为负数标志位
	uint8_t enshow;
	int z_temp,f_temp;
	if(num<0)
	{
		z_len+=1;
		i=1;
		num=-num;
	}
	z_temp=(int)num;
	//整数部分
	for(t=0;t<z_len;t++)
	{
		temp=(z_temp/oled_pow(10,z_len-t-1))%10;
		if(enshow==0 && t<(z_len-1))
		{
			if(temp==0)
			{
				DrawChar(x+8*t,y,' ');
				continue;
			}
			else
			enshow=1;
		}
		DrawChar(x+8*t,y,temp+'0');
	}
	//小数点
	DrawChar(x+8*(z_len),y,'.');

	f_temp=(int)((num-z_temp)*(oled_pow(10,f_len)));
  //小数部分
	for(t=0;t<f_len;t++)
	{
		temp=(f_temp/oled_pow(10,f_len-t-1))%10;
		DrawChar(x+8*(t+z_len)+5,y,temp+'0');
	}
	if(i==1)//如果为负，就将最前的一位赋值‘-’
	{
		DrawChar(x,y,'-');
		i=0;
	}
}

void OLED_LOGO(void)
{
  static uint8_t OLED_LOGOtmp;
  static uint8_t OLED_LOGOx, OLED_LOGOy, OLED_LOGOi;

  //	OLED_Clear();
  for (OLED_LOGOy = 0; OLED_LOGOy < 64; OLED_LOGOy += 8)
  {
    for (OLED_LOGOx = 0; OLED_LOGOx < 128; OLED_LOGOx++)
    {
      OLED_LOGOtmp = LOGO_RealSteel[OLED_LOGOx][OLED_LOGOy / 8];

      for (OLED_LOGOi = 0; OLED_LOGOi < 8; OLED_LOGOi++)
      {
        if (OLED_LOGOtmp & 0x80)
        {
					SetDrawColor(pix_white);
          DrawPixel(OLED_LOGOx, OLED_LOGOy + OLED_LOGOi);
        }
        else
        {
					SetDrawColor(pix_black);
          DrawPixel(OLED_LOGOx, OLED_LOGOy + OLED_LOGOi);
        }
        OLED_LOGOtmp <<= 1;
      }
    }
  }
//  OLED_Refresh_Gram(); //更新显示
}

