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
	
	此c文件用于对颜色进行操作
	在这里主要是画线线条(一般为白)和填充(一般为白)的颜色
	(在oled.font中还有一个字体背景的颜色(一般为黑))
	在划线画图和打点时都会访问此文件函数获取打点的颜色
*/

#include "oled_color.h"

//如有需要还可以自己迭代字体颜色 文字背景颜色等
static Type_color _Draw=pix_white;
static Type_color _fill=pix_white;


void SetDrawColor(Type_color value)	//就是划线 线条的颜色
{
	_Draw=value;
}
Type_color GetDrawColor(void)
{
	return _Draw;
}

void SetFillcolor(Type_color value)	//就是填充 实心图形内的颜色
{
	_fill=value;
}
Type_color GetFillColor(void)
{
	return _fill;
}
