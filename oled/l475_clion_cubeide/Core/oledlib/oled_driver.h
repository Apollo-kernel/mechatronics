#ifndef __OLED_DRIVER_H
#define __OLED_DRIVER_H
//#include "stm32f10x.h"
#include "oled_config.h"
#include "oled_basic.h"

#if (TRANSFER_METHOD == HW_IIC)

	#define OLED_ADDRESS  0x78 //通过调整0R电阻,屏可以0x78和0x7A两个地址 -- 默认0x78
//	#if (USE_HW_IIC == IIC_1)
//		/*STM32F103C8T6芯片的硬件I2C: PB6 -- SCL; PB7 -- SDA */
//		#define IIC_RCC_APB1Periph_I2CX     RCC_APB1Periph_I2C1
//		#define IIC_RCC_APB2Periph_GPIOX    RCC_APB2Periph_GPIOB
//		#define IIC_GPIOX                   GPIOB
//		#define IIC_SCL_Pin_X               GPIO_Pin_6
//		#define IIC_SDA_Pin_X               GPIO_Pin_7
//		#define I2CX                        I2C1
//	#elif (USE_HW_IIC == IIC_2)
//		#define IIC_RCC_APB1Periph_I2CX     RCC_APB1Periph_I2C2
//		#define IIC_RCC_APB2Periph_GPIOX    RCC_APB2Periph_GPIOB
//		#define IIC_GPIOX                   GPIOB
//		#define IIC_SCL_Pin_X               GPIO_Pin_10
//		#define IIC_SDA_Pin_X               GPIO_Pin_11
//		#define I2CX                        I2C2
//	#endif
	void I2C_Configuration(void);
	void I2C_WriteByte(uint8_t addr, uint8_t data);
	
#elif (TRANSFER_METHOD == SW_IIC)//软件I2C依赖主频，在F103（72M）测试下通过，其余请自行测试

	#define OLED_ADDRESS  0x78 	//通过调整0R电阻,屏可以0x78和0x7A两个地址 -- 默认0x78
	#define IIC_SCL             BIT_ADDR(GPIOE_ODR_Addr, 12) 	//SCL
	#define IIC_SDA             BIT_ADDR(GPIOE_ODR_Addr, 15) 	//SDA
	#define OLED_GPIO_PORT_I2C	GPIOE							/* GPIO端口 */
	#define OLED_RCC_I2C_PORT 	RCC_APB2Periph_GPIOE			/* GPIO端口时钟 */
	#define OLED_I2C_SCL_PIN		GPIO_Pin_12					/* 连接到SCL时钟线的GPIO */
	#define OLED_I2C_SDA_PIN		GPIO_Pin_15					/* 连接到SDA数据线的GPIO */
	void I2C_Configuration(void);
	void I2C_WriteByte(uint8_t addr, uint8_t data);


#elif (TRANSFER_METHOD ==HW_SPI)

	#if (USE_HW_SPI==SPI_1)
	//硬件SPI1还没实现
	#elif (USE_HW_SPI==SPI_2)
		#define SPIX                      SPI2
		//使能SPI2时钟
		#define SPI_RCC_APB1Periph_SPIX   RCC_APB1Periph_SPI2
		//使能gpio时钟，使用的GPIO不一样时可定义如下：
		//#define SPI_RCC_APB2Periph_GPIOX  RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC
		#define SPI_RCC_APB2Periph_GPIOX  RCC_APB2Periph_GPIOB
		//CS片选（软件片选）
		#define SPI_CS_Pin_X              GPIO_Pin_12
		#define SPI_CS_GPIOX              GPIOB
		//SPI2 时钟、mosi、miso引脚
		#define SPI_HW_ALL_PINS           (GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15)
		#define SPI_HW_ALL_GPIOX          GPIOB
		//复位引脚
		#define SPI_RES_PIN               GPIO_Pin_10
		#define SPI_RES_GPIOX             GPIOB
		//控制引脚
		#define SPI_DC_PIN                GPIO_Pin_11
		#define SPI_DC_GPIOX              GPIOB
	#endif
	void SPI_Configuration(void);
	void SPI_WriterByte(unsigned char dat);
	
#elif (TRANSFER_METHOD ==SW_SPI)
	
	#define OLED_SCLK_Clr() GPIO_ResetBits(GPIOA,GPIO_Pin_4)  //CLK
	#define OLED_SCLK_Set() GPIO_SetBits(GPIOA,GPIO_Pin_4)

	#define OLED_SDIN_Clr() GPIO_ResetBits(GPIOA,GPIO_Pin_5)  //DIN
	#define OLED_SDIN_Set() GPIO_SetBits(GPIOA,GPIO_Pin_5)

	#define OLED_RST_Clr() GPIO_ResetBits(GPIOA,GPIO_Pin_6)   //RES
	#define OLED_RST_Set() GPIO_SetBits(GPIOA,GPIO_Pin_6)

	#define OLED_DC_Clr() GPIO_ResetBits(GPIOA,GPIO_Pin_7)    //DC
	#define OLED_DC_Set() GPIO_SetBits(GPIOA,GPIO_Pin_7)

	#define OLED_CS_Clr()  GPIO_ResetBits(GPIOA,GPIO_Pin_8)   //CS
	#define OLED_CS_Set()  GPIO_SetBits(GPIOA,GPIO_Pin_8)

	#define OLED_CMD  0 //写命令
	#define OLED_DATA 1 //写数据
	void SPI_Configuration(void);
	void SPI_WriterByte(unsigned char dat);
	
#endif


void WriteCmd(unsigned char cmd);
void WriteDat(unsigned char Dat);

void OLED_Init(void);
void OLED_CLS(void);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_FILL(unsigned char BMP[]);

#endif //__OLED_DRIVER_H

