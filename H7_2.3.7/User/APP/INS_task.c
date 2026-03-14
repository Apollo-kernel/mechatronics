/**
  *********************************************************************
  * @file      ins_task.c/h
  * @brief     ����������mahony������ȡ������̬��ͬʱ��ȡ�����ھ�������ϵ�µ��˶����ٶ�
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "ins_task.h"
#include "controller.h"
// #include "QuaternionEKF.h"
// #include "bsp_PWM.h"
#include "mahony_filter.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#define RAD_TO_DEG 57.2957795f
INS_t INS;

struct MAHONY_FILTER_t mahony;
Axis3f Gyro,Accel;
float gravity[3] = {0, 0, 9.81f};

uint32_t INS_DWT_Count = 0;
float ins_dt = 0.0f;
float ins_time;
int stop_time;

static volatile uint8_t usart1_tx_busy = 0;
static char usart1_tx_buf[128];
static uint32_t ypr_print_tick = 0;

static void INS_PrintYPR_USART1_DMA(void)
{
	int len;

	if (usart1_tx_busy)
	{
		return;
	}

	// len = snprintf(usart1_tx_buf, sizeof(usart1_tx_buf),
	// 			   "Yaw=%.3f Pitch=%.3f Roll=%.3f\r\n",
	// 			   INS.Yaw, INS.Pitch, INS.Roll);

	len = snprintf(usart1_tx_buf, sizeof(usart1_tx_buf),
				   "Yaw=%.2fdeg Pitch=%.2fdeg Roll=%.2fdeg\r\n",
				   INS.Yaw   * RAD_TO_DEG,
				   INS.Pitch * RAD_TO_DEG,
				   INS.Roll  * RAD_TO_DEG);


	if (len <= 0)
	{
		return;
	}

	if (len >= (int)sizeof(usart1_tx_buf))
	{
		len = sizeof(usart1_tx_buf) - 1;
	}

	usart1_tx_busy = 1;
	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)usart1_tx_buf, (uint16_t)len) != HAL_OK)
	{
		usart1_tx_busy = 0;
	}
}


void INS_Init(void)
{ 
	 mahony_init(&mahony,1.0f,0.0f,0.001f);
   INS.AccelLPF = 0.0089f;
}

void INS_task(void)
{
	 INS_Init();
	ypr_print_tick = HAL_GetTick();
	 while(1)
	 {  
		ins_dt = DWT_GetDeltaT(&INS_DWT_Count);
    
		mahony.dt = ins_dt;

    BMI088_Read(&BMI088);

    INS.Accel[X] = BMI088.Accel[X];
    INS.Accel[Y] = BMI088.Accel[Y];
    INS.Accel[Z] = BMI088.Accel[Z];
	  Accel.x=BMI088.Accel[0];
	  Accel.y=BMI088.Accel[1];
		Accel.z=BMI088.Accel[2];
    INS.Gyro[X] = BMI088.Gyro[X];
    INS.Gyro[Y] = BMI088.Gyro[Y];
    INS.Gyro[Z] = BMI088.Gyro[Z];
  	Gyro.x=BMI088.Gyro[0];
		Gyro.y=BMI088.Gyro[1];
		Gyro.z=BMI088.Gyro[2];

		mahony_input(&mahony,Gyro,Accel);
		mahony_update(&mahony);
		mahony_output(&mahony);
	  RotationMatrix_update(&mahony);
				
		INS.q[0]=mahony.q0;
		INS.q[1]=mahony.q1;
		INS.q[2]=mahony.q2;
		INS.q[3]=mahony.q3;
       
      // �������ӵ�������ϵnת��������ϵb,�����ݼ��ٶȼ����ݼ����˶����ٶ�
		float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    for (uint8_t i = 0; i < 3; i++) // ͬ����һ����ͨ�˲�
    {
      INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * ins_dt / (INS.AccelLPF + ins_dt) 
														+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + ins_dt); 
//			INS.MotionAccel_b[i] = (INS.Accel[i] ) * dt / (INS.AccelLPF + dt) 
//														+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);			
		}
		BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // ת���ص���ϵn
		
		//��������
		if(fabsf(INS.MotionAccel_n[0])<0.02f)
		{
		  INS.MotionAccel_n[0]=0.0f;	//x��
		}
		if(fabsf(INS.MotionAccel_n[1])<0.02f)
		{
		  INS.MotionAccel_n[1]=0.0f;	//y��
		}
		if(fabsf(INS.MotionAccel_n[2])<0.04f)
		{
		  INS.MotionAccel_n[2]=0.0f;//z��
		}
   		
		if(ins_time>3000.0f)
		{
			INS.ins_flag=1;//��Ԫ���������������ٶ�Ҳ�������������Կ�ʼ��������
			// ��ȡ��������
			INS.Pitch=mahony.roll;
		  INS.Roll=mahony.pitch;
		  INS.Yaw=mahony.yaw;
		
		//INS.YawTotalAngle=INS.YawTotalAngle+INS.Gyro[2]*0.001f;
			
			if (INS.Yaw - INS.YawAngleLast > 3.1415926f)
			{
					INS.YawRoundCount--;
			}
			else if (INS.Yaw - INS.YawAngleLast < -3.1415926f)
			{
					INS.YawRoundCount++;
			}
			INS.YawTotalAngle = 6.283f* INS.YawRoundCount + INS.Yaw;
			INS.YawAngleLast = INS.Yaw;
			if (HAL_GetTick() - ypr_print_tick >= 20)
			{
				ypr_print_tick = HAL_GetTick();
				INS_PrintYPR_USART1_DMA();
			}
		}
		else
		{
		 ins_time++;
		}
		
    osDelay(1);
	}
} 


/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}




void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		usart1_tx_busy = 0;
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		usart1_tx_busy = 0;
	}
}