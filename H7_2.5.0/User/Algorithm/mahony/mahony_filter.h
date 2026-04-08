#ifndef _MAHONY_FILTER_H
#define _MAHONY_FILTER_H

#include <math.h>
#include <stdlib.h>
#include "stm32h7xx.h"
#include "math.h"

/*************************************
���ʱ�䣺2023��09��02�� 
���ܽ��ܣ�ʵ��mahony��̬�ǽ����㷨��ģ���װ
֪���˺ţ�����Ҳ
Bվ�˺ţ�����С����
***************************************/

#define DEG2RAD 0.0174533f
#define RAD2DEG 57.295671f

typedef struct Axis3f_t
{
  float x;
  float y;
  float z;
}Axis3f;


// ���� MAHONY_FILTER_t �ṹ�壬���ڷ�װ Mahony �˲��������ݺͺ���
struct MAHONY_FILTER_t
{
    // �������
    float Kp, Ki;          // �����ͻ�������
    float dt;              // ����ʱ����
    Axis3f  gyro, acc;     // �����Ǻͼ��ٶȼ�����

    // ���̲���
    float exInt, eyInt, ezInt;                // ��������ۼ�
    float q0, q1, q2, q3;            // ��Ԫ��
    float rMat[3][3];               // ��ת����

    // �������
    float pitch, roll, yaw;         // ��̬�ǣ������ǣ���ת�ǣ�ƫ����

    // ����ָ��
    void (*mahony_init)(struct MAHONY_FILTER_t *mahony_filter, float Kp, float Ki, float dt);
    void (*mahony_input)(struct MAHONY_FILTER_t *mahony_filter, Axis3f gyro, Axis3f acc);
    void (*mahony_update)(struct MAHONY_FILTER_t *mahony_filter);
    void (*mahony_output)(struct MAHONY_FILTER_t *mahony_filter);
    void (*RotationMatrix_update)(struct MAHONY_FILTER_t *mahony_filter);
};

// ��������
void mahony_init(struct MAHONY_FILTER_t *mahony_filter, float Kp, float Ki, float dt);          // ��ʼ������
void mahony_input(struct MAHONY_FILTER_t *mahony_filter, Axis3f gyro, Axis3f acc);              // �������ݺ���
void mahony_update(struct MAHONY_FILTER_t *mahony_filter);                                      // �����˲�������
void mahony_output(struct MAHONY_FILTER_t *mahony_filter);                                      // �����̬�Ǻ���
void RotationMatrix_update(struct MAHONY_FILTER_t *mahony_filter);                              // ������ת������

#endif

