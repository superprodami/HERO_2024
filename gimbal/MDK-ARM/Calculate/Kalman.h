#ifndef _KALMAN_H_
#define _KALMAN_H_
#include "main.h"
/*  �������˲�������  */
typedef struct KalmanType
{
	//Kailman ���
	float lastvalue ;
  float P1;
  float Q ;
  float R ;
  float Kg ;
  float P ;
}KalmanType;

extern float Kalman_Cal(KalmanType *p ,float nowvalue );
extern void Kalman_Init(KalmanType *p ,float Q ,float R);

#endif
