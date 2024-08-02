#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__

typedef enum 
{
   SHOOT_RUN 		= 0x00,
	 SHOOT_STOP   = 0x01,
}SHOOT_State;


typedef struct
{
	SHOOT_State State;
	float Shoot_target_speed;

}Shoot_t;	


extern Shoot_t Shoot;

void ShootInit(Shoot_t* shoot);
void ShootSelate(Shoot_t* shoot);
void Shoot_calc(Shoot_t* shoot);

#endif 


