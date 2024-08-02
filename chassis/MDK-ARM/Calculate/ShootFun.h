#ifndef __SHOOTFUN_H__
#define __SHOOTFUN_H__

#include "ShootTask.h"


extern void Shoot_Init(Shoot_t* shoot);
void Shoot_flag_Init(Shoot_t* shoot);
extern void Clean_Shoot_CtrlMsg(Shoot_t* shoot);

void Shoot_Mode_choose_remote(Shoot_t* shoot);
void Shoot_Mode_choose_keyboard(Shoot_t* shoot);
extern void Shoot_Mode_Choose(Shoot_t* shoot);

extern void Shoot_clc(void);
void Shoor_Ctl_STOP(Shoot_t* shoot);
void Shoor_Ctl_BERSERK(Shoot_t* shoot);
void Shoor_Ctl_SINGLE(Shoot_t* shoot);
void Shoor_Ctl_NORMAL(Shoot_t* shoot);
bool Revolver_Heat_Limit(void);


extern void ShootSelate(Shoot_t* shoot);
extern void Shoot_Statemachine_2_Init(Shoot_t* shoot);
extern void Shoot_Statemachine_2_Run(Shoot_t* shoot);
extern void Shoot_Statemachine_2_Stop(Shoot_t* shoot);


#endif

