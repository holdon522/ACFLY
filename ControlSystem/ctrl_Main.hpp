#pragma once

#define CtrlRateDiv 2
#define CtrlRateHz 400

//控制系统互斥锁
bool LockCtrl( double TIMEOUT );
void UnlockCtrl();

//上次控制时间
extern TIME last_XYCtrlTime;
extern TIME last_ZCtrlTime;

//控制线程句柄
extern TaskHandle_t ControlTaskHandle;
//MSafe任务句柄
extern TaskHandle_t MSafeTaskHandle;
//强制Safe控制
extern bool ForceMSafeCtrl;

void init_ControlSystem();