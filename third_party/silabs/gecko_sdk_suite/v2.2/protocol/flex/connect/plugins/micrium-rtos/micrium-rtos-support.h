// Copyright 2018 Silicon Laboratories, Inc.

#ifndef _MICRIUM_RTOS_SUPPORT_H_
#define _MICRIUM_RTOS_SUPPORT_H_

void emberPluginMicriumRtosCpuInit(void);

void emberPluginMicriumRtosInitAndRunConnectTask(void(*mainLoopFuncPtr)(void));

#endif // _MICRIUM_RTOS_SUPPORT_H_
