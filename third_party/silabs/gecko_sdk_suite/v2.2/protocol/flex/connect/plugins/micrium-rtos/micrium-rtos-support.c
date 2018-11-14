// *******************************************************************
// * micrium-rtos-main.c
// *
// *
// * Copyright 2016 Silicon Laboratories, Inc.                              *80*
// *******************************************************************

#include PLATFORM_HEADER
#include CONFIGURATION_HEADER
#include "stack/include/ember.h"
#include "rtcdriver.h"

#include <kernel/include/os.h>
#include "bsp_tick_rtcc.h"
#include "bsp_cpu.h"

#include "flex-callbacks.h"

//------------------------------------------------------------------------------
// Tasks variables and defines

#define SYSTEM_START_TASK_PRIORITY              4
#define CONNECT_STACK_TASK_PRIORITY             4

#define SYSTEM_START_TASK_STACK_SIZE            150

static OS_TCB systemStartTaskControlBlock;
static CPU_STK systemStartTaskStack[SYSTEM_START_TASK_STACK_SIZE];

static OS_TCB connectTaskControlBlock;
static CPU_STK connectTaskStack[EMBER_AF_PLUGIN_MICRIUM_RTOS_CONNECT_STACK_SIZE];

#if defined(EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK1)
static OS_TCB applicationTask1ControlBlock;
static CPU_STK applicationTask1Stack[EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK1_STACK_SIZE];
#endif // EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK1

#if defined(EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK2)
static OS_TCB applicationTask2ControlBlock;
static CPU_STK applicationTask2Stack[EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK2_STACK_SIZE];
#endif // EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK2

#if defined(EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK3)
static OS_TCB applicationTask3ControlBlock;
static CPU_STK applicationTask3Stack[EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK3_STACK_SIZE];
#endif // EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK3

//------------------------------------------------------------------------------
// Forward and external declarations

static void(*stackTaskMainLoop)(void) = NULL;

static void systemStartTask(void *p_arg);
static void wakeUpConnectStackTask(void);
void App_OS_SetAllHooks(void);

//------------------------------------------------------------------------------
// Implemented APIs

void emberPluginMicriumRtosInitAndRunConnectTask(void(*mainLoopFuncPtr)(void))
{
  RTOS_ERR err;

  stackTaskMainLoop = mainLoopFuncPtr;

  OS_TRACE_INIT();
  OSInit(&err);
  assert(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);

  App_OS_SetAllHooks();

  // Create the systemStart task which initializes things
  OSTaskCreate(&systemStartTaskControlBlock,
               "System Start",
               systemStartTask,
               NULL,
               SYSTEM_START_TASK_PRIORITY,
               &systemStartTaskStack[0],
               SYSTEM_START_TASK_STACK_SIZE / 10,
               SYSTEM_START_TASK_STACK_SIZE,
               0, // Not receiving messages
               0, // Default time quanta
               NULL, // No TCB extensions
               OS_OPT_TASK_STK_CLR | OS_OPT_TASK_STK_CHK,
               &err);
  assert(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);

  // Start the OS
  OSStart(&err);
  assert(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);
}

// Register interrupt vectors with the OS
void emberPluginMicriumRtosCpuInit(void)
{
  BSP_CPUInit();

  // Radio Interrupts can optionally be configured non-kernel aware at this
  // point.
}

//------------------------------------------------------------------------------
// Implemented callbacks

bool emberAfPluginIdleSleepRtosCallback(uint32_t *durationMs, bool sleepOk)
{
  uint32_t actualDurationMs = *durationMs;
  uint32_t deltaMs, startTicks = RTCDRV_GetWallClockTicks32();
  OS_TICK yieldTimeTicks = (OSCfg_TickRate_Hz * actualDurationMs) / 1000;
  RTOS_ERR err;
  CPU_TS ts;

  if (!sleepOk) {
    // TODO: disable deep sleep at the sleep manager.
    //rtosBlockDeepSleep();
  }

  INTERRUPTS_ON();

  // Yield the stack task.
  OSTaskSemPend(yieldTimeTicks, OS_OPT_PEND_BLOCKING, &ts, &err);

  if (!sleepOk) {
    // TODO: re-enable deep sleep at the sleep manager.
    //if (rtosGetDeepSleepBlockCount() > 0) {
    //  rtosUnblockDeepSleep();
    //}
  }

  deltaMs = RTCDRV_TicksToMsec(RTCDRV_GetWallClockTicks32() - startTicks);
  if ( deltaMs <= actualDurationMs ) {
    *durationMs = actualDurationMs - deltaMs;
  } else {
    *durationMs = 0;
  }

  return true;
}

bool emberAfStackIdleCallback(uint32_t *idleTimeMs)
{
  return emberAfPluginIdleSleepRtosCallback(idleTimeMs, false);
}

void emberAfPluginMicriumRtosStackIsr(void)
{
  wakeUpConnectStackTask();
}

void halPendSvIsr(void)
{
  PendSV_Handler();
}

void halSysTickIsr(void)
{
  SysTick_Handler();
}

//------------------------------------------------------------------------------
// Static functions

// This can be called from ISR.
static void wakeUpConnectStackTask(void)
{
  RTOS_ERR err;

  OSTaskSemPost(&connectTaskControlBlock,
                OS_OPT_POST_NONE,
                &err);
  assert(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);
}

static void connectTask(void *p_arg)
{
  stackTaskMainLoop();
}

#if defined(EMBER_AF_PLUGIN_MICRIUM_RTOS_POLL_CLI)

static void pollCliTimerCallback(void * p_tmr, void * p_arg)
{
  wakeUpConnectStackTask();
}

#define POLL_CLI_TASK_PERIOD_MS                 250

static void startPollCliTimer(void)
{
  static OS_TMR pollCliTimer;
  RTOS_ERR err;

  // Create a periodic software timer that wakes up the Connect task so that it
  // can poll the CLI.
  // TODO: for some reason the time params are working as they would be
  // expressed in 1/10 of a seconds units.
  OSTmrCreate(&pollCliTimer,
              "Poll CLI Timer",
              POLL_CLI_TASK_PERIOD_MS / 100, // Delay
              POLL_CLI_TASK_PERIOD_MS / 100, // Period
              OS_OPT_TMR_PERIODIC,
              pollCliTimerCallback,
              NULL,
              &err);
  EFM_ASSERT(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);

  OSTmrStart(&pollCliTimer, &err);
  EFM_ASSERT(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);
}

#endif // EMBER_AF_PLUGIN_MICRIUM_RTOS_POLL_CLI

static void systemStartTask(void *p_arg)
{
  RTOS_ERR err;

  // Setup OS tick
#if (OS_CFG_DYN_TICK_EN == DEF_ENABLED)
  BSP_RTCC_TickInit();
#else
  CPU_INT32U cnts = (SystemCoreClockGet() / (CPU_INT32U)OSCfg_TickRate_Hz);
  OS_CPU_SysTickInit(cnts);
#endif

#if defined(EMBER_AF_PLUGIN_MICRIUM_RTOS_CPU_USAGE)
  OSStatTaskCPUUsageInit(&err);
  assert(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);
#endif // EMBER_AF_PLUGIN_MICRIUM_RTOS_CPU_USAGE

  // Create Connect task.
  OSTaskCreate(&connectTaskControlBlock,
               "Connect Stack",
               connectTask,
               NULL,
               CONNECT_STACK_TASK_PRIORITY,
               &connectTaskStack[0],
               EMBER_AF_PLUGIN_MICRIUM_RTOS_CONNECT_STACK_SIZE / 10,
               EMBER_AF_PLUGIN_MICRIUM_RTOS_CONNECT_STACK_SIZE,
               0, // Not receiving messages
               0, // Default time quanta
               NULL, // No TCB extensions
               OS_OPT_TASK_STK_CLR | OS_OPT_TASK_STK_CHK,
               &err);
  assert(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);

#if defined(EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK1)

  emberAfPluginMicriumRtosAppTask1InitCallback();

  // Create Application Task 1.
  OSTaskCreate(&applicationTask1ControlBlock,
               "Application (1)",
               emberAfPluginMicriumRtosAppTask1MainLoopCallback,
               NULL,
               EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK1_PRIORITY,
               &applicationTask1Stack[0],
               EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK1_STACK_SIZE / 10,
               EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK1_STACK_SIZE,
               0, // Not receiving messages
               0, // Default time quanta
               NULL, // No TCB extensions
               OS_OPT_TASK_STK_CLR | OS_OPT_TASK_STK_CHK,
               &err);
  assert(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);

#endif // EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK1

#if defined(EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK2)

  emberAfPluginMicriumRtosAppTask2InitCallback();

  // Create Application Task 2.
  OSTaskCreate(&applicationTask2ControlBlock,
               "Application (2)",
               emberAfPluginMicriumRtosAppTask2MainLoopCallback,
               NULL,
               EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK2_PRIORITY,
               &applicationTask2Stack[0],
               EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK2_STACK_SIZE / 10,
               EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK2_STACK_SIZE,
               0, // Not receiving messages
               0, // Default time quanta
               NULL, // No TCB extensions
               OS_OPT_TASK_STK_CLR | OS_OPT_TASK_STK_CHK,
               &err);
  assert(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);

#endif // EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK2

#if defined(EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK3)

  emberAfPluginMicriumRtosAppTask3InitCallback();

  // Create Application Task 3.
  OSTaskCreate(&applicationTask3ControlBlock,
               "Application (3)",
               emberAfPluginMicriumRtosAppTask3MainLoopCallback,
               NULL,
               EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK3_PRIORITY,
               &applicationTask3Stack[0],
               EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK3_STACK_SIZE / 10,
               EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK3_STACK_SIZE,
               0, // Not receiving messages
               0, // Default time quanta
               NULL, // No TCB extensions
               OS_OPT_TASK_STK_CLR | OS_OPT_TASK_STK_CHK,
               &err);
  assert(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE);

#endif // EMBER_AF_PLUGIN_MICRIUM_RTOS_APP_TASK3

#if defined(EMBER_AF_PLUGIN_MICRIUM_RTOS_POLL_CLI)
  startPollCliTimer();
#endif // EMBER_AF_PLUGIN_MICRIUM_RTOS_POLL_CLI

  // Done starting everyone else so let's exit
  OSTaskDel((OS_TCB *)0, &err);
}
