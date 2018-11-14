/*
 *********************************************************************************************************
 *
 *                                   KERNEL OBJECTS CONFIGURATION FILE
 *
 * File : os_cfg_app.h
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                                MODULE
 *********************************************************************************************************
 *********************************************************************************************************
 */

#ifndef SILABS_OS_CFG_APP_H
#define SILABS_OS_CFG_APP_H

/*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                     KERNEL OBJECTS CONFIGURATION
 *********************************************************************************************************
 *********************************************************************************************************
 */

#define  OS_CFG_ISR_STK_SIZE                256
#define  OS_CFG_MSG_POOL_SIZE               50
#define  OS_CFG_IDLE_TASK_STK_SIZE          128
#define  OS_CFG_STAT_TASK_STK_SIZE          128
#define  OS_CFG_TICK_TASK_STK_SIZE          128
#define  OS_CFG_TMR_TASK_STK_SIZE           128
#define  OS_CFG_TASK_STK_LIMIT_PCT_EMPTY    10u
#define  OS_CFG_STAT_TASK_PRIO              3u
#define  OS_CFG_STAT_TASK_RATE_HZ           10u
#define  OS_CFG_TICK_RATE_HZ                1000u
#define  OS_CFG_TICK_TASK_PRIO              1u
#define  OS_CFG_TMR_TASK_PRIO               2u
#define  OS_CFG_TMR_TASK_RATE_HZ            10u

/*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                              MODULE END
 *********************************************************************************************************
 *********************************************************************************************************
 */

#endif
