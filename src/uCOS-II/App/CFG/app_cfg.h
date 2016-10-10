/********************  ********************
 * 文件名       ：app.c
 * 描述         ：uC/OS的用户任务函数配置头文件
 *

**********************************************************************************/

#ifndef  _APP_CFG_H_
#define  _APP_CFG_H_



/*
*********************************************************************************************************
*                                            任务优先级
*********************************************************************************************************
*/

#define  STARTUP_TASK_PRIO       4

#define  OS_TASK_TMR_PRIO                        (OS_LOWEST_PRIO - 2u)


/*
*********************************************************************************************************
*                                     任务栈大小（单位为 OS_STK ）
*********************************************************************************************************
*/

#define STARTUP_TASK_STK_SIZE   128u






#endif  //_APP_CFG_H_
