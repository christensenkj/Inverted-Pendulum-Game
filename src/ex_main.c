/***************************************************************************//**
 * @file
 * @brief
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement.
 * The software is governed by the sections of the MSLA applicable to Micrium
 * Software.
 *
 ******************************************************************************/

/*
*********************************************************************************************************
*
*                                             EXAMPLE MAIN
*
* File : ex_main.c
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*********************************************************************************************************
*/

#include  <bsp_os.h>
#include  "bsp.h"

#include  <cpu/include/cpu.h>
#include  <common/include/common.h>
#include  <kernel/include/os.h>
#include  <kernel/include/os_trace.h>

#include  <common/include/lib_def.h>
#include  <common/include/rtos_utils.h>
#include  <common/include/toolchains.h>

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_core.h"

#include "capsense.h"
#include "em_acmp.h"

#include "app.h"

#include "fifo.h"

#include "display.h"
#include <string.h>
#include "glib.h"
/*
*********************************************************************************************************
*********************************************************************************************************
*                                             LOCAL DEFINES
*********************************************************************************************************
*********************************************************************************************************
*/

#define  EX_MAIN_START_TASK_PRIO              		20u
#define  EX_MAIN_LED_OUTPUT_TASK_PRIO           	21u
#define  EX_MAIN_VEHICLE_DIRECTION_TASK_PRIO    	22u
#define  EX_MAIN_SPEED_SETPOINT_TASK_PRIO       	23u
#define  EX_MAIN_VEHICLE_MONITOR_TASK_PRIO	    	23u
#define  EX_MAIN_LCD_DISPLAY_TASK_PRIO    			23u
#define  EX_MAIN_IDLE_TASK_PRIO              		24u

#define  EX_MAIN_START_TASK_STK_SIZE         		512u
#define  EX_MAIN_LED_OUTPUT_TASK_STK_SIZE			512u
#define  EX_MAIN_IDLE_TASK_STK_SIZE			 		512u
#define  EX_MAIN_SPEED_SETPOINT_TASK_STK_SIZE	    512u
#define  EX_MAIN_VEHICLE_DIRECTION_TASK_STK_SIZE	512u
#define  EX_MAIN_VEHICLE_MONITOR_TASK_STK_SIZE		512u
#define  EX_MAIN_LCD_DISPLAY_TASK_STK_SIZE	    	512u
/*
*********************************************************************************************************
*********************************************************************************************************
*                                        LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*********************************************************************************************************
*/

/* Start Task Stack.                                    */
static  CPU_STK  Ex_MainStartTaskStk[EX_MAIN_START_TASK_STK_SIZE];
/* Start Task TCB.                                      */
static  OS_TCB   Ex_MainStartTaskTCB;

/* Idle Task Stack.                                    */
static  CPU_STK  Ex_MainIdleTaskStk[EX_MAIN_IDLE_TASK_STK_SIZE];
/* Idle Task TCB.                                      */
static  OS_TCB   Ex_MainIdleTaskTCB;

/* LedOutput Task Stack.                                    */
static  CPU_STK  Ex_MainLedOutputTaskStk[EX_MAIN_LED_OUTPUT_TASK_STK_SIZE];
/* LedOutput Task TCB.                                      */
static  OS_TCB   Ex_MainLedOutputTaskTCB;

/* Speed Setpoint Task Stack.                                    */
static  CPU_STK  Ex_MainSpeedSetpointTaskStk[EX_MAIN_SPEED_SETPOINT_TASK_STK_SIZE];
/* Speed Setpoint Task TCB.                                      */
static  OS_TCB   Ex_MainSpeedSetpointTaskTCB;

/* Vehicle Direction Task Stack.                                    */
static  CPU_STK  Ex_MainVehicleDirectionTaskStk[EX_MAIN_VEHICLE_DIRECTION_TASK_STK_SIZE];
/* LedOutput Task TCB.                                      */
static  OS_TCB   Ex_MainVehicleDirectionTaskTCB;

/* Vehicle Monitoring Task Stack.                                    */
static  CPU_STK  Ex_MainVehicleMonitorTaskStk[EX_MAIN_VEHICLE_MONITOR_TASK_STK_SIZE];
/* LedOutput Task TCB.                                      */
static  OS_TCB   Ex_MainVehicleMonitorTaskTCB;

/* LCD Display Task Stack.                                    */
static  CPU_STK  Ex_MainLcdDisplayTaskStk[EX_MAIN_LCD_DISPLAY_TASK_STK_SIZE];
/* LedOutput Task TCB.                                      */
static  OS_TCB   Ex_MainLcdDisplayTaskTCB;

/* Create OS Task Flags */

/* Create OS Task Flags for vehicle monitor task*/
static OS_FLAG_GRP	vehicle_monitor_flg;

/* Create OS Task Flags for LED task*/
static OS_FLAG_GRP	led_output_flg;

/* Create semaphore */
static OS_SEM		sem;

/* Create semaphore to keep track of btn action additions to fifo*/
static OS_SEM		fifo_sem;

/* Create a mutex to protect access to speed setpoint data */
static OS_MUTEX  	speed_setpoint_mutex;

/* Create a mutex to protect access to vehicle direction data */
static OS_MUTEX  	vehicle_direction_mutex;

/* Create OS Timer */
static OS_TMR		tmr;

/* Create an enum to store vehicle direction states */
enum vehicle_dir_state {
	straight,
	hard_left,
	soft_left,
	soft_right,
	hard_right,
};

/* Declare a fifo for button actions */
//struct node_t* btn0_fifo;
//struct node_t* btn1_fifo;

uint8_t btn0_fifo[10];
uint8_t btn1_fifo[10];
uint8_t btn0_fifo_rd;
uint8_t btn0_fifo_wr;
uint8_t btn1_fifo_rd;
uint8_t btn1_fifo_wr;

// Debug variable purely for debug purposes
int debug_flag;

// Data types for Car simulation
struct speed_setpoint_data_struct {
	uint8_t cur_speed;
	uint8_t up_cnt;
	uint8_t dn_cnt;
} speed_setpoint_data;

struct vehicle_direction_data_struct {
	uint8_t cur_dir;
	uint8_t cur_dir_time;
	uint8_t left_cnt;
	uint8_t right_cnt;
} vehicle_direction_data;

/* Global glib context */
GLIB_Context_t gc;


/*
*********************************************************************************************************
*********************************************************************************************************
*                                       LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*********************************************************************************************************
*/
// task main functions
static  void  Ex_MainStartTask (void  *p_arg);
static  void  Ex_MainIdleTask (void  *p_arg);
static  void  Ex_MainLedOutputTask (void  *p_arg);
static  void  Ex_MainSpeedSetpointTask (void  *p_arg);
static  void  Ex_MainVehicleDirectionTask (void  *p_arg);
static  void  Ex_MainVehicleMonitorTask (void  *p_arg);
static  void  Ex_MainLcdDisplayTask (void  *p_arg);
// callback function for OSTimer
static void MyCallback(OS_TMR p_tmr, void *p_arg);


/*
*********************************************************************************************************
*********************************************************************************************************
*                                       GLOBAL VARIABLES
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*********************************************************************************************************
*                                          GLOBAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C applications. It is assumed that your code will
*               call main() once you have performed all necessary initialization.
*
* Argument(s) : None.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
/***************************************************************************//**
 * @brief
 *   Creates the Startup Task and then begins the OS.
 *
 * @details
 * 	 Creates the Startup Task which in turn creates all other tasks.
 * 	 Then the OS is started, which runs indefinitely and does not return.
 *
 *
 ******************************************************************************/
int  main (void)
 {
    RTOS_ERR  err;


    BSP_SystemInit();                                           /* Initialize System.                                   */
    CPU_Init();                                                 /* Initialize CPU.                                      */

    OS_TRACE_INIT();
    OSInit(&err);                                               /* Initialize the Kernel.                               */
                                                                /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


    OSTaskCreate(&Ex_MainStartTaskTCB,                          /* Create the Start Task.                               */
                 "Ex Main Start Task",
                  Ex_MainStartTask,
                  DEF_NULL,
                  EX_MAIN_START_TASK_PRIO,
                 &Ex_MainStartTaskStk[0],
                 (EX_MAIN_START_TASK_STK_SIZE / 10u),
                  EX_MAIN_START_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

    /* Start the kernel.                                    */
    OSStart(&err);
    /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


    return (1);
}


/***************************************************************************//**
 * @brief
 *   Creates all other user-defined tasks.
 *
 * @details
 * 	 Creates the button input task, idle task, LED control task, and slider input task.
 *
 *
 ******************************************************************************/

static  void  Ex_MainStartTask (void  *p_arg)
{
    RTOS_ERR  err;


    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif

    Common_Init(&err);                                          /* Call common module initialization example.           */
    APP_RTOS_ASSERT_CRITICAL(err.Code == RTOS_ERR_NONE, ;);

    BSP_OS_Init();                                              /* Initialize the BSP. It is expected that the BSP ...  */
                                                                /* ... will register all the hardware controller to ... */
                                                                /* ... the platform manager at this moment.             */
    // initialize common modules for all tasks
    cmu_open();
    /* Create the Idle Task.                               */
    OSTaskCreate(&Ex_MainIdleTaskTCB,
                 "Ex Main Idle Task",
                  Ex_MainIdleTask,
                  DEF_NULL,
                  EX_MAIN_IDLE_TASK_PRIO,
                 &Ex_MainIdleTaskStk[0],
                 (EX_MAIN_IDLE_TASK_STK_SIZE / 10u),
                  EX_MAIN_IDLE_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    /* Create the Led Output Task.                               */
    OSTaskCreate(&Ex_MainLedOutputTaskTCB,
                 "Ex Main Led Output Task",
                  Ex_MainLedOutputTask,
                  DEF_NULL,
                  EX_MAIN_LED_OUTPUT_TASK_PRIO,
                 &Ex_MainLedOutputTaskStk[0],
                 (EX_MAIN_LED_OUTPUT_TASK_STK_SIZE / 10u),
                  EX_MAIN_LED_OUTPUT_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    /* Create the Speed Setpoint Task.                               */
    OSTaskCreate(&Ex_MainSpeedSetpointTaskTCB,
                 "Ex Main Speed Setpoint Task",
                  Ex_MainSpeedSetpointTask,
                  DEF_NULL,
                  EX_MAIN_SPEED_SETPOINT_TASK_PRIO,
                 &Ex_MainSpeedSetpointTaskStk[0],
                 (EX_MAIN_SPEED_SETPOINT_TASK_STK_SIZE / 10u),
                  EX_MAIN_SPEED_SETPOINT_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    /* Create the Vehicle Direction Task.                               */
    OSTaskCreate(&Ex_MainVehicleDirectionTaskTCB,
                 "Ex Main Vehicle Direction Task",
                  Ex_MainVehicleDirectionTask,
                  DEF_NULL,
                  EX_MAIN_VEHICLE_DIRECTION_TASK_PRIO,
                 &Ex_MainVehicleDirectionTaskStk[0],
                 (EX_MAIN_VEHICLE_DIRECTION_TASK_STK_SIZE / 10u),
                  EX_MAIN_VEHICLE_DIRECTION_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    /* Create the Vehicle Monitor Task.                               */
    OSTaskCreate(&Ex_MainVehicleMonitorTaskTCB,
                 "Ex Main Vehicle Monitor Task",
                  Ex_MainVehicleMonitorTask,
                  DEF_NULL,
                  EX_MAIN_VEHICLE_MONITOR_TASK_PRIO,
                 &Ex_MainVehicleMonitorTaskStk[0],
                 (EX_MAIN_VEHICLE_MONITOR_TASK_STK_SIZE / 10u),
                  EX_MAIN_VEHICLE_MONITOR_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    /* Create the LCD Display Task.                               */
    OSTaskCreate(&Ex_MainLcdDisplayTaskTCB,
                 "Ex Main LCD Display Task",
                  Ex_MainLcdDisplayTask,
                  DEF_NULL,
                  EX_MAIN_LCD_DISPLAY_TASK_PRIO,
                 &Ex_MainLcdDisplayTaskStk[0],
                 (EX_MAIN_LCD_DISPLAY_TASK_STK_SIZE / 10u),
                  EX_MAIN_LCD_DISPLAY_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    // Create the event flag
    OSFlagCreate((OS_FLAG_GRP *)
    		&vehicle_monitor_flg,
			(CPU_CHAR *) "vehicle monitor flag",
			(OS_FLAGS) 0,
			(RTOS_ERR *)&err);
    // Create the event flag
    OSFlagCreate((OS_FLAG_GRP *)
    		&led_output_flg,
			(CPU_CHAR *) "led output flag",
			(OS_FLAGS) 0,
			(RTOS_ERR *)&err);
    // Create the semaphore
    OSSemCreate ((OS_SEM *) &sem,
            (CPU_CHAR *) "Slider Semaphore",
            (OS_SEM_CTR) 0,
            (RTOS_ERR *) &err);
    // Create the semaphore
    OSSemCreate ((OS_SEM *) &fifo_sem,
            (CPU_CHAR *) "FIFO Semaphore",
            (OS_SEM_CTR) 0,
            (RTOS_ERR *) &err);
    // Create the OSTimer
    OSTmrCreate ((OS_TMR *) &tmr,
			(CPU_CHAR *) "OS Timer",
			(OS_TICK) 10,
			(OS_TICK) 10,
			(OS_OPT) OS_OPT_TMR_PERIODIC,
			(OS_TMR_CALLBACK_PTR) &MyCallback,
			(void *) 0,
			(RTOS_ERR *) &err);
    // Create the mutex
    OSMutexCreate ((OS_MUTEX *) &speed_setpoint_mutex,
			(CPU_CHAR *) "Speed Setpoint Mutex",
			(RTOS_ERR *) &err);
    // Create the mutex
    OSMutexCreate ((OS_MUTEX *) &vehicle_direction_mutex,
			(CPU_CHAR *) "Vehicle Direction Mutex",
			(RTOS_ERR *) &err);
//    // Declare the two FIFOs
//    btn0_fifo = create_queue();
//    btn1_fifo = create_queue();
    btn0_fifo_rd = 0;
    btn0_fifo_wr = 0;
    btn1_fifo_rd = 0;
    btn1_fifo_wr = 0;

    // Start the timer
    OSTmrStart ((OS_TMR *) &tmr, (RTOS_ERR *) &err);

    while (DEF_ON) {
                                                                /* Delay Start Task execution for                       */
        OSTimeDly( 1000,                                        /*   1000 OS Ticks                                      */
                   OS_OPT_TIME_DLY,                             /*   from now.                                          */
                  &err);
                                                                /*   Check error code.                                  */
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/***************************************************************************//**
 * @brief
 *   Puts the CPU to EM1 energy mode every time the task is executed.
 *
 * @details
 * 	 For the duration of timer ticks specified in the OSTimeDly function, will
 * 	 ready and run the task this frequently, putting the processor into EM1.
 *
 *
 ******************************************************************************/

static  void  Ex_MainIdleTask (void  *p_arg)
{
    RTOS_ERR  err;


    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif
                                                                /* ... will register all the hardware controller to ... */
                                                                /* ... the platform manager at this moment.             */

    while (DEF_ON) {
    	EMU_EnterEM1();
        /* Delay Start Task execution for                       */
		OSTimeDly( 10,                                        /*   1000 OS Ticks                                      */
					OS_OPT_TIME_DLY,                             /*   from now.                                          */
					&err);
                                                                /*   Check error code.                                  */
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}



/***************************************************************************//**
 * @brief
 *   Updates the Speed Setpoint Data
 *
 * @details
 * 	 This task is run as often as is specified by the OS timer tick value in the OSTimeDly
 * 	 function.
 *
 *
 ******************************************************************************/

static  void  Ex_MainSpeedSetpointTask (void  *p_arg)
{
    RTOS_ERR  err;
    uint8_t btn0_status = 0;
    uint8_t btn1_status = 0;


    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif
                                                                /* ... the platform manager at this moment.             */
    buttons_setup();
    while (DEF_ON) {
    	// pend on semaphore
    	OSSemPend (&fifo_sem, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    	// lock access to speed setpoint data structure using mutex
    	OSMutexPend (&speed_setpoint_mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    	// interpret results from fifo 1
//    	if (!is_empty(&btn0_fifo)) {
    	if (!(btn0_fifo_rd == btn0_fifo_wr)) {
//    		btn0_status = peek(&btn0_fifo);
    		btn0_status = btn0_fifo[btn0_fifo_rd];
    		btn0_fifo_rd += 1;
    		if (btn0_fifo_rd == 10) {
    			btn0_fifo_rd = 0;
    		}
//    		pop(&btn0_fifo);
    	}
    	// interpret results from fifo 2
//    	if (!is_empty(&btn1_fifo)) {
		if (!(btn1_fifo_rd == btn1_fifo_wr)) {
//    		btn1_status = peek(&btn1_fifo);
			btn1_status = btn1_fifo[btn1_fifo_rd];
			btn1_fifo_rd += 1;
    		if (btn1_fifo_rd == 10) {
    			btn1_fifo_rd = 0;
    		}
//    		pop(&btn1_fifo);
    	}
    	// access speed setpoint data structure here
    	if (btn0_status != 0) {
    		speed_setpoint_data.up_cnt++;
    		speed_setpoint_data.cur_speed += 5;
    	}
    	if (btn1_status != 0) {
    		speed_setpoint_data.dn_cnt++;
    		speed_setpoint_data.cur_speed -= 5;
    	}
    	// release access to speed setpoint
    	OSMutexPost (&speed_setpoint_mutex, OS_OPT_POST_NONE, &err);

    	// set the event flag for the speed setpoint change
		OSFlagPost ((OS_FLAG_GRP *) &vehicle_monitor_flg,
			(OS_FLAGS)      0x01,
			(OS_OPT)        OS_OPT_POST_FLAG_SET,
			(RTOS_ERR *)	&err);

        /* Delay Start Task execution for  */
		OSTimeDly( 10, OS_OPT_TIME_DLY, &err);
		/*   Check error code.                                  */
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/***************************************************************************//**
 * @brief
 *   Updates the Vehicle Direction Data
 *
 * @details
 * 	 This task is run as often as is specified by the OS timer tick value in the OSTimeDly
 * 	 function.
 *
 *
 ******************************************************************************/

static  void  Ex_MainVehicleDirectionTask (void  *p_arg)
{
    RTOS_ERR  err;
    uint8_t slider_pos;

    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif
                                                                /* ... the platform manager at this moment.             */
    slider_setup();
    while (DEF_ON) {
    	// pend on the semaphore that is to be posted by the OSTimer counter handler
        OSSemPend (&sem, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
        // retrieve the slider status
        slider_position(&slider_pos);

    	// lock access to vehicle direction data structure using mutex
    	OSMutexPend (&vehicle_direction_mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    	// modify vehicle direction
    	uint8_t prev_vehicle_direction = vehicle_direction_data.cur_dir;
    	vehicle_direction_data.cur_dir = slider_pos;

    	// see if still traveling in same direction
    	if (prev_vehicle_direction == HARD_LEFT || prev_vehicle_direction == SOFT_LEFT) {
    		if (slider_pos == HARD_LEFT || slider_pos == SOFT_LEFT) {
    			vehicle_direction_data.cur_dir_time += 1;
    		}
    		else {
    			vehicle_direction_data.cur_dir_time = 0;
    		}
    	}
    	else if (prev_vehicle_direction == HARD_RIGHT || prev_vehicle_direction == SOFT_RIGHT) {
    		if (slider_pos == HARD_RIGHT || slider_pos == SOFT_RIGHT) {
    			vehicle_direction_data.cur_dir_time += 1;
    		}
    		else {
    			vehicle_direction_data.cur_dir_time = 0;
    		}
    	}
    	else if (prev_vehicle_direction == INACTIVE) {
    		if (slider_pos == INACTIVE) {
    			vehicle_direction_data.cur_dir_time += 1;
    		}
    		else {
    			vehicle_direction_data.cur_dir_time = 0;
    		}
    	}
    	else {
    		vehicle_direction_data.cur_dir_time = 0;
    	}
    	// upgrade the count
    	if (slider_pos == HARD_LEFT || slider_pos == SOFT_LEFT) {
    		vehicle_direction_data.left_cnt++;
    	}
    	else if (slider_pos == HARD_RIGHT || slider_pos == SOFT_RIGHT) {
    		vehicle_direction_data.right_cnt++;
    	}
    	// release access to vehicle direction
    	OSMutexPost (&vehicle_direction_mutex, OS_OPT_POST_NONE, &err);
		// set the event flag for the speed setpoint change
		OSFlagPost ((OS_FLAG_GRP *) &vehicle_monitor_flg,
			(OS_FLAGS)      0x02,
			(OS_OPT)        OS_OPT_POST_FLAG_SET,
			(RTOS_ERR *)	&err);

    	/* Delay Start Task execution for  */
		OSTimeDly( 10,OS_OPT_TIME_DLY,&err);
		/*   Check error code.                                  */
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/***************************************************************************//**
 * @brief
 *   Updates the Vehicle Monitor Data
 *
 * @details
 * 	 This task is run as often as is specified by the OS timer tick value in the OSTimeDly
 * 	 function.
 *
 *
 ******************************************************************************/

static  void  Ex_MainVehicleMonitorTask (void  *p_arg)
{
    RTOS_ERR  err;
    OS_FLAGS flag;
    uint8_t cur_speed;
    uint8_t cur_dir_time;
    uint8_t cur_dir;

    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif
                                                                /* ... the platform manager at this moment.             */
    while (DEF_ON) {
    	// check the event flag
    	flag = OSFlagPend ((OS_FLAG_GRP  *) &vehicle_monitor_flg,
			(OS_FLAGS)      0x03,
			(OS_TICK)       0,
			(OS_OPT)        OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_BLOCKING + OS_OPT_PEND_FLAG_CONSUME,
			(CPU_TS *)      NULL,
			(RTOS_ERR *)    &err);

    	// Check for changes to various parameters
    	switch(flag) {
    	// Speed Change
    	case 0x01:
        	// lock access to speed data structure using mutex
        	OSMutexPend (&speed_setpoint_mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
        	cur_speed = speed_setpoint_data.cur_speed;
        	// release access to speed
        	OSMutexPost (&speed_setpoint_mutex, OS_OPT_POST_NONE, &err);
        	break;
		// Direction Change
		case 0x02:
			// lock access to vehicle direction data structure using mutex
			OSMutexPend (&vehicle_direction_mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
			cur_dir_time = vehicle_direction_data.cur_dir_time;
			cur_dir = vehicle_direction_data.cur_dir;
			// release access to vehicle direction
			OSMutexPost (&vehicle_direction_mutex, OS_OPT_POST_NONE, &err);
			break;
		default:
			break;
		}
    	// set the event flag for the led output for speeding 75 over
    	if (cur_speed > 75) {
    		OSFlagPost ((OS_FLAG_GRP *) &led_output_flg,
    			(OS_FLAGS)      0x02,
    			(OS_OPT)        OS_OPT_POST_FLAG_CLR,
    			(RTOS_ERR *)	&err);
    		OSFlagPost ((OS_FLAG_GRP *) &led_output_flg,
    			(OS_FLAGS)      0x01,
    			(OS_OPT)        OS_OPT_POST_FLAG_SET,
    			(RTOS_ERR *)	&err);
    	}
    	// set the event flag for the led output
    	else {
    		OSFlagPost ((OS_FLAG_GRP *) &led_output_flg,
    			(OS_FLAGS)      0x01,
    			(OS_OPT)        OS_OPT_POST_FLAG_CLR,
    			(RTOS_ERR *)	&err);
    		OSFlagPost ((OS_FLAG_GRP *) &led_output_flg,
    			(OS_FLAGS)      0x02,
    			(OS_OPT)        OS_OPT_POST_FLAG_SET,
    			(RTOS_ERR *)	&err);
    	}
    	// set the event flag for the led output for turning while 45 over
    	if (cur_speed > 45 && cur_dir != straight) {
    		OSFlagPost ((OS_FLAG_GRP *) &led_output_flg,
    			(OS_FLAGS)      0x08,
    			(OS_OPT)        OS_OPT_POST_FLAG_CLR,
    			(RTOS_ERR *)	&err);
    		OSFlagPost ((OS_FLAG_GRP *) &led_output_flg,
    			(OS_FLAGS)      0x04,
    			(OS_OPT)        OS_OPT_POST_FLAG_SET,
    			(RTOS_ERR *)	&err);
    	}
    	// set the event flag for the led output
    	else {
    		OSFlagPost ((OS_FLAG_GRP *) &led_output_flg,
    			(OS_FLAGS)      0x04,
    			(OS_OPT)        OS_OPT_POST_FLAG_CLR,
    			(RTOS_ERR *)	&err);
    		OSFlagPost ((OS_FLAG_GRP *) &led_output_flg,
    			(OS_FLAGS)      0x08,
    			(OS_OPT)        OS_OPT_POST_FLAG_SET,
    			(RTOS_ERR *)	&err);
    	}

    	// set the event flag for the led output for going in same direction for at least 5 seconds
    	if (cur_dir_time > 40) {
    		OSFlagPost ((OS_FLAG_GRP *) &led_output_flg,
    			(OS_FLAGS)      0x20,
    			(OS_OPT)        OS_OPT_POST_FLAG_CLR,
    			(RTOS_ERR *)	&err);
    		OSFlagPost ((OS_FLAG_GRP *) &led_output_flg,
    			(OS_FLAGS)      0x10,
    			(OS_OPT)        OS_OPT_POST_FLAG_SET,
    			(RTOS_ERR *)	&err);
    	}
    	// set the event flag for the led output
    	else {
    		OSFlagPost ((OS_FLAG_GRP *) &led_output_flg,
    			(OS_FLAGS)      0x10,
    			(OS_OPT)        OS_OPT_POST_FLAG_CLR,
    			(RTOS_ERR *)	&err);
    		OSFlagPost ((OS_FLAG_GRP *) &led_output_flg,
    			(OS_FLAGS)      0x20,
    			(OS_OPT)        OS_OPT_POST_FLAG_SET,
    			(RTOS_ERR *)	&err);
    	}
		/* Delay Start Task execution for  */
		OSTimeDly( 10, OS_OPT_TIME_DLY, &err);
		/*   Check error code.                                  */
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}


/***************************************************************************//**
 * @brief
 *   Drives the LEDs based on the pushbutton and slider values.
 *
 * @details
 * 	 This task is run as often as is specified by the OS timer tick value in the OSTimeDly
 * 	 function.
 *
 *
 ******************************************************************************/

static  void  Ex_MainLedOutputTask (void  *p_arg)
{
    RTOS_ERR  err;
    OS_FLAGS flag;
    uint8_t speed75;
    uint8_t turn45;
    uint8_t samefor5;

    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif
                                                                /* ... the platform manager at this moment.             */
    gpio_open();
    while (DEF_ON) {
        speed75 = 0;
        turn45 = 0;
        samefor5 = 0;
    	// check the event flag
    	flag = OSFlagPend ((OS_FLAG_GRP  *) &led_output_flg,
			(OS_FLAGS)      0xFF,
			(OS_TICK)       0,
			(OS_OPT)        OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_BLOCKING,
			(CPU_TS *)      NULL,
			(RTOS_ERR *)    &err);

    	if (flag & 0b00000001) {
    		speed75 = 1;
    	}
    	else {
    		speed75 = 0;
    	}
    	if (flag & 0b00000100) {
    		turn45 = 1;
    	}
    	else {
    		turn45 = 0;
    	}
    	if (flag & 0b00010000) {
    		samefor5 = 1;
    	}
    	else {
    		samefor5 = 0;
    	}
    	// If over 75 or turning while over 45, turn on LED0
    	if (speed75 || turn45) {
    		GPIO_PinOutSet(LED0_port, LED0_pin);
    	}
    	else {
    		GPIO_PinOutClear(LED0_port, LED0_pin);
    	}
    	// If straight for more than 5, turn on LED 1
    	if (samefor5) {
    		GPIO_PinOutSet(LED1_port, LED1_pin);
    	}
    	else {
    		GPIO_PinOutClear(LED1_port, LED1_pin);
    	}
        /* Delay Start Task execution for  */
		OSTimeDly( 10,OS_OPT_TIME_DLY,&err);
		/*   Check error code.                                  */
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/***************************************************************************//**
 * @brief
 *   Updates the LCD Display
 *
 * @details
 * 	 This task is run as often as is specified by the OS timer tick value in the OSTimeDly
 * 	 function.
 *
 *
 ******************************************************************************/

static  void  Ex_MainLcdDisplayTask (void  *p_arg)
{
    RTOS_ERR  err;
    EMSTATUS status;
    char clockString[16];

    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif
                                                                /* ... the platform manager at this moment.             */
    /* Initialize display module */
    status = DISPLAY_Init();
    if (DISPLAY_EMSTATUS_OK != status) {
      while (true)
        ;
    }
    /* Initialize the DMD module for the DISPLAY device driver. */
    status = DMD_init(0);
    if (DMD_OK != status) {
      while (true)
        ;
    }
    status = GLIB_contextInit(&gc);
    if (GLIB_OK != status) {
      while (true)
        ;
    }
    while (DEF_ON) {
        GLIB_setFont(&gc, (GLIB_Font_t *)&GLIB_FontNumber16x20);
        gc.backgroundColor = White;
        gc.foregroundColor = Black;
        GLIB_clear(&gc);
        sprintf(clockString, "%02d:%02d:%02d", vehicle_direction_data.cur_dir, vehicle_direction_data.cur_dir_time, speed_setpoint_data.cur_speed);
        GLIB_drawString(&gc, clockString, strlen(clockString), 1, 52, true);
        /* Update display */
        DMD_updateDisplay();
        /* Delay Start Task execution for  */
		OSTimeDly( 10,OS_OPT_TIME_DLY,&err);
		/*   Check error code.                                  */
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/***************************************************************************//**
 * @brief
 *   Callback function for kernel timer.
 *
 * @details
 * 	 Posts on a semaphore shared by the slider input task. This function is not
 * 	 called until the OSTimer reaches a 10 ms timeout. This function is called
 * 	 periodically every 10 ms to signal the slider input task to execute.
 *
 * @note
 *   This function is called every time the OSTimer counts 10 ms.
 *
 ******************************************************************************/
static void MyCallback (OS_TMR p_tmr, void *p_arg) {
	RTOS_ERR  err;
	OSSemPost (&sem, OS_OPT_POST_ALL, &err);
}


/***************************************************************************//**
 * @brief
 *   Interrupt handler for GPIO Even pins module.
 *
 *
 * @details
 * 	 Utilizes global variable PB0_status to poll the value of pushbutton 0
 * 	 using poll_PB0() function.
 *
 * @note
 *   This function is called every time PB0 is pressed.
 *
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
	RTOS_ERR  err;
	__disable_irq();
	/* Get and clear all pending GPIO interrupts */
	uint32_t interruptMask = GPIO_IntGet();
	GPIO_IntClear(interruptMask);
	bool PB0_status;
	poll_PB0(&PB0_status);
	if (PB0_status) {
		btn0_fifo[btn0_fifo_wr] = 1;
	    btn0_fifo_wr += 1;
	    if (btn0_fifo_wr == 10) {
	    	btn0_fifo_wr = 0;
	    }
//		push(&btn0_fifo, 1);
		OSSemPost (&fifo_sem, OS_OPT_POST_ALL, &err);
	}
	else {
		btn0_fifo[btn0_fifo_wr] = 0;
	    btn0_fifo_wr += 1;
	    if (btn0_fifo_wr == 10) {
	    	btn0_fifo_wr = 0;
	    }
//		push(&btn0_fifo, 0);
		OSSemPost (&fifo_sem, OS_OPT_POST_ALL, &err);
	}
	__enable_irq();
}

/***************************************************************************//**
 * @brief
 *   Interrupt handler for GPIO Odd pins module.
 *
 *
 * @details
 * 	 Utilizes global variable PB1_status to poll the value of pushbutton 1
 * 	 using poll_PB1() function.
 *
 * @note
 *   This function is called every time PB1 is pressed.
 *
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
	RTOS_ERR  err;
	__disable_irq();
	/* Get and clear all pending GPIO interrupts */
	uint32_t interruptMask = GPIO_IntGet();
	GPIO_IntClear(interruptMask);
	bool PB1_status;
	poll_PB1(&PB1_status);
	if (PB1_status) {
		btn1_fifo[btn1_fifo_wr] = 1;
	    btn1_fifo_wr += 1;
	    if (btn1_fifo_wr == 10) {
	    	btn1_fifo_wr = 0;
	    }
//		push(&btn1_fifo, 1);
		OSSemPost (&fifo_sem, OS_OPT_POST_ALL, &err);
	}
	else {
		btn1_fifo[btn1_fifo_wr] = 0;
	    btn1_fifo_wr += 1;
	    if (btn1_fifo_wr == 10) {
	    	btn1_fifo_wr = 0;
	    }
//		push(&btn1_fifo, 0);
		OSSemPost (&fifo_sem, OS_OPT_POST_ALL, &err);
	}

	__enable_irq();
}
