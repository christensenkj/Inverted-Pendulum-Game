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

#include <math.h>
/*
*********************************************************************************************************
*********************************************************************************************************
*                                             LOCAL DEFINES
*********************************************************************************************************
*********************************************************************************************************
*/

#define  EX_MAIN_START_TASK_PRIO              		20u
#define  EX_MAIN_FORCE_TASK_PRIO       				21u
#define  EX_MAIN_PHYSICS_TASK_PRIO    				22u
#define  EX_MAIN_GAIN_TASK_PRIO	    				23u
#define  EX_MAIN_LCD_DISPLAY_TASK_PRIO    			24u
#define  EX_MAIN_IDLE_TASK_PRIO              		25u

#define  EX_MAIN_START_TASK_STK_SIZE         		512u
#define  EX_MAIN_PHYSICS_TASK_STK_SIZE			 	512u
#define  EX_MAIN_FORCE_TASK_STK_SIZE	    		512u
#define  EX_MAIN_GAIN_TASK_STK_SIZE					512u
#define  EX_MAIN_LCD_DISPLAY_TASK_STK_SIZE	    	512u
#define  EX_MAIN_IDLE_TASK_STK_SIZE					512u
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

/* Speed Setpoint Task Stack.                                    */
static  CPU_STK  Ex_MainPhysicsTaskStk[EX_MAIN_PHYSICS_TASK_STK_SIZE];
/* Speed Setpoint Task TCB.                                      */
static  OS_TCB   Ex_MainPhysicsTaskTCB;

/* Vehicle Direction Task Stack.                                    */
static  CPU_STK  Ex_MainForceTaskStk[EX_MAIN_FORCE_TASK_STK_SIZE];
/* LedOutput Task TCB.                                      */
static  OS_TCB   Ex_MainForceTaskTCB;

/* Vehicle Monitoring Task Stack.                                    */
static  CPU_STK  Ex_MainGainTaskStk[EX_MAIN_GAIN_TASK_STK_SIZE];
/* LedOutput Task TCB.                                      */
static  OS_TCB   Ex_MainGainTaskTCB;

/* LCD Display Task Stack.                                    */
static  CPU_STK  Ex_MainLcdDisplayTaskStk[EX_MAIN_LCD_DISPLAY_TASK_STK_SIZE];
/* LedOutput Task TCB.                                      */
static  OS_TCB   Ex_MainLcdDisplayTaskTCB;

/* Create OS Task Flags */

/* Create OS Task Flags for vehicle monitor task*/
static OS_FLAG_GRP	input_flag;

/* Create OS Task Flags for LED task*/
static OS_FLAG_GRP	output_flag;

/* Create semaphore */
static OS_SEM		timer_sem;

/* Create semaphore to keep track of btn action additions to fifo*/
static OS_SEM		fifo_sem;

/* Create a mutex to protect access to speed setpoint data */
static OS_MUTEX  	gain_mutex;

/* Create a mutex to protect access to vehicle direction data */
static OS_MUTEX  	physics_mutex;

/* Create OS Timer */
static OS_TMR		tmr;

/* Create an enum to store vehicle direction states */
enum force {
	none,
	hard_left,
	soft_left,
	soft_right,
	hard_right,
};

/* Declare a fifo for button actions */
struct fifo_struct btn0_fifo;
struct fifo_struct btn1_fifo;

// global gain and force ints
uint32_t gain_data;
double force_data;

// Data type for physics task
struct physics_data_struct {
	uint8_t version;
	uint8_t game_state;
	double gravity;
	double bob_mass;
	double cart_mass;
	double len;
	uint32_t xmin;
	uint32_t xmax;
	uint8_t variant;
	double max_force;
	uint8_t num_obstacles;
	double theta;
	double x_pos;
	double w;
	double v;
} physics_data;

/* Global glib context */
GLIB_Context_t gc;

// System time
static volatile uint32_t msTicks; /* counts 1ms timeTicks */
static volatile uint32_t start = 0; /* counts 1ms timeTicks */

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
static  void  Ex_MainPhysicsTask (void  *p_arg);
static  void  Ex_MainForceTask (void  *p_arg);
static  void  Ex_MainGainTask (void  *p_arg);
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



/***************************************************************************//**
 * @brief
 *   Calculates time derivative of angle
 ******************************************************************************/
double dthetadt(double w) {
    return w;
}
/***************************************************************************//**
 * @brief
 *   Calculates time derivative of position along x axis
 ******************************************************************************/
double dxdt(double v) {
    return v;
}
/***************************************************************************//**
 * @brief
 *   Calculates time derivative of angular velocity
 ******************************************************************************/
double dwdt(double F, double theta, double w, double L, double M, double m, double g) {
    return ( 1/((M+m) - m*cos(theta)*cos(theta)) * (F*cos(theta)/L + (g/L)*((M+m)*sin(theta)) - m*w*w*sin(theta)*cos(theta)) );
}
/***************************************************************************//**
 * @brief
 *   Calculates time derivative of positional velocity along x axis
 ******************************************************************************/
double dvdt(double F, double theta, double w, double L, double M, double m, double g) {
    return ( 1/( cos(theta)*cos(theta) - (M+m)/(m)) * ( (w*w*L*sin(theta) - F/(m) - g*sin(theta)*cos(theta)) ) );
}



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

    /* Initialize System.                                   */
    BSP_SystemInit();
    /* Initialize CPU.                                      */
    CPU_Init();

    OS_TRACE_INIT();
    /* Initialize the Kernel.                               */
    OSInit(&err);
    /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

    /* Create the Start Task.                               */
    OSTaskCreate(&Ex_MainStartTaskTCB,
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
    msTicks = 0;

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
    gpio_open();
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
    /* Create the Speed Setpoint Task.                               */
    OSTaskCreate(&Ex_MainPhysicsTaskTCB,
                 "Ex Main Physics Task",
                  Ex_MainPhysicsTask,
                  DEF_NULL,
                  EX_MAIN_PHYSICS_TASK_PRIO,
                 &Ex_MainPhysicsTaskStk[0],
                 (EX_MAIN_PHYSICS_TASK_STK_SIZE / 10u),
                  EX_MAIN_PHYSICS_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    /* Create the Vehicle Direction Task.                               */
    OSTaskCreate(&Ex_MainForceTaskTCB,
                 "Ex Main Force Task",
                  Ex_MainForceTask,
                  DEF_NULL,
                  EX_MAIN_FORCE_TASK_PRIO,
                 &Ex_MainForceTaskStk[0],
                 (EX_MAIN_FORCE_TASK_STK_SIZE / 10u),
                  EX_MAIN_FORCE_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    /* Create the Vehicle Monitor Task.                               */
    OSTaskCreate(&Ex_MainGainTaskTCB,
                 "Ex Main Gain Task",
                  Ex_MainGainTask,
                  DEF_NULL,
                  EX_MAIN_GAIN_TASK_PRIO,
                 &Ex_MainGainTaskStk[0],
                 (EX_MAIN_GAIN_TASK_STK_SIZE / 10u),
                  EX_MAIN_GAIN_TASK_STK_SIZE,
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
    		&input_flag,
			(CPU_CHAR *) "input flag",
			(OS_FLAGS) 0,
			(RTOS_ERR *)&err);
    // Create the event flag
    OSFlagCreate((OS_FLAG_GRP *)
    		&output_flag,
			(CPU_CHAR *) "output flag",
			(OS_FLAGS) 0,
			(RTOS_ERR *)&err);
    // Create the semaphore
    OSSemCreate ((OS_SEM *) &timer_sem,
            (CPU_CHAR *) "Slider Timer Semaphore",
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
			(OS_TICK) 5,
			(OS_TICK) 5,
			(OS_OPT) OS_OPT_TMR_PERIODIC,
			(OS_TMR_CALLBACK_PTR) &MyCallback,
			(void *) 0,
			(RTOS_ERR *) &err);
    // Create the mutex
    OSMutexCreate ((OS_MUTEX *) &physics_mutex,
			(CPU_CHAR *) "Physics Mutex",
			(RTOS_ERR *) &err);
    // Create the mutex
    OSMutexCreate ((OS_MUTEX *) &gain_mutex,
			(CPU_CHAR *) "Gain Mutex",
			(RTOS_ERR *) &err);
    // Declare the two FIFOs
    fifo_init(&btn0_fifo);
    fifo_init(&btn1_fifo);

    // Start the timer
    OSTmrStart ((OS_TMR *) &tmr, (RTOS_ERR *) &err);

    /* Setup SysTick timer for 1 msec interrupts  */
    if (SysTick_Config(SystemCoreClockGet() / 1000)) {
      while (1) ;
    }

    // suspend the start task once all init is done
    OSTaskSuspend (&Ex_MainStartTaskTCB, &err);

    while (DEF_ON) {
        /* Delay Start Task execution for                       */
        OSTimeDly( 1000, OS_OPT_TIME_DLY, &err);
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

    /* Prevent compiler warning.                            */
    PP_UNUSED_PARAM(p_arg);

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif
                                                                /* ... will register all the hardware controller to ... */
                                                                /* ... the platform manager at this moment.             */

    while (DEF_ON) {
    	EMU_EnterEM1();
        /* Delay Start Task execution for                       */
//		OSTimeDly(10, OS_OPT_TIME_DLY, &err);
        /*   Check error code.                                  */
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}



/***************************************************************************//**
 * @brief
 *   Updates the Physics data structure to reflect time-accurate and physics-accurate changes
 *
 * @details
 * 	 Time is kept using Systick Timer with resolution of 1 ms.
 * 	 The physics uses a Semi-Implicit Euler's method for solving the second order system of nonlinear differential
 * 	 equations characteristic to the inverted pendulum problem.
 *
 *
 ******************************************************************************/

static  void  Ex_MainPhysicsTask (void  *p_arg)
{
    RTOS_ERR  err;
    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */
#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif
                                                                /* ... the platform manager at this moment.             */

    // set the initial conditions
    // initial system time
    uint32_t start_time = 0;
    // set all physics data structure parameters
    physics_data.version = 2;
    physics_data.game_state = 0;
	physics_data.gravity = 9.81;
	physics_data.bob_mass = 20;
	physics_data.cart_mass = 20;
	physics_data.len = 30;
	physics_data.xmin = 0;
	physics_data.xmax = 125;
	physics_data.variant = 1;
	physics_data.max_force =
	physics_data.theta = 0.01;
	physics_data.x_pos = 0;
	physics_data.w = 0;
	physics_data.v = 0;
	physics_data.num_obstacles = 0;
	// set the game to initially play mode
    uint8_t game_reset = 0;
    // local variables to store values for the solver
    double v_int;
    double w_int;
    double x_int;
    double theta_int;
    uint32_t x_min;
    uint32_t x_max;

    while (DEF_ON) {
    	// Check that the game is currently being played
    	if (!game_reset) {

    		// get the end time of the last iteration
    		uint32_t end_time = msTicks;
    		double dt = (double)(end_time-start_time)/1000;
        	double F = force_data*gain_data;

        	// obtain access to physics data
        	OSMutexPend (&physics_mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
        	double x = physics_data.x_pos;
        	double v = physics_data.v;
        	double theta = physics_data.theta;
        	double w = physics_data.w;
        	double L = physics_data.len;
        	double M = physics_data.cart_mass;
        	double m = physics_data.bob_mass;
        	double g = physics_data.gravity;
        	x_min = physics_data.xmin;
        	x_max = physics_data.xmax;
        	// release access to physics data
        	OSMutexPost (&physics_mutex, OS_OPT_POST_NONE, &err);

            // Solve the parameters of the physics system using Semi-Implicit Euler
            v_int = v + dt*dvdt(F, theta, w, L, M, m, g);
            // In semi-implicit Euler, update the position coordinate using the
            // velocity at the new time.
            x_int = x + dt*dxdt(v_int);
            // Similarly for the angles.
            w_int = w + dt*dwdt(F, theta, w, L, M, m, g);
            theta_int = theta + dt*dthetadt(w_int);

        	// obtain access to physics data
        	OSMutexPend (&physics_mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
        	physics_data.x_pos = x_int;
        	physics_data.v = v_int;
        	physics_data.w = w_int;
        	physics_data.theta = theta_int;
        	// release access to physics data
        	OSMutexPost (&physics_mutex, OS_OPT_POST_NONE, &err);

        	// get the start time for the next iteration
            start_time = msTicks;

            // Check game ending conditions
            // Has the post fallen?
            if (fabs(theta_int) > 1.57) {
            	game_reset = 1;
            }
            // has the cart reached the end points?
            if (((int) (66 + x_int)) < x_min ||
            		((int) (66 + x_int)) > x_max) {
            	game_reset = 1;
            }
    	}
    	// If the game is ended, stay here
    	else {
    		// obtain access to physics data
    		OSMutexPend (&physics_mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    		physics_data.game_state = 1;
        	// release access to physics data
        	OSMutexPost (&physics_mutex, OS_OPT_POST_NONE, &err);
        	GPIO_PinOutSet(LED0_port, LED0_pin);
    	}

        /* Delay Start Task execution for 8 ms */
		OSTimeDly(8, OS_OPT_TIME_DLY, &err);
		/*   Check error code.                                  */
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/***************************************************************************//**
 * @brief
 *   Updates the Force data
 *
 * @details
 * 	 This task is used to update the force applied to the cart
 *
 *
 ******************************************************************************/

static  void  Ex_MainForceTask (void  *p_arg)
{
    RTOS_ERR  err;
    uint8_t slider_pos;
    // set the necessary force parameters
    physics_data.max_force = 150.0;
    double nominal_force = 100.0;

    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif
                                                                /* ... the platform manager at this moment.             */
    slider_setup();
    while (DEF_ON) {
    	// Pend on a timer semaphore
    	OSSemPend (&timer_sem, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
        // retrieve the slider status
        slider_position(&slider_pos);
        // Interpret the slider value
        switch (slider_pos) {
        case INACTIVE:
        	force_data = 0;
        	break;
        case HARD_LEFT:
        	force_data = -1.5*nominal_force;
        	break;
        case SOFT_LEFT:
        	force_data = -1*nominal_force;
        	break;
        case SOFT_RIGHT:
        	force_data = nominal_force;
        	break;
        case HARD_RIGHT:
        	force_data = 1.5*nominal_force;
        	break;
        default:
        	break;
        }
		/*   Check error code.                                  */
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/***************************************************************************//**
 * @brief
 *   Updates the Vehicle Monitor Data
 *
 * @details
 * 	 This task is used to check vehicle parameters and signal the LCD task to display and alert the LED task of warnings
 *
 *
 ******************************************************************************/

static  void  Ex_MainGainTask (void  *p_arg)
{
    RTOS_ERR  err;
    gain_data = 0;
    uint8_t btn0_status = 0;
    uint8_t btn1_status = 0;

    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif
    buttons_setup();                                           /* ... the platform manager at this moment.             */
    // Initialize the gain data to 6 out of 20
    gain_data = 6;
    while (DEF_ON) {
    	// pend on semaphore
    	OSSemPend (&fifo_sem, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    	// perform fifo operations atomically
    	CORE_DECLARE_IRQ_STATE;           // Storage for saving IRQ state prior
    	CORE_ENTER_ATOMIC();
    	// interpret results from fifo 0
    	if (!fifo_isempty(&btn0_fifo)) {
    		btn0_status = fifo_pop(&btn0_fifo);
    	}
    	// interpret results from fifo 1
    	if (!fifo_isempty(&btn1_fifo)) {
    		btn1_status = fifo_pop(&btn1_fifo);
    	}
		CORE_EXIT_ATOMIC();

		// Apply changes to the gain data as indicated by values in the button fifos
		if (btn0_status && (gain_data > 0)) {
			gain_data-=2;
		}
		else if (btn1_status && (gain_data < 20)) {
			gain_data+=2;
		}

		/*   Check error code.                                  */
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/***************************************************************************//**
 * @brief
 *   Updates the LCD Display
 *
 * @details
 * 	 This task is used to display the post, the cart, the count at the bottom, and the ground
 *
 ******************************************************************************/

static  void  Ex_MainLcdDisplayTask (void  *p_arg)
{
    RTOS_ERR  err;
    EMSTATUS status;
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
    // declare a rectangle object
    GLIB_Rectangle_t pRect;
    // set the length to 40
    double len = 40;
    // initialize an array to hold the time string
    char time[4];
    // local variables to hold physics data
    double x_pos;
    double theta;
    // variables to keep track
    uint8_t game_state;
    uint32_t sys_time;
    while (DEF_ON) {
    	// initialize the display object
    	GLIB_clear(&gc);
    	GLIB_setFont(&gc, (GLIB_Font_t *)&GLIB_FontNarrow6x8);
        gc.backgroundColor = White;
        gc.foregroundColor = Black;

    	// obtain access to physics data
    	OSMutexPend (&physics_mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    	x_pos = physics_data.x_pos;
    	theta = physics_data.theta;
		game_state = physics_data.game_state;
    	// release access to physics data
    	OSMutexPost (&physics_mutex, OS_OPT_POST_NONE, &err);

    	// calculate positions of LCD elements
        int startx = 66 + x_pos;
        int starty = 66;
        int endx = (int) (startx - sin(theta)*len);
        int endy = (int) (starty - cos(theta)*len);
        pRect.xMax = startx+10;
        pRect.xMin = startx-10;
        pRect.yMax = starty+5;
        pRect.yMin = starty;
        // draw LCD shapes
	    GLIB_drawLine(&gc, startx, starty, endx, endy);
	    GLIB_drawCircleFilled(&gc, endx, endy, 4);
	    GLIB_drawRectFilled(&gc, &pRect);
	    GLIB_drawLine(&gc, 0, 72, 125, 72);
	    if (!game_state) {
	    	sys_time = (int) (msTicks/1000);
		    /* Print time */
		    sprintf(time, "%lu", sys_time);
	    }
	    GLIB_setFont(&gc, (GLIB_Font_t *)&GLIB_FontNumber16x20);
	    GLIB_drawString(&gc, time, 4, 52, 90, 0);
	    /* Update display */
	    DMD_updateDisplay();
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
	OSSemPost (&timer_sem, OS_OPT_POST_ALL, &err);
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
	    fifo_push(&btn1_fifo, 1);
		OSSemPost (&fifo_sem, OS_OPT_POST_ALL, &err);
	}
	else {
	    fifo_push(&btn1_fifo, 0);
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
	    fifo_push(&btn0_fifo, 1);
		OSSemPost (&fifo_sem, OS_OPT_POST_ALL, &err);
	}
	else {
	    fifo_push(&btn0_fifo, 0);
		OSSemPost (&fifo_sem, OS_OPT_POST_ALL, &err);
	}

	__enable_irq();
}


/***************************************************************************//**
 * @brief
 *   SysTick_Handler. Interrupt Service Routine for system tick counter
 ******************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;
  // determine times for PWM
  if (msTicks%20 == 0) {
	  start = msTicks;
  }
  if (msTicks < start + gain_data) {
	  GPIO_PinOutSet(LED1_port, LED1_pin);
  }
  else{
	  GPIO_PinOutClear(LED1_port, LED1_pin);
  }
}
