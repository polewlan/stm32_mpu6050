#include <includes.h>
#include "gpio.h"
#include "mpu6050.h"

struct MPU_DATA
{
	short gx;
	short gy;
	short gz;
	short ax;
	short ay;
	short az;
	float roll;
	float pitch;
	float yaw;
	/* data */
};


static CPU_STK   Task_Start_STK[APP_TASK_START_STK_SIZE] ;
static CPU_STK   Task_Led1_STK[APP_TASK_START_STK_SIZE] ;






static OS_TCB Task_Start_TCB ;
static OS_TCB Task_Led1_TCB ;





void Task_Start(void *p_arg) ;
void Task_Led1(void *p_arg) ;



struct MPU_DATA mpu_data;


void OS_Start(void ) 
{
	OS_ERR err ;
	
	OSInit (&err) ;
	
	OSTaskCreate ( (OS_TCB     *) &Task_Start_TCB,
							   (CPU_CHAR   *) "Task_Start",
	               (OS_TASK_PTR ) Task_Start ,
								 (void       *) 0 ,
								 (OS_PRIO     ) 4 ,
                 (CPU_STK    *) Task_Start_STK ,
                 (CPU_STK_SIZE)	APP_TASK_START_STK_SIZE /10 ,
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE ,
                 (OS_MSG_QTY  ) 0u ,
                 (OS_TICK     ) 0u ,
                 (void       *) 0 ,
                 (OS_OPT      ) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),	
                 (OS_ERR     *) &err );

  OSStart (&err ) ;								 
								
}

void Task_Start(void *p_arg)
{
	CPU_INT32U cpu_clk_freq ;
	CPU_INT32U cnts ;
	OS_ERR err ;
	
	(void) p_arg ;
	
	
	
	
	BSP_Init ();  
	//led_on ;
	
	CPU_Init () ;  
	
	cpu_clk_freq = BSP_CPU_ClkFreq () ; 
	
	cnts = cpu_clk_freq / (CPU_INT32U ) OSCfg_TickRate_Hz ;
	
	OS_CPU_SysTickInit (cnts );
	
	Mem_Init () ;
	
	
	OSTaskCreate ( (OS_TCB     *) &Task_Led1_TCB,
							   (CPU_CHAR   *) "Task_Led1",
	               (OS_TASK_PTR ) Task_Led1 ,
								 (void       *) 0 ,
								 (OS_PRIO     ) 4 ,
                 (CPU_STK    *) Task_Led1_STK ,
                 (CPU_STK_SIZE)	APP_TASK_START_STK_SIZE /10 ,
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE ,
                 (OS_MSG_QTY  ) 0u ,
                 (OS_TICK     ) 0u ,
                 (void       *) 0 ,
                 (OS_OPT      ) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),	
                 (OS_ERR     *) &err );
		



       OSTaskDel ( & Task_Start_TCB, & err );


}	

void Task_Led1(void *p_arg)
{
	
	
	  OS_ERR      err;


    while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.       */
			led1_toggle ;
			
			MPU_Get_Gyroscope(&mpu_data.gx,&mpu_data.gy,&mpu_data.gz);

			printf("gx : %hd gy : %hd gz : %hd\n",mpu_data.gx,mpu_data.gy,mpu_data.gz);
			
			OSTimeDly ( 1000, OS_OPT_TIME_DLY, & err );
    }

	
}
















