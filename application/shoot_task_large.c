#include "board.h"
#include "dbus.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"


#include "init.h"
#include "chassis_task.h"
#include "gimbal_task.h"

#include "protocol.h"
#include "referee_system.h"
#include "hero_cmd.h"
#include "shoot_task_large.h"
//Timer necessary
#include "tim.h"
#include "stm32f4xx_hal.h"


void set_large_fric_speed(double input_speed){
//Input between 0-100, 0 means 0% speed, 100% means 100% speed
//Convert input percentage to number between 1000-2000, because the motor can only take 1000us-2000us pulse widths
uint16_t pw_value=round(input_speed*10+1000);
	
//Set speed for left wheel
	__HAL_TIM_SET_COMPARE(FRIC_LARGE_L_TIMER,FRIC_LARGE_L_CHANNEL,pw_value);
  __HAL_TIM_SET_COMPARE(FRIC_LARGE_R_TIMER,FRIC_LARGE_R_CHANNEL,pw_value);
//Set speed for right wheel
}

void init_large_frics(){
	  uint32_t period = osKernelSysTick();
	  HAL_TIM_PWM_Start(FRIC_LARGE_L_TIMER,FRIC_LARGE_L_CHANNEL);
	  HAL_TIM_PWM_Start(FRIC_LARGE_R_TIMER,FRIC_LARGE_R_CHANNEL);
		//HAL_Delay(1750);
		__HAL_TIM_SET_COMPARE(FRIC_LARGE_L_TIMER,FRIC_LARGE_L_CHANNEL,2000);
	  __HAL_TIM_SET_COMPARE(FRIC_LARGE_R_TIMER,FRIC_LARGE_R_CHANNEL,2000);
		  osDelayUntil(&period, 2000);
		__HAL_TIM_SET_COMPARE(FRIC_LARGE_L_TIMER,FRIC_LARGE_L_CHANNEL,1000);
	  __HAL_TIM_SET_COMPARE(FRIC_LARGE_R_TIMER,FRIC_LARGE_R_CHANNEL,1000);
		  osDelayUntil(&period, 1750);

}


void shoot_task_large(void const *argument){
	///System variables
	 uint32_t period = osKernelSysTick();
	 
	//Initialize the PWM ports (start PWM and give initialization signal)
	 init_large_frics();

	
	//Finish initilization of ports
	  //double speed=0;
    while(1){
			//If condition is okay, then shoot large task should set the friction wheels at the same input speed, will only worry about left wheel for now
			set_large_fric_speed(5);
			osDelayUntil(&period,5);

    }
}



