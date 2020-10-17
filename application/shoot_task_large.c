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

void set_large_fric_speed(uint16_t input_speed){
//Input between 1000-2000, 1000 for no spin, 2000 for fastest speed
//Set speed for left wheel
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,input_speed);

//Set speed for right wheel
}


void shoot_task_large(void const *argument){
	///System variables
	 uint32_t period = osKernelSysTick();
	 
	//Initialize the PWM ports (start PWM and give initialization signal)
			HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
		//HAL_Delay(1750);
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,2000);
		  osDelayUntil(&period, 2000);
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,1000);
		  osDelayUntil(&period, 1750);
	//Finish initilization of ports
	  uint16_t speed=0;
    while(1){
			//If condition is okay, then shoot large task should set the friction wheels at the same input speed, will only worry about left wheel for now
			while (speed<2000){
			set_large_fric_speed(speed++);
			osDelayUntil(&period,5);
			}
			while (speed>1000){
			set_large_fric_speed(speed--);
			osDelayUntil(&period,5);
		  }   
    }
}



