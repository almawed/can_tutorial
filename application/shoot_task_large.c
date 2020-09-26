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

#include "stm32f4xx_hal.h"

void shoot_task_large(void const *argument){
    while(1){
        HAL_Delay(1000);
        HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);   
    }
}