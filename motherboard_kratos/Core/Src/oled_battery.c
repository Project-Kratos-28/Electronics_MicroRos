#include "oled_battery.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"
#include <kratos_teerth_rohan_pandya_msgs/msg/arm_cmd.h>
#include <kratos_teerth_rohan_pandya_msgs/msg/wheel_cmd.h>
void oled_battery_Run(void *argument)
{
    /* Initialization code for oled_battery goes here (runs once) */
    
    for (;;)
    {
        /* Infinite task logic goes here */
        
        osDelay(10); /* Always yield the CPU */
    }
}
