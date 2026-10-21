#include "motor_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"
#include <kratos_teerth_rohan_pandya_msgs/msg/rover_status.h>
void motor_control_Run(void *argument)
{
    /* Initialization code for motor_control goes here (runs once) */
    
    for (;;)
    {
        /* Infinite task logic goes here */
        
        osDelay(10); /* Always yield the CPU */
    }
}
