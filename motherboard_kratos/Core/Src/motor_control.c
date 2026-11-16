#include "motor_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"
#include <kratos_teerth_rohan_pandya_msgs/msg/arm_cmd.h>
#include <kratos_teerth_rohan_pandya_msgs/msg/wheel_cmd.h>
#include <std_msgs/msg/bool.h>
#include <stdbool.h>
volatile bool   flag;
extern volatile double arm_cmd[7];
extern volatile double wheel_info[8];
void motor_control_Run(void *argument)
{
    /* Initialization code for motor_control goes here (runs once) */
    
    for (;;)
    {
    	if (flag == true)

    	{
    		//run ARM code here
    	}

    	else
		{
    		//run LD motor code here
		}

        /* Infinite task logic goes here */

        osDelay(10); /* Always yield the CPU */
    }
}
