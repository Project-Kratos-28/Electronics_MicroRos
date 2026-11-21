#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <stdbool.h>

extern volatile double arm_cmd[7];
extern volatile double wheel_info[8];
extern volatile double ld_cmd[10];
extern volatile bool flag;
void motor_control_Run(void *argument);

#endif /* MOTOR_CONTROL_H */
