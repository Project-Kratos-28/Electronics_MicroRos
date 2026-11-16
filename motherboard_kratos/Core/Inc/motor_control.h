#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <stdbool.h>

extern volatile bool flag;
void motor_control_Run(void *argument);

#endif /* MOTOR_CONTROL_H */
