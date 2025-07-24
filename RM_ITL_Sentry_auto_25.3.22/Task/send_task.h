#ifndef __SEND_TASK_H
#define __SEND_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

void Send_task(void *p_arg);
#endif