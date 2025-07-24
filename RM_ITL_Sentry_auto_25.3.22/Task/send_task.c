#include "chassis_task.h"
#include "chassis_power_control.h"
#include "cmsis_os.h"
#include "main.h"
#include "arm_math.h"
#include "detect_task.h"
#include "INS_task.h"
#include "CAN_receive.h"
#include "send_task.h"
#include "referee.h"
#include "USART_receive.h"

extern chassis_move_t chassis_move;
extern ext_game_robot_state_t robot_state;
extern ext_power_heat_data_t power_heat_data_t;
extern int c_switch;

void Send_task(void *p_arg)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	while (1)
	{
//		flag=1;
		Can_Send_Msg2(c_switch, chassis_move.motor_chassis[1].give_current, robot_state.chassis_power_limit, power_heat_data_t.chassis_power_buffer); // CAN1传出电机速度数据
		//		Can_Send_Msg(output_201,output_202,output_203,output_204);//CAN1传出电调电流数据
		vTaskDelay(1);
	}
}