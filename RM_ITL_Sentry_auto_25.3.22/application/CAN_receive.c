/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "detect_task.h"

#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
float _207_angle[2];

/**
 * @brief data of capacity
 * @param PowerData[0] input voltage
 * @param PowerData[1] cap voltage
 * @param PowerData[2] input current
 * @param PowerData[3] target power
 */
float PowerData[4];

int16_t round_207;
float this_angle_207;
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }
static motor_measure_t motor_chassis[4];  //底盘电机
static motor_measure_t motor_gimbal[2];   //云台电机
static motor_measure_t motor_trigger;     //拨弹电机
static motor_measure_t motor_friction[2]; //摩擦轮电机
static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static CAN_TxHeaderTypeDef tx_message;
static uint8_t chassis_can_send_data[8];
static uint8_t tx_message_data[8];
static CAN_TxHeaderTypeDef cap_tx_message;
static uint8_t cap_can_send_data[8];

/**
 * @brief           计算3508电机累计旋转角度
 * @param[out]      motor:电机结构数据指针
 */
void Calc_motor_Angle(motor_measure_t *motor)
{
  if (motor->ecd - motor->last_ecd > 4095.5)
  {
    motor->round--;
  }
  else if (motor->ecd - motor->last_ecd < -4095.5)
  {
    motor->round++;
  }
  motor->angle = (motor->round * ANGLE_T + motor->last_ecd) * 0.04395067757f; //转换为360度一圈
}

/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;

  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  if (hcan == &hcan1)
  {
    switch (rx_header.StdId)
    {
    case CAN_3508_M1_ID:
    {
      get_motor_measure(&motor_chassis[0], rx_data); //读取数据
      // Calc_motor_Angle(&motor_chassis[0]);//角度计算
      detect_hook(CHASSIS_MOTOR1_TOE); //错误检测
      break;
    }
    case CAN_3508_M2_ID:
    {
      get_motor_measure(&motor_chassis[1], rx_data);
      // Calc_motor_Angle(&motor_chassis[1]);
      detect_hook(CHASSIS_MOTOR2_TOE);
      break;
    }
    case CAN_3508_M3_ID:
    {
      get_motor_measure(&motor_chassis[2], rx_data);
      // Calc_motor_Angle(&motor_chassis[2]);
      detect_hook(CHASSIS_MOTOR3_TOE);
      break;
    }
    case CAN_3508_M4_ID:
    {
      get_motor_measure(&motor_chassis[3], rx_data);
      // Calc_motor_Angle(&motor_chassis[3]);
      detect_hook(CHASSIS_MOTOR4_TOE);
      break;
    }
    case CAN_YAW_MOTOR_ID:
    {
      get_motor_measure(&motor_gimbal[0], rx_data);
      // Calc_motor_Angle(&motor_gimbal[0]);
      detect_hook(YAW_GIMBAL_MOTOR_TOE);
      break;
    }
    case CAN_PIT_MOTOR_ID:
    {
      get_motor_measure(&motor_gimbal[1], rx_data);
      // Calc_motor_Angle(&motor_gimbal[1]);
      detect_hook(PITCH_GIMBAL_MOTOR_TOE);
      break;
    }
    case CAN_TRIGGER_MOTOR_ID:
    {
      get_motor_measure(&motor_trigger, rx_data);
      Calc_motor_Angle(&motor_trigger);
      detect_hook(TRIGGER_MOTOR_TOE);
      _207_angle[1] = _207_angle[0];
      _207_angle[0] = motor_trigger.ecd * 0.044;
      if (_207_angle[0] - _207_angle[1] > 180)
      {
        round_207--;
      }
      else if (_207_angle[0] - _207_angle[1] < -180)
      {
        round_207++;
      }
      this_angle_207 = round_207 * 360 + _207_angle[0];
      break;
    }
    case 0x211:
    {
      // extern float PowerData[4];
      uint16_t *pPowerData = (uint16_t *)rx_data;
      PowerData[0] = (float)pPowerData[0] / 100.f;
      PowerData[1] = (float)pPowerData[1] / 100.f;
      PowerData[2] = (float)pPowerData[2] / 100.f;
      PowerData[3] = (float)pPowerData[3] / 100.f;
      break;
    }

    default:
    {
      break;
    }
    }
  }
  else if (hcan == &hcan2)
  {
    switch (rx_header.StdId)
    {
    case CAN_LEFT_FRICTION_ID:
    {
      get_motor_measure(&motor_friction[0], rx_data); //读取数据
      detect_hook(LEFT_FRICTION_MOTOR_TOE);           //错误检测
      break;
    }
    case CAN_RIGHT_FRICTION_ID:
    {
      get_motor_measure(&motor_friction[1], rx_data); //读取数据
      detect_hook(RIGHT_FRICTION_MOTOR_TOE);          //错误检测
      break;
    }

    default:
    {
      break;
    }
    }
  }
}

/**
 * @brief 摩擦轮控制量发送函数 can2
 *
 * @param left_friction
 * @param rigit_friction
 * @param rev
 */
void CAN_cmd_friction(int16_t left_friction, int16_t rigit_friction)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_FRICTION_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = (left_friction >> 8);
  gimbal_can_send_data[1] = left_friction;
  gimbal_can_send_data[2] = (rigit_friction >> 8);
  gimbal_can_send_data[3] = rigit_friction;
  gimbal_can_send_data[4] = 0;
  gimbal_can_send_data[5] = 0;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief 拨弹电机控制量发送函数 can1
 *
 * @param shoot
 */
void CAN_cmd_shoot(int16_t shoot)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = 0x1FF;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = 0;
  gimbal_can_send_data[1] = 0;
  gimbal_can_send_data[2] = 0;
  gimbal_can_send_data[3] = 0;
  gimbal_can_send_data[4] = (shoot >> 8);
  gimbal_can_send_data[5] = shoot;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief          发送电机控制电流(0x205,0x206,0x207,0x208) can1
 * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
 * @retval         none
 */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = (yaw >> 8);
  gimbal_can_send_data[1] = yaw;
  gimbal_can_send_data[2] = (pitch >> 8);
  gimbal_can_send_data[3] = pitch;
  gimbal_can_send_data[4] = 0;
  gimbal_can_send_data[5] = 0;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204) can1
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void Can_Send_Msg2(int16_t current_1, int16_t current_2, int16_t current_3, int16_t current_4)
{
  uint32_t send_mail_box;
  tx_message.StdId = 0x102;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  tx_message_data[0] = (current_1 >> 8);
  tx_message_data[1] = current_1;
  tx_message_data[2] = (current_2 >> 8);
  tx_message_data[3] = current_2;
  tx_message_data[4] = (current_3 >> 8);
  tx_message_data[5] = current_3;
  tx_message_data[6] = (current_4 >> 8);
  tx_message_data[7] = current_4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &tx_message, tx_message_data, &send_mail_box);
}
/**
 * @brief          返回yaw 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_gimbal[0];
}

/**
 * @brief          返回pitch 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &motor_gimbal[1];
}

/**
 * @brief          返回拨弹电机 2006电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
  return &motor_trigger;
}

const motor_measure_t *get_left_friction_motor_measure_point(void)
{
  return &motor_friction[0];
}

const motor_measure_t *get_right_friction_motor_measure_point(void)
{
  return &motor_friction[1];
}

/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i & 0x03)];
}

/**
 * @brief     发送超级电容设定功率
 * @param Power   3000~13000 对应30W~130W
 * 10Hz
 */
void super_cap_send_power(uint16_t Power)
{
  uint32_t send_mail_box;
  cap_tx_message.StdId = 0x210;
  cap_tx_message.IDE = CAN_ID_STD;
  cap_tx_message.RTR = CAN_RTR_DATA;
  cap_tx_message.DLC = 0x08;
  cap_can_send_data[0] = Power >> 8;
  cap_can_send_data[1] = Power;
  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &cap_tx_message, cap_can_send_data, &send_mail_box);
}

