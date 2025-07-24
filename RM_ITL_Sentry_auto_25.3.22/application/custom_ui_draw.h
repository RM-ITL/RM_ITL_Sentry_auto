#ifndef CUSTOM_UI_DRAW_H
#define CUSTOM_UI_DRAW_H

#include "main.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "string.h"
#include "cmsis_os.h"

#ifndef PI
#define PI					3.14159265358979f
#endif

/*长度定义*/
#define MAX_SIZE 128
#define frameheader_len 5
#define cmd_len 2
#define crc_len 2

/**
 * @brief 步兵机器人ID
 * @param SENDER_ID   R3 3        蓝方 103
 * @param RECEIVER_ID 红方 0x0103   蓝方 0x0167
 */
#define SENDER_ID 103
#define RECEIVER_ID 0x0167

typedef struct
{
  uint16_t gx;
  uint16_t gy;
  float angle;
  float r;
} tank_ui_t;

typedef struct
{
  uint16_t aimy;
  float angle;
} aim_dynamic_t;

typedef struct
{
  float power_ratio;
  uint16_t jindu;
  uint16_t voltage;
} super_capacity_t;

#if 0 //帧头
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;
#endif

/**
  *@brief 交互数据段头信息
  *@param data_cmd_id:   内容id(删除、绘制x个图形，绘制字符)
          this parammeter can be any combination of the folowing values:
                0x0200~0x02FF   己方机器人间通信
                0x0100          客户端删除图形
                0x0101          客户端绘制一个图形
                0x0102          客户端绘制二个图形
                0x0103          客户端绘制五个图形
                0x0104          客户端绘制七个图形
                0x0110          客户端绘制字符图形
  *@param sender_ID:     发送者id，需校验
          this parammeter can be any combination of the folowing values:
                1，英雄(红)；2，工程(红)；3/4/5，步兵(红)；6，空中(红)；7，哨兵(红)；9，雷达站（红）
                101，英雄(蓝)；102，工程(蓝)；103/104/105，步兵(蓝)；106，空中(蓝)；107，哨兵(蓝)； 109，雷达站（蓝）
  *@param receiver_ID:   接收者id，需校验
          this parammeter can be any combination of the folowing values:
                客户端 ID：0x0101 为英雄操作手客户端(红)；0x0102，工程操作手客户端((红)；0x0103/0x0104/0x0105，
                步兵操作手客户端(红)；0x0106，空中操作手客户端((红)； 0x0165，英雄操作手客户端(蓝)；0x0166，工
                程操作手客户端(蓝)；0x0167/0x0168/0x0169，步兵操作手客户端步兵(蓝)；0x016A，空中操作手客户端 (蓝)。
  */
typedef __packed struct
{
  uint16_t data_cmd_id;
  uint16_t sender_ID;
  uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

/**
 * @brief 图形数据结构体
 *        有3个u8，3个u32，对u32位进行操作，定义成位段，可直接赋值，不必移位
 * @param operate_type  图形操作    bit0-2:   0 空操作，1 增加，2 修改，3 删除
 * @param graphic_tpye  图形类别    bit3-5：  0：直线；1：矩形；2：整圆；3：椭圆；4：圆弧；5：浮点数；6：整型数；7：字符；
 * @param layer         图层数      bit6-9:   0~9
 * @param color         颜色        bit10-13  0：红蓝主色；1：黄色；2：绿色；3：橙色；
 *                                            4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
 * @param start_angle   起始角度    bit14-22   0~360
 * @param end_angle     终止角度    bit23-32   0~360
 * --------------------------------------------------
 * @param width         线宽        bit0-9
 * @param start_x                   bit10-20
 * @param start_y                   bit21-31
 * ----------------------------------------------------
 * @param radius        字体大小或半径 bit0-9
 * @param end_x                     bit10-20
 * @param end_y                     bit21-31
 */
typedef __packed struct
{
  uint8_t graphic_name[3];
  uint32_t operate_tpye : 3;
  uint32_t graphic_tpye : 3;
  uint32_t layer : 4;
  uint32_t color : 4;
  uint32_t start_angle : 9;
  uint32_t end_angle : 9;
  uint32_t width : 10;
  uint32_t start_x : 11;
  uint32_t start_y : 11;
  uint32_t radius : 10;
  uint32_t end_x : 11;
  uint32_t end_y : 11;
} graphic_data_struct_t;

/**
 * @brief 绘制浮点数和整数用
 * @param data: 最后32位是数据位
 *
 */
typedef __packed struct
{
  uint8_t graphic_name[3];
  uint32_t operate_tpye : 3;
  uint32_t graphic_tpye : 3;
  uint32_t layer : 4;
  uint32_t color : 4;
  uint32_t start_angle : 9;
  uint32_t end_angle : 9;
  uint32_t width : 10;
  uint32_t start_x : 11;
  uint32_t start_y : 11;
  uint32_t data : 32;

} Data_graphic_data_struct_t;

/**
 * @brief 客户端绘制一个图形
 */
typedef __packed struct
{
  ext_student_interactive_header_data_t header_data_t;
  graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;
/**
 * @brief 绘制2个图形
 * @brief cmd : 0x0102
 */
typedef __packed struct
{
  ext_student_interactive_header_data_t header_data_t;
  graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

/**
 * @brief 绘制5个图形
 *
 */
typedef __packed struct
{
  ext_student_interactive_header_data_t header_data_t;
  graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

/*绘制数*/
typedef __packed struct
{
  ext_student_interactive_header_data_t header_data_t;
  Data_graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t_data;

/*********客户端绘制7个图形*****/
typedef __packed struct
{
  ext_student_interactive_header_data_t header_data_t;
  graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

//绘制字符
typedef __packed struct
{
  ext_student_interactive_header_data_t header_data_t;
  graphic_data_struct_t grapic_data_struct;
  uint8_t data[30];
} ext_client_custom_character_t;

/**
 * @brief 删除图形
 * @param operate_type:  0 空操作 ,1 删除图层 ，2 删除所有
 * @param layer ： 图层 0~9
 */
typedef __packed struct
{
  ext_student_interactive_header_data_t header_data_t;
  uint8_t operate_tpye;
  uint8_t layer;
} ext_client_custom_graphic_delete_t;

//-----------------------------------------------------------------------------------------------//

void referee_data_pack_handle(uint8_t sof, uint16_t cmd_id, uint8_t *p_data, uint16_t len);

/*sender_id 1  receiver id 0x0101*/
void test_7_graphic(uint16_t sender_id, uint16_t receiver_id);
void Send_Speed(uint16_t sender_id, uint16_t receiver_id);

/***********删除图形*************/
void clear_all_layer(uint16_t sender_id, uint16_t receiver_id);
void clear_single_layer(uint16_t sender_id, uint16_t receiver_id, uint8_t layer);

/***********绘制数据**************/

void draw_single_float(uint16_t sender_id, uint16_t receiver_id, float data);
void draw_single_integral(uint16_t sender_id, uint16_t receiver_id, uint16_t start_x, uint16_t start_y, int32_t data, uint16_t op_type);

/**********绘制图形*****************/
void draw_single_line(uint16_t sender_id, uint16_t receiver_id, uint16_t start_x,
                      uint16_t start_y, uint16_t end_x, uint16_t end_y,
                      uint8_t color, uint8_t layer, uint8_t width, uint16_t op_type);

void draw_single_circle(uint16_t sender_id, uint16_t receiver_id, uint16_t start_x, uint16_t start_y,
                        uint16_t radius, uint8_t color, uint8_t layer, uint8_t width, uint16_t op_type);

void draw_single_ellipse(uint16_t sender_id, uint16_t receiver_id, uint16_t start_x, uint16_t start_y,
                         uint16_t end_x, uint16_t end_y, uint8_t color, uint8_t layer, uint8_t width);

void draw_single_arc(uint16_t sender_id, uint16_t receiver_id, uint16_t start_angle, uint16_t end_angle,
                     uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y,
                     uint8_t color, uint8_t layer, uint8_t width);
/**
 * @brief test for single graphic
 * @param graphic name start with 1
 */
void draw_single_line_test(uint16_t sender_id, uint16_t receiver_id,uint16_t op_type);
void draw_single_tangle_test(uint16_t sender_id, uint16_t receiver_id,uint16_t op_type);








/*******************动态ui*********/
void draw_dynamic_ui(uint16_t sender_id, uint16_t receiver_id, uint16_t origin_x,
                     uint16_t origin_y, uint16_t dx, uint16_t dy, uint16_t ddx, uint16_t ddy, uint16_t op_type);

/****************Application*******************/
void draw_init_all(uint16_t sender_id, uint16_t receiver_id);
void draw_chassis_mode(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type);       // $3
void draw_chassis_mode_state(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type); // $1
void draw_gimbal_mode(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type);
void draw_aim_assitant1(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type, uint16_t layer); // $2
void draw_aim_assitant2(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type, uint16_t layer); // $7
void draw_chassis_run_bordor(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type);           // $4
void draw_gimbal_relative_angle_tangle(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type);// $6
void cal_draw_gimbal_relative_angle_tangle(tank_ui_t *tank); //计算角度
void draw_state_of_top(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type); //$8
void draw_state_of_top_circle(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type); //$9
void draw_capacity_number(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type); // $10
void draw_capacity_char(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type); //
void draw_capacity_tangle(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type);
void cal_draw_pitch_aim(aim_dynamic_t *aim_dynamic);
void audio_enemy_godlike(uint16_t sender_id, uint16_t receiver_id);
void cal_capacity(super_capacity_t * super_capacity);
void draw_get_robot_id(void);
void draw_servo_bull(uint16_t sender_id, uint16_t receiver_id,uint16_t op_type);
void draw_top_speed(uint16_t sender_id, uint16_t receiver_id,uint16_t op_type);
#endif
