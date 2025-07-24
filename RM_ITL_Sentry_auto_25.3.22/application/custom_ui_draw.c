#include "custom_ui_draw.h"
#include "bsp_usart.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "shoot_task.h"
#include "referee.h"

/*外部调用*/
extern chassis_move_t chassis_move;
extern ext_game_robot_state_t robot_state;
extern int servo_flag;
extern int n;
super_capacity_t super_capacity;
uint16_t send_id;
uint16_t receive_id;
// extern gimbal_control_t gimbal_control;

uint32_t FONT_SIZE = 16;

/*location*/
#define WIDTH 1
#define SCREEN_WIDTH 1080
#define SCREEN_LENGTH 1920
#define CENTER_X 1300
#define CENTER_Y 820
#define VERTICAL 50
#define HORIZONTAL 90

#define chassis_mode_x CENTER_X
#define chassis_mode_y CENTER_Y
#define CHASSIS_RUN_BORDER_START_X 550
#define CHASSIS_RUN_BORDER_START_Y 60
#define CHASSIS_RUN_BORDER_END_X 440
#define CHASSIS_RUN_BORDER_END_Y 260
#define gimbal_angle_center_x 1600
#define gimbal_angle_center_y 500
#define AIM_CENTER_X 960
#define AIM_CENTER_Y 510
#define AIM_CENTER_Y_UP 580
#define AIM_VERTICAL_LONG_Y 80
#define AIM_VERTICAL_SHORT_Y 20
#define AIM_HORIAZONTAL_SHORT_X 15
#define AIM_HORIAZONTAL_LONG_X 90
#define LEFT_STATE_AREA_X 100
#define LEFT_STATE_AREA_Y 700
#define LEFT_STATE_CIRCLE_DX 270
#define LEFT_STATE_DY 60
#define CAP_START_X 1160
#define CAP_START_Y 700
#define CAP_TANGLE_START_X (960 - 150)
#define CAP_TANGLE_START_Y 100
#define CAP_TANGLE_END_X (960 + 150)
#define CAP_TANGLE_END_Y (CAP_TANGLE_START_Y + 30)
/*********private variable*************/
tank_ui_t tank;
aim_dynamic_t aim_dynamic;

/*打包发送*/
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart6;
uint8_t seq = 0;

void referee_data_pack_handle(uint8_t sof, uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
    uint8_t tx_buff[MAX_SIZE];
    uint16_t frame_length = frameheader_len + cmd_len + crc_len + len;

    memset(tx_buff, 0, MAX_SIZE); //存储数据的数组清零

    /*****帧头打包*****/
    tx_buff[0] = sof;                                  //数据帧起始字节
    memcpy(&tx_buff[1], (uint8_t *)&len, sizeof(len)); //数据帧中data的长度
    tx_buff[3] = seq;                                  //包序号
    append_CRC8_check_sum(tx_buff, frameheader_len);   //帧头校验CRC8

    /*****命令码打包*****/
    memcpy(&tx_buff[frameheader_len], (uint8_t *)&cmd_id, cmd_len);

    /*****数据打包*****/
    memcpy(&tx_buff[frameheader_len + cmd_len], p_data, len);
    append_CRC16_check_sum(tx_buff, frame_length); //一帧数据校验CRC16
    if (seq == 0xff)
        seq = 0;
    else
        seq++;

    // HAL_UART_Transmit(&huart6,tx_buff,frame_length,13);
    usart6_tx_dma_enable(tx_buff, frame_length);
}

/*sender_id 1  receiver id 0x0101*/
#if 1
void test_7_graphic(uint16_t sender_id, uint16_t receiver_id)
{
    ext_client_custom_graphic_seven_t test_draw;

    test_draw.header_data_t.data_cmd_id = 0x0104;
    test_draw.header_data_t.sender_ID = sender_id;
    test_draw.header_data_t.receiver_ID = receiver_id;

    test_draw.grapic_data_struct[0].graphic_name[0] = 0;
    test_draw.grapic_data_struct[0].graphic_name[1] = 97;
    test_draw.grapic_data_struct[0].graphic_name[2] = 0;

    test_draw.grapic_data_struct[0].operate_tpye = 1;
    test_draw.grapic_data_struct[0].graphic_tpye = 0;
    test_draw.grapic_data_struct[0].layer = 0;
    test_draw.grapic_data_struct[0].color = 8;
    test_draw.grapic_data_struct[0].start_angle = 0;
    test_draw.grapic_data_struct[0].end_angle = 0;
    test_draw.grapic_data_struct[0].width = 2;
    test_draw.grapic_data_struct[0].start_x = SCREEN_LENGTH / 2 - 250;
    test_draw.grapic_data_struct[0].start_y = SCREEN_WIDTH / 2 - 75;
    test_draw.grapic_data_struct[0].end_x = SCREEN_LENGTH / 2 + 250;
    test_draw.grapic_data_struct[0].end_y = SCREEN_WIDTH / 2 - 75;
    test_draw.grapic_data_struct[0].radius = 0;

    test_draw.grapic_data_struct[1].graphic_name[0] = 0;
    test_draw.grapic_data_struct[1].graphic_name[1] = 96;
    test_draw.grapic_data_struct[1].graphic_name[2] = 0; //图形名
    //上面三个字节代表的是图形名，用于图形索引，可自行定义
    test_draw.grapic_data_struct[1].operate_tpye = 1; //图形操作，0：空操作；1：增加；2：修改；3：删除；
    test_draw.grapic_data_struct[1].graphic_tpye = 0; //图形类型，0为直线，其他的查看用户手册
    test_draw.grapic_data_struct[1].layer = 0;        //图层数
    test_draw.grapic_data_struct[1].color = 8;        //颜色
    test_draw.grapic_data_struct[1].start_angle = 0;
    test_draw.grapic_data_struct[1].end_angle = 0;
    test_draw.grapic_data_struct[1].width = 2;
    test_draw.grapic_data_struct[1].start_x = SCREEN_LENGTH / 2 - 200;
    test_draw.grapic_data_struct[1].start_y = SCREEN_WIDTH / 2 - 125;
    test_draw.grapic_data_struct[1].end_x = SCREEN_LENGTH / 2 + 200;
    test_draw.grapic_data_struct[1].end_y = SCREEN_WIDTH / 2 - 125;
    test_draw.grapic_data_struct[1].radius = 0;

    test_draw.grapic_data_struct[2].graphic_name[0] = 0;
    test_draw.grapic_data_struct[2].graphic_name[1] = 95;
    test_draw.grapic_data_struct[2].graphic_name[2] = 0; //图形名
    //上面三个字节代表的是图形名，用于图形索引，可自行定义
    test_draw.grapic_data_struct[2].operate_tpye = 1; //图形操作，0：空操作；1：增加；2：修改
    test_draw.grapic_data_struct[2].graphic_tpye = 0; //图形类型，0为直线，其他的查看用户手册
    test_draw.grapic_data_struct[2].layer = 0;        //图层数
    test_draw.grapic_data_struct[2].color = 8;        //颜色
    test_draw.grapic_data_struct[2].start_angle = 0;
    test_draw.grapic_data_struct[2].end_angle = 0;
    test_draw.grapic_data_struct[2].width = 2;
    test_draw.grapic_data_struct[2].start_x = SCREEN_LENGTH / 2 - 150;
    test_draw.grapic_data_struct[2].start_y = SCREEN_WIDTH / 2 - 175;
    test_draw.grapic_data_struct[2].end_x = SCREEN_LENGTH / 2 + 150;
    test_draw.grapic_data_struct[2].end_y = SCREEN_WIDTH / 2 - 175;
    test_draw.grapic_data_struct[2].radius = 0;

    test_draw.grapic_data_struct[3].graphic_name[0] = 0;
    test_draw.grapic_data_struct[3].graphic_name[1] = 94;
    test_draw.grapic_data_struct[3].graphic_name[2] = 0; //图形名
    //上面三个字节代表的是图形名于图形索引，可自行定义
    test_draw.grapic_data_struct[3].operate_tpye = 1; //图形操作，0：空操作；1：增加；2：修改
    test_draw.grapic_data_struct[3].graphic_tpye = 0; //图形类型，0为直线，其他的查看用户手册
    test_draw.grapic_data_struct[3].layer = 0;        //图层数
    test_draw.grapic_data_struct[3].color = 8;        //颜色
    test_draw.grapic_data_struct[3].start_angle = 0;
    test_draw.grapic_data_struct[3].end_angle = 0;
    test_draw.grapic_data_struct[3].width = 2;
    test_draw.grapic_data_struct[3].start_x = SCREEN_LENGTH / 2 - 100;
    test_draw.grapic_data_struct[3].start_y = SCREEN_WIDTH / 2 - 225;
    test_draw.grapic_data_struct[3].end_x = SCREEN_LENGTH / 2 + 100;
    test_draw.grapic_data_struct[3].end_y = SCREEN_WIDTH / 2 - 225;
    test_draw.grapic_data_struct[3].radius = 0;

    test_draw.grapic_data_struct[4].graphic_name[0] = 0;
    test_draw.grapic_data_struct[4].graphic_name[1] = 93;
    test_draw.grapic_data_struct[4].graphic_name[2] = 0; //图形名
    //上面三个字节代表的是图形名4于图形索引，可自行定义
    test_draw.grapic_data_struct[4].operate_tpye = 1; //图形操作，0：空操作；1：增加；2：修改
    test_draw.grapic_data_struct[4].graphic_tpye = 0; //图形类型，0为直线，其他的查看用户手册
    test_draw.grapic_data_struct[4].layer = 0;        //图层数
    test_draw.grapic_data_struct[4].color = 8;        //颜色
    test_draw.grapic_data_struct[4].start_angle = 0;
    test_draw.grapic_data_struct[4].end_angle = 0;
    test_draw.grapic_data_struct[4].width = 2;
    test_draw.grapic_data_struct[4].start_x = SCREEN_LENGTH / 2;
    test_draw.grapic_data_struct[4].start_y = SCREEN_WIDTH / 2;
    test_draw.grapic_data_struct[4].end_x = SCREEN_LENGTH / 2;
    test_draw.grapic_data_struct[4].end_y = SCREEN_WIDTH / 2 - 250;
    test_draw.grapic_data_struct[4].radius = 0;

    test_draw.grapic_data_struct[5].graphic_name[0] = 0;
    test_draw.grapic_data_struct[5].graphic_name[1] = 92;
    test_draw.grapic_data_struct[5].graphic_name[2] = 0; //图形名
    //上面三个字节代表的是图形6用5于图形索引，可自行定义
    test_draw.grapic_data_struct[5].operate_tpye = 0; //图形操作，0：空操作；1：增加；2：修改
    test_draw.grapic_data_struct[5].graphic_tpye = 0; //图形类型，0为直线，其他的查看用户手册
    test_draw.grapic_data_struct[5].layer = 0;        //图层数
    test_draw.grapic_data_struct[5].color = 8;        //颜色
    test_draw.grapic_data_struct[5].start_angle = 0;
    test_draw.grapic_data_struct[5].end_angle = 0;
    test_draw.grapic_data_struct[5].width = 2;
    test_draw.grapic_data_struct[5].start_x = SCREEN_LENGTH / 2 + 280;
    test_draw.grapic_data_struct[5].start_y = SCREEN_WIDTH / 2;
    test_draw.grapic_data_struct[5].end_x = SCREEN_LENGTH / 2 + 880;
    test_draw.grapic_data_struct[5].end_y = SCREEN_WIDTH / 2;
    test_draw.grapic_data_struct[5].radius = 0;

    //圆弧
    test_draw.grapic_data_struct[6].graphic_name[0] = 0;
    test_draw.grapic_data_struct[6].graphic_name[1] = 91;
    test_draw.grapic_data_struct[6].graphic_name[2] = 0; //图形名
    //上面三个字节代表的是图形名，用于图形索引，可自行定义
    test_draw.grapic_data_struct[6].operate_tpye = 0; //图形操作，0：空操作；1：增加；2：修改
    test_draw.grapic_data_struct[6].graphic_tpye = 4; //图形类型，0为直线，其他的查看用户手册
    test_draw.grapic_data_struct[6].layer = 0;        //图层数
    test_draw.grapic_data_struct[6].color = 8;        //颜色
    test_draw.grapic_data_struct[6].start_angle = 135;
    test_draw.grapic_data_struct[6].end_angle = 180;
    test_draw.grapic_data_struct[6].width = 2;
    test_draw.grapic_data_struct[6].start_x = SCREEN_LENGTH / 2 + 280;
    test_draw.grapic_data_struct[6].start_y = SCREEN_WIDTH / 2 - 125;
    test_draw.grapic_data_struct[6].end_x = 300;
    test_draw.grapic_data_struct[6].end_y = 200;
    test_draw.grapic_data_struct[6].radius = 0;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&test_draw, sizeof(test_draw));
}
#endif

void Send_Speed(uint16_t sender_id, uint16_t receiver_id)
{
    ext_client_custom_character_t draw_init;
    draw_init.header_data_t.data_cmd_id = 0x0110;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct.graphic_name[0] = 1;
    draw_init.grapic_data_struct.graphic_name[1] = 0;
    draw_init.grapic_data_struct.graphic_name[2] = 0;
    draw_init.grapic_data_struct.operate_tpye = 1;
    draw_init.grapic_data_struct.graphic_tpye = 7;
    draw_init.grapic_data_struct.layer = 2;
    draw_init.grapic_data_struct.color = 8;
    draw_init.grapic_data_struct.start_angle = 15; //字符大小
    draw_init.grapic_data_struct.end_angle = 6;    //字符长度
    draw_init.grapic_data_struct.width = 3;
    draw_init.grapic_data_struct.start_x = 1400;
    draw_init.grapic_data_struct.start_y = 820;
    draw_init.grapic_data_struct.end_x = SCREEN_LENGTH / 2 + 250;
    draw_init.grapic_data_struct.end_y = SCREEN_WIDTH / 2 - 75;
    draw_init.grapic_data_struct.radius = 0;
    draw_init.data[0] = 'S';
    draw_init.data[1] = 'P';
    draw_init.data[2] = 'E';
    draw_init.data[3] = 'E';
    draw_init.data[4] = 'D';
    draw_init.data[5] = 00;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void clear_all_layer(uint16_t sender_id, uint16_t receiver_id)
{

    ext_client_custom_graphic_delete_t init_struct;

    init_struct.header_data_t.data_cmd_id = 0x0100;
    init_struct.header_data_t.receiver_ID = receiver_id;
    init_struct.header_data_t.sender_ID = sender_id;

    init_struct.operate_tpye = 2;
    init_struct.layer = 0;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&init_struct, sizeof(init_struct));
}

void clear_single_layer(uint16_t sender_id, uint16_t receiver_id, uint8_t layer)
{
    ext_client_custom_graphic_delete_t init_struct;

    init_struct.header_data_t.data_cmd_id = 0x0100;
    init_struct.header_data_t.receiver_ID = receiver_id;
    init_struct.header_data_t.sender_ID = sender_id;

    init_struct.operate_tpye = 1;
    init_struct.layer = layer;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&init_struct, sizeof(init_struct));
}

void draw_single_float(uint16_t sender_id, uint16_t receiver_id, float data)
{
    ext_client_custom_graphic_single_t_data init_struct;

    init_struct.header_data_t.data_cmd_id = 0x0101;
    init_struct.header_data_t.sender_ID = sender_id;
    init_struct.header_data_t.receiver_ID = receiver_id;

    init_struct.grapic_data_struct.graphic_name[0] = 0;
    init_struct.grapic_data_struct.graphic_name[1] = 0;
    init_struct.grapic_data_struct.graphic_name[2] = 0;
    init_struct.grapic_data_struct.operate_tpye = 1;
    init_struct.grapic_data_struct.graphic_tpye = 7;
    init_struct.grapic_data_struct.layer = 1;
    init_struct.grapic_data_struct.color = 3;
    init_struct.grapic_data_struct.start_x = 1920 / 2;
    init_struct.grapic_data_struct.start_y = 1080 / 2 - 300;

    init_struct.grapic_data_struct.start_angle = 15;
    init_struct.grapic_data_struct.end_angle = 2;
    init_struct.grapic_data_struct.width = 3;

    data *= 1000;
    init_struct.grapic_data_struct.data = (int32_t)data;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&init_struct, sizeof(init_struct));
}

void draw_single_integral(uint16_t sender_id, uint16_t receiver_id, uint16_t start_x, uint16_t start_y, int32_t data, uint16_t op_type)
{
    ext_client_custom_graphic_single_t_data init_struct;

    init_struct.header_data_t.data_cmd_id = 0x0101;
    init_struct.header_data_t.sender_ID = sender_id;
    init_struct.header_data_t.receiver_ID = receiver_id;

    init_struct.grapic_data_struct.graphic_name[0] = 0;
    init_struct.grapic_data_struct.graphic_name[1] = 0;
    init_struct.grapic_data_struct.graphic_name[2] = 1;
    init_struct.grapic_data_struct.operate_tpye = op_type;
    init_struct.grapic_data_struct.graphic_tpye = 6;
    init_struct.grapic_data_struct.layer = 1;
    init_struct.grapic_data_struct.color = 4;
    init_struct.grapic_data_struct.start_x = start_x;
    init_struct.grapic_data_struct.start_y = start_y;

    init_struct.grapic_data_struct.start_angle = 15;
    init_struct.grapic_data_struct.end_angle = 0;
    init_struct.grapic_data_struct.width = 3;

    init_struct.grapic_data_struct.data = data;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&init_struct, sizeof(init_struct));
}

void draw_single_line(uint16_t sender_id, uint16_t receiver_id, uint16_t start_x, uint16_t start_y,
                      uint16_t end_x, uint16_t end_y, uint8_t color, uint8_t layer, uint8_t width, uint16_t op_type)
{
    ext_client_custom_graphic_single_t init_struct;

    init_struct.header_data_t.data_cmd_id = 0x0101;
    init_struct.header_data_t.sender_ID = sender_id;
    init_struct.header_data_t.receiver_ID = receiver_id;

    init_struct.grapic_data_struct.graphic_name[0] = 2;
    init_struct.grapic_data_struct.graphic_name[1] = 2;
    init_struct.grapic_data_struct.graphic_name[2] = 0;
    init_struct.grapic_data_struct.operate_tpye = 1;
    init_struct.grapic_data_struct.graphic_tpye = 0;
    init_struct.grapic_data_struct.layer = layer;
    init_struct.grapic_data_struct.color = color;
    init_struct.grapic_data_struct.start_x = start_x;
    init_struct.grapic_data_struct.start_y = start_y;
    init_struct.grapic_data_struct.end_x = end_x;
    init_struct.grapic_data_struct.end_y = end_y;
    init_struct.grapic_data_struct.start_angle = 0;
    init_struct.grapic_data_struct.end_angle = 0;
    init_struct.grapic_data_struct.width = width;
    init_struct.grapic_data_struct.radius = 0;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&init_struct, sizeof(init_struct));
}

void draw_single_circle(uint16_t sender_id, uint16_t receiver_id, uint16_t start_x, uint16_t start_y,
                        uint16_t radius, uint8_t color, uint8_t layer, uint8_t width, uint16_t op_type)
{

    ext_client_custom_graphic_single_t init_struct;

    init_struct.header_data_t.data_cmd_id = 0x0101;
    init_struct.header_data_t.sender_ID = sender_id;
    init_struct.header_data_t.receiver_ID = receiver_id;

    init_struct.grapic_data_struct.graphic_name[0] = 4;
    init_struct.grapic_data_struct.graphic_name[1] = 4;
    init_struct.grapic_data_struct.graphic_name[2] = 0;
    init_struct.grapic_data_struct.operate_tpye = op_type;
    init_struct.grapic_data_struct.graphic_tpye = 2;
    init_struct.grapic_data_struct.layer = layer;
    init_struct.grapic_data_struct.color = color;
    init_struct.grapic_data_struct.start_x = start_x;
    init_struct.grapic_data_struct.start_y = start_y;
    init_struct.grapic_data_struct.width = width;
    init_struct.grapic_data_struct.radius = radius;
    init_struct.grapic_data_struct.start_angle = 0;
    init_struct.grapic_data_struct.end_angle = 0;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&init_struct, sizeof(init_struct));
}

void draw_single_ellipse(uint16_t sender_id, uint16_t receiver_id, uint16_t start_x, uint16_t start_y,
                         uint16_t end_x, uint16_t end_y, uint8_t color, uint8_t layer, uint8_t width)
{
    ext_client_custom_graphic_single_t init_struct;

    init_struct.header_data_t.data_cmd_id = 0x0101;
    init_struct.header_data_t.receiver_ID = receiver_id;
    init_struct.header_data_t.sender_ID = sender_id;

    init_struct.grapic_data_struct.graphic_name[0] = 0;
    init_struct.grapic_data_struct.graphic_name[1] = 0;
    init_struct.grapic_data_struct.graphic_name[2] = 4;
    init_struct.grapic_data_struct.operate_tpye = 1;
    init_struct.grapic_data_struct.graphic_tpye = 3;
    init_struct.grapic_data_struct.start_x = start_x;
    init_struct.grapic_data_struct.start_y = start_y;
    init_struct.grapic_data_struct.end_x = end_x;
    init_struct.grapic_data_struct.end_y = end_y;
    init_struct.grapic_data_struct.color = color;
    init_struct.grapic_data_struct.layer = layer;
    init_struct.grapic_data_struct.width = width;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&init_struct, sizeof(init_struct));
}

void draw_single_arc(uint16_t sender_id, uint16_t receiver_id, uint16_t start_angle, uint16_t end_angle,
                     uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y,
                     uint8_t color, uint8_t layer, uint8_t width)
{

    ext_client_custom_graphic_single_t init_struct;

    init_struct.header_data_t.data_cmd_id = 0x0101;
    init_struct.header_data_t.receiver_ID = receiver_id;
    init_struct.header_data_t.sender_ID = sender_id;

    init_struct.grapic_data_struct.graphic_name[0] = 0;
    init_struct.grapic_data_struct.graphic_name[1] = 0;
    init_struct.grapic_data_struct.graphic_name[2] = 5;
    init_struct.grapic_data_struct.operate_tpye = 1;
    init_struct.grapic_data_struct.graphic_tpye = 4;
    init_struct.grapic_data_struct.start_angle = start_angle;
    init_struct.grapic_data_struct.end_angle = end_angle;
    init_struct.grapic_data_struct.start_x = start_x;
    init_struct.grapic_data_struct.start_y = start_y;
    init_struct.grapic_data_struct.end_x = end_x;
    init_struct.grapic_data_struct.end_y = end_y;
    init_struct.grapic_data_struct.color = color;
    init_struct.grapic_data_struct.width = width;
    init_struct.grapic_data_struct.layer = layer;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&init_struct, sizeof(init_struct));
}

void draw_dynamic_ui(uint16_t sender_id, uint16_t receiver_id, uint16_t origin_x,
                     uint16_t origin_y, uint16_t dx, uint16_t dy, uint16_t ddx, uint16_t ddy, uint16_t op_type)
{

    ext_client_custom_graphic_seven_t init_struct;

    init_struct.header_data_t.data_cmd_id = 0x0104;
    init_struct.header_data_t.sender_ID = sender_id;
    init_struct.header_data_t.receiver_ID = receiver_id;

    init_struct.grapic_data_struct[0].graphic_name[0] = 0;
    init_struct.grapic_data_struct[0].graphic_name[1] = 0;
    init_struct.grapic_data_struct[0].graphic_name[2] = 2;
    init_struct.grapic_data_struct[0].operate_tpye = op_type;
    init_struct.grapic_data_struct[0].graphic_tpye = 0;
    init_struct.grapic_data_struct[0].start_x = origin_x;
    init_struct.grapic_data_struct[0].start_y = origin_y;
    init_struct.grapic_data_struct[0].end_x = origin_x - dx; // left
    init_struct.grapic_data_struct[0].end_y = origin_y + dy;
    init_struct.grapic_data_struct[0].width = 3;
    init_struct.grapic_data_struct[0].color = 4;
    init_struct.grapic_data_struct[0].layer = 8;
    init_struct.grapic_data_struct[0].start_angle = 0;
    init_struct.grapic_data_struct[0].end_angle = 0;
    init_struct.grapic_data_struct[0].radius = 0;

    init_struct.grapic_data_struct[1].graphic_name[0] = 0;
    init_struct.grapic_data_struct[1].graphic_name[1] = 0;
    init_struct.grapic_data_struct[1].graphic_name[2] = 3;
    init_struct.grapic_data_struct[1].operate_tpye = op_type;
    init_struct.grapic_data_struct[1].graphic_tpye = 0;
    init_struct.grapic_data_struct[1].start_x = origin_x;
    init_struct.grapic_data_struct[1].start_y = origin_y;
    init_struct.grapic_data_struct[1].end_x = origin_x + dx; // right
    init_struct.grapic_data_struct[1].end_y = origin_y + dy;
    init_struct.grapic_data_struct[1].width = 3;
    init_struct.grapic_data_struct[1].color = 4;
    init_struct.grapic_data_struct[1].layer = 8;
    init_struct.grapic_data_struct[1].start_angle = 0;
    init_struct.grapic_data_struct[1].end_angle = 0;
    init_struct.grapic_data_struct[1].radius = 0;

    init_struct.grapic_data_struct[2].graphic_name[0] = 0;
    init_struct.grapic_data_struct[2].graphic_name[1] = 0;
    init_struct.grapic_data_struct[2].graphic_name[2] = 4;
    init_struct.grapic_data_struct[2].operate_tpye = op_type;
    init_struct.grapic_data_struct[2].graphic_tpye = 0;
    init_struct.grapic_data_struct[2].start_x = origin_x - dx;
    init_struct.grapic_data_struct[2].start_y = origin_y + dy;
    init_struct.grapic_data_struct[2].end_x = origin_x - dx + ddx; // left
    init_struct.grapic_data_struct[2].end_y = origin_y + dy + ddy;
    init_struct.grapic_data_struct[2].width = 3;
    init_struct.grapic_data_struct[2].color = 4;
    init_struct.grapic_data_struct[2].layer = 8;
    init_struct.grapic_data_struct[2].start_angle = 0;
    init_struct.grapic_data_struct[2].end_angle = 0;
    init_struct.grapic_data_struct[2].radius = 0;

    init_struct.grapic_data_struct[3].graphic_name[0] = 0;
    init_struct.grapic_data_struct[3].graphic_name[1] = 0;
    init_struct.grapic_data_struct[3].graphic_name[2] = 5;
    init_struct.grapic_data_struct[3].operate_tpye = op_type;
    init_struct.grapic_data_struct[3].graphic_tpye = 0;
    init_struct.grapic_data_struct[3].start_x = origin_x + dx;
    init_struct.grapic_data_struct[3].start_y = origin_y + dy;
    init_struct.grapic_data_struct[3].end_x = origin_x + dx - ddx; // right
    init_struct.grapic_data_struct[3].end_y = origin_y + dy + ddy;
    init_struct.grapic_data_struct[3].width = 3;
    init_struct.grapic_data_struct[3].color = 4;
    init_struct.grapic_data_struct[3].layer = 8;
    init_struct.grapic_data_struct[3].start_angle = 0;
    init_struct.grapic_data_struct[3].end_angle = 0;
    init_struct.grapic_data_struct[3].radius = 0;

    init_struct.grapic_data_struct[4].graphic_name[0] = 0;
    init_struct.grapic_data_struct[4].graphic_name[1] = 0;
    init_struct.grapic_data_struct[4].graphic_name[2] = 6;
    init_struct.grapic_data_struct[4].operate_tpye = op_type;
    init_struct.grapic_data_struct[4].graphic_tpye = 0;
    init_struct.grapic_data_struct[4].start_x = origin_x - dx + ddx;
    init_struct.grapic_data_struct[4].start_y = origin_y + dy + ddy;
    init_struct.grapic_data_struct[4].end_x = origin_x + dx - ddx; // left to right
    init_struct.grapic_data_struct[4].end_y = origin_y + dy + ddy;
    init_struct.grapic_data_struct[4].width = 3;
    init_struct.grapic_data_struct[4].color = 2;
    init_struct.grapic_data_struct[4].layer = 8;
    init_struct.grapic_data_struct[4].start_angle = 0;
    init_struct.grapic_data_struct[4].end_angle = 0;
    init_struct.grapic_data_struct[4].radius = 0;

    init_struct.grapic_data_struct[5].graphic_name[0] = 0;
    init_struct.grapic_data_struct[5].graphic_name[1] = 0;
    init_struct.grapic_data_struct[5].graphic_name[2] = 7;
    init_struct.grapic_data_struct[5].operate_tpye = op_type;
    init_struct.grapic_data_struct[5].graphic_tpye = 2;
    init_struct.grapic_data_struct[5].start_x = origin_x - dx + ddx;
    init_struct.grapic_data_struct[5].start_y = origin_y + dy + ddy;
    init_struct.grapic_data_struct[5].end_x = 0; // left node
    init_struct.grapic_data_struct[5].end_y = 0;
    init_struct.grapic_data_struct[5].width = 3;
    init_struct.grapic_data_struct[5].color = 5;
    init_struct.grapic_data_struct[5].layer = 8;
    init_struct.grapic_data_struct[5].start_angle = 0;
    init_struct.grapic_data_struct[5].end_angle = 0;
    init_struct.grapic_data_struct[5].radius = 5;

    init_struct.grapic_data_struct[6].graphic_name[0] = 0;
    init_struct.grapic_data_struct[6].graphic_name[1] = 0;
    init_struct.grapic_data_struct[6].graphic_name[2] = 8;
    init_struct.grapic_data_struct[6].operate_tpye = op_type;
    init_struct.grapic_data_struct[6].graphic_tpye = 2;
    init_struct.grapic_data_struct[6].start_x = origin_x + dx - ddx;
    init_struct.grapic_data_struct[6].start_y = origin_y + dy + ddy;
    init_struct.grapic_data_struct[6].end_x = 0; // left node
    init_struct.grapic_data_struct[6].end_y = 0;
    init_struct.grapic_data_struct[6].width = 3;
    init_struct.grapic_data_struct[6].color = 5;
    init_struct.grapic_data_struct[6].layer = 8;
    init_struct.grapic_data_struct[6].start_angle = 0;
    init_struct.grapic_data_struct[6].end_angle = 0;
    init_struct.grapic_data_struct[6].radius = 5;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&init_struct, sizeof(init_struct));
}

/*********Application************/
void draw_init_all(uint16_t sender_id, uint16_t receiver_id)
{
    clear_all_layer(sender_id, receiver_id);
    osDelay(100);
    draw_aim_assitant2(sender_id, receiver_id, 1, 1);
    osDelay(100);
    draw_chassis_mode(sender_id, receiver_id, 1);
    osDelay(100);
    draw_chassis_mode_state(sender_id, receiver_id, 1);
    osDelay(100);
    draw_aim_assitant1(sender_id, receiver_id, 1, 1);
    osDelay(100);
    draw_chassis_run_bordor(sender_id, receiver_id, 1);
    osDelay(100);
    draw_gimbal_relative_angle_tangle(sender_id, receiver_id, 1);
    osDelay(100);
    draw_state_of_top(sender_id, receiver_id, 1);
    osDelay(200);
    draw_state_of_top_circle(sender_id, receiver_id, 1);
    osDelay(100);
    draw_servo_bull(send_id, receive_id, 1);
    osDelay(100);
    draw_top_speed(send_id, receive_id, 1);
    osDelay(100);
}

void draw_chassis_mode_state(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_character_t draw_init;
    draw_init.header_data_t.data_cmd_id = 0x0110;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct.graphic_name[0] = 0;
    draw_init.grapic_data_struct.graphic_name[1] = 0;
    draw_init.grapic_data_struct.graphic_name[2] = 9;
    draw_init.grapic_data_struct.operate_tpye = op_type;
    draw_init.grapic_data_struct.graphic_tpye = 7;
    draw_init.grapic_data_struct.layer = 1;

    draw_init.grapic_data_struct.start_angle = FONT_SIZE; //字符大小

    draw_init.grapic_data_struct.width = 2;
    draw_init.grapic_data_struct.start_x = chassis_mode_x;
    draw_init.grapic_data_struct.start_y = chassis_mode_y;
    draw_init.grapic_data_struct.end_x = 0;
    draw_init.grapic_data_struct.end_y = 0;
    draw_init.grapic_data_struct.radius = 0;
    memset(draw_init.data, 0, sizeof(draw_init.data));
    switch (chassis_move.chassis_mode)
    {
    case CHASSIS_ZERO_FORCE:
        draw_init.grapic_data_struct.color = 8;
        draw_init.grapic_data_struct.end_angle = 10; //字符长度
        draw_init.data[0] = 'Z';
        draw_init.data[1] = 'E';
        draw_init.data[2] = 'R';
        draw_init.data[3] = 'O';
        draw_init.data[4] = '_';
        draw_init.data[5] = 'F';
        draw_init.data[6] = 'O';
        draw_init.data[7] = 'R';
        draw_init.data[8] = 'C';
        draw_init.data[9] = 'E';
        break;
    case CHASSIS_NO_MOVE:
        draw_init.grapic_data_struct.color = 8;
        draw_init.grapic_data_struct.end_angle = 8; //字符长度
        draw_init.data[0] = 'N';
        draw_init.data[1] = 'O';
        draw_init.data[2] = '_';
        draw_init.data[3] = 'M';
        draw_init.data[4] = 'O';
        draw_init.data[5] = 'V';
        draw_init.data[6] = 'E';

        break;
    case CHASSIS_FOLLOW_GIMBAL:
        draw_init.grapic_data_struct.color = 8;
        draw_init.grapic_data_struct.end_angle = 6; //字符长度
        draw_init.data[0] = 'F';
        draw_init.data[1] = 'O';
        draw_init.data[2] = 'L';
        draw_init.data[3] = 'L';
        draw_init.data[4] = 'O';
        draw_init.data[5] = 'W';
        break;
    case CHASSIS_TOP:
        draw_init.grapic_data_struct.color = 1;
        draw_init.grapic_data_struct.end_angle = 3; //字符长度
        draw_init.data[0] = 'T';
        draw_init.data[1] = 'O';
        draw_init.data[2] = 'P';
        break;
    case CHASSIS_NO_FOLLOW_GIMBAL:
        draw_init.grapic_data_struct.color = 8;
        draw_init.grapic_data_struct.end_angle = 10; //字符长度
        draw_init.data[0] = 'N';
        draw_init.data[1] = 'O';
        draw_init.data[2] = '_';
        draw_init.data[3] = 'F';
        draw_init.data[4] = 'O';
        draw_init.data[5] = 'L';
        draw_init.data[6] = 'L';
        draw_init.data[7] = 'O';
        draw_init.data[8] = 'W';
        break;
    case CHASSIS_OPEN:
        draw_init.grapic_data_struct.color = 8;
        draw_init.grapic_data_struct.end_angle = 5; //字符长度
        draw_init.data[0] = 'O';
        draw_init.data[1] = 'P';
        draw_init.data[2] = 'E';
        draw_init.data[3] = 'N';
        break;
    }
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void draw_aim_assitant1(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type, uint16_t layer)
{

    ext_client_custom_graphic_seven_t test_draw;

    test_draw.header_data_t.data_cmd_id = 0x0104;
    test_draw.header_data_t.sender_ID = sender_id;
    test_draw.header_data_t.receiver_ID = receiver_id;

    test_draw.grapic_data_struct[0].graphic_name[0] = 0;
    test_draw.grapic_data_struct[0].graphic_name[1] = 0;
    test_draw.grapic_data_struct[0].graphic_name[2] = 10;

    test_draw.grapic_data_struct[0].operate_tpye = op_type;
    test_draw.grapic_data_struct[0].graphic_tpye = 0;
    test_draw.grapic_data_struct[0].layer = layer;
    test_draw.grapic_data_struct[0].color = 0;
    test_draw.grapic_data_struct[0].start_angle = 0;
    test_draw.grapic_data_struct[0].end_angle = 0;
    test_draw.grapic_data_struct[0].width = WIDTH;
    test_draw.grapic_data_struct[0].start_x = AIM_CENTER_X - AIM_HORIAZONTAL_LONG_X;
    test_draw.grapic_data_struct[0].start_y = AIM_CENTER_Y;
    test_draw.grapic_data_struct[0].end_x = AIM_CENTER_X + AIM_HORIAZONTAL_LONG_X;
    test_draw.grapic_data_struct[0].end_y = AIM_CENTER_Y;
    test_draw.grapic_data_struct[0].radius = 0;

    test_draw.grapic_data_struct[1].graphic_name[0] = 0;
    test_draw.grapic_data_struct[1].graphic_name[1] = 0;
    test_draw.grapic_data_struct[1].graphic_name[2] = 11; //图形名
    //上面三个字节代表的是图形名，用于图形索引，可自行定义
    test_draw.grapic_data_struct[1].operate_tpye = op_type; //图形操作，0：空操作；1：增加；2：修改；3：删除；
    test_draw.grapic_data_struct[1].graphic_tpye = 0;       //图形类型，0为直线，其他的查看用户手册
    test_draw.grapic_data_struct[1].layer = layer;          //图层数
    test_draw.grapic_data_struct[1].color = 1;              //颜色
    test_draw.grapic_data_struct[1].start_angle = 0;
    test_draw.grapic_data_struct[1].end_angle = 0;
    test_draw.grapic_data_struct[1].width = WIDTH;
    test_draw.grapic_data_struct[1].start_x = AIM_CENTER_X - AIM_HORIAZONTAL_SHORT_X;
    test_draw.grapic_data_struct[1].start_y = AIM_CENTER_Y - AIM_VERTICAL_SHORT_Y;
    test_draw.grapic_data_struct[1].end_x = AIM_CENTER_X + AIM_HORIAZONTAL_SHORT_X;
    test_draw.grapic_data_struct[1].end_y = AIM_CENTER_Y - AIM_VERTICAL_SHORT_Y;
    test_draw.grapic_data_struct[1].radius = 0;

    test_draw.grapic_data_struct[2].graphic_name[0] = 0;
    test_draw.grapic_data_struct[2].graphic_name[1] = 0;
    test_draw.grapic_data_struct[2].graphic_name[2] = 12; //图形名
    //上面三个字节代表的是图形名，用于图形索引，可自行定义
    test_draw.grapic_data_struct[2].operate_tpye = op_type; //图形操作，0：空操作；1：增加；2：修改
    test_draw.grapic_data_struct[2].graphic_tpye = 0;       //图形类型，0为直线，其他的查看用户手册
    test_draw.grapic_data_struct[2].layer = layer;          //图层数
    test_draw.grapic_data_struct[2].color = 2;              //颜色
    test_draw.grapic_data_struct[2].start_angle = 0;
    test_draw.grapic_data_struct[2].end_angle = 0;
    test_draw.grapic_data_struct[2].width = WIDTH;
    test_draw.grapic_data_struct[2].start_x = AIM_CENTER_X - AIM_HORIAZONTAL_SHORT_X;
    test_draw.grapic_data_struct[2].start_y = AIM_CENTER_Y - 2 * AIM_VERTICAL_SHORT_Y;
    test_draw.grapic_data_struct[2].end_x = AIM_CENTER_X + AIM_HORIAZONTAL_SHORT_X;
    test_draw.grapic_data_struct[2].end_y = AIM_CENTER_Y - 2 * AIM_VERTICAL_SHORT_Y;
    test_draw.grapic_data_struct[2].radius = 0;

    test_draw.grapic_data_struct[3].graphic_name[0] = 0;
    test_draw.grapic_data_struct[3].graphic_name[1] = 0;
    test_draw.grapic_data_struct[3].graphic_name[2] = 13; //图形名
    //上面三个字节代表的是图形名于图形索引，可自行定义
    test_draw.grapic_data_struct[3].operate_tpye = op_type; //图形操作，0：空操作；1：增加；2：修改
    test_draw.grapic_data_struct[3].graphic_tpye = 0;       //图形类型，0为直线，其他的查看用户手册
    test_draw.grapic_data_struct[3].layer = layer;          //图层数
    test_draw.grapic_data_struct[3].color = 3;              //颜色
    test_draw.grapic_data_struct[3].start_angle = 0;
    test_draw.grapic_data_struct[3].end_angle = 0;
    test_draw.grapic_data_struct[3].width = WIDTH;
    test_draw.grapic_data_struct[3].start_x = AIM_CENTER_X - AIM_HORIAZONTAL_SHORT_X;
    test_draw.grapic_data_struct[3].start_y = AIM_CENTER_Y - 3 * AIM_VERTICAL_SHORT_Y;
    test_draw.grapic_data_struct[3].end_x = AIM_CENTER_X + AIM_HORIAZONTAL_SHORT_X;
    test_draw.grapic_data_struct[3].end_y = AIM_CENTER_Y - 3 * AIM_VERTICAL_SHORT_Y;
    test_draw.grapic_data_struct[3].radius = 0;

    test_draw.grapic_data_struct[4].graphic_name[0] = 0;
    test_draw.grapic_data_struct[4].graphic_name[1] = 0;
    test_draw.grapic_data_struct[4].graphic_name[2] = 14; //图形名
    //上面三个字节代表的是图形名4于图形索引，可自行定义
    test_draw.grapic_data_struct[4].operate_tpye = op_type; //图形操作，0：空操作；1：增加；2：修改
    test_draw.grapic_data_struct[4].graphic_tpye = 0;       //图形类型，0为直线，其他的查看用户手册
    test_draw.grapic_data_struct[4].layer = layer;          //图层数
    test_draw.grapic_data_struct[4].color = 4;              //颜色
    test_draw.grapic_data_struct[4].start_angle = 0;
    test_draw.grapic_data_struct[4].end_angle = 0;
    test_draw.grapic_data_struct[4].width = WIDTH;
    test_draw.grapic_data_struct[4].start_x = AIM_CENTER_X - AIM_HORIAZONTAL_LONG_X + 15;
    test_draw.grapic_data_struct[4].start_y = AIM_CENTER_Y - AIM_VERTICAL_LONG_Y;
    test_draw.grapic_data_struct[4].end_x = AIM_CENTER_X + AIM_HORIAZONTAL_LONG_X - 15;
    test_draw.grapic_data_struct[4].end_y = AIM_CENTER_Y - AIM_VERTICAL_LONG_Y;
    test_draw.grapic_data_struct[4].radius = 0;

    test_draw.grapic_data_struct[5].graphic_name[0] = 0;
    test_draw.grapic_data_struct[5].graphic_name[1] = 0;
    test_draw.grapic_data_struct[5].graphic_name[2] = 15; //图形名
    //上面三个字节代表的是图形6用5于图形索引，可自行定义
    test_draw.grapic_data_struct[5].operate_tpye = op_type; //图形操作，0：空操作；1：增加；2：修改
    test_draw.grapic_data_struct[5].graphic_tpye = 0;       //图形类型，0为直线，其他的查看用户手册
    test_draw.grapic_data_struct[5].layer = layer;          //图层数
    test_draw.grapic_data_struct[5].color = 5;              //颜色
    test_draw.grapic_data_struct[5].start_angle = 0;
    test_draw.grapic_data_struct[5].end_angle = 0;
    test_draw.grapic_data_struct[5].width = WIDTH;
    test_draw.grapic_data_struct[5].start_x = AIM_CENTER_X - AIM_HORIAZONTAL_SHORT_X;
    test_draw.grapic_data_struct[5].start_y = AIM_CENTER_Y - 5 * AIM_VERTICAL_SHORT_Y;
    test_draw.grapic_data_struct[5].end_x = AIM_CENTER_X + AIM_HORIAZONTAL_SHORT_X;
    test_draw.grapic_data_struct[5].end_y = AIM_CENTER_Y - 5 * AIM_VERTICAL_SHORT_Y;
    test_draw.grapic_data_struct[5].radius = 0;

    test_draw.grapic_data_struct[6].graphic_name[0] = 0;
    test_draw.grapic_data_struct[6].graphic_name[1] = 0;
    test_draw.grapic_data_struct[6].graphic_name[2] = 16; //图形名

    test_draw.grapic_data_struct[6].operate_tpye = op_type; //图形操作，0：空操作；1：增加；2：修改
    test_draw.grapic_data_struct[6].graphic_tpye = 0;       //图形类型，0为直线，其他的查看用户手册
    test_draw.grapic_data_struct[6].layer = layer;          //图层数
    test_draw.grapic_data_struct[6].color = 6;              //颜色
    test_draw.grapic_data_struct[6].start_angle = 0;
    test_draw.grapic_data_struct[6].end_angle = 0;
    test_draw.grapic_data_struct[6].width = WIDTH;
    test_draw.grapic_data_struct[6].start_x = AIM_CENTER_X - AIM_HORIAZONTAL_SHORT_X;
    test_draw.grapic_data_struct[6].start_y = AIM_CENTER_Y - 6 * AIM_VERTICAL_SHORT_Y;
    test_draw.grapic_data_struct[6].end_x = AIM_CENTER_X + AIM_HORIAZONTAL_SHORT_X;
    test_draw.grapic_data_struct[6].end_y = AIM_CENTER_Y - 6 * AIM_VERTICAL_SHORT_Y;
    test_draw.grapic_data_struct[6].radius = 0;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&test_draw, sizeof(test_draw));
}

void draw_chassis_mode(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_character_t draw_init;
    draw_init.header_data_t.data_cmd_id = 0x0110;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct.graphic_name[0] = 0;
    draw_init.grapic_data_struct.graphic_name[1] = 0;
    draw_init.grapic_data_struct.graphic_name[2] = 17;
    draw_init.grapic_data_struct.operate_tpye = op_type;
    draw_init.grapic_data_struct.graphic_tpye = 7;
    draw_init.grapic_data_struct.layer = 1;
    draw_init.grapic_data_struct.color = 8;
    draw_init.grapic_data_struct.start_angle = FONT_SIZE; //字符大小

    draw_init.grapic_data_struct.width = 2;
    draw_init.grapic_data_struct.start_x = chassis_mode_x - 40;
    draw_init.grapic_data_struct.start_y = chassis_mode_y + VERTICAL;
    draw_init.grapic_data_struct.end_x = 0;
    draw_init.grapic_data_struct.end_y = 0;
    draw_init.grapic_data_struct.radius = 0;

    memset(draw_init.data, 0, sizeof(draw_init.data));
    draw_init.grapic_data_struct.end_angle = 13; //字符长度
    draw_init.data[0] = 'C';
    draw_init.data[1] = 'H';
    draw_init.data[2] = 'A';
    draw_init.data[3] = 'S';
    draw_init.data[4] = 'S';
    draw_init.data[5] = 'I';
    draw_init.data[6] = 'S';
    draw_init.data[7] = ' ';
    draw_init.data[8] = 'M';
    draw_init.data[9] = 'O';
    draw_init.data[10] = 'D';
    draw_init.data[11] = 'E';
    draw_init.data[12] = ':';

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void draw_chassis_run_bordor(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    int w = 3;
    ext_client_custom_graphic_five_t draw_init;
    draw_init.header_data_t.data_cmd_id = 0x0104;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct[0].graphic_name[0] = 0;
    draw_init.grapic_data_struct[0].graphic_name[1] = 0;
    draw_init.grapic_data_struct[0].graphic_name[2] = 18;

    draw_init.grapic_data_struct[0].operate_tpye = op_type;
    draw_init.grapic_data_struct[0].graphic_tpye = 0;
    draw_init.grapic_data_struct[0].layer = 3;
    draw_init.grapic_data_struct[0].color = 3;
    draw_init.grapic_data_struct[0].start_angle = 0;
    draw_init.grapic_data_struct[0].end_angle = 0;
    draw_init.grapic_data_struct[0].width = w;

    draw_init.grapic_data_struct[0].start_x = SCREEN_LENGTH / 2 - CHASSIS_RUN_BORDER_START_X; //- 300 - pangle;
    draw_init.grapic_data_struct[0].start_y = CHASSIS_RUN_BORDER_START_Y;
    draw_init.grapic_data_struct[0].end_x = SCREEN_LENGTH / 2 - CHASSIS_RUN_BORDER_END_X; //- 200 - pangle;
    draw_init.grapic_data_struct[0].end_y = CHASSIS_RUN_BORDER_END_Y;
    draw_init.grapic_data_struct[0].radius = 0;

    draw_init.grapic_data_struct[1].graphic_name[0] = 0;
    draw_init.grapic_data_struct[1].graphic_name[1] = 0;
    draw_init.grapic_data_struct[1].graphic_name[2] = 19; //图形名

    draw_init.grapic_data_struct[1].operate_tpye = op_type; //图形操作，0：空操作；1：增加；2：修改；3：删除；
    draw_init.grapic_data_struct[1].graphic_tpye = 0;       //图形类型，0为直线，其他的查看用户手册
    draw_init.grapic_data_struct[1].layer = 3;              //图层数
    draw_init.grapic_data_struct[1].color = 3;              //颜色
    draw_init.grapic_data_struct[1].start_angle = 0;
    draw_init.grapic_data_struct[1].end_angle = 0;
    draw_init.grapic_data_struct[1].width = w;
    draw_init.grapic_data_struct[1].start_x = SCREEN_LENGTH / 2 + CHASSIS_RUN_BORDER_START_X;
    draw_init.grapic_data_struct[1].start_y = CHASSIS_RUN_BORDER_START_Y;
    draw_init.grapic_data_struct[1].end_x = SCREEN_LENGTH / 2 + CHASSIS_RUN_BORDER_END_X;
    draw_init.grapic_data_struct[1].end_y = CHASSIS_RUN_BORDER_END_Y;
    draw_init.grapic_data_struct[1].radius = 0;

    draw_init.grapic_data_struct[2].graphic_name[0] = 0;
    draw_init.grapic_data_struct[2].graphic_name[1] = 1;
    draw_init.grapic_data_struct[2].graphic_name[2] = 99; //图形名

    draw_init.grapic_data_struct[2].operate_tpye = op_type; //图形操作，0：空操作；1：增加；2：修改；3：删除；
    draw_init.grapic_data_struct[2].graphic_tpye = 0;       //图形类型，0为直线，其他的查看用户手册
    draw_init.grapic_data_struct[2].layer = 3;              //图层数
    draw_init.grapic_data_struct[2].color = 3;              //颜色
    draw_init.grapic_data_struct[2].start_angle = 0;
    draw_init.grapic_data_struct[2].end_angle = 0;
    draw_init.grapic_data_struct[2].width = w;
    draw_init.grapic_data_struct[2].start_x = SCREEN_LENGTH / 2 - CHASSIS_RUN_BORDER_END_X;
    draw_init.grapic_data_struct[2].start_y = CHASSIS_RUN_BORDER_END_Y;
    draw_init.grapic_data_struct[2].end_x = SCREEN_LENGTH / 2 - CHASSIS_RUN_BORDER_END_X + 100;
    draw_init.grapic_data_struct[2].end_y = CHASSIS_RUN_BORDER_END_Y;
    draw_init.grapic_data_struct[2].radius = 0;

    draw_init.grapic_data_struct[3].graphic_name[0] = 0;
    draw_init.grapic_data_struct[3].graphic_name[1] = 1;
    draw_init.grapic_data_struct[3].graphic_name[2] = 98; //图形名

    draw_init.grapic_data_struct[3].operate_tpye = op_type; //图形操作，0：空操作；1：增加；2：修改；3：删除；
    draw_init.grapic_data_struct[3].graphic_tpye = 0;       //图形类型，0为直线，其他的查看用户手册
    draw_init.grapic_data_struct[3].layer = 3;              //图层数
    draw_init.grapic_data_struct[3].color = 3;              //颜色
    draw_init.grapic_data_struct[3].start_angle = 0;
    draw_init.grapic_data_struct[3].end_angle = 0;
    draw_init.grapic_data_struct[3].width = w;
    draw_init.grapic_data_struct[3].start_x = SCREEN_LENGTH / 2 + CHASSIS_RUN_BORDER_END_X;
    draw_init.grapic_data_struct[3].start_y = CHASSIS_RUN_BORDER_END_Y;
    draw_init.grapic_data_struct[3].end_x = SCREEN_LENGTH / 2 + CHASSIS_RUN_BORDER_END_X - 100;
    draw_init.grapic_data_struct[3].end_y = CHASSIS_RUN_BORDER_END_Y;
    draw_init.grapic_data_struct[3].radius = 0;

    draw_init.grapic_data_struct[4].graphic_name[0] = 0;
    draw_init.grapic_data_struct[4].graphic_name[1] = 1;
    draw_init.grapic_data_struct[4].graphic_name[2] = 3; //图形名

    //考虑做成动态
    draw_init.grapic_data_struct[4].operate_tpye = op_type; //图形操作，0：空操作；1：增加；2：修改；3：删除；
    draw_init.grapic_data_struct[4].graphic_tpye = 0;       //图形类型，0为直线，其他的查看用户手册
    draw_init.grapic_data_struct[4].layer = 3;              //图层数
    draw_init.grapic_data_struct[4].color = 3;              //颜色
    draw_init.grapic_data_struct[4].start_angle = 0;
    draw_init.grapic_data_struct[4].end_angle = 0;
    draw_init.grapic_data_struct[4].width = w;
    draw_init.grapic_data_struct[4].start_x = SCREEN_LENGTH / 2 - 20;
    draw_init.grapic_data_struct[4].start_y = CHASSIS_RUN_BORDER_END_Y;
    draw_init.grapic_data_struct[4].end_x = SCREEN_LENGTH / 2 + 20;
    draw_init.grapic_data_struct[4].end_y = CHASSIS_RUN_BORDER_END_Y;
    draw_init.grapic_data_struct[4].radius = 0;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void cal_draw_gimbal_relative_angle_tangle(tank_ui_t *tank)
{

    tank->r = VERTICAL * 2.2;
    tank->angle = gimbal_control.gimbal_yaw_motor.relative_angle;
    if (tank->angle >= -PI && tank->angle <= 0)
    {
        tank->angle = -tank->angle;
        tank->angle = tank->angle - GIMBAL_PITCH_OFFSET_ECD / 2;
    }
    else if (tank->angle >= 0 && tank->angle <= PI)
    {
        tank->angle = -tank->angle;
        tank->angle = tank->angle - GIMBAL_PITCH_OFFSET_ECD / 2;
    }
    tank->gx = (int)(arm_cos_f32(tank->angle) * tank->r);
    tank->gy = (int)(arm_sin_f32(tank->angle) * tank->r);
}

void draw_gimbal_relative_angle_tangle(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{

    ext_client_custom_graphic_double_t draw_init;

    draw_init.header_data_t.data_cmd_id = 0x0102;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct[0].graphic_name[0] = 0;
    draw_init.grapic_data_struct[0].graphic_name[1] = 0;
    draw_init.grapic_data_struct[0].graphic_name[2] = 25;

    draw_init.grapic_data_struct[0].graphic_tpye = 1;
    draw_init.grapic_data_struct[0].operate_tpye = op_type;
    draw_init.grapic_data_struct[0].layer = 9;
    draw_init.grapic_data_struct[0].color = 0;
    draw_init.grapic_data_struct[0].width = 3;
    draw_init.grapic_data_struct[0].start_x = gimbal_angle_center_x - 70;
    draw_init.grapic_data_struct[0].start_y = gimbal_angle_center_y - 50;
    draw_init.grapic_data_struct[0].end_x = gimbal_angle_center_x + 70;
    draw_init.grapic_data_struct[0].end_y = gimbal_angle_center_y + 50;

    draw_init.grapic_data_struct[1].graphic_name[0] = 0;
    draw_init.grapic_data_struct[1].graphic_name[1] = 0;
    draw_init.grapic_data_struct[1].graphic_name[2] = 26;
    draw_init.grapic_data_struct[1].graphic_tpye = 0;
    draw_init.grapic_data_struct[1].operate_tpye = op_type;
    draw_init.grapic_data_struct[1].layer = 7;
    draw_init.grapic_data_struct[1].color = 3;
    draw_init.grapic_data_struct[1].width = 2;
    draw_init.grapic_data_struct[1].start_x = gimbal_angle_center_x;
    draw_init.grapic_data_struct[1].start_y = gimbal_angle_center_y;
    draw_init.grapic_data_struct[1].end_x = gimbal_angle_center_x + tank.gx;
    draw_init.grapic_data_struct[1].end_y = gimbal_angle_center_y + tank.gy;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void draw_single_tangle_test(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_graphic_single_t draw_test;

    draw_test.header_data_t.data_cmd_id = 0x0101;
    draw_test.header_data_t.sender_ID = sender_id;
    draw_test.header_data_t.receiver_ID = receiver_id;

    draw_test.grapic_data_struct.graphic_name[0] = 1;
    draw_test.grapic_data_struct.graphic_name[1] = 0;
    draw_test.grapic_data_struct.graphic_name[2] = 0;

    draw_test.grapic_data_struct.graphic_tpye = 1;
    draw_test.grapic_data_struct.operate_tpye = op_type;
    draw_test.grapic_data_struct.layer = 9;
    draw_test.grapic_data_struct.color = 8;
    draw_test.grapic_data_struct.width = 2;
    draw_test.grapic_data_struct.start_x = 960;
    draw_test.grapic_data_struct.start_y = 500;
    draw_test.grapic_data_struct.end_x = 960 + 100;
    draw_test.grapic_data_struct.end_y = 500 + 100;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_test, sizeof(draw_test));
}

void draw_single_line_test(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_graphic_single_t draw_test;

    draw_test.header_data_t.data_cmd_id = 0x0101;
    draw_test.header_data_t.sender_ID = sender_id;
    draw_test.header_data_t.receiver_ID = receiver_id;

    draw_test.grapic_data_struct.graphic_name[0] = 1;
    draw_test.grapic_data_struct.graphic_name[1] = 0;
    draw_test.grapic_data_struct.graphic_name[2] = 1;

    draw_test.grapic_data_struct.graphic_tpye = 0;
    draw_test.grapic_data_struct.operate_tpye = op_type;
    draw_test.grapic_data_struct.layer = 9;
    draw_test.grapic_data_struct.color = 8;
    draw_test.grapic_data_struct.width = 2;
    draw_test.grapic_data_struct.start_x = 960;
    draw_test.grapic_data_struct.start_y = 500;
    draw_test.grapic_data_struct.end_x = 960 + 200;
    draw_test.grapic_data_struct.end_y = 500 + 200;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_test, sizeof(draw_test));
}

void draw_aim_assitant2(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type, uint16_t layer)
{

    ext_client_custom_graphic_seven_t test_draw;

    test_draw.header_data_t.data_cmd_id = 0x0104;
    test_draw.header_data_t.sender_ID = sender_id;
    test_draw.header_data_t.receiver_ID = receiver_id;
    //竖线
    test_draw.grapic_data_struct[0].graphic_name[0] = 0;
    test_draw.grapic_data_struct[0].graphic_name[1] = 0;
    test_draw.grapic_data_struct[0].graphic_name[2] = 28;

    test_draw.grapic_data_struct[0].operate_tpye = op_type;
    test_draw.grapic_data_struct[0].graphic_tpye = 0;
    test_draw.grapic_data_struct[0].layer = layer;
    test_draw.grapic_data_struct[0].color = 6;
    test_draw.grapic_data_struct[0].start_angle = 0;
    test_draw.grapic_data_struct[0].end_angle = 0;
    test_draw.grapic_data_struct[0].width = WIDTH;
    test_draw.grapic_data_struct[0].start_x = AIM_CENTER_X;
    test_draw.grapic_data_struct[0].start_y = AIM_CENTER_Y + 20;
    test_draw.grapic_data_struct[0].end_x = AIM_CENTER_X;
    test_draw.grapic_data_struct[0].end_y = AIM_CENTER_Y - 350; //竖线
    test_draw.grapic_data_struct[0].radius = 0;

    test_draw.grapic_data_struct[1].graphic_name[0] = 0;
    test_draw.grapic_data_struct[1].graphic_name[1] = 0;
    test_draw.grapic_data_struct[1].graphic_name[2] = 29; //图形名
    //上面三个字节代表的是图形名，用于图形索引，可自行定义
    test_draw.grapic_data_struct[1].operate_tpye = op_type; //图形操作，0：空操作；1：增加；2：修改；3：删除；
    test_draw.grapic_data_struct[1].graphic_tpye = 0;       //图形类型，0为直线，其他的查看用户手册
    test_draw.grapic_data_struct[1].layer = layer;          //图层数
    test_draw.grapic_data_struct[1].color = 1;              //颜色
    test_draw.grapic_data_struct[1].start_angle = 0;
    test_draw.grapic_data_struct[1].end_angle = 0;
    test_draw.grapic_data_struct[1].width = WIDTH;
    test_draw.grapic_data_struct[1].start_x = AIM_CENTER_X - AIM_HORIAZONTAL_SHORT_X;
    test_draw.grapic_data_struct[1].start_y = AIM_CENTER_Y - 7 * AIM_VERTICAL_SHORT_Y;
    test_draw.grapic_data_struct[1].end_x = AIM_CENTER_X + AIM_HORIAZONTAL_SHORT_X;
    test_draw.grapic_data_struct[1].end_y = AIM_CENTER_Y - 7 * AIM_VERTICAL_SHORT_Y;
    test_draw.grapic_data_struct[1].radius = 0;

    test_draw.grapic_data_struct[2].graphic_name[0] = 0;
    test_draw.grapic_data_struct[2].graphic_name[1] = 0;
    test_draw.grapic_data_struct[2].graphic_name[2] = 30; //图形名
    //上面三个字节代表的是图形名，用于图形索引，可自行定义
    test_draw.grapic_data_struct[2].operate_tpye = op_type; //图形操作，0：空操作；1：增加；2：修改
    test_draw.grapic_data_struct[2].graphic_tpye = 0;       //图形类型，0为直线，其他的查看用户手册
    test_draw.grapic_data_struct[2].layer = layer;          //图层数
    test_draw.grapic_data_struct[2].color = 2;              //颜色
    test_draw.grapic_data_struct[2].start_angle = 0;
    test_draw.grapic_data_struct[2].end_angle = 0;
    test_draw.grapic_data_struct[2].width = WIDTH;
    test_draw.grapic_data_struct[2].start_x = AIM_CENTER_X - AIM_HORIAZONTAL_LONG_X + 30;
    test_draw.grapic_data_struct[2].start_y = AIM_CENTER_Y - 8 * AIM_VERTICAL_SHORT_Y;
    test_draw.grapic_data_struct[2].end_x = AIM_CENTER_X + AIM_HORIAZONTAL_LONG_X - 30;
    test_draw.grapic_data_struct[2].end_y = AIM_CENTER_Y - 8 * AIM_VERTICAL_SHORT_Y;
    test_draw.grapic_data_struct[2].radius = 0;

    test_draw.grapic_data_struct[3].graphic_name[0] = 0;
    test_draw.grapic_data_struct[3].graphic_name[1] = 0;
    test_draw.grapic_data_struct[3].graphic_name[2] = 31; //图形名
    //上面三个字节代表的是图形名于图形索引，可自行定义
    test_draw.grapic_data_struct[3].operate_tpye = op_type; //图形操作，0：空操作；1：增加；2：修改
    test_draw.grapic_data_struct[3].graphic_tpye = 0;       //图形类型，0为直线，其他的查看用户手册
    test_draw.grapic_data_struct[3].layer = layer;          //图层数
    test_draw.grapic_data_struct[3].color = 3;              //颜色
    test_draw.grapic_data_struct[3].start_angle = 0;
    test_draw.grapic_data_struct[3].end_angle = 0;
    test_draw.grapic_data_struct[3].width = WIDTH;
    test_draw.grapic_data_struct[3].start_x = AIM_CENTER_X - AIM_HORIAZONTAL_SHORT_X;
    test_draw.grapic_data_struct[3].start_y = AIM_CENTER_Y - 9 * AIM_VERTICAL_SHORT_Y;
    test_draw.grapic_data_struct[3].end_x = AIM_CENTER_X + AIM_HORIAZONTAL_SHORT_X;
    test_draw.grapic_data_struct[3].end_y = AIM_CENTER_Y - 9 * AIM_VERTICAL_SHORT_Y;
    test_draw.grapic_data_struct[3].radius = 0;

    test_draw.grapic_data_struct[4].graphic_name[0] = 0;
    test_draw.grapic_data_struct[4].graphic_name[1] = 0;
    test_draw.grapic_data_struct[4].graphic_name[2] = 32; //图形名
    //上面三个字节代表的是图形名4于图形索引，可自行定义
    test_draw.grapic_data_struct[4].operate_tpye = op_type; //图形操作，0：空操作；1：增加；2：修改
    test_draw.grapic_data_struct[4].graphic_tpye = 0;       //图形类型，0为直线，其他的查看用户手册
    test_draw.grapic_data_struct[4].layer = layer;          //图层数
    test_draw.grapic_data_struct[4].color = 4;              //颜色
    test_draw.grapic_data_struct[4].start_angle = 0;
    test_draw.grapic_data_struct[4].end_angle = 0;
    test_draw.grapic_data_struct[4].width = WIDTH;
    test_draw.grapic_data_struct[4].start_x = AIM_CENTER_X - AIM_HORIAZONTAL_SHORT_X;
    test_draw.grapic_data_struct[4].start_y = AIM_CENTER_Y - 10 * AIM_VERTICAL_SHORT_Y;
    test_draw.grapic_data_struct[4].end_x = AIM_CENTER_X + AIM_HORIAZONTAL_SHORT_X;
    test_draw.grapic_data_struct[4].end_y = AIM_CENTER_Y - 10 * AIM_VERTICAL_SHORT_Y;
    test_draw.grapic_data_struct[4].radius = 0;

    test_draw.grapic_data_struct[5].graphic_name[0] = 0;
    test_draw.grapic_data_struct[5].graphic_name[1] = 0;
    test_draw.grapic_data_struct[5].graphic_name[2] = 33; //图形名
    //上面三个字节代表的是图形6用5于图形索引，可自行定义
    test_draw.grapic_data_struct[5].operate_tpye = op_type; //图形操作，0：空操作；1：增加；2：修改
    test_draw.grapic_data_struct[5].graphic_tpye = 0;       //图形类型，0为直线，其他的查看用户手册
    test_draw.grapic_data_struct[5].layer = layer;          //图层数
    test_draw.grapic_data_struct[5].color = 8;              //颜色
    test_draw.grapic_data_struct[5].start_angle = 0;
    test_draw.grapic_data_struct[5].end_angle = 0;
    test_draw.grapic_data_struct[5].width = 1;
    test_draw.grapic_data_struct[5].start_x = AIM_CENTER_X - 20; //十字
    test_draw.grapic_data_struct[5].start_y = 1080 / 2;
    test_draw.grapic_data_struct[5].end_x = AIM_CENTER_X + 20;
    test_draw.grapic_data_struct[5].end_y = 1080 / 2;
    test_draw.grapic_data_struct[5].radius = 0;

    test_draw.grapic_data_struct[6].graphic_name[0] = 0;
    test_draw.grapic_data_struct[6].graphic_name[1] = 0;
    test_draw.grapic_data_struct[6].graphic_name[2] = 34; //图形名

    test_draw.grapic_data_struct[6].operate_tpye = op_type; //图形操作，0：空操作；1：增加；2：修改
    test_draw.grapic_data_struct[6].graphic_tpye = 0;       //图形类型，0为直线，其他的查看用户手册
    test_draw.grapic_data_struct[6].layer = layer;          //图层数
    test_draw.grapic_data_struct[6].color = 8;              //颜色
    test_draw.grapic_data_struct[6].start_angle = 0;
    test_draw.grapic_data_struct[6].end_angle = 0;
    test_draw.grapic_data_struct[6].width = 1;
    test_draw.grapic_data_struct[6].start_x = AIM_CENTER_X;
    test_draw.grapic_data_struct[6].start_y = 1080 / 2 + 20;
    test_draw.grapic_data_struct[6].end_x = AIM_CENTER_X;
    test_draw.grapic_data_struct[6].end_y = 1080 / 2 - 20;
    test_draw.grapic_data_struct[6].radius = 0;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&test_draw, sizeof(test_draw));
}

void draw_state_of_top(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type) //$8
{
    ext_client_custom_character_t draw_init;
    draw_init.header_data_t.data_cmd_id = 0x0110;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct.graphic_name[0] = 0;
    draw_init.grapic_data_struct.graphic_name[1] = 0;
    draw_init.grapic_data_struct.graphic_name[2] = 40;
    draw_init.grapic_data_struct.operate_tpye = op_type;
    draw_init.grapic_data_struct.graphic_tpye = 7;
    draw_init.grapic_data_struct.layer = 2;
    draw_init.grapic_data_struct.color = 8;
    draw_init.grapic_data_struct.start_angle = FONT_SIZE; //字符大小

    draw_init.grapic_data_struct.width = 2;
    draw_init.grapic_data_struct.start_x = LEFT_STATE_AREA_X;
    draw_init.grapic_data_struct.start_y = LEFT_STATE_AREA_Y;
    draw_init.grapic_data_struct.end_x = 0;
    draw_init.grapic_data_struct.end_y = 0;
    draw_init.grapic_data_struct.radius = 0;

    memset(draw_init.data, 0, sizeof(draw_init.data));
    draw_init.grapic_data_struct.end_angle = 12; //字符长度
    draw_init.data[0] = 'C';
    draw_init.data[1] = 'H';
    draw_init.data[2] = 'A';
    draw_init.data[3] = 'S';
    draw_init.data[4] = 'S';
    draw_init.data[5] = 'I';
    draw_init.data[6] = 'S';
    draw_init.data[7] = ' ';
    draw_init.data[8] = 'T';
    draw_init.data[9] = 'O';
    draw_init.data[10] = 'P';
    draw_init.data[11] = ':';

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void draw_state_of_top_circle(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_graphic_single_t draw_init;
    draw_init.header_data_t.data_cmd_id = 0x0101;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct.graphic_name[0] = 0;
    draw_init.grapic_data_struct.graphic_name[1] = 0;
    draw_init.grapic_data_struct.graphic_name[2] = 41;
    draw_init.grapic_data_struct.operate_tpye = op_type;
    draw_init.grapic_data_struct.graphic_tpye = 2;
    draw_init.grapic_data_struct.layer = 1;
    draw_init.grapic_data_struct.start_x = LEFT_STATE_AREA_X + 220;
    draw_init.grapic_data_struct.start_y = LEFT_STATE_AREA_Y - 10;
    draw_init.grapic_data_struct.width = 3;
    draw_init.grapic_data_struct.radius = 20;

    if (chassis_move.chassis_mode == CHASSIS_TOP)
    {
        draw_init.grapic_data_struct.color = 0;
    }
    else
    {
        draw_init.grapic_data_struct.color = 8;
    }
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void draw_capacity_number(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_graphic_single_t_data init_struct;

    init_struct.header_data_t.data_cmd_id = 0x0101;
    init_struct.header_data_t.sender_ID = sender_id;
    init_struct.header_data_t.receiver_ID = receiver_id;

    init_struct.grapic_data_struct.graphic_name[0] = 0;
    init_struct.grapic_data_struct.graphic_name[1] = 0;
    init_struct.grapic_data_struct.graphic_name[2] = 42;
    init_struct.grapic_data_struct.operate_tpye = op_type;
    init_struct.grapic_data_struct.graphic_tpye = 6;
    init_struct.grapic_data_struct.layer = 1;
    init_struct.grapic_data_struct.color = 4;
    init_struct.grapic_data_struct.start_x = CAP_START_X;
    init_struct.grapic_data_struct.start_y = CAP_START_Y;

    init_struct.grapic_data_struct.start_angle = 25; // font size
    init_struct.grapic_data_struct.end_angle = 0;
    init_struct.grapic_data_struct.width = 2;

    init_struct.grapic_data_struct.data = PowerData[3];

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&init_struct, sizeof(init_struct));
}

void draw_capacity_char(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_character_t draw_init;
    draw_init.header_data_t.data_cmd_id = 0x0110;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct.graphic_name[0] = 0;
    draw_init.grapic_data_struct.graphic_name[1] = 0;
    draw_init.grapic_data_struct.graphic_name[2] = 43;
    draw_init.grapic_data_struct.operate_tpye = op_type;
    draw_init.grapic_data_struct.graphic_tpye = 7;
    draw_init.grapic_data_struct.layer = 1;
    draw_init.grapic_data_struct.color = 8;
    draw_init.grapic_data_struct.start_angle = 25; //字符大小

    draw_init.grapic_data_struct.width = 2;
    draw_init.grapic_data_struct.start_x = CAP_START_X + 50;
    draw_init.grapic_data_struct.start_y = CAP_START_Y;
    draw_init.grapic_data_struct.end_x = 0;
    draw_init.grapic_data_struct.end_y = 0;
    draw_init.grapic_data_struct.radius = 0;

    memset(draw_init.data, 0, sizeof(draw_init.data));
    draw_init.grapic_data_struct.end_angle = 1; //字符长度
    draw_init.data[0] = '%';

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void cal_draw_pitch_aim(aim_dynamic_t *aim_dynamic)
{
    aim_dynamic->angle = gimbal_control.gimbal_pitch_motor.absolute_angle;
}

void draw_capacity_tangle(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{

    // if(super_capacity.interprolation<super_capacity.jindu) super_capacity.interprolation+=3;

    ext_client_custom_graphic_double_t draw_init;

    draw_init.header_data_t.data_cmd_id = 0x0102;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct[0].graphic_name[0] = 0;
    draw_init.grapic_data_struct[0].graphic_name[1] = 0;
    draw_init.grapic_data_struct[0].graphic_name[2] = 45;

    draw_init.grapic_data_struct[0].graphic_tpye = 1;
    draw_init.grapic_data_struct[0].operate_tpye = op_type;
    draw_init.grapic_data_struct[0].layer = 9;
    draw_init.grapic_data_struct[0].color = 4;
    draw_init.grapic_data_struct[0].width = 3;
    draw_init.grapic_data_struct[0].start_x = CAP_TANGLE_START_X;
    draw_init.grapic_data_struct[0].start_y = CAP_TANGLE_START_Y;
    draw_init.grapic_data_struct[0].end_x = CAP_TANGLE_END_X;
    draw_init.grapic_data_struct[0].end_y = CAP_TANGLE_END_Y;

    draw_init.grapic_data_struct[1].graphic_name[0] = 0;
    draw_init.grapic_data_struct[1].graphic_name[1] = 0;
    draw_init.grapic_data_struct[1].graphic_name[2] = 46;
    draw_init.grapic_data_struct[1].graphic_tpye = 0;
    draw_init.grapic_data_struct[1].operate_tpye = op_type;
    draw_init.grapic_data_struct[1].layer = 7;
    draw_init.grapic_data_struct[1].color = 2;
    draw_init.grapic_data_struct[1].width = 28;
    draw_init.grapic_data_struct[1].start_x = CAP_TANGLE_START_X + 2;
    draw_init.grapic_data_struct[1].start_y = CAP_TANGLE_START_Y + 15;
    draw_init.grapic_data_struct[1].end_x = CAP_TANGLE_START_X + super_capacity.jindu;
    draw_init.grapic_data_struct[1].end_y = CAP_TANGLE_START_Y + 15;

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void audio_enemy_godlike(uint16_t sender_id, uint16_t receiver_id)
{
    static uint16_t op_type = 1;
    static int op = 0;
    static uint16_t x = 0;
    x += 50;
    if (x == 600)
        x = 0;
    op_type = op_type ? 0 : 1;
    op = op_type ? 3 : 1;

    ext_client_custom_character_t draw_init;
    draw_init.header_data_t.data_cmd_id = 0x0110;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct.graphic_name[0] = 0;
    draw_init.grapic_data_struct.graphic_name[1] = 0;
    draw_init.grapic_data_struct.graphic_name[2] = 47;
    draw_init.grapic_data_struct.operate_tpye = op;
    draw_init.grapic_data_struct.graphic_tpye = 7;
    draw_init.grapic_data_struct.layer = 2;
    draw_init.grapic_data_struct.color = 8;
    draw_init.grapic_data_struct.start_angle = FONT_SIZE; //字符大小

    draw_init.grapic_data_struct.width = 2;
    draw_init.grapic_data_struct.start_x = 200 + x;
    draw_init.grapic_data_struct.start_y = 800;
    draw_init.grapic_data_struct.end_x = 0;
    draw_init.grapic_data_struct.end_y = 0;
    draw_init.grapic_data_struct.radius = 0;

    memset(draw_init.data, 0, sizeof(draw_init.data));
    draw_init.grapic_data_struct.end_angle = 22; //字符长度
    draw_init.data[0] = 'E';
    draw_init.data[1] = 'N';
    draw_init.data[2] = 'E';
    draw_init.data[3] = 'M';
    draw_init.data[4] = 'Y';
    draw_init.data[5] = ' ';
    draw_init.data[6] = 'H';
    draw_init.data[7] = 'A';
    draw_init.data[8] = 'S';
    draw_init.data[9] = ' ';
    draw_init.data[10] = 'B';
    draw_init.data[11] = 'E';
    draw_init.data[12] = 'E';
    draw_init.data[13] = 'N';
    draw_init.data[14] = ' ';
    draw_init.data[15] = 'G';
    draw_init.data[16] = 'O';
    draw_init.data[17] = 'D';
    draw_init.data[18] = 'L';
    draw_init.data[19] = 'I';
    draw_init.data[20] = 'K';
    draw_init.data[21] = 'E';

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}
void cal_capacity(super_capacity_t *super_capacity)
{
    super_capacity->power_ratio = PowerData[1] / PowerData[0];
    super_capacity->jindu = (uint16_t)(super_capacity->power_ratio * 300);
    super_capacity->voltage = (uint16_t)(PowerData[1] * 100);
}

void draw_get_robot_id()
{
    switch (robot_state.robot_id)
    {
    case RED_HERO:
        send_id = 1;
        receive_id = 0x0101;
        break;
    case RED_ENGINEER:
        send_id = 2;
        receive_id = 0x0102;
        break;
    case RED_STANDARD_1:
        send_id = 3;
        receive_id = 0x0103;
        break;
    case RED_STANDARD_2:
        send_id = 4;
        receive_id = 0x0104;
        break;
    case RED_STANDARD_3:
        send_id = 5;
        receive_id = 0x0105;
        break;
    case RED_AERIAL:
        send_id = 6;
        receive_id = 0x0106;
        break;
    case BLUE_HERO:
        send_id = 101;
        receive_id = 0x0165;
        break;
    case BLUE_ENGINEER:
        send_id = 102;
        receive_id = 0x0166;
        break;
    case BLUE_STANDARD_1:
        send_id = 103;
        receive_id = 0x0167;
        break;
    case BLUE_STANDARD_2:
        send_id = 104;
        receive_id = 0x0168;
        break;
    case BLUE_STANDARD_3:
        send_id = 105;
        receive_id = 0x0169;
        break;
    case BLUE_AERIAL:
        send_id = 106;
        receive_id = 0x016A;
        break;
    default:
        send_id = 0;
        receive_id = 0;
        break;
    }
}

void draw_servo_bull(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_character_t draw_init;
    draw_init.header_data_t.data_cmd_id = 0x0110;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct.graphic_name[0] = 0;
    draw_init.grapic_data_struct.graphic_name[1] = 0;
    draw_init.grapic_data_struct.graphic_name[2] = 49;
    draw_init.grapic_data_struct.operate_tpye = op_type;
    draw_init.grapic_data_struct.graphic_tpye = 7;
    draw_init.grapic_data_struct.layer = 2;
    draw_init.grapic_data_struct.color = 8;
    draw_init.grapic_data_struct.start_angle = FONT_SIZE; //字符大小

    draw_init.grapic_data_struct.width = 2;
    draw_init.grapic_data_struct.start_x = LEFT_STATE_AREA_X;
    draw_init.grapic_data_struct.start_y = LEFT_STATE_AREA_Y - 2 * LEFT_STATE_DY;
    draw_init.grapic_data_struct.end_x = 0;
    draw_init.grapic_data_struct.end_y = 0;
    draw_init.grapic_data_struct.radius = 0;

    memset(draw_init.data, 0, sizeof(draw_init.data));

    draw_init.grapic_data_struct.end_angle = 9; //字符长度
    draw_init.data[0] = 'S';
    draw_init.data[1] = 'E';
    draw_init.data[2] = 'R';
    draw_init.data[3] = 'V';
    draw_init.data[4] = 'O';
    draw_init.data[5] = ' ';
    draw_init.data[6] = 'O';
    draw_init.data[7] = 'F';
    draw_init.data[8] = 'F';

    if (servo_flag)
    {
        draw_init.grapic_data_struct.end_angle = 8; //字符长度

        draw_init.data[6] = 'O';
        draw_init.data[7] = 'N';
        draw_init.data[8] = 0;
    }

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}

void draw_top_speed(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type)
{
    ext_client_custom_character_t draw_init;
    draw_init.header_data_t.data_cmd_id = 0x0110;
    draw_init.header_data_t.sender_ID = sender_id;
    draw_init.header_data_t.receiver_ID = receiver_id;

    draw_init.grapic_data_struct.graphic_name[0] = 0;
    draw_init.grapic_data_struct.graphic_name[1] = 0;
    draw_init.grapic_data_struct.graphic_name[2] = 50;
    draw_init.grapic_data_struct.operate_tpye = op_type;
    draw_init.grapic_data_struct.graphic_tpye = 7;
    draw_init.grapic_data_struct.layer = 2;
    draw_init.grapic_data_struct.color = 8;
    draw_init.grapic_data_struct.start_angle = FONT_SIZE; //字符大小

    draw_init.grapic_data_struct.width = 2;
    draw_init.grapic_data_struct.start_x = LEFT_STATE_AREA_X;
    draw_init.grapic_data_struct.start_y = LEFT_STATE_AREA_Y - LEFT_STATE_DY;
    draw_init.grapic_data_struct.end_x = 0;
    draw_init.grapic_data_struct.end_y = 0;
    draw_init.grapic_data_struct.radius = 0;

    memset(draw_init.data, 0, sizeof(draw_init.data));

    if (n == 0)
    {
        draw_init.grapic_data_struct.end_angle = 3; //字符长度
        draw_init.data[0] = 'L';
        draw_init.data[1] = 'O';
        draw_init.data[2] = 'W';
    }
    else
    {
        draw_init.grapic_data_struct.end_angle = 4; //字符长度
        draw_init.data[0] = 'H';
        draw_init.data[1] = 'I';
        draw_init.data[2] = 'G';
        draw_init.data[3] = 'H';
    }

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&draw_init, sizeof(draw_init));
}
