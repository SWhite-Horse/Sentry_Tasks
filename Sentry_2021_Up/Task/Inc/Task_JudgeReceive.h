#ifndef __TASKJUDGERECEIVE_H
#define __TASKJUDGERECEIVE_H

#include "Task_init.h"
extern uint8_t Turn_sign;


/*--------------CmdID(2-Byte)----------------*/
#define GAME_STATE              0X0001          //比赛状态数据，1Hz 周期发送
#define GAME_RESULT             0X0002          //比赛结果数据，比赛结束后发送
#define ROBOT_HP                0X0003          //比赛机器人血量数据，1Hz周期发送
#define DART_STATUS             0X0004          //飞镖发射状态，飞镖发射后发送
#define EVENT_DATA              0X0101          //场地事件数据，事件改变后发送
#define SUPPLY_ACTION           0X0102          //场地补给站动作标识数据，动作改变后发送
#define REFEREE_WARNING         0X0104          //裁判警告数据，警告发生后发送 
#define DART_REMAINING_TIME     0X0105          //飞镖发射口倒计时，1Hz 周期发送 
#define GAME_ROBOT_STATE        0X0201          //机器人状态数据，10Hz 周期发送
#define POWER_HEAT_DATA         0X0202          //实时功率热量数据，50Hz 周期发送
#define GAME_ROBOT_POS          0X0203          //机器人位置数据，10Hz 发送
#define BUFF_MUSK               0X0204          //机器人增益数据，增益状态改变后发送
#define AERIAL_ROBOT_ENERGY     0X0205          //空中机器人能量状态数据，10Hz 周期发送，只有空中机器人主控发送
#define ROBOT_HURT              0X0206          //伤害状态数据，伤害发生后发送
#define SHOOT_DATA              0X0207          //实时射击数据，子弹发射后发送
#define REMAIN_BULLET           0X0208          //子弹剩余发送数，空中机器人以及哨兵机器人发送，1Hz周期发送
#define RFID_STATE              0X0209          //机器人 RFID 状态，1Hz 周期发送
#define DART_CLIENT_CMD         0X020A          //飞镖机器人客户端指令数据，10Hz周期发送
#define INTERACTIVE_HEADER      0X0301          //机器人间交互数据，发送方触发发送，上限10Hz
#define VIDEO_TRANSMITTER       0X0304          //键盘、鼠标信息，通过图传串口发送
/*--------------CmdID(2-Byte)----------------*/

/*--------------DataSize----------------*/
#define GAME_STATE_DATA_SIZE            (11)
#define GAME_RESULT_DATA_SIZE           (1)
#define ROBOT_HP_DATA_SIZE              (32)
#define DART_STATUS_DATE_SIZE           (3)
#define EVENTDATA_DATA_SIZE             (4)
#define SUPPLY_ACTION_DATA_SIZE         (4)
#define REFEREE_WARNING_DATA_SIZE       (2)
#define DART_REMAINING_TIME_DATA_SIZE   (1)
#define GAMEROBOT_STATE_DATA_SIZE       (27)
#define POWER_HEAT_DATA_SIZE            (16)
#define GAME_ROBOT_POS_DATA_SIZE        (16)
#define BUFF_DATA_SIZE                  (1)
#define AERIAL_ROBOT_ENERGY_DATA_SIZE   (1)
#define ROBOT_HURT_DATA_SIZE            (1)
#define SHOOTDATA_DATA_SIZE             (7)
#define REMAIN_BULLET_DATA_SIZE         (6)
#define RFID_STATE_DATA_SIZE            (4)
#define DART_CLIENT_CMD_DATA_SIZE       (12)
//#define DIY_CONTROL_DATA_SIZE           (?)//长度30以内自定
//#define MINI_MAP_DATA_SIZE              (15)
#define VIDEO_TRANSMITTER_DATA_SIZE     (12)

#define INTERACTIVEHEADER_DATA_SIZE(n) (n + 9)
#define JUDGE_DATA_LENGTH(n) (n + 9)
/*--------------DataSize----------------*/

/*--------------偏移位置----------------*/
//接收数据
#define JUDGE_SOF_OFFSET (0)
#define JUDGE_DATALENGTH_OFFSET (1)
#define JUDGE_SEQ_OFFSET (3)
#define JUDGE_CRC8_OFFSET (4)
#define JUDGE_CMDID_OFFSET (5)
#define JUDGE_DATA_OFFSET (7)
#define JUDGE_CRC16_OFFSET(n) (n + JUDGE_DATA_OFFSET)
//发送数据
#define TRAMSINIT_LENGTH 128
#define TRAMSINIT_HEAD_OFFSET 0
#define TRAMSINIT_SENDID_OFFSET 2
#define TRAMSINIT_CLIENT_OFFSET 4
/*--------------偏移位置----------------*/


/*-------------------------------------------CRC校验---------------------------------------------------*/
/**
  * @brief  裁判系统数据校验
  * @param  __RECEIVEBUFFER__：  接收到的裁判系统数据头帧所在地址
  * @param  __DATALENGTH__：     一帧数据内的数据量/Bytes（内容）
  * @retval 1：                  校验正确
  * @retval 0：                  校验错误
  * @note	None
  */
#define Verify_CRC_Check_Sum(__RECEIVEBUFFER__, __DATALENGTH__)      (Verify_CRC8_Check_Sum(__RECEIVEBUFFER__, JUDGE_CRC8_OFFSET+1)\
                                                    &&Verify_CRC16_Check_Sum(__RECEIVEBUFFER__, __DATALENGTH__+JUDGE_DATA_LENGTH(0)))

/**
  * @brief  裁判系统添加校验
  * @param  __TRANSMITBUFFER__： 发送到裁判系统的数据中头帧所在地址
  * @param  __DATALENGTH__：     一帧数据内的数据量/Bytes（内容）
  * @retval None
  * @note	None
  */
#define Append_CRC_Check_Sum(__TRANSMITBUFFER__, __DATALENGTH__)                    \
do                                                                                  \
{                                                                                   \
    Append_CRC8_Check_Sum(__TRANSMITBUFFER__, JUDGE_CRC8_OFFSET+1);                 \
    Append_CRC16_Check_Sum(__TRANSMITBUFFER__, __DATALENGTH__+JUDGE_DATA_LENGTH(0));\
}while(0U)



//1.	比赛机器人状态(0x0001)
typedef __packed struct
{
    /*
    0-3 bit：比赛类型
    • 1：RoboMaster 机甲大师赛；
    • 2：RoboMaster 机甲大师单项赛；
    • 3：ICRA RoboMaster 人工智能挑战赛
    • 4：RoboMaster 联盟赛3V3
    • 5：RoboMaster 联盟赛1V1
    */
    uint8_t game_type : 4;

    /*
    4-7 bit：当前比赛阶段
    • 0：未开始比赛；
    • 1：准备阶段；
    • 2：自检阶段；
    • 3：5s倒计时；
    • 4：对战中；
    • 5：比赛结算中
    */
    uint8_t game_progress : 4;

    /*
    当前阶段剩余时间，单位 s
    */
    uint16_t stage_remain_time;

    /*
    机器人接收到该指令的精确Unix时间，当机载端收到有效的NTP服务器授时后生效
    */
   uint64_t SyncTimeStamp;

} ext_game_state_t;

//2.比赛结果数据：0x0002。发送频率：比赛结束后发送
typedef __packed struct
{
    /*0 平局 1 红方胜利 2 蓝方胜利*/
    uint8_t winner;
} ext_game_result_t;

//3. 机器人血量数据：0x0003。发送频率：1Hz
typedef __packed struct
{
    uint16_t red_1_robot_HP;    
    uint16_t red_2_robot_HP;    
    uint16_t red_3_robot_HP;    
    uint16_t red_4_robot_HP;    
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;    
    uint16_t red_outpost_HP;   
    uint16_t red_base_HP;    

    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;    
    uint16_t blue_3_robot_HP;    
    uint16_t blue_4_robot_HP;    
    uint16_t blue_5_robot_HP;    
    uint16_t blue_7_robot_HP;    
    uint16_t blue_outpost_HP;   
    uint16_t blue_base_HP; 
} ext_game_robot_HP_t;

//4. 飞镖发射状态：0x0004。发送频率：飞镖发射后发送，发送范围：所有机器人。 
typedef __packed struct 
{   /*
    发射飞镖的队伍：
    1：红方飞镖
    2：蓝方飞镖
    */
    uint8_t dart_belong;    
    /*
    发射时的剩余比赛时间，单位s
    */
    uint16_t stage_remaining_time;  
} ext_dart_status_t; 

//5.人工智能挑战赛加成与惩罚区状态：0x0005。发送频率：1Hz周期发送，发送范围：所有机器人
//暂时用不到，就没写

//6.场地事件数据：0x0101。发送频率：事件改变后发送
typedef __packed struct
{
    /*
    bit 0-2：
    bit 0：己方补给站 1号补血点占领状态 1为已占领；
    bit 1：己方补给站 2号补血点占领状态 1为已占领；
    bit 2：己方补给站 3号补血点占领状态 1为已占领；

    bit 3-5：己方能量机关状态：
    • bit 3为打击点占领状态，1为占领；
    • bit 4为小能量机关激活状态，1为已激活；
    • bit 5为大能量机关激活状态，1为已激活；

    bit 6：己方R2环形高地占领状态 1为已占领；
    bit 7：己方R3梯形高地占领状态 1为已占领；
    bit 8：己方R4梯形高地占领状态 1为已占领；

    bit 9：己方基地护盾状态：
    • 1 为基地有虚拟护盾血量；
    • 0为基地无虚拟护盾血量；

    bit 10：己方前哨战状态：
    • 1为前哨战存活；
    • 0为前哨战被击毁；

    bit 10 -31: 保留
    */
    uint32_t event_type;
} ext_event_data_t;

//7. 补给站动作标识：0x0102。发送频率：动作改变后发送。发送范围：己方机器人
typedef __packed struct
{
    /*
    补给站口 ID：
    1：1 号补给口；
    2：2 号补给口
    */
    uint8_t supply_projectile_id;

    /*
    补弹机器人 ID：0 为当前无机器人补弹，1 为红方英雄机器人补弹，2 为红方工程
    机器人补弹，3/4/5 为红方步兵机器人补弹，101 为蓝方英雄机器人补弹，102 为蓝方
    工程机器人补弹，103/104/105 为蓝方步兵机器人补弹
    */
    uint8_t supply_robot_id;

    /*
    出弹口开闭状态：0 为关闭，1 为子弹准备中，2 为子弹下落
    */
    uint8_t supply_projectile_step;

    /*
    补弹数量：
    50：50 颗子弹；
    100：100 颗子弹；
    150：150 颗子弹；
    200：200 颗子弹。
    */
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

//8. 裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送，发送范围：己方机器人。 
typedef __packed struct 
{   
    /*
    警告等级：
    1：黄牌
    2：红牌
    3：判负
    */
    uint8_t level;
    /*
    犯规机器人ID：
    判负时，机器人ID为0
    黄牌、红牌时，机器人ID为犯规机器人ID
    */
    uint8_t foul_robot_id;  
} ext_referee_warning_t; 

//9. 飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人。 
typedef __packed struct 
{   
    uint8_t dart_remaining_time;   //15s 倒计时 
} ext_dart_remaining_time_t; 

//10.比赛机器人状态：0x0201。发送频率：10Hz
typedef __packed struct
{
    /*
    机器人 ID：
    1：红方英雄机器人；
    2：红方工程机器人；
    3/4/5：红方步兵机器人；
    6：红方空中机器人；
    7：红方哨兵机器人；
    8：红方飞镖机器人；
    9：红方雷达站；  
    101：蓝方英雄机器人；
    102：蓝方工程机器人；
    103/104/105：蓝方步兵机器人；
    106：蓝方空中机器人；
    107：蓝方哨兵机器人。
    108：蓝方飞镖机器人； 
    109：蓝方雷达站。 
    */
    uint8_t robot_id;

    /*
    机器人等级：
    1：一级；2：二级；3：三级。
    */
    uint8_t robot_level;

    /*
    机器人剩余血量
    */
    uint16_t remain_HP;

    /*
    机器人上限血量
    */
    uint16_t max_HP;

    /*
    机器人1号17mm枪口每秒冷却值
    */
    uint16_t shooter_id1_17mm_cooling_rate;

    /*
    机器人1号17mm枪口热量上限
    */
    uint16_t shooter_id1_17mm_cooling_limit;

    /*
    机器人1号17mm 枪口上限速度 单位 m/s
    */
    uint16_t shooter_id1_17mm_speed_limit;

    /*
    机器人2号17mm枪口每秒冷却值
     */
    uint16_t shooter_id2_17mm_cooling_rate;

    /*
    机器人2号17mm枪口热量上限
    */
    uint16_t shooter_id2_17mm_cooling_limit; 

    /*
    机器人2号17mm 枪口上限速度 单位 m/s
    */
    uint16_t shooter_id2_17mm_speed_limit;   

    /*
    机器人42mm枪口每秒冷却值
    */
    uint16_t shooter_id1_42mm_cooling_rate; 

    /*
    机器人42mm枪口热量上限
    */
    uint16_t shooter_id1_42mm_cooling_limit;

    /*
    机器人42mm枪口上限速度 单位 m/s
    */
    uint16_t shooter_id1_42mm_speed_limit;

    /*
    机器人底盘功率限制上限
    */
    uint16_t chassis_power_limit; 

    /*
    主控电源输出情况：
    0 bit：gimbal口输出： 1为有24V输出，0为无24v输出；
    1 bit：chassis口输出：1为有24V输出，0为无24v输出；
    2 bit：shooter口输出：1为有24V输出，0为无24v输出；
    */
    uint8_t mains_power_gimbal_output : 1; 
    uint8_t mains_power_chassis_output : 1; 
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

//11.实时功率热量数据：0x0202。发送频率：50Hz
typedef __packed struct
{
    //底盘输出电压 单位 毫伏
    uint16_t chassis_volt;
    //底盘输出电流 单位 毫安
    uint16_t chassis_current;
    //底盘输出功率 单位 W 瓦
    float chassis_power;
    //底盘功率缓冲 单位 J 焦耳
    uint16_t chassis_power_buffer;
    //1号17mm 枪口热量
    uint16_t shooter_id1_17mm_cooling_heat;
    //2号17mm枪口热量
    uint16_t shooter_id2_17mm_cooling_heat;
    //42mm 枪口热量
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

//12.机器人位置：0x0203。发送频率：10Hz
typedef __packed struct
{
    float x;   //位置 x 坐标，单位 m
    float y;   //位置 y 坐标，单位 m
    float z;   //位置 z 坐标，单位 m
    float yaw; //位置枪口，单位度
} ext_game_robot_pos_t;

//13. 机器人增益：0x0204。发送频率：1Hz
typedef __packed struct
{
    /*
    bit 0：机器人血量补血状态
    bit 1：枪口热量冷却加速
    bit 2：机器人防御加成
    bit 3：机器人攻击加成
    其他 bit 保留
    */
    uint8_t power_rune_buff;
} ext_buff_t;

//14. 空中机器人能量状态：0x0205。发送频率：10Hz
typedef __packed struct
{
    uint8_t attack_time;  //可攻击时间 单位 s。30s 递减至 0
} aerial_robot_energy_t;

//15. 伤害状态：0x0206。发送频率：伤害发生后发送
typedef __packed struct
{
    /*
    bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人
    的五个装甲片，其他血量变化类型，该变量数值为 0。
    */
    uint8_t armor_id : 4;
    /*bit 4-7：血量变化类型
    0x0 装甲伤害扣血；
    0x1 模块掉线扣血；
    0x2 超射速扣血；
    0x3 超枪口热量扣血；
    0x4 超底盘功率扣血；
    0x5 装甲撞击扣血。
    */
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

//16. 实时射击信息：0x0207。发送频率：射击后发送
typedef __packed struct
{
    /*子弹类型: 1：17mm弹丸 2：42mm弹丸*/
    uint8_t bullet_type; 
    /*
    发射机构ID：
    1：1号17mm发射机构
    2：2号17mm发射机构
    3：42mm 发射机构
    */
    uint8_t shooter_id; 
    /*子弹射频 单位 Hz*/
    uint8_t bullet_freq; 
    /*子弹射速 单位 m/s*/
    float bullet_speed; 
} ext_shoot_data_t;

//17. 子弹剩余发射数：0x0208。发送频率：10Hz周期发送，所有机器人发送 
typedef __packed struct 
{   
    uint16_t bullet_remaining_num_17mm;//17mm子弹剩余发射数目
    uint16_t bullet_remaining_num_42mm;//42mm子弹剩余发射数目
    uint16_t coin_remaining_num;//剩余金币数量
} ext_bullet_remaining_t;

//18. 机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人。 
typedef __packed struct 
{   
    /*
    bit 0：基地增益点 RFID 状态； 
    bit 1：高地增益点 RFID 状态； 
    bit 2：能量机关激活点 RFID 状态； 
    bit 3：飞坡增益点 RFID 状态； 
    bit 4：前哨岗增益点 RFID 状态； 
    bit 5：资源岛增益点 RFID 状态； 
    bit 6：补血点增益点 RFID 状态； 
    bit 7：工程机器人补血卡 RFID 状态； 
    bit 8-31：保留
    RFID 状态不完全代表对应的增益或处罚状态，例如敌方已占领的高地增益点，不能获取对应的增益效果。
    */
    uint32_t rfid_status;
} ext_rfid_status_t; 

//19. 飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人
typedef __packed struct 
{ 
    /*
    当前飞镖发射口的状态
    0：关闭；
    1：正在开启或者关闭中
    2：已经开启
    */
    uint8_t dart_launch_opening_status;
    /*
    飞镖的打击目标，默认为前哨站；
    1：前哨站；
    2：基地。
    */
    uint8_t dart_attack_target;
    /*切换打击目标时的比赛剩余时间，单位秒，从未切换默认为0。*/
    uint16_t target_change_time;
    /*检测到的第一枚飞镖速度，单位 0.1m/s/LSB, 未检测是为0。*/
    uint8_t first_dart_speed;
    /*检测到的第二枚飞镖速度，单位 0.1m/s/LSB，未检测是为0。*/
    uint8_t second_dart_speed;
    /*检测到的第三枚飞镖速度，单位 0.1m/s/LSB，未检测是为0。*/
    uint8_t third_dart_speed;
    /*检测到的第四枚飞镖速度，单位 0.1m/s/LSB，未检测是为0。*/
    uint8_t fourth_dart_speed;
    /*最近一次的发射飞镖的比赛剩余时间，单位秒，初始值为0。*/
    uint16_t last_dart_launch_time;
    /*最近一次操作手确定发射指令时的比赛剩余时间，单位秒, 初始值为0。*/
    uint16_t operate_launch_cmd_time; 
} ext_dart_client_cmd_t;

//20.图传遥控信息标识：0x0304。发送频率：30Hz。
//2021赛季新增功能，据说比遥控器链路稳定，没用过
typedef __packed struct
{
    /*鼠标X轴信息*/
    int16_t mouse_x;
    /*鼠标Y轴信息*/
    int16_t mouse_y;
    /*鼠标滚轮信息*/
    int16_t mouse_z;
    /*鼠标左键按下*/
    int8_t left_button_down;
    /*鼠标右键按下*/
    int8_t right_button_down;
    /*
    键盘信息
    bit 0：键盘W是否按下
    bit 1：键盘S是否按下
    bit 2：键盘A是否按下
    bit 3：键盘D是否按下
    bit 4：键盘SHIFT是否按下
    bit 5：键盘CTRL是否按下
    bit 6：键盘Q是否按下
    bit 7：键盘E是否按下
    bit 8：键盘R是否按下
    bit 9：键盘F是否按下
    bit 10：键盘G是否按下
    bit 11：键盘Z是否按下
    bit 12：键盘X是否按下
    bit 13：键盘C是否按下
    bit 14：键盘V是否按下
    bit 15：键盘B是否按下
    */
    uint16_t keyboard_value;
    /*保留位*/
    uint16_t reserved;
} ext_video_transmitter_t; 


extern ext_game_state_t ext_game_state;
extern ext_game_result_t ext_game_result;
extern ext_game_robot_HP_t ext_game_robot_HP;
extern ext_dart_status_t ext_dart_status;
extern ext_event_data_t ext_event_data;
extern ext_supply_projectile_action_t ext_supply_projectile_action;
extern ext_referee_warning_t ext_referee_warning;
extern ext_dart_remaining_time_t ext_dart_remaining_time;
extern ext_game_robot_state_t ext_game_robot_state;
extern ext_power_heat_data_t ext_power_heat_data;
extern ext_game_robot_pos_t ext_game_robot_pos;
extern ext_buff_t ext_buff;
extern aerial_robot_energy_t aerial_robot_energy;
extern ext_robot_hurt_t ext_robot_hurt;
extern ext_shoot_data_t ext_shoot_data;
extern ext_bullet_remaining_t ext_bullet_remaining;
extern ext_rfid_status_t ext_rfid_status;
extern ext_dart_client_cmd_t ext_dart_client_cmd;
extern ext_video_transmitter_t ext_video_transmitter;


void JudgeConnection_Init(UART_HandleTypeDef *huart);
void Judge_IDLECallback(UART_HandleTypeDef *huart);
void JudgeTransmit(void);
void JudgeReceive(void);
void Judge_Receive_Data_Processing(uint8_t SOF, uint16_t CmdID);

/*--------------------------------------------------校验函数--------------------------------------------------*/
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
/*--------------------------------------------------校验函数--------------------------------------------------*/

#endif
