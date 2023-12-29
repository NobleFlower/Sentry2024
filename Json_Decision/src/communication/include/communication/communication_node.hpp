#include <ros/ros.h>
#include "global_interface/CarPos.h"
#include "global_interface/GameInfo.h"
#include "global_interface/ObjHP.h"
#include "global_interface/Serial.h"

class Receive {
private:
    ros::NodeHandle nh;
    ros::Publisher GameInfo_pub_;
    ros::Publisher ObjHP_pub_;
    ros::Publisher Serial_pub_;
public:
    Receive();
    void Re();
    void Re_GameInfo();
    void Re_ObjHP();
    void Re_CarPos();
    void Re_Serial();
    void Re_Remaining_bullets();
};

/**
 * @brief 比赛相关信息
 */
typedef struct {
    // 剩余时间
    global_interface::GameInfo::_timestamp_type TimeStamp;
    // 当前比赛阶段
    global_interface::GameInfo::_game_stage_type GameStage;
}Game_Info_t;

/**
 * @brief 场上兵种/设施血量 
 */
typedef struct {
    uint16_t r_Hero_HP;       //!<@brief 红方英雄机器人血量
    uint16_t r_Engineer_HP;   //!<@brief 红方工程机器人血量
    uint16_t r_Infantry_3_HP; //!<@brief 红方三号步兵机器人血量
    uint16_t r_Infantry_4_HP; //!<@brief 红方四号步兵机器人血量
    uint16_t r_Infantry_5_HP; //!<@brief 红方五号步兵机器人血量
    uint16_t r_Sentry_HP;     //!<@brief 红方哨兵机器人血量
    uint16_t r_Outpost_HP;    //!<@brief 红方前哨站血量
    uint16_t r_Base_HP;       //!<@brief 红方基地血量

    uint16_t b_Hero_HP;       //!<@brief 蓝方英雄机器人血量
    uint16_t b_Engineer_HP;   //!<@brief 蓝方工程机器人血量
    uint16_t b_Infantry_3_HP; //!<@brief 蓝方三号步兵机器人血量
    uint16_t b_Infantry_4_HP; //!<@brief 蓝方四号步兵机器人血量
    uint16_t b_Infantry_5_HP; //!<@brief 蓝方五号步兵机器人血量
    uint16_t b_Sentry_HP;     //!<@brief 蓝方哨兵机器人血量
    uint16_t b_Outpost_HP;    //!<@brief 蓝方前哨站血量
    uint16_t b_Base_HP;       //!<@brief 蓝方基地血量
}ObjHP_t;

typedef struct {
    
}Serial_t;

