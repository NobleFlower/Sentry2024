/*
 * @Author: NobleFlower 1619239422@qq.com
 * @Date: 2023-11-21 20:27:39
 * @LastEditors: NobleFlower 1619239422@qq.com
 * @LastEditTime: 2023-12-15 20:47:19
 * @FilePath: /Sentry2024/Json_Decision/src/robot_decision/include/robot_decision/structs.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _RD_STRUCTS_H
#define _RD_STRUCTS_H

#include "./public.h"

    /**
     * @brief 机器人位置信息
     */
    typedef struct RobotPosition
    {
        int robot_id = -1;
        double curTime;
        float vehicleX;
        float vehicleY;
        float vehicleZ;
        RobotPosition(){};
        RobotPosition(int id, const nav_msgs::Odometry::ConstPtr& pose)
        {
            this->robot_id = id;
            this->curTime = pose->header.stamp.toSec();
            this->vehicleX = pose->pose.pose.position.x;
            this->vehicleY = pose->pose.pose.position.y;
            this->vehicleZ = pose->pose.pose.position.z;
        };
    } RobotPosition;

    /**
     * @brief 路径点信息
     */
    typedef struct WayPoint
    {
        int id;
        int type;
        geometry_msgs::Point point;
        std::map<int, int> enemyWeights;
        std::vector<int> connection;
        WayPoint(){};
        WayPoint(int id, double x, double y, double z)
        {
            this->id = id;
            this->point.x = x;
            this->point.y = y;
            this->point.z = z;
        };
    } WayPoint;

    /**
     * @brief 决策信息
     */
    typedef struct Decision
    {
        bool if_auto = true;
        int id;
        const char *name;
        std::vector<int> wayPointID;
        int weight;
        // condition
        int robot_mode;
        int start_time;
        int end_time;
        int _minHP;
        int _maxHP;
        int out_post_HP_min;
        int out_post_HP_max;
        int base_HP_min;
        std::vector<std::vector<int>> enemy_position;
        std::vector<std::vector<int>> friend_position;
        // decision
        int decide_mode;
        int decide_wayPoint;
        bool if_succession;
        bool if_reverse;
    } Decision;

    enum Mode
    {
        AUTOAIM = 8,
        MANUAL_ATTACK = 8,
        MANUAL_BACKDEFENSE = 11
    };

    enum GameStage
    {
        COMPETITON_NOT_STARTED = 0,
        PREPARATION_STAGE = 1,
        SELF_INSPECTION_STAGE = 2,
        FIVE_S_COUNTDOWN = 3,
        IN_BATTLE = 4,
        END = 5
    };


#endif