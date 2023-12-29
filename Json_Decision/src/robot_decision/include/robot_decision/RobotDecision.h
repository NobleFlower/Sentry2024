#ifndef _RD_DECISION_H
#define _RD_DECISION_H

#include "./structs.h"

/**
 * @brief 裁判系统信息处理类
 * 处理存储比赛信息
 */
class GameHandler {
private:
    std::chrono::time_point<std::chrono::system_clock,
                            std::chrono::microseconds>
        lastUpdateTime;
    int gameTime = -1;

public:
    GameHandler();
    ~GameHandler();

    void update(int &gameTime);
};
/**
 * @brief 决策系统类
 * 提供机器人决策相关处理接口
 */
class RobotDecisionSys {
private:
    float       _REAL_WIDTH;
    float       _REAL_HEIGHT;
    float       _STEP_DISTANCE;
    float       _CAR_SEEK_FOV;

private:
    std::vector<std::shared_ptr<WayPoint>> wayPointMap;
    std::vector<std::shared_ptr<Decision>> decisions;

    float _distance_THR = 0.;
    float _seek_THR = 5.0;

private:
    std::map<int, std::vector<int>> connection_map;
    std::shared_ptr<GameHandler>    myGameHandler = nullptr;

private:
    /**
     * @brief 计算机器人当前所在路径点
     * @param pos
     * 当前机器人坐标信息
     */
    int calculatePosition(RobotPosition &pos);

public:
    RobotDecisionSys(float &_distance_THR, float &_seek_THR, float &_REAL_WIDTH,
                     float &_REAL_HEIGHT, float &_STEP_DISTANCE, float &_CAR_SEEK_FOV);
    ~RobotDecisionSys();

    /**
     * @brief 解码路径点Json文件
     * @param filePath
     * 路径点Json文件地址
     * @return
     * 是否成功
     */
    bool decodeWayPoints(std::string &filePath);
    /**
     * @brief 解码决策Json文件
     * @param filePath
     * 决策Json文件地址
     * @return
     * 是否成功
     */
    bool decodeDecisions(std::string &filePath);

    /**
     * @brief 检查机器人当前所在路径点
     * @param x
     * 机器人坐标x
     * @param y
     * 机器人坐标y
     * @return
     * 路径点ID
     */
    int checkNowWayPoint(float x, float y);
    /**
     * @brief 检查机器人当前所在路径点
     * @param pos
     * 当前机器人坐标信息
     * @return
     * 路径点ID
     */
    int checkNowWayPoint(RobotPosition pos);
    /**
     * @brief 机器人决策
     * @param wayPointID
     * 当前机器人路径点ID
     * @param robot_mode
     * 当前机器人模式
     * @param _HP
     * 当前机器人血量
     * @param nowtime
     * 当前比赛剩余时间
     * @param now_out_post_HP
     * 当前前哨站血量
     * @param now_base_HP
     * 当前基地血量
     * @param friendPositions
     * 友方机器人位置（真实坐标）
     * @param enemtPositions
     * 敌方机器人位置（真实坐标）
     * @param availableDecisionID
     * 输入输出符合条件的决策ID
     * @param id_pos_f
     * 友方位置（路径点）
     * @param id_pos_e
     * 敌方位置（路径点）
     * @return
     * 决策
     */
    std::shared_ptr<Decision> decide(
        int wayPointID, int robot_mode, int _HP, int nowtime,
        int now_out_post_HP, int now_base_HP,
        std::vector<RobotPosition> &friendPositions,
        std::vector<RobotPosition> &enemyPositions,
        std::vector<int> &availableDecisionID, std::map<int, int> &id_pos_f,
        std::map<int, int> &id_pos_e);
    /**
     * @brief 根据ID获取路径点
     * @param id
     * 路径点ID
     * @return
     * 路径点
     */
    std::shared_ptr<WayPoint> getWayPointByID(int id);
    /**
     * @brief 根据ID获取决策
     * @param id
     * 决策ID
     * @return
     * 决策
     */
    std::shared_ptr<Decision> getDecisionByID(int id);

    /**
     * @brief 决策打击目标
     * @param _IsBlue
     * 是否是蓝方
     * @param mypos
     * 当前机器人位置
     * @param enemyPositions
     * 敌方机器人位置
     * @param detectedEnemy
     * 检测到的敌方目标ID
     * @param enemyHP
     * 敌方血量(仅填入车辆血量)
     * @param distance_weight
     * 距离权重
     * @param hp_weight
     * 血量权重
     * @return
     * 打击权重
     */
    std::map<int, float> decideAimTarget(
        bool _IsBlue, RobotPosition &mypos,
        std::vector<RobotPosition> &enemyPositions, std::vector<int> &enemyHP,
        float distance_weight_ratio, float hp_weight_ratio,
        bool enemyOutpostDown);
    /**
     * @brief 获取距离阈值，用于计算路径点
     * @return
     * 距离阈值
     */
    float getDistanceTHR();
    /**
     * @brief 设置距离阈值，用于计算路径点
     * @param thr
     * 距离阈值
     */
    void setDistanceTHR(float thr);
    /**
     * @brief 获取距离阈值，用于索敌
     * @return
     * 距离阈值
     */
    float getSeekTHR();
    /**
     * @brief 设置距离阈值，用于索敌
     * @param thr
     * 距离阈值
     */
    void setSeekTHR(float thr);
    
};
#endif