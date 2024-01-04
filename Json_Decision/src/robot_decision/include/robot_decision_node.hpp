#ifndef _ROBOT_DECISION_NODE_HPP_
#define _ROBOT_DECISION_NODE_HPP_

#include "./robot_decision/RobotDecision.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/smart_ptr/scoped_ptr.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <memory>
#include <string>
#include <tf2/utils.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "global_interface/Autoaim.h"
#include "global_interface/CarPos.h"
#include "global_interface/Decision.h"
#include "global_interface/Detection.h"
#include "global_interface/DetectionArray.h"
#include "global_interface/GameInfo.h"
#include "global_interface/ModeSet.h"
#include "global_interface/ObjHP.h"
#include "global_interface/Point2f.h"
#include "global_interface/Point3f.h"
#include "global_interface/Serial.h"
#include "global_interface/StrikeLicensing.h"

#include "nav_msgs/Odometry.h"
#include "ros/duration.h"
#include "ros/node_handle.h"
#include "ros/time.h"
#include "ros/timer.h"
#include "ros/wall_timer.h"

using namespace std::chrono_literals;

class RobotDecisionNode {
public:
    bool _Debug = false;
    bool _auto_mode = true;

    double delta_yaw = 0;

private:
    std::string _WayPointsPath;
    std::string _DecisionsPath;
    float _INIT_DISTANCE_THR;
    float _INIT_SEEK_THR;
    bool _INIT_IsBlue;
    int _INIT_SELFINDEX;
    int _INIT_FRIENDOUTPOSTINDEX;
    int _INIT_FRIENDBASEINDEX;
    int _GAME_TIME;
    int _TIME_THR;

    // std::string _MAP_PATH;
    float _REAL_WIDTH;
    float _REAL_HEIGHT;
    float _STEP_DISTANCE;
    float _CAR_SEEK_FOV;
private:
    // RealParamValues:
    int _selfIndex = 5;
    int _friendOutPostIndex = 6;
    int _friendBaseIndex = 7;
    bool _IsBlue = false;

    // TempParams:
    float _distance_THR_Temp = 0.5;
    float _seek_THR_Temp = 5.0;
    bool _IsBlue_Temp = false;

    const std::map<std::string, int> type_id = {
        std::map<std::string, int>::value_type("R1", 0),
        std::map<std::string, int>::value_type("R2", 1),
        std::map<std::string, int>::value_type("R3", 2),
        std::map<std::string, int>::value_type("R4", 3),
        std::map<std::string, int>::value_type("R5", 4),
        std::map<std::string, int>::value_type("R7", 5),
        std::map<std::string, int>::value_type("B1", 6),
        std::map<std::string, int>::value_type("B2", 7),
        std::map<std::string, int>::value_type("B3", 8),
        std::map<std::string, int>::value_type("B4", 9),
        std::map<std::string, int>::value_type("B5", 10),
        std::map<std::string, int>::value_type("B7", 11)};

    const std::vector<int> _car_ids = {0, 1, 2, 3, 4, 6};

    const double PI = 3.1415926;

    double waypointXYRadius = 0.5;
    double waypointZBound = 5.0;
    double waitTime = 0;
    double waitTimeStart = 0;
    bool isWaiting = false;
    double frameRate = 5.0;  // 帧率
    double speed = 1.0;
    bool sendSpeed = true;
    bool sendBoundary = true;
    

private:
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    ros::Publisher  decision_pub_;
    ros::Publisher  strikeLicensing_pub_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber modeSet_sub_;

    ros::Subscriber subPose;
    ros::Publisher pubWaypoint;
    ros::Publisher pubSpeed;
    geometry_msgs::PointStamped waypointMsgs;
    std_msgs::Float32 speedMsgs;

    ros::Timer  timer_;

    message_filters::Subscriber<global_interface::ObjHP> objHP_sub_;
    message_filters::Subscriber<global_interface::CarPos> carPos_sub_;
    message_filters::Subscriber<global_interface::GameInfo> gameInfo_sub_;
    message_filters::Subscriber<global_interface::Serial> serial_sub_;

    typedef message_filters::sync_policies::ApproximateTime<global_interface::ObjHP, 
                                                            global_interface::CarPos,
                                                            global_interface::GameInfo, 
                                                            global_interface::Serial>
        ApproximateSyncPolicy;
    typedef message_filters::Synchronizer<ApproximateSyncPolicy> sync;
    boost::scoped_ptr<sync> TS_sync_;
private:
    std::shared_ptr<RobotDecisionSys> myRDS;
    std::shared_ptr<::global_interface::Decision> excuting_decision = nullptr;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    geometry_msgs::TransformStamped::Ptr _transformStamped = nullptr;

    // std::vector<geometry_msgs::PointStamped> acummulated_poses_;
    float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
    double curTime = 0, waypointTime = 0;
    std::chrono::milliseconds server_timeout_;

    std::shared_timed_mutex myMutex_status;
    std::shared_timed_mutex myMutex_NTP_FeedBack;
    std::shared_timed_mutex myMutex_joint_states;
    std::shared_timed_mutex myMutex_detectionArray;
    std::shared_timed_mutex myMutex_modeSet;
    std::shared_timed_mutex myMutex_autoaim;

    sensor_msgs::JointState::Ptr          joint_states_msg = nullptr;
    global_interface::Autoaim::Ptr        autoaim_msg = nullptr;
    global_interface::DetectionArray::Ptr detectionArray_msg = nullptr;
    global_interface::ModeSet::Ptr        modeSet_msg = nullptr;

public:
    /**
     * @brief 初始化
     */
    RobotDecisionNode();
    ~RobotDecisionNode();
    void clearGoals();
    /**
     * @brief 将消息位置转换成RobotPosition结构体
     * @param pos
     * 位置集合
     * @return
     * RobotPosition集合
     */
    std::vector<RobotPosition> point2f2Position(
        boost::array<global_interface::Point2f, 12UL> pos);
    /**
     * @brief 消息回调，由message_filters处理
     */
    void messageCallBack(
        const boost::shared_ptr<::global_interface::ObjHP const > &carHP_msg_,
        const boost::shared_ptr<::global_interface::CarPos const > &carPos_msg_,
        const boost::shared_ptr<::global_interface::GameInfo const > &gameInfo_msg_,
        const boost::shared_ptr<::global_interface::Serial const > &serial_msg_);
    /**
     * @brief joint_states消息回调
     */
    void jointStateCallBack(const sensor_msgs::JointState::Ptr &msg);
    /**
     * @brief 检测目标消息回调
     */
    void detectionArrayCallBack(const global_interface::DetectionArray::Ptr msg);
    /**
     * @brief 云台手模式设置消息回调
     */
    void modeSetCallBack(const global_interface::ModeSet::Ptr msg);
    void poseHandler(const nav_msgs::Odometry::ConstPtr& pose);
    /**
     * @brief 解码Json文件，载入配置信息
     * @return 是否成功
     */
    bool decodeConfig();
    /**
     * @brief 向目标缓冲区添加目标点
     * @param x 目标x坐标
     * @param y 目标y坐标
     * @param z 目标z坐标
     */
    void makeNewGoal(double x, double y, double z);
    /**
     * @brief 决策处理一次
     * @param _HP 当前血量
     * @param mode 当前模式
     * @param _x 当前x轴坐标
     * @param _y 当前y轴坐标
     * @param time 比赛时间
     * @param now_out_post_HP 友方前哨站当前血量
     * @param friendPositions 友方位置集合
     * @param enemyPositions 敌方位置集合
     * @return 处理是否成功
     */
    bool process_once(int &_HP, int &mode, float &_x, float &_y, int &time,
                      int &now_out_post_HP, int &now_base_HP,
                      std::vector<RobotPosition> &friendPositions,
                      std::vector<RobotPosition> &enemyPositions,
                      geometry_msgs::TransformStamped::Ptr transformStamped);
    /**
     * @brief 周期性参数回调
     */
    void respond(const ros::TimerEvent& event);
    /**
     * @brief 根据决策创建消息
     * @param decision 决策
     * @param theta 车辆朝向（弧度制）（偏角）
     * @return 决策消息
     */
    // global_interface::Decision makeDecisionMsg(
    //     std::shared_ptr<Decision> decision, double &theta);
    // global_interface::Decision makeDecisionMsg(
    //     int mode, double theta, float _x, float _y);
};


#endif


