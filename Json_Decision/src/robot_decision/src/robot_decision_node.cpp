#include "../include/robot_decision_node.hpp"
#include <ros/package.h>
#include <pcl/point_cloud.h>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <mutex>
#include <pcl/impl/point_types.hpp>
#include <shared_mutex>
#include <string>
#include <vector>
#include "Json/json.h"
#include "boost/bind.hpp"
#include "geometry_msgs/PointStamped.h"
#include "global_interface/Decision.h"
#include "nav_msgs/Odometry.h"
#include "robot_decision/public.h"
#include "robot_decision/structs.h"
#include "std_msgs/Float32.h"
#include "tf2/transform_storage.h"

void RobotDecisionNode::jointStateCallBack(const sensor_msgs::JointState::Ptr &msg) {
    std::unique_lock<std::shared_timed_mutex> ulk(this->myMutex_joint_states);
    if (msg->name[0] == "gimbal_yaw_joint" && !std::isnan(msg->position[0])) {
        this->joint_states_msg = msg;
    }
    ulk.unlock();
    ROS_DEBUG("jointState Recived: yaw = %lf , pitch = %lf", msg->position[0], msg->position[1]);
}

void RobotDecisionNode::clearGoals() {
    this->acummulated_poses_.clear();
}
pcl::PointCloud<pcl::PointXYZ>::Ptr waypoints(new pcl::PointCloud<pcl::PointXYZ>());
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
double curTime = 0, waypointTime = 0;

void RobotDecisionNode::makeNewGoal(double x, double y, double z) {
    waypoints->clear();
    pcl::PointXYZ point;
    int val1, val2, val3;
    point.x = x;
    point.y = y;
    point.z = z;
    waypoints->push_back(point);
}
/*
    * 不知道是啥
*/
void RobotDecisionNode::modeSetCallBack(const global_interface::ModeSet::Ptr msg) {
    int mode = msg->mode;
    float _x = msg->x;
    float _y = msg->y;
    float _z = msg->z;
    ROS_INFO("Manual mode: %d x:%lf y:%lf z:%lf", mode, _x, _y, _z);
    if (mode == 0) { return; }
    bool check = true;
    std::unique_lock<std::shared_timed_mutex> ulk(this->myMutex_modeSet);
    if (this->modeSet_msg == nullptr) {
        this->modeSet_msg = msg;
        check = true;
    } else if (this->modeSet_msg != nullptr && 
                this->modeSet_msg->mode == mode && \
                this->modeSet_msg->x == _x && 
                this->modeSet_msg->y == _y && 
                this->modeSet_msg->z == _z) {
        check = false;
    } else {
        this->modeSet_msg = msg;
        check = true;
    }
    global_interface::Decision newDecision_msg;
    this->decision_pub_.publish(newDecision_msg);
    if (check) {
        this->clearGoals();
        this->makeNewGoal(_x, _y, _z);
        /*
            * 没写完！！！！！！！！！！！！！！！！！！！！！！！！！
        */
    }
}
void RobotDecisionNode::poseHandler(const nav_msgs::Odometry::ConstPtr& pose) {
    // 获取当前时间戳并将其转换为秒 数
    curTime = pose->header.stamp.toSec();

    vehicleX = pose->pose.pose.position.x;
    vehicleY = pose->pose.pose.position.y;
    vehicleZ = pose->pose.pose.position.z;
}
void RobotDecisionNode::respond(const ros::TimerEvent& event) {
    nh.getParam("distance_thr", this->_distance_THR_Temp);
    nh.getParam("seek_thr", this->_seek_THR_Temp);
    nh.getParam("IsBlue", this->_IsBlue_Temp);

    if (this->myRDS->getDistanceTHR() != this->_distance_THR_Temp)
    {
        ROS_INFO("set _distance_THR to %f", this->_distance_THR_Temp);
        this->myRDS->setDistanceTHR(this->_distance_THR_Temp);
    }
    if (this->myRDS->getSeekTHR() != this->_seek_THR_Temp)
    {
        ROS_INFO("set _seek_THR to %f", this->_seek_THR_Temp);
        this->myRDS->setSeekTHR(this->_seek_THR_Temp);
    }
    if (this->_IsBlue != this->_IsBlue_Temp)
    {
        ROS_INFO("set _IsBlue to %d", this->_IsBlue_Temp);
        this->_IsBlue = this->_IsBlue_Temp;
    }
}
bool RobotDecisionNode::decodeConfig() {
    std::string package_share_directory = ros::package::getPath("robot_decision");
    Json::Reader jsonReader;
    Json::Value jsonValue;
    std::ifstream jsonFile(package_share_directory + "/" + "JsonFile/config.json");
    if (!jsonReader.parse(jsonFile, jsonValue, true)) {
        std::cout << "read error" << std::endl;
        jsonFile.close();
        return false;
    }
    Json::Value arrayValue = jsonValue["config"];
    this->_Debug = arrayValue["Debug"].asBool();
    this->_WayPointsPath = package_share_directory + "/JsonFile/" + arrayValue["WayPointsPATH"].asCString();
    this->_DecisionsPath = package_share_directory + "/JsonFile/" + arrayValue["DecisionsPath"].asCString();
    this->_INIT_DISTANCE_THR = arrayValue["INIT_DISTANCE_THR"].asFloat();
    this->_INIT_SEEK_THR = arrayValue["INIT_SEEK_THR"].asFloat();
    this->_INIT_IsBlue = arrayValue["INIT_ISBLUE"].asBool();
    this->_INIT_SELFINDEX = arrayValue["INIT_SELFINDEX"].asInt();
    this->_INIT_FRIENDOUTPOSTINDEX = arrayValue["INIT_FRIENDOUTPOSTINDEX"].asInt();
    this->_INIT_FRIENDBASEINDEX = arrayValue["INIT_FRIENDBASEINDEX"].asInt();
    this->_GAME_TIME = arrayValue["GAME_TIME"].asInt();
    this->_TIME_THR = arrayValue["TIME_THR"].asInt();
    this->_REAL_WIDTH = arrayValue["REAL_WIDTH"].asFloat();
    this->_REAL_HEIGHT = arrayValue["REAL_HEIGHT"].asFloat();
    this->_STEP_DISTANCE = arrayValue["STEP_DISTANCE"].asFloat();
    this->_CAR_SEEK_FOV = arrayValue["CAR_SEEK_FOV"].asFloat();
    return true;
}

RobotDecisionNode::RobotDecisionNode() {
    nhPrivate.getParam("distance_thr", this->_INIT_DISTANCE_THR);
    nhPrivate.getParam("seek_thr", this->_INIT_SEEK_THR);
    nhPrivate.getParam("IsBlue", this->_INIT_IsBlue);

    nhPrivate.getParam("waypointXYRadius", this->waypointXYRadius);
    nhPrivate.getParam("waypointZBound", this->waypointZBound);
    nhPrivate.getParam("waitTime", this->waitTime);
    nhPrivate.getParam("frameRate", this->frameRate);
    nhPrivate.getParam("speed", this->speed);
    nhPrivate.getParam("sendSpeed", this->sendSpeed);

    this->_selfIndex = this->_INIT_SELFINDEX;
    this->_friendOutPostIndex = this->_INIT_FRIENDOUTPOSTINDEX;
    this->_friendBaseIndex = this->_INIT_FRIENDBASEINDEX;
    this->myRDS = std::make_shared<RobotDecisionSys>(RobotDecisionSys(
        this->_distance_THR_Temp, this->_seek_THR_Temp, this->_REAL_WIDTH,
        this->_REAL_HEIGHT, this->_STEP_DISTANCE, this->_CAR_SEEK_FOV));
    this->timer_ = nh.createTimer(ros::Duration(200), &RobotDecisionNode::respond, this);

    if (!this->myRDS->decodeWayPoints(this->_WayPointsPath))
        ROS_ERROR("Decode waypoints failed");
    if (!this->myRDS->decodeDecisions(this->_DecisionsPath))
        ROS_ERROR("Decode decisions failed");
    
    this->modeSet_sub_ = nh.subscribe("/mode_set", 10, &RobotDecisionNode::modeSetCallBack, this);
   
    objHP_sub_.subscribe(nh, "/obj_hp", 10);
    carPos_sub_.subscribe(nh, "/car_pos", 10);
    gameInfo_sub_.subscribe(nh, "/game_info", 10);
    serial_sub_.subscribe(nh, "/serial_msg", 10);

    TS_sync_.reset(
        new message_filters::Synchronizer<ApproximateSyncPolicy>(
            ApproximateSyncPolicy(10), objHP_sub_, carPos_sub_, gameInfo_sub_, serial_sub_));
    TS_sync_->registerCallback(boost::bind(&RobotDecisionNode::messageCallBack, this, _1, _2, _3, _4));

    this->decision_pub_ =
        nh.advertise<global_interface::Decision>("robot_decision/decision", 10);
    this->strikeLicensing_pub_ =
        nh.advertise<global_interface::StrikeLicensing>(
            "robot_decision/strikeLicensing", 10);
    
    this->subPose = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, &RobotDecisionNode::poseHandler, this);
    this->pubWaypoint = nh.advertise<geometry_msgs::PointStamped>("way_point", 5);
    geometry_msgs::PointStamped waypointMsgs;
    waypointMsgs.header.frame_id = "map";

    this->pubSpeed = nh.advertise<std_msgs::Float32>("/speed", 5);
}

bool RobotDecisionNode::process_once(int &HP, int &mode, float &_x, float &_y, int &time, int &now_out_post_HP, int &now_base_HP, std::vector<RobotPosition> &friendPositions, std::vector<RobotPosition> &enemyPositions, geometry_msgs::TransformStamped::Ptr transformStamped) {
    ROS_INFO("Heartbeat Processing");
    if (_x == 0.0 || _y == 0.0 || std::isnan(_x) || std::isnan(_y)) {
        if (transformStamped != nullptr) {
            _x = transformStamped->transform.translation.x;
            _y = transformStamped->transform.translation.y;
        } else if (this->_transformStamped != nullptr && abs(ros::Time::now().toSec() - this->_transformStamped->header.stamp.sec) < this->_TIME_THR) {
            _x = this->_transformStamped->transform.translation.x;
            _y = this->_transformStamped->transform.translation.y;
        } else {
            ROS_ERROR("Cannot get current Position!");
            return false;
        }
    }
    /* 获取当前位姿 */
    return true;
}

std::vector<RobotPosition> RobotDecisionNode::point2f2Position(std::array<global_interface::Point2f, 12UL> pos) {
    if (pos.size() != CARPOS_NUM * 2) {
        ROS_ERROR("Position msg not valid !");
        return {};
    }
    std::vector<RobotPosition> result;
    for (int i = 0; i < int(pos.size()); ++i) {
        // result.emplace_back(RobotPosition(i, pos[i].x, pos[i].y));
    }
    return result;
}

void RobotDecisionNode::messageCallBack(const boost::shared_ptr<::global_interface::ObjHP const > &carHP_msg_, 
                                        const boost::shared_ptr<::global_interface::CarPos const > &carPos_msg_, 
                                        const boost::shared_ptr<::global_interface::GameInfo const > &gameInfo_msg_, 
                                        const boost::shared_ptr<::global_interface::Serial const > &serial_msg_) {
    ROS_INFO("------------------------------------------------------------------------------[ONCE PROCESS]");
    auto start_t = std::chrono::system_clock::now().time_since_epoch();
    geometry_msgs::TransformStamped::Ptr transformStamped = nullptr;
    for (int i = 0; i < int(carHP_msg_->hp.size()); i++) {
        if (std::isnan(carHP_msg_->hp[i])) {
            ROS_ERROR("Receive ObjHP Msg ERROR!");
            return;
        }
        ROS_DEBUG("Receive ObjHP Msg %d: %d", i, carHP_msg_->hp[i]);
    }
    for (int i = 0; i < int(carPos_msg_->pos.size()); i++) {  
        if (std::isnan(carPos_msg_->pos[i].x || carPos_msg_->pos[i].y)) {
            ROS_ERROR("Receive ObjPos Msg ERROR");
            return;
        }
        ROS_DEBUG("Receive ObjPos Msg %d: x=%lf, y=%lf", i, carPos_msg_->pos[i].x, carPos_msg_->pos[i].y);
    }
    if (std::isnan(gameInfo_msg_->timestamp) || std::isnan(gameInfo_msg_->game_stage)) {
        ROS_ERROR("Receive GameInfo Msg ERROR");
        return;
    }
    ROS_DEBUG("Receive GameInfo Msg : timestamp=%d", gameInfo_msg_->timestamp);
    if (std::isnan(serial_msg_->mode) || std::isnan(serial_msg_->theta)) {
        ROS_ERROR("Receive Serial Msg Error");
        return;
    }
    ROS_DEBUG("Receive Serial Msg : mode=%d, theta=%lf", serial_msg_->mode,
        serial_msg_->theta);
    int currentSelfIndex = this->_selfIndex;
    int currentFriendOutpostIndex_hp = this->_friendOutPostIndex;
    int currentEnemyOutpostIndex_hp = this->_friendOutPostIndex + OBJHP_NUM;
    int currentBaseIndex_hp = this->_friendBaseIndex;
    int currentSelfIndex_hp = this->_selfIndex;
    if (this->_IsBlue) {
        currentSelfIndex_hp = this->_selfIndex + OBJHP_NUM;
        currentFriendOutpostIndex_hp = this->_friendOutPostIndex + OBJHP_NUM;
        currentEnemyOutpostIndex_hp = this->_friendOutPostIndex;
        currentBaseIndex_hp = this->_friendBaseIndex + OBJHP_NUM;
        currentSelfIndex = this->_selfIndex + CARPOS_NUM;
    }
    if (gameInfo_msg_->game_stage != GameStage::IN_BATTLE && !this->_Debug) {
        ROS_WARN_ONCE("Wait for game start ...");
        return;
    }

    int myHP = carHP_msg_->hp[currentSelfIndex];
    float myPos_x_ = carPos_msg_->pos[currentSelfIndex].x;
    float myPos_y_ = carPos_msg_->pos[currentSelfIndex].y;
    int nowTime = this->_GAME_TIME - gameInfo_msg_->timestamp;
    int mode = serial_msg_->mode;
    int now_out_post_HP = carHP_msg_->hp[currentFriendOutpostIndex_hp];
    int now_out_post_HP_enemy = (now_out_post_HP == 0);
    bool _if_enemy_outpost_down = (now_out_post_HP_enemy == 0);
    int now_base_HP = carHP_msg_->hp[currentBaseIndex_hp];
    std::vector<int> _car_hps;
    for (auto& it: this->_car_ids) {
        _car_hps.emplace_back(carHP_msg_->hp[it + !this->_IsBlue * OBJHP_NUM]);
    }
    // std::vector<RobotDecisionNode> allPositions = this->point2f2Position(carPos_msg_->pos);
}

int main (int argc, char** argv) {
    RobotDecisionNode node;
    return 0;
}
