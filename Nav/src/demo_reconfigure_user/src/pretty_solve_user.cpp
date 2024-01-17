#include "ros/duration.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include <boost/bind/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <cstdint>
#include <cstdio>
#include <geometry_msgs/TwistStamped.h>
#include "demo_reconfigure_user/demo_configConfig.h"
#include "dynamic_reconfigure/server.h"
#include "ros/timer.h"
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>

using namespace std;

const double PI = 3.1415926;

bool Navigation_Mode;
bool Patrol_Mode;
bool Rotate_Mode;
bool Counterattack_Mode;    

string waypoint_file_dir;
double waypointXYRadius = 0.5;
double waypointZBound = 5.0;
double waitTime = 0;
double waitTimeStart = 0;
bool isWaiting = false;
double frameRate = 5.0;  // 帧率
double speed = 1.0;
bool sendSpeed = true;

// 创建PointCloud对象，用于存储航点和边界信息
pcl::PointCloud<pcl::PointXYZ>::Ptr waypoints(
    new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointXYZ point1(0, 0, 0);
pcl::PointXYZ point2(2, 0, 0);
int wayPointID = 0;

// 定义变量，用于存储车辆的位置信息
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
double curTime = 0, waypointTime = 0;
geometry_msgs::TwistStamped cmd_vel_user;
geometry_msgs::TwistStamped cmd_vel;
geometry_msgs::PointStamped waypointMsgs;
std_msgs::Float32 speedMsgs;

ros::Subscriber subPose;
ros::Publisher pubWaypoint;
ros::Publisher pubSpeed;
ros::Publisher pubCmd_Vel_User;

namespace demo_reconfigure_user{
    void callback(demo_configConfig &config, uint32_t level)
    {
        if (!config.Navigation_Mode) 
        {
            if (config.Rotate_Mode && !config.Patrol_Mode && !config.Counterattack_Mode) 
            {
                // Rotation
                cmd_vel_user.twist.linear.x  = 0;
                cmd_vel_user.twist.linear.y  = 0;
                cmd_vel_user.twist.angular.z = 0.75;    // 0.75

                Navigation_Mode     = config.Navigation_Mode;
                Patrol_Mode         = config.Patrol_Mode;
                Rotate_Mode         = config.Rotate_Mode;
                Counterattack_Mode  = config.Counterattack_Mode;

                ROS_INFO("Robot is in rotation mode\n");
            }
            else if (!config.Rotate_Mode && config.Patrol_Mode && !config.Counterattack_Mode) 
            {
                // Patrol

                Navigation_Mode     = config.Navigation_Mode;
                Patrol_Mode         = config.Patrol_Mode;
                Rotate_Mode         = config.Rotate_Mode;
                Counterattack_Mode  = config.Counterattack_Mode;

                ROS_INFO("Robot is in patrol mode\n");
            }
            else if (!config.Rotate_Mode && !config.Patrol_Mode && config.Counterattack_Mode) 
            {
                //Counterattack
                cmd_vel_user.twist.linear.x  = 0;
                cmd_vel_user.twist.linear.y  = 0;
                cmd_vel_user.twist.angular.z = 1.25;  // 1.25

                Navigation_Mode    = config.Navigation_Mode;
                Patrol_Mode        = config.Patrol_Mode;
                Rotate_Mode        = config.Rotate_Mode;
                Counterattack_Mode = config.Counterattack_Mode;

                ROS_INFO("Robot is in counterattack mode\n");
            }
            else 
            {
                cmd_vel_user.twist.linear.x  = 0;
                cmd_vel_user.twist.linear.y  = 0;
                cmd_vel_user.twist.angular.z = 0;

                Navigation_Mode     = config.Navigation_Mode;
                Patrol_Mode         = config.Patrol_Mode;
                Rotate_Mode         = config.Rotate_Mode;
                Counterattack_Mode  = config.Counterattack_Mode;
                
                ROS_INFO("If robot is not in navigation mode. Please ensure that the robot has only one valid state\n");
            }
        }
        else 
        {
            ROS_INFO("Robot is in navigation mode\n");
            return;
        }

    }
};

void timer_callback_pub(const ros::TimerEvent& event)
{
    if (!Navigation_Mode && !Patrol_Mode) 
    {
        pubCmd_Vel_User.publish(cmd_vel_user);
    }
}

// vehicle pose callback function
void poseHandler(const nav_msgs::Odometry::ConstPtr& pose) {
    // 获取当前时间戳并将其转换为秒 数
    curTime = pose->header.stamp.toSec();

    vehicleX = pose->pose.pose.position.x;
    vehicleY = pose->pose.pose.position.y;
    vehicleZ = pose->pose.pose.position.z;
}
void timer_callback_back (const ros::TimerEvent& event) {
    if (!Navigation_Mode && !Rotate_Mode && Patrol_Mode && !Counterattack_Mode) 
        {
            waypoints->push_back(point1);
            waypoints->push_back(point2);
            float disX = vehicleX - waypoints->points[wayPointID].x;
            float disY = vehicleY - waypoints->points[wayPointID].y;
            float disZ = vehicleZ - waypoints->points[wayPointID].z;

            // start waiting if the current waypoint is reached
            if (sqrt(disX * disX + disY * disY) < waypointXYRadius &&
                fabs(disZ) < waypointZBound && !isWaiting) {
                waitTimeStart = curTime;
                isWaiting = true;
            }

            // move to the next waypoint after waiting is over
            if (isWaiting && waitTimeStart + waitTime < curTime) {
                wayPointID = wayPointID ? 0 : 1;
                isWaiting = false;
            }

            // publish waypoint, speed, and boundary messages at certain frame rate
            if (curTime - waypointTime > 1.0 / frameRate) {
                if (!isWaiting) {
                    waypointMsgs.header.stamp = ros::Time().fromSec(curTime);
                    waypointMsgs.point.x = waypoints->points[wayPointID].x;
                    waypointMsgs.point.y = waypoints->points[wayPointID].y;
                    waypointMsgs.point.z = waypoints->points[wayPointID].z;
                    pubWaypoint.publish(waypointMsgs);
                }

                speedMsgs.data = speed;
                pubSpeed.publish(speedMsgs);

                waypointTime = curTime;
            }
        }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "temp_solve_user");
    ros::NodeHandle nh;

    nh.param<bool>("Navigation_Mode", Navigation_Mode, true);
    nh.param<bool>("Patrol_Mode", Patrol_Mode, false);
    nh.param<bool>("Rotate_Mode", Rotate_Mode, false);
    nh.param<bool>("Counterattack_Mode", Counterattack_Mode, false);
    nh.getParam("waypoint_file_dir", waypoint_file_dir);
    nh.getParam("waypointXYRadius", waypointXYRadius);
    nh.getParam("waypointZBound", waypointZBound);
    nh.getParam("waitTime", waitTime);
    nh.getParam("frameRate", frameRate);
    nh.getParam("speed", speed);
    nh.getParam("sendSpeed", sendSpeed);

    waypointMsgs.header.frame_id = "map";

    dynamic_reconfigure::Server<demo_reconfigure_user::demo_configConfig> srv;
    dynamic_reconfigure::Server<demo_reconfigure_user::demo_configConfig>::CallbackType ca = boost::bind(&demo_reconfigure_user::callback, _1, _2);

    srv.setCallback(ca);

    pubCmd_Vel_User = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 5);

    // poseHandler 获取车辆的位置信息
    subPose = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, poseHandler);

    pubWaypoint = nh.advertise<geometry_msgs::PointStamped>("/way_point", 5);

    pubSpeed = nh.advertise<std_msgs::Float32>("/speed", 5);
    
    ros::Timer timer_pub = nh.createTimer(ros::Duration(0.01), timer_callback_pub);

    ros::Timer timer_back = nh.createTimer(ros::Duration(1), timer_callback_back);
    

    ros::Rate rate(100);

    // while (ros::ok()) {
    //     ros::spinOnce();

    //     nh.setParam("Navigation_Mode_now", Navigation_Mode);


    //     rate.sleep();
    // }

    ros::spin();

    return 0;
}
