#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <map>
#include <float.h>
#include <memory>
#include <shared_mutex>
#include <mutex>
#include <algorithm>
#include <ctime>
#include <chrono>
#include <stack>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Point.h"
#include <geometry_msgs/PointStamped.h>

// #include "opencv2/opencv.hpp"
#include "../../include/Json/json.h"

#define OBJHP_NUM (int)8
#define CARPOS_NUM (int)6