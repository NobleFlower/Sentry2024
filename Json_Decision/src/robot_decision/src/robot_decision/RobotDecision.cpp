#include "../../include/robot_decision/RobotDecision.h"

    GameHandler::GameHandler()
    {
    }

    GameHandler::~GameHandler()
    {
    }

    void GameHandler::update(int &gameTime)
    {
        this->lastUpdateTime = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
        this->gameTime = gameTime;
    }

    int RobotDecisionSys::calculatePosition(RobotPosition &pos)
    {
        double distance = FLT_MAX;
        int id = -1;
        for (auto it : this->wayPointMap)
        {
            double tempDistance = sqrtf(powf(double(it->point.x - pos.vehicleX), 2) + powf(double(it->point.y - pos.vehicleY), 2));
            if (tempDistance - distance < 0.)
            {
                distance = tempDistance;
                id = it->id;
            }
        }
        return distance - this->_distance_THR > 0. ? -1 : id;
    }


    RobotDecisionSys::RobotDecisionSys(float &_distance_THR, float &_seek_THR, float &_REAL_WIDTH, float &_REAL_HEIGHT, float &_STEP_DISTANCE, float &_CAR_SEEK_FOV)
    {
        this->_distance_THR  = _distance_THR;
        this->_seek_THR      = _seek_THR;
        this->_REAL_WIDTH    = _REAL_WIDTH;
        this->_REAL_HEIGHT   = _REAL_HEIGHT;
        this->_STEP_DISTANCE = _STEP_DISTANCE;
        this->_CAR_SEEK_FOV  = _CAR_SEEK_FOV;

        this->myGameHandler  = std::make_shared<GameHandler>(GameHandler());
        // this->decisionMap    = cv::imread(this->_MAP_PATH);
        // cv::resize(this->decisionMap, this->decisionMap, cv::Size(int(this->_REAL_WIDTH / this->_REAL_HEIGHT * 1080.), 1080));
        // cv::cvtColor(this->decisionMap, this->decisionMap_Gray, cv::COLOR_BGR2GRAY, 1);
    }

    RobotDecisionSys::~RobotDecisionSys()
    {
    }

    bool RobotDecisionSys::decodeWayPoints(std::string &filePath)
    {
        Json::Reader jsonReader;
        Json::Value jsonValue;
        std::ifstream jsonFile(filePath);
        if (!jsonReader.parse(jsonFile, jsonValue, true))
        {
            std::cout << "read error" << std::endl;
            jsonFile.close();
            return false;
        }
        Json::Value arrayValue = jsonValue["data"];
        std::vector<WayPoint> tempwayPointMap;
        for (int i = 0; i < int(arrayValue.size()); ++i)
        {
            std::shared_ptr<WayPoint> wayPoint = std::make_shared<WayPoint>(WayPoint());
            wayPoint->id = arrayValue[i]["id"].asInt();
            wayPoint->type = arrayValue[i]["type"].asInt();
            wayPoint->point.x = arrayValue[i]["x"].asFloat();
            wayPoint->point.y = arrayValue[i]["y"].asFloat();
            // wayPoint->point.z = arrayValue[i]["angle"].asDouble();
            Json::Value connectedPoints = arrayValue[i]["connect"];
            for (int j = 0; j < int(connectedPoints.size()); ++j)
            {
                wayPoint->connection.emplace_back(connectedPoints[j].asInt());
            }
            Json::Value enemyWeightsArray = arrayValue[i]["enemyWeights"];
            for (int j = 0; j < int(enemyWeightsArray.size()); ++j)
            {
                wayPoint->enemyWeights[j] = enemyWeightsArray[j].asInt();
            }
            this->wayPointMap.emplace_back(wayPoint);
            this->connection_map.insert(std::make_pair(wayPoint->id, wayPoint->connection));
        }
        jsonFile.close();
        return true;
    }

    bool RobotDecisionSys::decodeDecisions(std::string &filePath)
    {
        Json::Reader jsonReader;
        Json::Value jsonValue;
        std::ifstream jsonFile(filePath);
        if (!jsonReader.parse(jsonFile, jsonValue, true))
        {
            std::cout << "read error" << std::endl;
            jsonFile.close();
            return false;
        }
        Json::Value arrayValue = jsonValue["data"];
        for (int i = 0; i < int(arrayValue.size()); ++i)
        {
            std::shared_ptr<Decision> decision = std::make_shared<Decision>(Decision());
            decision->id = arrayValue[i]["id"].asInt();
            decision->name = arrayValue[i]["name"].asCString();
            Json::Value wayPointIDArray = arrayValue[i]["wayPointID"];
            for (int j = 0; j < int(wayPointIDArray.size()); ++j)
            {
                decision->wayPointID.emplace_back(wayPointIDArray[j].asInt());
            }
            decision->weight = arrayValue[i]["weight"].asInt();
            decision->start_time = arrayValue[i]["start_time"].asInt();
            decision->end_time = arrayValue[i]["end_time"].asInt();
            decision->robot_mode = arrayValue[i]["robot_mode"].asInt();
            decision->_minHP = arrayValue[i]["minHP"].asInt();
            decision->_maxHP = arrayValue[i]["maxHP"].asInt();
            decision->decide_mode = arrayValue[i]["decide_mode"].asInt();
            decision->decide_wayPoint = arrayValue[i]["decide_wayPoint"].asInt();
            decision->out_post_HP_min = arrayValue[i]["out_post_HP_min"].asInt();
            decision->out_post_HP_max = arrayValue[i]["out_post_HP_max"].asInt();
            decision->base_HP_min = arrayValue[i]["base_HP_min"].asInt();
            decision->if_succession = arrayValue[i]["if_succession"].asBool();
            decision->if_reverse = arrayValue[i]["if_reverse"].asBool();
            Json::Value enemyPositionArray = arrayValue[i]["enemyPosition"];
            for (int j = 0; j < int(enemyPositionArray.size()); ++j)
            {
                std::vector<int> temp;
                for (int k = 0; k < int(enemyPositionArray[j].size()); ++k)
                {
                    temp.emplace_back(enemyPositionArray[j][k].asInt());
                }
                decision->enemy_position.emplace_back(temp);
            }
            Json::Value friendPositionArray = arrayValue[i]["friendPosition"];
            for (int j = 0; j < int(friendPositionArray.size()); ++j)
            {
                std::vector<int> temp;
                for (int k = 0; k < int(friendPositionArray[j].size()); ++k)
                {
                    temp.emplace_back(friendPositionArray[j][k].asInt());
                }
                decision->friend_position.emplace_back(temp);
            }
            this->decisions.emplace_back(decision);
        }
        jsonFile.close();
        return true;
    }

    int RobotDecisionSys::checkNowWayPoint(float x, float y)
    {
        RobotPosition pos;
        pos.vehicleX = x;
        pos.vehicleY = y;
        // pos.vehicleZ = z;
        return this->calculatePosition(pos);
    }

    int RobotDecisionSys::checkNowWayPoint(RobotPosition pos)
    {
        return this->calculatePosition(pos);
    }

    std::shared_ptr<Decision> RobotDecisionSys::decide(int wayPointID, int robot_mode, int _HP, int nowtime, int now_out_post_HP, int now_base_HP, std::vector<RobotPosition> &friendPositions, std::vector<RobotPosition> &enemyPositions, std::vector<int> &availableDecisionID, std::map<int, int> &id_pos_f, std::map<int, int> &id_pos_e)
    {
        this->myGameHandler->update(nowtime);
        std::vector<std::shared_ptr<Decision>> tempDecision;
        for (auto &it : friendPositions)
        {
            id_pos_f.insert(std::make_pair(it.robot_id, this->calculatePosition(it)));
        }
        for (auto &it : enemyPositions)
        {
            id_pos_e.insert(std::make_pair(it.robot_id, this->calculatePosition(it)));
        }
        for (auto it : this->decisions)
        {

            if (it->wayPointID[0] != -1)
            {
                bool check = false;
                for (auto &jt : it->wayPointID)
                {
                    check = jt == wayPointID;
                    if (check)
                        break;
                }
                if (!check)
                    continue;
            }
            if ((it->robot_mode != -1 && it->robot_mode != robot_mode) ||
                (it->_maxHP != -1 && _HP > it->_maxHP) ||
                (it->_minHP != -1 && _HP <= it->_minHP) ||
                (it->end_time != -1 && nowtime > it->end_time) ||
                (it->start_time != -1 && nowtime <= it->start_time) ||
                (it->out_post_HP_min != -1 && now_out_post_HP <= it->out_post_HP_min) ||
                (it->out_post_HP_max != -1 && now_out_post_HP > it->out_post_HP_max) ||
                (it->base_HP_min != -1 && now_base_HP < it->base_HP_min))
            {
                continue;
            }
            bool fpFLAG = true;
            for (int i = 0; i < int(it->friend_position.size()); ++i)
            {
                int temp_pos = id_pos_f.find(i)->second;
                if (it->friend_position[i][0] == -1)
                {
                    continue;
                }
                if (std::find(it->friend_position[i].begin(), it->friend_position[i].end(), temp_pos) == it->friend_position[i].end())
                {
                    fpFLAG = false;
                    break;
                }
            }
            bool epFLAG = true;
            for (int i = 0; i < int(it->enemy_position.size()); ++i)
            {
                int temp_pos = id_pos_e.find(i)->second;
                if (it->enemy_position[i][0] == -1)
                {
                    continue;
                }
                if (std::find(it->enemy_position[i].begin(), it->enemy_position[i].end(), temp_pos) == it->enemy_position[i].end())
                {
                    fpFLAG = false;
                    break;
                }
            }
            if (epFLAG && fpFLAG)
                tempDecision.emplace_back(it);
        }
        int max_weight = 0;
        std::shared_ptr<Decision> decision = nullptr;
        for (auto it : tempDecision)
        {
            if (it->weight > max_weight)
            {
                max_weight = it->weight;
                decision = it;
                decision->if_auto = false;
            }
            availableDecisionID.emplace_back(it->id);
        }
        if (decision != nullptr && decision->decide_mode == -1)
            decision->decide_mode = decision->robot_mode;
        if (decision != nullptr && decision->decide_wayPoint == -1)
            decision->decide_wayPoint = wayPointID;
        return decision;
    }

    std::shared_ptr<WayPoint> RobotDecisionSys::getWayPointByID(int id)
    {
        for (auto &it : this->wayPointMap)
        {
            if (it->id == id)
            {
                return it;
            }
        }
        return nullptr;
    }

    std::shared_ptr<Decision> RobotDecisionSys::getDecisionByID(int id)
    {
        for (auto &it : this->decisions)
        {
            if (it->id == id)
            {
                return it;
            }
        }
        return nullptr;
    }

    std::map<int, float> RobotDecisionSys::decideAimTarget(bool _IsBlue, RobotPosition &mypos, std::vector<RobotPosition> &enemyPositions, std::vector<int> &enemyHP, float distance_weight_ratio, float hp_weight_ratio, bool enemyOutpostDown)
    {
        std::map<int, float> result;
        float max_distance = 0.0000001;
        std::map<int, float> distances;
        std::map<int, float> distance_weight;
        for (auto &it : enemyPositions)
        {
            float tempDistance = sqrtf(powf(double(it.vehicleX - mypos.vehicleX), 2) + powf(double(it.vehicleY - mypos.vehicleY), 2));
            distances[it.robot_id] = tempDistance;
            if (tempDistance > max_distance)
                max_distance = tempDistance;
        }
        std::map<int, float>::iterator iter_distance_weight;
        for (iter_distance_weight = distances.begin(); iter_distance_weight != distances.end(); iter_distance_weight++)
        {
            distance_weight[iter_distance_weight->first] = (1. - (iter_distance_weight->second / max_distance)) * distance_weight_ratio;
        }
        float max_hp = 0.0000001;
        std::map<int, float> hps;
        std::map<int, float> hp_weight;
        for (int i = 0; i < int(enemyHP.size()); ++i)
        {
            hps[!_IsBlue * CARPOS_NUM + i] = float(enemyHP[i]);
            if (enemyHP[i] > max_hp)
                max_hp = float(enemyHP[i]);
        }
        std::map<int, float>::iterator iter_hp_weight;
        for (iter_hp_weight = hps.begin(); iter_hp_weight != hps.end(); iter_hp_weight++)
        {
            hp_weight[iter_hp_weight->first] = (1. - (iter_hp_weight->second / max_hp)) * hp_weight_ratio;
        }
        for (int i = 0; i < CARPOS_NUM; ++i)
        {
            result[i] = (hp_weight.find(!_IsBlue * CARPOS_NUM + i) != hp_weight.end() ? hp_weight.find(!_IsBlue * CARPOS_NUM + i)->second : -0.5) + (distance_weight.find(!_IsBlue * CARPOS_NUM + i) != distance_weight.end() ? distance_weight.find(!_IsBlue * CARPOS_NUM + i)->second : -0.5) + 0.000001;
        }
        // if (!enemyOutpostDown)
            result[5] = 0;
        return result;
    }

    // double RobotDecisionSys::decideAngleByEnemyPos(float _x, float _y, std::vector<RobotPosition> &enemyPositions)
    // {
    //     int index = -1;
    //     float min_distance = MAXFLOAT;
    //     for (int i = 0; i < int(enemyPositions.size()); ++i)
    //     {
    //         // if(i == 5 || i == 10)
    //         //     continue;
    //         if (_x == enemyPositions[i].x && _y == enemyPositions[i].y)
    //             continue;
    //         if (enemyPositions[i].x == 0 || enemyPositions[i].y == 0)
    //             continue;
    //         float temp_distance = sqrtf(powf(double(enemyPositions[i].x - _x), 2) + powf(double(enemyPositions[i].y - _y), 2));
    //         if (temp_distance > this->_seek_THR)
    //             continue;
    //         double temp_angle = this->calculateAngle(_x, _y, enemyPositions[i].x, enemyPositions[i].y);
    //         // if (this->checkBlock(this->transformPoint(_x, _y, this->_REAL_WIDTH, this->_REAL_HEIGHT, int((this->_REAL_WIDTH / this->_REAL_HEIGHT) * 1080), 1080), temp_angle, int(temp_distance * (1080. / this->_REAL_HEIGHT))))
    //         //     continue;
    //         if (temp_distance < min_distance)
    //         {
    //             min_distance = temp_distance;
    //             index = i;
    //         }
            
    //     }
    //     return index == -1 ? -1 : this->calculateAngle(_x, _y, enemyPositions[index].x, enemyPositions[index].y);
    // }

    // double RobotDecisionSys::calculateAngle(double x1, double y1, double x2, double y2)
    // {
    //     if (x1 == x2 && y1 == y2)
    //         return 0;
    //     double angle_temp;
    //     double xx, yy;
    //     xx = x2 - x1;
    //     yy = y2 - y1;
    //     if (xx == 0.0)
    //         angle_temp = CV_PI / 2.0;
    //     else
    //         angle_temp = atan(fabs(yy / xx));
    //     if ((xx < 0.0) && (yy >= 0.0))
    //         angle_temp = CV_PI - angle_temp;
    //     else if ((xx < 0.0) && (yy < 0.0))
    //         angle_temp = CV_PI + angle_temp;
    //     else if ((xx >= 0.0) && (yy < 0.0))
    //         angle_temp = CV_PI * 2.0 - angle_temp;
    //     return angle_temp;
    // }

    float RobotDecisionSys::getDistanceTHR()
    {
        return this->_distance_THR;
    }

    void RobotDecisionSys::setDistanceTHR(float thr)
    {
        this->_distance_THR = thr;
    }

    float RobotDecisionSys::getSeekTHR()
    {
        return this->_seek_THR;
    }

    void RobotDecisionSys::setSeekTHR(float thr)
    {
        this->_seek_THR = thr;
    }
