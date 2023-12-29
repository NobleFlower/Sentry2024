#include <VCOMCOMM.h>

#include "../include/communication/communication_node.hpp"
#include "global_interface/ObjHP.h"
#include "global_interface/Serial.h"
#include "ros/time.h"

Receive::Receive() {
    GameInfo_pub_ = nh.advertise<::global_interface::GameInfo>("/game_info", 10);
    ObjHP_pub_ = nh.advertise<::global_interface::ObjHP>("/obj_hp", 10);
    Serial_pub_ = nh.advertise<::global_interface::Serial>("/serial_msg", 10);
}

void Receive::Re_GameInfo()
{
    VCOMCOMM vcom_receive1;
    QByteArray QReceive_data1;
    Game_Info_t *temp1;
    global_interface::GameInfo Game_Info;

    vcom_receive1.receiveData(0x01, 0x01, QReceive_data1);
    temp1 = (Game_Info_t *)QReceive_data1.data();
    Game_Info.timestamp = temp1->TimeStamp;
    Game_Info.game_stage = temp1->GameStage;
    Game_Info.header.stamp = ros::Time::now();

    GameInfo_pub_.publish(Game_Info);
}

void Receive::Re_ObjHP()
{
    VCOMCOMM vcom_receive2;
    QByteArray QReceive_data2;
    ObjHP_t *temp2;
    global_interface::ObjHP ObjHP;

    vcom_receive2.receiveData(0x02, 0x02, QReceive_data2);
    temp2 = (ObjHP_t *)QReceive_data2.data();
    ObjHP.hp[0] = temp2->r_Hero_HP;
    ObjHP.hp[1] = temp2->r_Engineer_HP;
    ObjHP.hp[2] = temp2->r_Infantry_3_HP;
    ObjHP.hp[3] = temp2->r_Infantry_4_HP;
    ObjHP.hp[4] = temp2->r_Infantry_5_HP;
    ObjHP.hp[5] = temp2->r_Sentry_HP;
    ObjHP.hp[6] = temp2->r_Outpost_HP;
    ObjHP.hp[7] = temp2->r_Base_HP;
    ObjHP.hp[8] = temp2->b_Hero_HP;
    ObjHP.hp[9] = temp2->b_Engineer_HP;
    ObjHP.hp[10] = temp2->b_Infantry_3_HP;
    ObjHP.hp[11] = temp2->b_Infantry_4_HP;
    ObjHP.hp[12] = temp2->b_Infantry_5_HP;
    ObjHP.hp[13] = temp2->b_Sentry_HP;
    ObjHP.hp[14] = temp2->b_Outpost_HP;
    ObjHP.hp[15] = temp2->b_Base_HP;
    ObjHP.header.stamp = ros::Time::now();

    ObjHP_pub_.publish(ObjHP);
}

void Receive::Re_Serial()
{
    VCOMCOMM vcom_receive3;
    QByteArray QReceive_data3;
    Serial_t *temp3;
    global_interface::Serial Serial;

    vcom_receive3.receiveData(0x03, 0x03, QReceive_data3);
    temp3 = (Serial_t *)QReceive_data3.data();

    Serial.header.stamp = ros::Time::now();

    Serial_pub_.publish(Serial);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Communication");
    Receive rece;
    return 0;
}