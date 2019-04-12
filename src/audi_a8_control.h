/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: audi_control_2.h
*   Author  : lubing.han
*   Date    : 2017-8-16
*   Describe:
*
********************************************************/
#ifndef AUDI_A8_CONTROL_H
#define AUDI_A8_CONTROL_H

#include "CANTrans.h"
#include "audi_a8_msgs_processor.h"
// #include <std_msgs/String.h>
// #include <sensor_msgs/TimeReference.h>
// #include <ros/package.h>
// #include <ros/ros.h>
// #include <autodrive_msgs/CANbrief.h>
using namespace std;

namespace control {
namespace audi_a8 {

#define AUDI_A8_VEHICLE_DEBUG 0
#define IGNORE_MANUAL_EXIT 0

class AudiA8Control
{
public:
	AudiA8Control();
	~AudiA8Control();
	void run();

private:
	void car_control_callback(const struct Control &msg);
	// void gps_time_callback(const sensor_msgs::TimeReference::ConstPtr &msg);
	string getStrings(const vector<can_adapter::CanBase>& canBases) const;

	float _gps_time,start_time;
	int _autodrive_status; // 0: manual, 1: entering autodrive, 2: leaving autodrive, 3: autodrive
	int _count, _control_watch, _time_watch;
	// ros::Subscriber _sub_control, _sub_time;
	// ros::Publisher _pub_status, _pub_can_recv, _pub_can_send, _pub_can_brief;
	// autodrive_msgs::Control _control_msg, _last_control_msg;
	// autodrive_msgs::VehicleStatus _status_msg;
    // autodrive_msgs::CANbrief _can_brief;
    struct Control recv_control,_control_msg, _last_control_msg;

	AudiA8MsgsProcessor _msgs_processor;//调用AudiA8MsgsProcessor的构造函数，并加载参数文件　DBC unit  
	can_adapter::CANTrans _canTrans;
	vector<can_adapter::CanBase> _send_cans[2];
	//定义_Send_cands[0],_Send_cans[1] 两个　vector<can_adapter::CanBase>　类型的vector  对应两个can通道
	vector<can_adapter::CanBase> _recv_cans[2];
	bool _manual_exit, _initialized;
};

}
}

#endif
