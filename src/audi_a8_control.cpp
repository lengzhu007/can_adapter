/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: audi_control.cpp
*   Author  : lubing.han
*   Date    : 2017-02-17
*   Describe:
*
********************************************************/
#include "audi_a8_control.h"
#include <unistd.h>
#include <sys/time.h>
using namespace std;

namespace control {
namespace audi_a8 {

AudiA8Control::AudiA8Control(ros::NodeHandle nh): _canTrans({20, 20}, {20, 20}) {
	_autodrive_status = 0;
	_gps_time = _ros_time = ros::Time::now();
	_count = _control_watch = _time_watch = 0;
	_sub_control = nh.subscribe("/car/control", 1, &AudiA8Control::car_control_callback, this);
	_sub_time = nh.subscribe("/sensor/gnss/time_reference", 1, &AudiA8Control::gps_time_callback, this);
	_pub_status = nh.advertise<autodrive_msgs::VehicleStatus>("/vehicle/status", 1, true);
	_pub_can_recv = nh.advertise<std_msgs::String>("/can/recv", 1, true);
	_pub_can_send = nh.advertise<std_msgs::String>("/can/send", 1, true);
    _pub_can_brief = nh.advertise<autodrive_msgs::CANbrief>("/vehicle/can_brief", 1, true);
	_msgs_processor.AUDI_A8_DEBUG = AUDI_A8_VEHICLE_DEBUG;
	_manual_exit = false;
	_initialized = false;
	_last_control_msg.gear = 0;
	_last_control_msg.throttle = 0.0;
	_last_control_msg.brake = 0.0;
	double steer_offset = 0.0;
	nh.getParam("/control/steer_offset", steer_offset);
	_msgs_processor.set_steer_offset(steer_offset);
	//_last_control_msg.turn_left_light = 1;
	//_last_control_msg.turn_right_light = 1;
}

AudiA8Control::~AudiA8Control() {

}

void AudiA8Control::run() {
	if (_canTrans.start()) {
		ROS_WARN("Start with manual drive mode");
		while (ros::ok()) {
			ros::spinOnce();
			++_control_watch;
			++_time_watch;
			if (_control_watch > 1000) {
				ROS_WARN("Vehicle Control Error: Can not receive control message");
				if (_autodrive_status == 1 || _autodrive_status == 3) {
					_autodrive_status = 3;
					_manual_exit = true;
				}
				_control_watch = 0;
			}
			// if (_time_watch > 1000) {
			// 	ROS_WARN("Vehicle Control Error: Can not receive gps time reference");
			// 	_time_watch = 0;
			// }
			_recv_cans[0] = _canTrans.get_can_bases(0);
			for (auto iter = _recv_cans[0].begin(); iter != _recv_cans[0].end(); ) {
				if (iter->get_id() == 289) iter = _recv_cans[0].erase(iter);
				else ++iter;
			}
			_recv_cans[1] = _canTrans.get_can_bases(1);
			if (_recv_cans[0].size() > 0) {
				string recvS = getStrings(_recv_cans[0]);
				std_msgs::String recvSMsg;
				recvSMsg.data = recvS;
				_pub_can_recv.publish(recvSMsg);
			}
			if (_recv_cans[1].size() > 0) {
				string recvS = getStrings(_recv_cans[1]);
				std_msgs::String recvSMsg;
				recvSMsg.data = recvS;
				_pub_can_recv.publish(recvSMsg);
			}
			// if (!_initialized) {
				//usleep(1000);
				//continue;
			// }
			if (_recv_cans[0].size() > 0) _msgs_processor.process_recv(_can_brief, _status_msg, _recv_cans[0]);
			if (_recv_cans[1].size() > 0) _msgs_processor.process_recv(_can_brief, _status_msg, _recv_cans[1]);
			if (++_count % 20 == 0) {
				_status_msg.header.stamp = _gps_time + (ros::Time::now() - _ros_time);
				_pub_status.publish(_status_msg);
                _pub_can_brief.publish(_can_brief);
			}
			if (_msgs_processor.got_all_messages()) {
				if (_autodrive_status == 0) {
					// if (_msgs_processor.get_wheel_button() == 5) {
					if (_manual_exit) _manual_exit = _control_msg.autodrive_mode;
					if (_control_msg.autodrive_mode == true && !_manual_exit) {
						// enter autodrive mode
						_last_control_msg = _control_msg;
						_autodrive_status = 1;
						ROS_WARN("Entering autodrive mode");
						_msgs_processor.enter_autodrive_mode();
					}
					else {
						// _last_control_msg = _control_msg;
						// do nothing
					}
				}
				else if (_autodrive_status == 1) {
					if (_msgs_processor.all_in_autodrive()) {
						_last_control_msg = _control_msg;
						_autodrive_status = 3;
						ROS_WARN("Autodrive mode ok");
						_msgs_processor.entered_autodrive_mode();
					}
					else if (_control_msg.autodrive_mode == false)
						_autodrive_status = 3;
					else {
						_last_control_msg = _control_msg;
					}
				}
				else if (_autodrive_status == 2) {
					if (true) {
						_autodrive_status = 0;
						ROS_WARN("Manual drive mode ok");
						_msgs_processor.left_autodrive_mode();
					}
					else if (_control_msg.autodrive_mode == true && !_manual_exit)
						_autodrive_status = 0;
					else {
						// do nothing
					}
				}
				else if (_autodrive_status == 3) {
					//cout << _manual_exit << (int)(_control_msg.autodrive_mode) << int(_msgs_processor.manual_exit()) << _msgs_processor.all_in_manual() << endl;
					if (_msgs_processor.manual_exit() || _manual_exit
						|| (bool)((int)(_control_msg.autodrive_mode)) == false || _msgs_processor.all_in_manual()) {
						// leave autodrive mode
						if (_control_msg.autodrive_mode) _manual_exit = true;
						_last_control_msg = _control_msg;
						_last_control_msg.gear = 0;
						_last_control_msg.brake = 0.0;
						_last_control_msg.throttle = 0.0;
						_last_control_msg.turn_left_light = _last_control_msg.turn_right_light = 0;
						_autodrive_status = 2;
						ROS_WARN("Leaving autodrive mode");
						_msgs_processor.leave_autodrive_mode();
					}
					else {
						_last_control_msg = _control_msg;
					}
				}
			}
			else {
				// initializing
				// do nothing
			}
			// _last_control_msg = _control_msg;
			if (_canTrans.has_sent(0)) {
				_send_cans[0] = _msgs_processor.process_send(_last_control_msg);
				_canTrans.append_can_bases(_send_cans[0], 0);
				if (_send_cans[0].size() > 0) {
					string sendS = getStrings(_send_cans[0]);
					std_msgs::String sendSMsg;
					sendSMsg.data = sendS;
					_pub_can_send.publish(sendSMsg);
					if (IGNORE_MANUAL_EXIT && _autodrive_status == 0 && _manual_exit) _manual_exit = false;
				}
			}
			usleep(1000);
		}
		_msgs_processor.leave_autodrive_mode();
		for (int i = 0; i < 5; ++i) {
			_msgs_processor.leave_autodrive_mode();
			_last_control_msg.throttle = 0.0;
			_last_control_msg.brake = 0.0;
			_last_control_msg.turn_left_light = _last_control_msg.turn_right_light = 0;
			_send_cans[0] = _msgs_processor.process_send(_last_control_msg);
			_canTrans.append_can_bases(_send_cans[0], 0);
			usleep(20000);
		}
		_canTrans.stop();
	}
	else {
		ROS_ERROR("AudiControl error: can not start CAN");
	}
}

string AudiA8Control::getStrings(const vector<can_adapter::CanBase>& canBases) const {
	string s;
	for (int i = 0; i < canBases.size(); ++i)
		s += canBases[i].toString();
	return s;
}

void AudiA8Control::car_control_callback(const autodrive_msgs::Control::ConstPtr &msg) {
	_control_msg = *msg;
	_control_watch = 0;
}

void AudiA8Control::gps_time_callback(const sensor_msgs::TimeReference::ConstPtr &msg) {
	_gps_time = msg->time_ref;
	_ros_time = msg->header.stamp;
	_initialized = true;
	_time_watch = 0;
}

}
}
