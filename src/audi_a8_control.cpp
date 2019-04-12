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
#include "spi_dbus_hal.h"
using namespace std;

namespace control {
namespace audi_a8 {

AudiA8Control::AudiA8Control(): _canTrans({20, 20}, {20, 20}) {
	std::cout << "015"<< std::endl;
	//can_adapter::CANTrans _canTrans;	／／class CANTrans 先执行CANTrans的构造函数　
	_autodrive_status = 0;
	_gps_time = GetTimeStamp();
	_count = _control_watch = _time_watch = 0;
	std::cout << "016"<< std::endl;
	//_sub_control = nh.subscribe("/car/control", 1, &AudiA8Control::car_control_callback, this);//接受控制指令
	//_sub_time = nh.subscribe("/sensor/gnss/time_reference", 1, &AudiA8Control::gps_time_callback, this); 　
	// _pub_status = nh.advertise<autodrive_msgs::VehicleStatus>("/vehicle/status", 1, true);
	// _pub_can_recv = nh.advertise<std_msgs::String>("/can/recv", 1, true);
	// _pub_can_send = nh.advertise<std_msgs::String>("/can/send", 1, true);
    // _pub_can_brief = nh.advertise<autodrive_msgs::CANbrief>("/vehicle/can_brief", 1, true);
	_msgs_processor.AUDI_A8_DEBUG = AUDI_A8_VEHICLE_DEBUG;//0
	_manual_exit = false;
	_initialized = false;
	_last_control_msg.gear = 0;
	_last_control_msg.throttle = 0.0;
	_last_control_msg.brake = 0.0;
	double steer_offset = 0.0;
	//nh.getParam("/control/steer_offset", steer_offset);//
	//_msgs_processor.set_steer_offset(steer_offset);//
	//_last_control_msg.turn_left_light = 1;
	//_last_control_msg.turn_right_light = 1;
}

AudiA8Control::~AudiA8Control() {

}

void AudiA8Control::run() {
	bool flag1 = false;
	if (_canTrans.start()) {
		 flag1 = true;
		//初始化　can  创建线程  并调用recv_callback -->每1ms　循环调用recv_message 将spi_can_frame的数据存在　recv_buff][0]　_new_recv_buff　
		//初始化　can  创建线程  并调用send_callback -->send_message 将send_buff[0]的数据存在　outputcan 调用　send can spi发出去　
		std::cout<<"Start with manual drive mode"<<std::endl;
		while(flag1){//将收到的数据打印出来　并经由DBC解析　出来　物理值
			// start_time = GetTimeStamp();
		//while (ros::ok()) { 只要ros正常就一直循环
			//ros::ok()在以下几种情况下也会返回false：（1）
		//按下Ctrl-C时（2）我们被一个同名同姓的节点从网络中踢出（3）ros::shutdown()被应用程序的另一部分调用
		//（4）所有的ros::NodeHandles都被销毁了。一旦ros::ok()返回false，所有的ROS调用都会失败。

			//ros::spinOnce();//ROS消息回调处理函数。它俩通常会出现在ROS的主循环中，程序需要不断调用ros::spin() 或
			//ros::spinOnce()，两者区别在于前者调用后不会再返回，也就是你的主程序到这儿就不往下执行了，
			//而后者在调用后还可以继续执行之后的程序。
			// if (recv_control.brake != 0) car_control_callback(recv_control);//只要收到control的指令　就调用car_control_callback
			recv_control.Orentation_Roll =5 ;
			car_control_callback(recv_control);//将recv_control　的控制指令给到　_control_msg　．
			// if (gps_time )//只要收到control的指令　就调用car_control_callback
			//　收到control信息，调用car_control_callback，将control 信息　传给　_control_msg　．然后

			++_control_watch;//car_control_callback 中归零
		
			if (_control_watch > 1000) {
				std::cout<<"Vehicle Control Error: Can not receive control message"<<std::endl;
				if (_autodrive_status == 1 || _autodrive_status == 3) {
					_autodrive_status = 3;
					_manual_exit = true;
				}
				_control_watch = 0;
			}
	
			_recv_cans[0] = _canTrans.get_can_bases(0);//从new_recb_buff 拿出数据 
			// for (auto iter = _recv_cans[0].begin(); iter != _recv_cans[0].end(); ) {//擦除id = 289的报文
			// 	if (iter->get_id() == 289) iter = _recv_cans[0].erase(iter);//
			// 	// iterator erase( iterator _Where);//删除指定位置的元素，返回值是一个迭代器，指向删除元素的下一个元素；
			// 	// 调用erase()方法后，vector后面的元素会向前移位,一般在调用该方法后将迭代器自减一

			// 	else ++iter;
			// }
			_recv_cans[1] = _canTrans.get_can_bases(1);
			if (_recv_cans[0].size() > 0) {
				string recvS = getStrings(_recv_cans[0]);
				string recvSMsg;
				recvSMsg = recvS;
				cout << "/can/recv[0] = " << recvSMsg <<endl;
				// printf("/can/recv[0] = %s \n",recvSMsg);//把收到的原始报文发出去
				//_pub_can_recv.publish(recvSMsg);
			}
			if (_recv_cans[1].size() > 0) {
				string recvS = getStrings(_recv_cans[1]);
				string recvSMsg;//#include <std_msgs>???
				recvSMsg = recvS;
				cout << "/can/recv[1] = " << recvSMsg <<endl;
                // printf("recvSMSG[1] = %s \n",recvSMsg);//把收到的原始报文发出去

				// _pub_can_recv.publish(recvSMsg);
			}
			// if (!_initialized) {
				//usleep(1000);
				//continue;
			// }
			CANbrief _can_brief;
			VehicleStatus _status_msg;
			if (_recv_cans[0].size() > 0) _msgs_processor.process_recv(_can_brief, _status_msg, _recv_cans[0]);
			if (_recv_cans[1].size() > 0) _msgs_processor.process_recv(_can_brief, _status_msg, _recv_cans[1]);
			if (++_count % 20 == 0) {//每20ms输出一次　vehicle status
				_status_msg.header = GetTimeStamp();
				std::cout << "rr_speed = " <<_status_msg.rr_speed << std::endl;
				std::cout <<"speed = " << _status_msg.speed << std::endl;
				std::cout <<"gear = " << _status_msg.gear << std::endl;
				std::cout <<"yaw_rate =" << _status_msg.yaw_rate << std::endl;
				// _pub_status.publish(_status_msg);
                // _pub_can_brief.publish(_can_brief);
			}

			// std::cout << "300   "  <<std::endl;


			/*/////////////////////////////////////////////////////////////////
			if (_msgs_processor.got_all_messages()) {//没看懂？？？？？？？？？？？
				if (_autodrive_status == 0) {
					// if (_msgs_processor.get_wheel_button() == 5) {
					if (_manual_exit) _manual_exit = _control_msg.autodrive_mode;
					if (_control_msg.autodrive_mode == true && !_manual_exit) {
						// enter autodrive mode
						_last_control_msg = _control_msg;
						_autodrive_status = 1;
						std::cout<<"Entering autodrive mode"<<std::endl;
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
						std::cout<<"Autodrive mode ok"<<std::endl;
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
						std::cout<<"Manual drive mode ok"<<std::endl;
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
						std::cout<<"Leaving autodrive mode"<<std::endl;
						_msgs_processor.leave_autodrive_mode();
					}
					else {
						_last_control_msg = _control_msg;
					}
				}
			}
			
			else {
				// std::cout << "310   "  <<std::endl;
				// initializing
				// do nothing
			}
           */////////////////////////////////////////////////////////////
			// std::cout << "320   "  <<std::endl;
			_last_control_msg = _control_msg;
			// _last_control_msg.Orentation_Roll = 5;
			// cout << "_last_control_msg "<< _last_control_msg.Orentation_Roll << endl;
			if (_canTrans.has_sent(0)) {
				_send_cans[0] = _msgs_processor.process_send(_last_control_msg);//
				// cout << "_send_cans[0].size " <<_send_cans[0].size() << endl;
				//通过_last_control_msg 组成　can 形式的报文 放到　_send_msgs 里
				_canTrans.append_can_bases(_send_cans[0], 0);//将_send_cans 里的　新的报文　放到　_new_send_buff
				//_new_send_buff  通过调用子进程　实现发送
				
				if (_send_cans[0].size() > 0) {
					string sendS = getStrings(_send_cans[0]);
					string sendSMsg;
					sendSMsg = sendS;//将组合好的信息放到sendSMsg 
					cout << "/can/send[0] = " << sendSMsg <<endl;//将发送的原始报文打印出来
            
					// _pub_can_send.publish(sendSMsg);
					if (IGNORE_MANUAL_EXIT && _autodrive_status == 0 && _manual_exit) _manual_exit = false;
				}
				
			}
			// sleep(1);
			  usleep(1000);//每隔1ms执行一次
		//}
		}//每隔1ms执行一次

		std::cout<<"060"<<std::endl;
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
		std::cout<<"AudiControl error: can not start CAN"<<std::endl;
	}
}

string AudiA8Control::getStrings(const vector<can_adapter::CanBase>& canBases) const {
	string s;
	for (int i = 0; i < canBases.size(); ++i)
		s += canBases[i].toString();
	return s;
}

void AudiA8Control::car_control_callback(const struct Control &msg) {
	_control_msg = msg;
	_control_watch = 0;
}

// void AudiA8Control::gps_time_callback(const sensor_msgs::TimeReference::ConstPtr &msg) {
// 	_gps_time = msg->time_ref;
// 	_ros_time = msg->header.stamp;
// 	_initialized = true;
// 	_time_watch = 0;
// }

}
}
