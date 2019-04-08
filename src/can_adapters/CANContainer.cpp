/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: CANContainer.cpp
*   Author  : lubing.han
*   Date    : 2017-02-07
*   Describe:
*
********************************************************/

#include "CANContainer.h"
#include "spi_dbus_hal.h"
#include "service_handler_module.h"
#include "recv_send.h"
#include <iostream>
#include <string>
using namespace std;

namespace can_adapter
{

bool stop_can()
{
	void spi_dbus_exit();
	void spi_slave_dbus_exit();
}

bool init_can()
{
	ServiceHandler *service_handler = new ServiceHandler();
	service_handler->Init();
}

void send_messages(const vector<CanBase> &send_buff, int can_number)
{
	
	//将sendbuff 转换为　                     outputcan
	//CanBase1 CanBase2  CanBase3           canframe1 canframe2 canframe3
	//
	#define MAX_LENGTH 4*1024//???????????????
	 if (send_buff.size() == 0)
		return;
	int n = send_buff.size() < MAX_LENGTH ? send_buff.size() : MAX_LENGTH;
    OutputCAN *outputcan = new OutputCAN();
	for (int i = 0; i < n; ++i)
	{
		(*outputcan)[i].can_id = send_buff[i].get_id();
		(*outputcan)[i].can_dlc = 8;

		for (int j = 0; j < 8; ++j)
			(*outputcan)[i].data[j] = 0;//清零

		const BYTE *send_bytes = send_buff[i].get_bytes();
		for (int j = 0; j < 8; ++j)
			(*outputcan)[i].data[j] = send_bytes[j];
	}
  
	send_can_spi(outputcan);///???????? 可以否　
}

void recv_messages(vector<CanBase> &recv_buff, int can_number)
{
	recv_buff.clear();

	SpiCanFrameType spi_can_frame;
	recv_can_spi(spi_can_frame);//调用函数 调用spi_can_frame return spi_can_frame //


	int n = spi_can_frame.can_frame_.size();
	for (int i = 0; i < n; ++i)
	{
		CanBase recvBase(spi_can_frame.can_frame_[i].can_id);
		recvBase.set_time_stamp((double)spi_can_frame.ts_);
		recvBase.set_bytes(spi_can_frame.can_frame_[i].data);
		recv_buff.push_back(recvBase);
	}
}

} // namespace can_adapter
