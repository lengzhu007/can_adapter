/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: CANTrans.h
*   Author  : lubing.han
*   Date    : 2017-02-07
*   Describe:
*
********************************************************/
#ifndef CANTRANS_H
#define CANTRANS_H

#include "CANContainer.h"
#include <iostream>
#include <string>
#include <vector>
#include <pthread.h>
using namespace std;

namespace can_adapter {

class CANTrans
{
public:
	CANTrans(vector<int> send_peroid = {20, 20}, vector<int> recv_period = {20, 20}); // ms
	~CANTrans();
	bool start();
	void stop();
	vector<CanBase> get_can_bases(int can_number = 0);
	bool has_sent(int can_number = 0);
	void append_can_bases(const vector<CanBase>& can_bases, int can_number = 0);
private:
	static void* send_callback(void* self_ptr);
	static void* recv_callback(void* self_ptr);
	
	int _send_period[2], _recv_period[2];
	vector<CanBase> *_send_buff[2], *_new_send_buff[2];
	vector<CanBase> *_recv_buff[2], *_new_recv_buff[2];
	pthread_t _send_thread, _recv_thread;
	pthread_mutex_t _send_mutex[2], _recv_mutex[2];
	bool _initialized, _stop;
};

}

#endif