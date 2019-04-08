/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: CANTrans.cpp
*   Author  : lubing.han
*   Date    : 2017-02-07
*   Describe:
*
********************************************************/

#include "CANTrans.h"
#include <unistd.h>
#include <sys/time.h>
using namespace std;

namespace can_adapter {

CANTrans::CANTrans(vector<int> send_period, vector<int> recv_period) {
	_send_period[0] = send_period[0], _send_period[1] = send_period[1]; 
	_recv_period[0] = recv_period[0], _recv_period[1] = recv_period[1];
	_send_buff[0] = new vector<CanBase>(); _send_buff[0]->reserve(8000);
	_send_buff[1] = new vector<CanBase>(); _send_buff[1]->reserve(8000);
	_new_send_buff[0] = new vector<CanBase>(); _new_send_buff[0]->reserve(8000);
	_new_send_buff[1] = new vector<CanBase>(); _new_send_buff[1]->reserve(8000);
	_recv_buff[0] = new vector<CanBase>(); _recv_buff[0]->reserve(8000);
	_recv_buff[1] = new vector<CanBase>(); _recv_buff[1]->reserve(8000);
	_new_recv_buff[0] = new vector<CanBase>(); _new_recv_buff[0]->reserve(8000);
	_new_recv_buff[1] = new vector<CanBase>(); _new_recv_buff[1]->reserve(8000);
	_initialized = false;
	_stop = true;
}

CANTrans::~CANTrans() {
	delete _send_buff[0], delete _send_buff[1];
	delete _new_send_buff[0], delete _new_send_buff[1];
	delete _recv_buff[0], delete _recv_buff[1];
	delete _new_recv_buff[0], delete _new_recv_buff[1];
}

bool CANTrans::start() {
	if (_initialized) {
		cout << "CANTrans: start: already initialized" << endl;
		return false;
	}
	if (!init_can()) return false;
	_stop = false;
	_initialized = true;
	pthread_mutex_init(&_send_mutex[0], NULL);
	pthread_mutex_init(&_send_mutex[1], NULL);
	pthread_mutex_init(&_recv_mutex[0], NULL);
	pthread_mutex_init(&_recv_mutex[1], NULL);
	pthread_create(&_send_thread, NULL, &CANTrans::send_callback, this);
	pthread_create(&_recv_thread, NULL, &CANTrans::recv_callback, this);
	return true;
}

void CANTrans::stop() {
	if (!_initialized) return;
	_stop = true;
	pthread_join(_send_thread, NULL);
	pthread_join(_recv_thread, NULL);
	stop_can();
	_send_buff[0]->clear();
	_send_buff[1]->clear();
	_new_send_buff[0]->clear();
	_new_send_buff[1]->clear();
	_recv_buff[0]->clear();
	_recv_buff[1]->clear();
	_new_recv_buff[0]->clear();
	_new_recv_buff[1]->clear();
	_initialized = false;
}

vector<CanBase> CANTrans::get_can_bases(int can_number) {
	if (can_number != 0) can_number = 1;
	vector<CanBase> recvBases;
	if (!_initialized) return recvBases;
	pthread_mutex_lock(&_recv_mutex[can_number]);
	recvBases = *_new_recv_buff[can_number];
	_new_recv_buff[can_number]->clear();
	pthread_mutex_unlock(&_recv_mutex[can_number]);
	return recvBases;
}

bool CANTrans::has_sent(int can_number) {
	if (can_number != 0) can_number = 1;
	pthread_mutex_lock(&_send_mutex[can_number]);
	bool flag = _new_send_buff[can_number]->size() == 0;
	pthread_mutex_unlock(&_send_mutex[can_number]);
//  cout << "flag" << flag << endl;
	return flag;  
}

void CANTrans::append_can_bases(const vector<CanBase>& can_bases, int can_number) {
	if (can_number != 0) can_number = 1;
	if (!_initialized) return;
	pthread_mutex_lock(&_send_mutex[can_number]);
	if (_new_send_buff[can_number]->size() > 5000) _new_send_buff[can_number]->clear();
	_new_send_buff[can_number]->insert(_new_send_buff[can_number]->end(), can_bases.begin(), can_bases.end());
	pthread_mutex_unlock(&_send_mutex[can_number]);
}

void* CANTrans::send_callback(void* self_ptr) {
	CANTrans* canTrans = (CANTrans*)self_ptr;
	struct timeval t1[2], t2;
	double time_offsets[2] = {0.0, 3.14};
	double dt, cur_time, last_time;
	gettimeofday(&t1[0], NULL);
	gettimeofday(&t1[1], NULL);
	while (!canTrans->_stop) {
		gettimeofday(&t2, NULL);
		cur_time = double(t2.tv_sec)*1000.0 + double(t2.tv_usec)/1000.0;
		last_time = double(t1[0].tv_sec)*1000.0 + double(t1[0].tv_usec)/1000.0;
		dt = cur_time - last_time;
		if (canTrans->_send_period[0] > 0 && dt > canTrans->_send_period[0] - time_offsets[0]) {
			pthread_mutex_lock(&canTrans->_send_mutex[0]);
			canTrans->_send_buff[0]->clear();
			vector<CanBase>* tmp = canTrans->_send_buff[0];

      for (int i=0;i<tmp->size();i++)
             {
            CanBase  a = tmp->at(i);
         cout << "tmp=" << a.toString() << endl;
              }

			canTrans->_send_buff[0] = canTrans->_new_send_buff[0];
      canTrans->_new_send_buff[0] = tmp;
			pthread_mutex_unlock(&canTrans->_send_mutex[0]);
			if (canTrans->_send_buff[0]->size() > 0)
				send_messages(*(canTrans->_send_buff[0]), 0);
			t1[0] = t2;
			time_offsets[0] = dt - (canTrans->_send_period[0] - time_offsets[0]);
		}
		gettimeofday(&t2, NULL);
		cur_time = double(t2.tv_sec)*1000.0 + double(t2.tv_usec)/1000.0;
		last_time = double(t1[1].tv_sec)*1000.0 + double(t1[1].tv_usec)/1000.0;
		dt = cur_time - last_time;
		if (canTrans->_send_period[1] > 0 && dt > canTrans->_send_period[1] - time_offsets[1]) {
			pthread_mutex_lock(&canTrans->_send_mutex[1]);
			canTrans->_send_buff[1]->clear();
			vector<CanBase>* tmp = canTrans->_send_buff[1];
			canTrans->_send_buff[1] = canTrans->_new_send_buff[1];
			canTrans->_new_send_buff[1] = tmp;
			pthread_mutex_unlock(&canTrans->_send_mutex[1]);
			if (canTrans->_send_buff[1]->size() > 0)
				send_messages(*(canTrans->_send_buff[1]), 1);
			t1[1] = t2;
			time_offsets[1] = dt - (canTrans->_send_period[1] - time_offsets[1]);
		}
		usleep(1000);
	}
	pthread_exit(0);
}

void* CANTrans::recv_callback(void* self_ptr) {
	CANTrans* canTrans = (CANTrans*)self_ptr;
	struct timeval t1[2], t2;
	double time_offsets[2] = {3.14 * 3, 3.14 * 4};
	double dt, cur_time, last_time;
	gettimeofday(&t1[0], NULL);
	gettimeofday(&t1[1], NULL);
	while (!canTrans->_stop) {
		gettimeofday(&t2, NULL);
		cur_time = double(t2.tv_sec)*1000.0 + double(t2.tv_usec)/1000.0;
		last_time = double(t1[0].tv_sec)*1000.0 + double(t1[0].tv_usec)/1000.0;
		dt = cur_time - last_time;
		if (canTrans->_recv_period[0] > 0 && dt > canTrans->_recv_period[0] - time_offsets[0]) {
			canTrans->_recv_buff[0]->clear();
			recv_messages(*(canTrans->_recv_buff[0]), 0);
			pthread_mutex_lock(&canTrans->_recv_mutex[0]);
			if (canTrans->_new_recv_buff[0]->size() > 5000)
				canTrans->_new_recv_buff[0]->clear();
			canTrans->_new_recv_buff[0]->insert(canTrans->_new_recv_buff[0]->end(),
				canTrans->_recv_buff[0]->begin(), canTrans->_recv_buff[0]->end());
    		pthread_mutex_unlock(&canTrans->_recv_mutex[0]);
			t1[0] = t2;
			time_offsets[0] = dt - (canTrans->_recv_period[0] - time_offsets[0]);
		}
		gettimeofday(&t2, NULL);
		cur_time = double(t2.tv_sec)*1000.0 + double(t2.tv_usec)/1000.0;
		last_time = double(t1[1].tv_sec)*1000.0 + double(t1[1].tv_usec)/1000.0;
		dt = cur_time - last_time;
		if (canTrans->_recv_period[1] > 0 && dt > canTrans->_recv_period[1] - time_offsets[1]) {
			canTrans->_recv_buff[1]->clear();
			recv_messages(*(canTrans->_recv_buff[1]), 1);
			pthread_mutex_lock(&canTrans->_recv_mutex[1]);
			if (canTrans->_new_recv_buff[1]->size() > 5000)
				canTrans->_new_recv_buff[1]->clear();
			canTrans->_new_recv_buff[1]->insert(canTrans->_new_recv_buff[1]->end(),
				canTrans->_recv_buff[1]->begin(), canTrans->_recv_buff[1]->end());
			pthread_mutex_unlock(&canTrans->_recv_mutex[1]);
			t1[1] = t2;
			time_offsets[1] = dt - (canTrans->_recv_period[1] - time_offsets[1]);
		}
		usleep(1000);
	}
	pthread_exit(0);
}


}
