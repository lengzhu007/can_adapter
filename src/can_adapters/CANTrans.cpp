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
#include <pthread.h>
using namespace std;

namespace can_adapter
{

CANTrans::CANTrans(vector<int> send_period, vector<int> recv_period)
{
	// std::cout << "cantrans0000000000000"<< std::endl;
	_send_period[0] = send_period[0], _send_period[1] = send_period[1];
	_recv_period[0] = recv_period[0], _recv_period[1] = recv_period[1];
	_send_buff[0] = new vector<CanBase>();
	_send_buff[0]->reserve(8000); //reserve用来（预留空间，）改变capacity，不改变size，会去分配内存，但不会构造出对象
	_send_buff[1] = new vector<CanBase>();
	_send_buff[1]->reserve(8000);
	_new_send_buff[0] = new vector<CanBase>();
	_new_send_buff[0]->reserve(8000);
	_new_send_buff[1] = new vector<CanBase>();
	_new_send_buff[1]->reserve(8000);
	_recv_buff[0] = new vector<CanBase>();
	_recv_buff[0]->reserve(8000);
	_recv_buff[1] = new vector<CanBase>();
	_recv_buff[1]->reserve(8000);
	_new_recv_buff[0] = new vector<CanBase>();
	_new_recv_buff[0]->reserve(8000);
	_new_recv_buff[1] = new vector<CanBase>();
	_new_recv_buff[1]->reserve(8000);
	_initialized = false;
	_stop = true;
}

CANTrans::~CANTrans()
{
	delete _send_buff[0], delete _send_buff[1];
	delete _new_send_buff[0], delete _new_send_buff[1];
	delete _recv_buff[0], delete _recv_buff[1];
	delete _new_recv_buff[0], delete _new_recv_buff[1];
}

bool CANTrans::start()
{
	// std::cout << "045 " << std::endl;
	if (_initialized)
	{
		cout << "CANTrans: start: already initialized" << endl;
		return false;
	}
	// std::cout << "050 " << std::endl;
	if (!init_can())
		return false; //
	_stop = false;
	_initialized = true;
	pthread_mutex_init(&_send_mutex[0], NULL);
	pthread_mutex_init(&_send_mutex[1], NULL);
	pthread_mutex_init(&_recv_mutex[0], NULL);
	pthread_mutex_init(&_recv_mutex[1], NULL);							 //初始化互斥变量
	pthread_create(&_send_thread, NULL, &CANTrans::send_callback, this); //创见线程　　并从CANTrans::send_callback这个函数开始执行
	pthread_create(&_recv_thread, NULL, &CANTrans::recv_callback, this); //创见线程　　并从CANTrans::recv_callback这个函数开始执行
	return true;
}

void CANTrans::stop()
{
	if (!_initialized)
		return;
	_stop = true;
	pthread_join(_send_thread, NULL); //pthread_join()函数，以阻塞的方式等待thread指定的线程结束//本进程等待　＿send_thread的结束
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

vector<CanBase> CANTrans::get_can_bases(int can_number)
{
	if (can_number != 0)
		can_number = 1;
	vector<CanBase> recvBases;
	if (!_initialized)
		return recvBases;
	pthread_mutex_lock(&_recv_mutex[can_number]);
	recvBases = *_new_recv_buff[can_number];
	_new_recv_buff[can_number]->clear();
	pthread_mutex_unlock(&_recv_mutex[can_number]);
	return recvBases;
}

bool CANTrans::has_sent(int can_number)
{
	if (can_number != 0)
		can_number = 1;
	pthread_mutex_lock(&_send_mutex[can_number]);
	bool flag = _new_send_buff[can_number]->size() == 0;
	pthread_mutex_unlock(&_send_mutex[can_number]);
	//  cout << "flag" << flag << endl;
	return true;
	//flag;
}

void CANTrans::append_can_bases(const vector<CanBase> &can_bases, int can_number)
{
	if (can_number != 0)
		can_number = 1;
	if (!_initialized)
		return;
	pthread_mutex_lock(&_send_mutex[can_number]);
	if (_new_send_buff[can_number]->size() > 5000)
		_new_send_buff[can_number]->clear();
	_new_send_buff[can_number]->insert(_new_send_buff[can_number]->end(), can_bases.begin(), can_bases.end());
	pthread_mutex_unlock(&_send_mutex[can_number]);
}

void *CANTrans::send_callback(void *self_ptr)
{
	CANTrans *canTrans = (CANTrans *)self_ptr;
	struct timeval t1[2], t2;
	double time_offsets[2] = {0.0, 3.14};
	double dt, cur_time, last_time;
	gettimeofday(&t1[0], NULL);
	gettimeofday(&t1[1], NULL);
	while (!canTrans->_stop)
	{
		gettimeofday(&t2, NULL);
		cur_time = double(t2.tv_sec) * 1000.0 + double(t2.tv_usec) / 1000.0;
		last_time = double(t1[0].tv_sec) * 1000.0 + double(t1[0].tv_usec) / 1000.0;
		dt = cur_time - last_time;
		if (canTrans->_send_period[0] > 0 && dt > canTrans->_send_period[0] - time_offsets[0])
		{
			pthread_mutex_lock(&canTrans->_send_mutex[0]);
			canTrans->_send_buff[0]->clear();
			//作用：将会清空temp中的所有元素，包括temp开辟的空间（size），但是capacity会保留，
			//即不可以以temp[1]这种形式赋初值，只能通过temp.push_back(value)的形式赋初值。
			vector<CanBase> *tmp = canTrans->_send_buff[0];

			for (int i = 0; i < tmp->size(); i++)
			{
				CanBase a = tmp->at(i);//at函数和[]函数使用可以说是一模一样. 都是为了访问对应index中存储的数据时
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
		cur_time = double(t2.tv_sec) * 1000.0 + double(t2.tv_usec) / 1000.0;
		last_time = double(t1[1].tv_sec) * 1000.0 + double(t1[1].tv_usec) / 1000.0;
		dt = cur_time - last_time;
		if (canTrans->_send_period[1] > 0 && dt > canTrans->_send_period[1] - time_offsets[1])
		{
			pthread_mutex_lock(&canTrans->_send_mutex[1]);
			canTrans->_send_buff[1]->clear();
			vector<CanBase> *tmp = canTrans->_send_buff[1];
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

void *CANTrans::recv_callback(void *self_ptr)
{
	CANTrans *canTrans = (CANTrans *)self_ptr;
	struct timeval t1[2], t2;
	double time_offsets[2] = {3.14 * 3, 3.14 * 4};
	double dt, cur_time, last_time;
	gettimeofday(&t1[0], NULL);
	gettimeofday(&t1[1], NULL);
	while (!canTrans->_stop)
	{
		gettimeofday(&t2, NULL);
		cur_time = double(t2.tv_sec) * 1000.0 + double(t2.tv_usec) / 1000.0;
		last_time = double(t1[0].tv_sec) * 1000.0 + double(t1[0].tv_usec) / 1000.0;
		dt = cur_time - last_time;
		if (canTrans->_recv_period[0] > 0 && dt > canTrans->_recv_period[0] - time_offsets[0])
		{
			//每隔（canTrans->_recv_period[0]＝20　） - 9.42)　这么长时间收一次can 报文
			canTrans->_recv_buff[0]->clear();
			recv_messages(*(canTrans->_recv_buff[0]), 0); //接受spi_can_frame的消息　存入_recv_buff[0]
			pthread_mutex_lock(&canTrans->_recv_mutex[0]);
			if (canTrans->_new_recv_buff[0]->size() > 5000)
				canTrans->_new_recv_buff[0]->clear();
			canTrans->_new_recv_buff[0]->insert(canTrans->_new_recv_buff[0]->end(),
												canTrans->_recv_buff[0]->begin(), canTrans->_recv_buff[0]->end()); //
																												   //将　_recv_buff的数据放入　_new_recv_buff　保证所有数据都是新的
			pthread_mutex_unlock(&canTrans->_recv_mutex[0]);
			t1[0] = t2;
			time_offsets[0] = dt - (canTrans->_recv_period[0] - time_offsets[0]);
		}
		gettimeofday(&t2, NULL);
		cur_time = double(t2.tv_sec) * 1000.0 + double(t2.tv_usec) / 1000.0;
		last_time = double(t1[1].tv_sec) * 1000.0 + double(t1[1].tv_usec) / 1000.0;
		dt = cur_time - last_time;
		if (canTrans->_recv_period[1] > 0 && dt > canTrans->_recv_period[1] - time_offsets[1])
		{
			canTrans->_recv_buff[1]->clear();
			recv_messages(*(canTrans->_recv_buff[1]), 1); //接受spi_can_frame的消息　存入_recv_buff[1]
			pthread_mutex_lock(&canTrans->_recv_mutex[1]);
			if (canTrans->_new_recv_buff[1]->size() > 5000)
				canTrans->_new_recv_buff[1]->clear();
			canTrans->_new_recv_buff[1]->insert(canTrans->_new_recv_buff[1]->end(),
												canTrans->_recv_buff[1]->begin(), canTrans->_recv_buff[1]->end());
			pthread_mutex_unlock(&canTrans->_recv_mutex[1]);
			t1[1] = t2;
			time_offsets[1] = dt - (canTrans->_recv_period[1] - time_offsets[1]);
		}
		usleep(1000); //每隔1ms循环一次
	}
	pthread_exit(0); //线程通过调用pthread_exit函数终止执行
}

} // namespace can_adapter
