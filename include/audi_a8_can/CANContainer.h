/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: CANContainer.h
*   Author  : lubing.han
*   Date    : 2017-02-07
*   Describe:
*
********************************************************/

#ifndef CANCONTAINER_H
#define CANCONTAINER_H

#include "can_base.h"
#include <vector>
using namespace std;

#define CAN_NUMBER 0

namespace can_adapter {

bool init_can();

bool stop_can();

void send_messages(const vector<CanBase>& send_buff, int can_number = -1);

void recv_messages(vector<CanBase>& recv_buff, int can_number = -1);

}

#endif
