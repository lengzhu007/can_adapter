/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: test_audi_can.cpp
*   Author  : lubing.han
*   Date    : 2017-08-16
*   Describe:
*
********************************************************/
#include "CANTrans.h"
#include "audi_a8_control.h"
#include <ros/ros.h>
using namespace std;
using namespace control;
using namespace control::audi_a8;
int main(int argc, char** argv) {
	ros::init(argc, argv, "audi_a8_adapter");
	ros::NodeHandle nh;
	AudiA8Control audia8Control(nh);
	audia8Control.run();
	return 0;
}