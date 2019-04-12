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
#include <iostream>
// #include <ros/ros.h>
using namespace std;
using namespace control;
using namespace control::audi_a8;

int main(int argc, char** argv) {
	// ros::init(argc, argv, "audi_a8_adapter");
	// ros::NodeHandle nh;
	// cout << "010"<< endl;
	AudiA8Control audia8Control;
	// std::map<int, svcFunc> service_recieve_map_;
	// std::cout<<"100"<<std::endl;
	audia8Control.run();
	// std::cout<<"200"<<std::endl;
	return 0;
	
}