/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: audi_msgs_processor.h
*   Author  : lubing.han
*   Date    : 2017-08-16
*   Describe:
*
********************************************************/
#ifndef AUDI_A8_MSGS_PROCESSOR_H
#define AUDI_A8_MSGS_PROCESSOR_H

#include "can_translator.h"
#include <autodrive_msgs/VehicleStatus.h>
#include <autodrive_msgs/Control.h>
#include <autodrive_msgs/CarStatus.h>
#include <autodrive_msgs/CANbrief.h>
using namespace std;
using namespace can_adapter;

namespace control {
namespace audi_a8 {

class AudiA8MsgsProcessor
{
public:
	AudiA8MsgsProcessor();
	~AudiA8MsgsProcessor();

	const vector<CanBase>& process_send(const autodrive_msgs::Control &msg);
	void process_recv(autodrive_msgs::CANbrief & can_brief, autodrive_msgs::VehicleStatus &vehicleStatus, const vector<CanBase> &canBases);

	// send funs
	void enter_autodrive_mode();
	void leave_autodrive_mode();
	void entered_autodrive_mode();
	void left_autodrive_mode();
	// recv funs
	bool got_all_messages() const;
	bool all_in_autodrive() const;
	bool all_in_manual() const;
	bool manual_exit() const;

	void set_steer_offset(double steer_offset) {_steer_offset = steer_offset;}
	bool AUDI_A8_DEBUG;
	
private:
	void process_recv_base(autodrive_msgs::VehicleStatus &vehicleStatus, const vector<CanBase> &canBases);

	vector<CanBase> _send_msgs;
	tdCL_CanTranslator _canTranslator;
	set<int> _received, _inAutodrived;
	int _steer_feedback;
	int _watch_dog;
	int _counter;
	int _lastGear = -1;
	double _lastSpeed = 0.0;
	int _gear_counter;
	bool _manual_exit = false;
  bool _in_auto = false;
  float _brake_cache = .0;
  double _steer_offset;
};

}
}
#endif
