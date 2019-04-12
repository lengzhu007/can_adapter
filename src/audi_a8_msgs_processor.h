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
#include <stdint.h>
#include <set>

// #include <autodrive_msgs/VehicleStatus.h>
// #include <autodrive_msgs/Control.h>
// #include <autodrive_msgs/CarStatus.h>
// #include <autodrive_msgs/CANbrief.h>
using namespace std;
using namespace can_adapter;

namespace control {
namespace audi_a8 {
// typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type
typedef struct CANbrief
{
float header;	
uint8_t park_status;
uint8_t back_status;
uint16_t light_status ;
int8_t brake;
float brake_pressure;
float steer;
int8_t accelerator;
uint8_t driving_mode;
} CANbrief;

typedef struct VehicleStatus
{
float header;	
float speed;
bool wheel_speeds_valid;
bool wheel_speeds_fl_valid;
bool wheel_speeds_fr_valid;
bool wheel_speeds_rl_valid;
bool wheel_speeds_rr_valid;
float fl_speed;
float fr_speed;
float rl_speed;
float rr_speed;
bool wheel_pulses_valid;
int32_t fl_pulse_direction;
int32_t fr_pulse_direction;
int32_t rl_pulse_direction;
int32_t rr_pulse_direction;
int32_t fl_pulse;
int32_t fr_pulse;
int32_t rl_pulse;
int32_t rr_pulse;
bool steer_angle_valid;
float steer_angle ;     // in degrees
bool gear_valid;
int32_t gear   ;            // 1->P, 2->R, 3->N, 4->D
bool torque_valid;
float torque;
bool yaw_rate_valid;
float yaw_rate;
float yaw_rate_offset;
bool left_light_status;
bool right_light_status;
bool brake_light_status ;
} VehicleStatus;

typedef struct Control//需要重新定义　trajecttory and odometry 包含以下结构体
{
float header;

double throttle;
double steer_target;
double steer_rate;
double brake;
bool turn_left_light;
bool turn_right_light;
bool brake_light;
bool autodrive_mode;
int32_t gear;
bool start_request;
bool stop_request;
bool downhill_signal;
bool uphill_signal;
bool emergency_signal;
double speed;
double steer_ref;


// typedef unsigned int vbittype;
  // typedef struct _Trajectory_Start //id = 256　　　10hz
  // {
  //   vbittype Trajectory_version : 8;
  //   vbittype Points_Number : 8;
  //   vbittype Emergency_brake_request : 1;
  //   vbittype Turn_indicator_request : 2;
  //   vbittype NULL_Bit1 : 32; //19-59//长度超了？？？
  //   vbittype NULL_Bit2 : 9;
  //   vbittype Trajectory_Counter : 4;
  // } _Trajectory_Start;

  // typedef struct _Points_ //id =306 -257//50各点
  // {
  //   vbittype Orentation_Yaw : 16;
  //   vbittype Speed : 9;
  //   vbittype Gear : 3;
  //   vbittype Position_X : 18;
  //   vbittype Position_Y : 18;
  // } _Points_;

  // typedef struct _Trajectory_End //id=457
  // {
  //   vbittype Trajectory_version : 8; //
  //   vbittype Points_Number : 8;
  //   vbittype NULL_Bit1 : 32; //20-59;
  //   vbittype NULL_Bit2 : 8;
  //   vbittype Trajectory_Counter : 4;
  // } _Trajectory_End;

  // typedef struct _Vehicle_2D_Odometry //id=458  50hz
  // {
  //   vbittype Position_X : 18; //
  //   vbittype Position_Y : 18;
  //   vbittype Orentation : 16;
  //   vbittype NULL_Bit : 8;          //空位？？？？？？？？？？？？？？  字节顺序　　　52-59
  //   vbittype Odometry_Counter : 4;
  // } _Vehicle_2D_Odometry;

  // typedef struct _Vehicle_3D_Odometry //ID=459
  // {
  //   vbittype Orentation_Roll : 16;
  //   vbittype Orentation_Pitch : 16;
  //   vbittype Position_Z : 18;
  //   vbittype NULL_Bit : 10; //空bit ？？？？？？？？？？？？？？  50-59
  //   vbittype Odometry_Counter : 4;
  // } _Vehicle_3D_Odometry;



} Control;


class AudiA8MsgsProcessor
{
public:
	AudiA8MsgsProcessor();
	~AudiA8MsgsProcessor();

	const vector<CanBase>& process_send(const struct Control &msg);
	void process_recv(struct CANbrief & can_brief, struct VehicleStatus &vehicleStatus, const vector<CanBase> &canBases);

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
	void process_recv_base(struct VehicleStatus &vehicleStatus, const vector<CanBase> &canBases);

	vector<CanBase> _send_msgs;
	tdCL_CanTranslator _canTranslator;
	set<int> _received, _inAutodrived;//元素按顺序排列　还得保证根节点左子树的高度与右子树高度相等。
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
