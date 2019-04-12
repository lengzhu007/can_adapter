/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: audi_msgs_processor.cpp
*   Author  : lubing.han
*   Date    : 2017-08-16
*   Describe:
*
********************************************************/
#include "audi_a8_msgs_processor.h"
// #include <autodrive_msgs/VehicleStatus.h>
// #include <autodrive_msgs/CANbrief.h>
// #include <ros/ros.h>
// #include <ros/package.h>
#include <iostream>
#include "spi_dbus_hal.h"
using namespace std;
using namespace can_adapter;

namespace control
{
namespace audi_a8
{

const unsigned HAZARD_LIGHT_MASK = 1;
const unsigned LEFT_LIGHT_MASK = 1 << 1;
const unsigned RIGHT_LIGHT_MASK = 1 << 2;
const unsigned FOG_LIGHT_MASK = 1 << 3;
const unsigned REAR_FOG_LIGHT_MASK = 1 << 4;
const unsigned REAR_LIGHT_MASK = 1 << 5;
const unsigned BRAKE_LIGHT_MASK = 1 << 6;
const unsigned LOW_BEAM_MASK = 1 << 7;
const unsigned HIGH_BEAM_MASK = 1 << 8;

// const double AUDI_A8_MAX_STEER_ANGLE = 500.0;
// const double AUDI_A8_STEER_ANGLE_FACTOR = double(0x8000) / 1005.0;
// const double AUDI_A8_STEER_ANGLE_OFFSET = double(0x4000);
// const double AUDI_A8_THROTTLE_BRAKE_FACTOR = double(0x8000) / 2.01;
// const double AUDI_A8_THROTTLE_BRAKE_OFFSET = double(0x4000);
AudiA8MsgsProcessor::AudiA8MsgsProcessor()
{
    _watch_dog = 0;
    _counter = 0;
    // ros::NodeHandle nh;

    vector<string> dbcPaths, keyPaths, unitPaths;
    // string basePath = ros::package::getPath("control");
    // if (!nh.getParam("dbc_files", dbcPaths))
    // {
    //     cout << "DBCCheryProcessor error: No dbc files" << endl;
    //     return;
    // }
    // if (!nh.getParam("key_files", keyPaths))
    // {
    //     cout << "DBCCheryProcessor error: No key files" << endl;
    //     return;
    // }
    // if (!nh.getParam("unit_files", unitPaths))
    // {
    //     cout << "DBCCheryProcessor error: No unit files" << endl;
    //     return;
    // }
    // for (auto &i : dbcPaths)//i用来遍历dbcPath中的每一个元素
    //     i = basePath + "/" + i;
    // for (auto &i : keyPaths)
    //     i = basePath + "/" + i;
    // for (auto &i : unitPaths)
    //     i = basePath + "/" + i;
     dbcPaths.push_back("/userdata/maji/parameters/Flexray_CAN.dbc");
     keyPaths.push_back("/userdata/maji/parameters/key.map");
     unitPaths.push_back("/userdata/maji/parameters/unit.map");
    _canTranslator.appendDbcFiles(dbcPaths);
    _canTranslator.appendKeyFiles(keyPaths);
    _canTranslator.appendUnitFiles(unitPaths);
    _gear_counter = 10;
    // _canTranslator.print();
    // set default keys
}

AudiA8MsgsProcessor::~AudiA8MsgsProcessor()
{
}

const vector<can_adapter::CanBase> &AudiA8MsgsProcessor::process_send(const struct Control &msg)
{
    // if (msg.Orentation_Roll > 0)
    // _canTranslator.SetKey(_send_msgs, "send_Orentation_Roll", Orentation_Roll);
    // _canTranslator.SetKey(_send_msgs, "send_Position_Z", Position_Z);
    // _canTranslator.SetKey(_send_msgs, "send_Orentation_Pitch", Orentation_Pitch);
    // _canTranslator.SetKey(_send_msgs, "send_Odometry_Counter", Odometry_Counter);

//发送的信息爆炸　如果一个一个信号的设置　　50个点的坐标　/////////////////////////////////////　
//是否改为　组包设置报文内容　 canb报文的组包  需要加入报文ＩＤ信息
    // for (int i = 0; i < 50 ; i++)
    // {
    //     _canTranslator.SetKey(_send_msgs, "send_Position_Y", Position_Y);
    // }
    // set other keys
    if (AUDI_A8_DEBUG)
    {
        cout << "send-------------------" << endl;
        for (auto &i : _send_msgs)
            i.print();
        cout << "-----------------------" << endl;
    }
    return _send_msgs;
}

void AudiA8MsgsProcessor::process_recv(struct CANbrief &can_brief, struct VehicleStatus &vehicleStatus, const vector<can_adapter::CanBase> &canBases)
{
    //can_brief.light_status = 0;
    can_brief.driving_mode = _in_auto;
    if (_in_auto)
    {
        can_brief.brake = 100 * _brake_cache;
    }
    else
    {
        can_brief.brake = 0;
    }
    if (0 == can_brief.brake)
    {
        can_brief.light_status &= ~BRAKE_LIGHT_MASK;
    }
    else
    {
        can_brief.light_status |= BRAKE_LIGHT_MASK;
    }
    vector<CanBase> newCanBases;
    for (auto &i : canBases)
        if (!i.get_remote_flag() && !i.get_extern_flag())
            newCanBases.push_back(i);
    process_recv_base(vehicleStatus, newCanBases);
    can_brief.header = GetTimeStamp();
    // can_brief.header.frame_id = "/vehicle/CANbrief";

    _watch_dog = newCanBases.size() > 0 ? 0 : _watch_dog + 1;
    if (_watch_dog > 50)
    {
        std::cout <<"Vehicle Status Error: can not receive CAN data from can" << std::endl;
        _watch_dog = 0;
    }


    int iValue;
    double fValue;
    

    //add by MJ   add new signal
    if (_canTranslator.GetKey(newCanBases, "recv_rr_speed", fValue))
        vehicleStatus.rr_speed = fValue;
    if (_canTranslator.GetKey(newCanBases, "recv_rl_speed", fValue))
        vehicleStatus.rl_speed = fValue;
    if (_canTranslator.GetKey(newCanBases, "recv_fr_speed", fValue))
        vehicleStatus.fr_speed = fValue;
    if (_canTranslator.GetKey(newCanBases, "recv_fl_speed", fValue))
        vehicleStatus.fl_speed = fValue;

    if (_canTranslator.GetKey(newCanBases, "recv_fl_pulse", iValue))
        vehicleStatus.fl_pulse;
    if (_canTranslator.GetKey(newCanBases, "recv_fr_pulse", iValue))
        vehicleStatus.fr_pulse;
    if (_canTranslator.GetKey(newCanBases, "recv_rl_pulse", iValue))
        vehicleStatus.rl_pulse;
    if (_canTranslator.GetKey(newCanBases, "recv_rr_pulse", iValue))
        vehicleStatus.rr_pulse;

    if (_canTranslator.GetKey(newCanBases, "recv_fl_direction", iValue))
        vehicleStatus.fl_pulse_direction;
    if (_canTranslator.GetKey(newCanBases, "recv_fr_direction", iValue))
        vehicleStatus.fr_pulse_direction;
    if (_canTranslator.GetKey(newCanBases, "recv_rl_direction", iValue))
        vehicleStatus.rl_pulse_direction;
    if (_canTranslator.GetKey(newCanBases, "recv_rr_direction", iValue))
        vehicleStatus.rr_pulse_direction;

    if (_canTranslator.GetKey(newCanBases, "recv_fl_pulsevalid", iValue))
        vehicleStatus.wheel_speeds_fl_valid;
    if (_canTranslator.GetKey(newCanBases, "recv_fr_pulsevalid", iValue))
        vehicleStatus.wheel_speeds_fr_valid;
    if (_canTranslator.GetKey(newCanBases, "recv_rl_pulsevalid", iValue))
        vehicleStatus.wheel_speeds_rl_valid;
    if (_canTranslator.GetKey(newCanBases, "recv_rr_pulsevalid", iValue))
        vehicleStatus.wheel_speeds_rr_valid;

    if (_canTranslator.GetKey(newCanBases, "recv_speed", fValue))
        vehicleStatus.speed = fValue;

    if (_canTranslator.GetKey(newCanBases, "recv_gear", iValue))
        vehicleStatus.gear = iValue;
    if (_canTranslator.GetKey(newCanBases, "recv_yawrate", fValue))
        vehicleStatus.yaw_rate = fValue;


}

void AudiA8MsgsProcessor::process_recv_base(struct VehicleStatus &vehicleStatus, const vector<CanBase> &canBases)
{
    if (AUDI_A8_DEBUG)
    {
        cout << "recv-------------------" << endl;
        for (auto &i : canBases)
            i.print();
        cout << "-----------------------" << endl;
    }
    for (auto &i : canBases)//将canBase的每一个元素　的id 放入＿receive中
        if (_received.find(i.get_id()) == _received.end())
            _received.insert(i.get_id());//
    vehicleStatus.header = GetTimeStamp();
    // vehicleStatus.header.frame_id = "/map";
}

void AudiA8MsgsProcessor::enter_autodrive_mode()
{
    _canTranslator.SetKey(_send_msgs, "send_clutch", 1);
    //_canTranslator.SetKey(_send_msgs, "send_neutral_pos", 1);
}
void AudiA8MsgsProcessor::leave_autodrive_mode()
{
    _canTranslator.SetKey(_send_msgs, "send_clutch", 0);
    //_canTranslator.SetKey(_send_msgs, "send_neutral_pos", 0);
}
void AudiA8MsgsProcessor::entered_autodrive_mode()
{
    _in_auto = true;
}
void AudiA8MsgsProcessor::left_autodrive_mode()
{
    _in_auto = false;
}

bool AudiA8MsgsProcessor::got_all_messages() const
{
    // return true;
    return _received.find(1793) != _received.end();//需要接受的报文id 的最大值　
}

bool AudiA8MsgsProcessor::all_in_autodrive() const
{
    return true;
}

bool AudiA8MsgsProcessor::all_in_manual() const
{
    return false;
}

bool AudiA8MsgsProcessor::manual_exit() const
{
    return _manual_exit;
}

} // namespace audi_a8
} // namespace control
