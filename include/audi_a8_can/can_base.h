/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: can_base.h
*   Author  : lubing.han
*   Date    : 2017-02-08
*   Describe:
*
********************************************************/
#ifndef CAN_ADAPTER_CAN_BASE_H
#define CAN_ADAPTER_CAN_BASE_H

#include <string>
#include <vector>
#include <stdlib.h>
using namespace std;

namespace can_adapter {

#ifndef BYTE
#define BYTE unsigned char
#endif
#ifndef USHORT
#define USHORT unsigned short
#endif
#ifndef UINT
#define UINT unsigned int
#endif
#ifndef INT8
#define INT8 char
#endif
#ifndef UINT8
#define UINT8 unsigned char
#endif
#ifndef INT16
#define INT16 short
#endif
#ifndef UINT16
#define UINT16 unsigned short
#endif
#ifndef INT32
#define INT32 int
#endif
#ifndef UINT32
#define UINT32 unsigned int
#endif
#ifndef LLONG
#define LLONG long long
#endif
#ifndef ULLONG
#define ULLONG unsigned long long
#endif

class CanBase
{
public:
	CanBase();
	CanBase(UINT32 id, UINT32 period = 0);
	~CanBase();

	UINT32 get_id() const;
	UINT32 get_period() const;
	bool get_time_flag() const;
	double get_time_stamp() const;
	bool get_remote_flag() const;
	bool get_extern_flag() const;
	void set_id(UINT32 id);
	void set_period(UINT32 period);
	void set_time_stamp(double time_stamp);
	void set_remote_flag(bool remote_flag);
	void set_extern_flag(bool extern_flag);
	void set_bytes(const UINT8* pbyte);
	UINT8* get_bytes();
	const UINT8* get_bytes() const;
	void print() const;
	string toString() const;
	static vector<CanBase> fromString(const string& canString);

	virtual const CanBase& get_message();
	void set_message(const CanBase& canBase);

public:
	void set_data(ULLONG data, int start, int length);
	ULLONG get_data(int start, int length) const;
	void set_data_motorola(ULLONG data, int start, int length);
	ULLONG get_data_motorola(int start, int length) const;

private:
	UINT32 _id;
	UINT32 _period;
	bool _time_flag;
	double _time_stamp;
	bool _remote_flag;
	bool _extern_flag;
	UINT8 _data[8];
};

}

#endif
