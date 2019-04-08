/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: can_base.cpp
*   Author  : lubing.han
*   Date    : 2017-02-08
*   Describe:
*
********************************************************/

#include "can_base.h"
#include <math.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <bitset>
using namespace std;

namespace can_adapter {

CanBase::CanBase() {

}

CanBase::CanBase(UINT32 id, UINT32 period): _id(id), _period(period) {
	_time_flag = false;
	_remote_flag = false;
	_extern_flag = false;
	for (int i = 0; i < 8; ++i)
		_data[i] = 0;
}

CanBase::~CanBase() {

}

UINT32 CanBase::get_id() const {
	return _id;
}

UINT32 CanBase::get_period() const {
	return _period;
}

bool CanBase::get_time_flag() const {
	return _time_flag;
}

double CanBase::get_time_stamp() const {
	return _time_stamp;
}

bool CanBase::get_remote_flag() const {
	return _remote_flag;
}

bool CanBase::get_extern_flag() const {
	return _extern_flag;
}

void CanBase::set_id(UINT32 id) {
	_id = id;
}

void CanBase::set_period(UINT32 period) {
	_period = period;
}

void CanBase::set_time_stamp(double time_stamp) {
	_time_flag = true; _time_stamp = time_stamp;
}

void CanBase::set_remote_flag(bool remote_flag) {
	_remote_flag = remote_flag;
}

void CanBase::set_extern_flag(bool extern_flag) {
	_extern_flag = extern_flag;
}

void CanBase::set_bytes(const UINT8* pbyte) {
	for (int i = 0; i < 8; ++i)
		_data[i] = pbyte[i];
}

UINT8* CanBase::get_bytes() {
	return _data;
}

const UINT8* CanBase::get_bytes() const {
	return _data;
}

void CanBase::print() const {
	ULLONG data = get_data(0, 64);
	ULLONG one = 1;
	cout << "id: 0x" << setiosflags(ios::uppercase) << hex << setw(3) << get_id();
	// cout << "  data: 7        6        5        4        3        2        1" << endl;
	for (int i = 0; i < 8; ++i) {
		cout << " ";
		for (int j = 0; j < 8; ++j)
			cout << ((data >> (63 - i * 8 - j)) & one);

	}
	cout << dec << endl;
}

vector<CanBase> CanBase::fromString(const string& canString) {
	vector<CanBase> cans;
	int n = 0;
	while (canString.size() >= n * 88 + 87) {
		string sid = canString.substr(n * 88 + 4, 5);
		if (sid[2] == ' ') sid.erase(2, 1);
		int id = (int)strtol(sid.c_str(), NULL, 0);
		CanBase can(id);
		for (int i = 0; i < 8; ++i) {
			string sbits = canString.substr(n * 88 + 16 + i * 9, 8);
			bitset<8> bits(sbits);
			can.set_data(bits.to_ulong(), (7 - i) * 8, 8);
		}
		cans.push_back(can);
		++n;
	}
	return cans;
}

string CanBase::toString() const {
	stringstream ss;
	ss << "id: 0x" << setiosflags(ios::uppercase) << hex << setw(3) << get_id() << " data:";
	for (int i = 7; i >= 0; --i)
		ss << " " << bitset<8>(get_data(8 * i, 8));
	ss << "\n";	
	return ss.str();
}

const CanBase& CanBase::get_message() {
	return *this;
}

void CanBase::set_message(const CanBase& canBase) {
	if (this != &canBase) *this = canBase;
}

void CanBase::set_data(ULLONG data, int start, int length) {
	ULLONG mask;
	if (length > 63) mask = ~((ULLONG)0);
	else mask = ((ULLONG)1 << length) - 1;
	data &= mask;
	mask = ~(mask << start);
	data = data << start;
	ULLONG* pbytes = (ULLONG*)get_bytes();
	*pbytes = *pbytes & mask;
	*pbytes = *pbytes | data;
}

ULLONG CanBase::get_data(int start, int length) const {
	ULLONG mask;
	if (length > 63) mask = ~((ULLONG)0);
	else mask = ((ULLONG)1 << length) - 1;
	mask = mask << start;
	ULLONG data = *((const ULLONG*)get_bytes());
	data &= mask;
	return data >> start;
}

void CanBase::set_data_motorola(ULLONG data, int start, int length) {
	ULLONG mask;
	int start1 = start % 8, start2 = start >> 3;
	int byte_len = (start1 + length - 1) >> 3;
	if (length > 63) mask = ~((ULLONG)0);
	else mask = ((ULLONG)1 << length) - 1;
	mask = mask << start1;
	data = data << start1;
	ULLONG new_data = 0, new_mask = 0;
	for (int i = 0, j = start2; i <= byte_len; ++i, --j) {
		*((UINT8*)(&new_data) + j) = *((UINT8*)(&data) + i);
		*((UINT8*)(&new_mask) + j) = *((UINT8*)(&mask) + i);
	}
	new_data &= new_mask;
	new_mask = ~new_mask;
	ULLONG* pbytes = (ULLONG*)get_bytes();
	*pbytes = *pbytes & new_mask;
	*pbytes = *pbytes | new_data;
}

ULLONG CanBase::get_data_motorola(int start, int length) const {
	ULLONG mask;
	int start1 = start % 8, start2 = start >> 3;
	int byte_len = (start1 + length - 1) >> 3;
	if (length > 63) mask = ~((ULLONG)0);
	else mask = ((ULLONG)1 << length) - 1;

	ULLONG data;
	const UINT8* pbytes = get_bytes();
	for (int i = 0, j = start2; i <= byte_len; ++i, --j)
		*((UINT8*)(&data) + i) = *(pbytes + j);
	data = data >> start1;
	data &= mask;
	return data;
}

}
