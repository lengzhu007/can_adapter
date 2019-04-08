/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename: can_signal.h
*   Author  : lubing.han
*   Date    : 2017-08-15
*   Describe: can_signal.h
*
********************************************************/

#ifndef CAN_TRANSLATOR_CAN_SIGNAL_H
#define CAN_TRANSLATOR_CAN_SIGNAL_H

#include <string>
#include <iostream>
#include <fstream>
#include "can_signal.h"
using namespace std;

namespace can_adapter
{

class tdST_CanSignal
{
public:
	tdST_CanSignal() {}
	tdST_CanSignal(unsigned int id, unsigned int len): MessageID(id), MessageLength(len) {}
	string SignalName;
	unsigned int MessageID;
	unsigned int MessageLength;
	unsigned int StartBit;
	unsigned int SignalSize;
	bool ByteOrder; // false: intel, true: motorola
	bool ValueType; // false: unsigned, true: signed
	double Factor;
	double Offset;
	double Minimum;
	double Maximum;
	string Unit;

	string setSignal(const string &line);
	bool check() const;
	void print() const;
};

}

#endif