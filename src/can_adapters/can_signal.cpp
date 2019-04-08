/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename: can_signal.cpp
*   Author  : lubing.han
*   Date    : 2017-08-15
*   Describe: can_signal.cpp
*
********************************************************/
#include "can_translator.h"
#include <math.h>
#include <ctype.h>
namespace can_adapter
{

string tdST_CanSignal::setSignal(const string &line) {
	// get signal name
	unsigned int SrchCharStart = 5;
	unsigned int SrchCharEnd = line.find(":", SrchCharStart + 1);
	SignalName = line.substr(SrchCharStart, SrchCharEnd - SrchCharStart - 1);

	// get start bit
	SrchCharStart = SrchCharEnd + 1;
	SrchCharEnd = line.find("|", SrchCharStart + 1);
	StartBit = atoi(line.substr(SrchCharStart, SrchCharEnd - SrchCharStart).c_str());

	// get signal size
	SrchCharStart = SrchCharEnd + 1;
	SrchCharEnd = line.find("@", SrchCharStart + 1);
	SignalSize = atoi(line.substr(SrchCharStart, SrchCharEnd - SrchCharStart).c_str());

	// get byte order
	SrchCharEnd = SrchCharEnd + 1;
	if(line.substr(SrchCharEnd, 1) == "0") ByteOrder = false;
	else ByteOrder = true;
	
	// only for dbc analyzer
	int inByteLen = StartBit - ((StartBit >> 3) << 3) + 1;
	if (ByteOrder) {
		if (inByteLen >= SignalSize) StartBit -= SignalSize - 1;
		else {
			int dByte = ((SignalSize - inByteLen - 1) >> 3) + 1;
			inByteLen = (SignalSize - inByteLen) & 7;
			if (inByteLen == 0) inByteLen = 8;
			inByteLen = 8 - inByteLen;
			StartBit = ((StartBit >> 3) << 3) + (dByte << 3) + inByteLen;
		}
	}

	// get byte order
	//SrchCharEnd = SrchCharEnd + 1;
	//if(line.substr(SrchCharEnd, 1) == "0") ByteOrder = false;
	//else ByteOrder = true;

	// get value type
	SrchCharEnd = SrchCharEnd + 1;
	if(line.substr(SrchCharEnd, 1) == "+") ValueType = false;
	else ValueType = true;

	// get factor
	SrchCharStart = line.find("(", SrchCharEnd + 1);
	SrchCharEnd = line.find(",", SrchCharStart + 1);
	Factor = atof(line.substr(SrchCharStart + 1, SrchCharEnd - SrchCharStart - 1).c_str());

	// get offset
	SrchCharStart = SrchCharEnd;
	SrchCharEnd = line.find(")", SrchCharStart + 1);
	Offset = atof(line.substr(SrchCharStart + 1, SrchCharEnd - SrchCharStart - 1).c_str());

	// get minimum
	SrchCharStart = line.find("[", SrchCharEnd + 1);
	SrchCharEnd = line.find("|", SrchCharStart + 1);
	Minimum = atof(line.substr(SrchCharStart + 1, SrchCharEnd - SrchCharStart - 1).c_str());

	// get maximum
	SrchCharStart = SrchCharEnd;
	SrchCharEnd = line.find("]", SrchCharStart + 1);
	Maximum = atof(line.substr(SrchCharStart + 1, SrchCharEnd - SrchCharStart - 1).c_str());

	SrchCharStart = line.find("\"", SrchCharStart + 1);
	SrchCharEnd = line.find("\"", SrchCharStart + 1);
	Unit = line.substr(SrchCharStart + 1, SrchCharEnd - SrchCharStart - 1);
	for (auto &c : Unit) if (c >= 'A' && c <= 'Z') c += 'a' - 'A';
	return SignalName;
}

void tdST_CanSignal::print() const {
	cout << "Name:" << SignalName
			 << " ID:" << MessageID
			 << " S|L:" << StartBit << "|" << SignalSize
			 << " O|T:" << ByteOrder << "|" << ValueType
			 << " F|O:" << Factor << "|" << Offset
			 << " Mi|Ma:" << Minimum << "|" << Maximum
			 << " Unit:" << Unit << endl;
}

bool tdST_CanSignal::check() const {
	if (!ByteOrder) {
		return StartBit <= 63 && SignalSize <= 64 && (StartBit + SignalSize <= 64);
	}
	else {
		if (!(StartBit <= 63 && SignalSize <= 64)) return false;
		UINT32 maxs = (StartBit / 8 + 1) * 8 - StartBit % 8;
		return maxs >= SignalSize;
	}
}

}
