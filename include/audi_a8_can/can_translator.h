/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: can_translator.h
*   Author  : WYC
*   Date    : 2017-02-08
*   Describe:
*
********************************************************/
#ifndef CAN_TRANSLATOR_H
#define CAN_TRANSLATOR_H

// #include "ros/ros.h"
// #include "std_msgs/String.h"
#include <vector>
#include <map>
#include "stdlib.h"
#include "can_base.h"
#include "can_signal.h"

using namespace std;

namespace can_adapter
{

class tdCL_CanTranslator
{
public:
	tdCL_CanTranslator() {};
	~tdCL_CanTranslator() {};

	void appendDbcFiles(const vector<string>& DbcFilePaths);
	void appendKeyFiles(const vector<string>& keyFilePaths);
	void appendUnitFiles(const vector<string>& unitFilePaths);

	template <class T>
	bool GetKey(const vector<CanBase> &canBases, const string &key, T& value, bool transUnit = true) {
		cout << "tdCL_CanTranslator: GetKey error: invalid value type." << endl;
		return false;
	}
	template <class T>
	bool SetKey(vector<CanBase> &canBases, const string &key, T value, bool transUnit = true) {
		cout << "tdCL_CanTranslator: SetKey error: invalid value type." << endl;
		return false;
	}

	vector<unsigned int> getMessageIDs() const;
	void print() const;
	UINT8* GetMsgBytesByKey(vector<CanBase> &canBases,const string &key);
	
private:
	bool getCanSignalByKey(const string &key, tdST_CanSignal &canSignal) const;
	int getCanBaseIndexById(int id, const vector<CanBase> &canBases) const;
	bool getUnitScale(const string &unit, double& unitScale) const;

	map<string, string> KeySignalMap;
	map<string, tdST_CanSignal> StrMapStSignal;
	map<string, double> UnitScaleMap;
};

template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, double& value, bool transUnit);
template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, ULLONG& value, bool transUnit);
template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, bool& value, bool transUnit);
template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, INT8& value, bool transUnit);
template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, UINT8& value, bool transUnit);
template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, INT16& value, bool transUnit);
template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, UINT16& value, bool transUnit);
template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, INT32& value, bool transUnit);
template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, UINT32& value, bool transUnit);
template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, LLONG& value, bool transUnit);
template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, float& value, bool transUnit);

template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, double value, bool transUnit);
template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, ULLONG value, bool transUnit);
template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, bool value, bool transUnit);
template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, INT8 value, bool transUnit);
template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, UINT8 value, bool transUnit);
template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, INT16 value, bool transUnit);
template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, UINT16 value, bool transUnit);
template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, INT32 value, bool transUnit);
template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, UINT32 value, bool transUnit);
template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, LLONG value, bool transUnit);
template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, float value, bool transUnit);

}

#endif