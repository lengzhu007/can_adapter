/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: can_translator.cpp
*   Author  : WYC
*   Date    : 2017-02-08
*   Describe: 
*	BugFound: 
********************************************************/
#include "can_translator.h"
#include <math.h>
namespace can_adapter
{

vector<unsigned int> tdCL_CanTranslator::getMessageIDs() const {
	vector<unsigned int> ids;
	for (auto i = StrMapStSignal.cbegin(); i != StrMapStSignal.cend(); ++i)
		ids.push_back(i->second.MessageID);
	return ids;
}

void tdCL_CanTranslator::print() const
{
	cout << "--------Signals-------" << endl;
	for(auto i = StrMapStSignal.cbegin(); i != StrMapStSignal.cend(); ++i)
		i->second.print();
	cout << "--------Keys----------" << endl;
	for (auto i = KeySignalMap.cbegin(); i != KeySignalMap.cend(); ++i)
		cout << i->first << " : " << i->second << endl;
	cout << "--------Units---------" << endl;
	for (auto i = UnitScaleMap.cbegin(); i != UnitScaleMap.cend(); ++i)
		cout << i->first << " : " << i->second << endl;
}

void tdCL_CanTranslator::appendDbcFiles(const vector<string>& DbcFilePaths) {
	unsigned int MessageID;
	unsigned int MessageLength;
	string SignalName;
	string DbcLine;
	unsigned int SrchCharStart;
	unsigned int SrchCharEnd;

	// read dbc file one by one
	for(unsigned char i = 0; i < DbcFilePaths.size(); ++i)
	{
		 // read a dbc file
		ifstream DbcFileContent(DbcFilePaths[i].c_str());

		while(getline(DbcFileContent, DbcLine))
		{
			// search for message tag
			if(DbcLine.find("BO_ ") == 0)
			{
				// get message id
				SrchCharStart = 4;
				SrchCharEnd = DbcLine.find(" ", SrchCharStart + 1);
				MessageID = atoi(DbcLine.substr(SrchCharStart, SrchCharEnd - SrchCharStart).c_str());

				// get message length
				SrchCharStart = DbcLine.find(" ", SrchCharEnd + 1);
				SrchCharEnd = DbcLine.find(" ", SrchCharStart + 1);
				MessageLength = atoi(DbcLine.substr(SrchCharStart, SrchCharEnd - SrchCharStart).c_str());				
			}

			// search for signal tag
			if(DbcLine.find(" SG_ ") == 0 && MessageID < (1 << 11))
			{
				tdST_CanSignal CanSignal(MessageID, MessageLength);
				SignalName = CanSignal.setSignal(DbcLine);
				if (CanSignal.check()) {
					if (StrMapStSignal.find(SignalName) != StrMapStSignal.end())
						cout << "tdCL_CanTranslator::appendDbcFiles warning: duplicated signal name: " << SignalName << endl;
					else StrMapStSignal[SignalName] = CanSignal;
				}
				else
					cout << "tdCL_CanTranslator::appendDbcFiles error: invalid CanSignal syntax: " << SignalName << endl;
			}
		}
		
		// close dbc file
		// cout << endl <<">> " << DbcFilePaths[i].c_str() << " loaded successfully" << endl;
		// cout << "Can signals found:  " << endl;
		DbcFileContent.close();
	}
}

void tdCL_CanTranslator::appendKeyFiles(const vector<string>& keyFilePaths) {
	string keyLine, key, signal;
	for (int i = 0; i < keyFilePaths.size(); ++i) {
		ifstream keyFileContent(keyFilePaths[i].c_str());
		while(getline(keyFileContent, keyLine)) {
			if (keyLine[0] == '#') continue;
			int n = keyLine.find(" ", 0);
			if(n > 0) {
				key = keyLine.substr(0, n);
				n = keyLine.find_first_not_of(" ", n + 1);
				--n;
				int m = keyLine.find_first_of(" \n\r", n + 1);
				if (m < 0) m = keyLine.size();
				signal = keyLine.substr(n + 1, m - n - 1);
				KeySignalMap[key] = signal;
			}
		}
		keyFileContent.close();
	}
}

void tdCL_CanTranslator::appendUnitFiles(const vector<string>& unitFilePaths) {
	string unitLine, unit;
	double scale;
	for (int i = 0; i < unitFilePaths.size(); ++i) {
		ifstream unitFileContent(unitFilePaths[i].c_str());
		while(getline(unitFileContent, unitLine)) {
			if (unitLine[0] == '#') continue;
			int n = unitLine.find(" ", 0);
			if(n > 0) {
				scale = atof(unitLine.substr(0, n).c_str());
				while (true) {
					n = unitLine.find_first_not_of(" ", n + 1);
					--n;
					int m = unitLine.find_first_of(" \n\r", n + 1);
					if (m < 0) {
						unit = unitLine.substr(n + 1, unitLine.size() - n - 1);
						break;
					}
					if (unitLine[m] != ' ') {
						unit = unitLine.substr(n + 1, m - n - 1);
						break;
					}
					unit = unitLine.substr(n + 1, m - n - 1);
					n = m;
					UnitScaleMap[unit] = scale;
				}
				UnitScaleMap[unit] = scale;
			}
		}
		unitFileContent.close();
	}
}

template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, double& value, bool transUnit) {
	tdST_CanSignal canSignal;
	if (!getCanSignalByKey(key, canSignal)) return false;
	int index = getCanBaseIndexById(canSignal.MessageID, canBases);
	if (index < 0) return false;
	ULLONG rawvalue;
	if (!canSignal.ByteOrder) rawvalue = canBases[index].get_data(canSignal.StartBit, canSignal.SignalSize);
	else rawvalue = canBases[index].get_data_motorola(canSignal.StartBit, canSignal.SignalSize);
	if (!canSignal.ValueType) value = double(rawvalue) * canSignal.Factor + canSignal.Offset;
	else {
		if ((rawvalue & ((ULLONG)1 << (canSignal.SignalSize - 1))) == (ULLONG)0)
			value = double(rawvalue);
		else
			value = -double(rawvalue ^ (((ULLONG)1 << canSignal.SignalSize) - (ULLONG)1)) - 1.0;
		value = value * canSignal.Factor + canSignal.Offset;
	}
	double unitScale;
	if (transUnit && getUnitScale(canSignal.Unit, unitScale)) value *= unitScale;
	return true;
}

template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, ULLONG& value, bool transUnit) {
		tdST_CanSignal canSignal;
	if (!getCanSignalByKey(key, canSignal)) return false;
	int index = getCanBaseIndexById(canSignal.MessageID, canBases);
	if (index < 0) return false;
	if (canSignal.Factor != 1 || canSignal.Offset != (double)(LLONG)canSignal.Offset || canSignal.Unit != "") {
		double dvalue = 0.0;
		bool flag = GetKey(canBases, key, dvalue);
		value = (ULLONG)(LLONG)dvalue;
		return flag;		
	}
	ULLONG rawvalue;
	if (!canSignal.ByteOrder) rawvalue = canBases[index].get_data(canSignal.StartBit, canSignal.SignalSize);
	else rawvalue = canBases[index].get_data_motorola(canSignal.StartBit, canSignal.SignalSize);
	if (!canSignal.ValueType) value = rawvalue + (ULLONG)(LLONG)round(canSignal.Offset);
	else {
		if ((rawvalue & ((ULLONG)1 << (canSignal.SignalSize - 1))) == (ULLONG)0)
			value = rawvalue;
		else
			value = ~(rawvalue ^ (((ULLONG)1 << canSignal.SignalSize) - (ULLONG)1));
		value = (ULLONG)((LLONG)value + (LLONG)round(canSignal.Offset));
	}
	return true;
}

template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, bool& value, bool transUnit) {
	ULLONG nvalue = 0;
	bool flag = GetKey(canBases, key, nvalue, transUnit);
	value = (bool)(LLONG)nvalue;
	return flag;
}
template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, INT8& value, bool transUnit) {
	ULLONG nvalue = 0;
	bool flag = GetKey(canBases, key, nvalue, transUnit);
	value = (INT8)(LLONG)nvalue;
	return flag;
}
template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, UINT8& value, bool transUnit) {
	ULLONG nvalue = 0;
	bool flag = GetKey(canBases, key, nvalue, transUnit);
	value = (UINT8)(LLONG)nvalue;
	return flag;
}
template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, INT16& value, bool transUnit) {
	ULLONG nvalue = 0;
	bool flag = GetKey(canBases, key, nvalue, transUnit);
	value = (INT16)(LLONG)nvalue;
	return flag;
}
template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, UINT16& value, bool transUnit) {
	ULLONG nvalue = 0;
	bool flag = GetKey(canBases, key, nvalue, transUnit);
	value = (UINT16)(LLONG)nvalue;
	return flag;
}
template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, INT32& value, bool transUnit) {
	ULLONG nvalue = 0;
	bool flag = GetKey(canBases, key, nvalue, transUnit);
	value = (INT32)(LLONG)nvalue;
	return flag;
}
template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, UINT32& value, bool transUnit) {
	ULLONG nvalue = 0;
	bool flag = GetKey(canBases, key, nvalue, transUnit);
	value = (UINT32)(LLONG)nvalue;
	return flag;
}
template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, LLONG& value, bool transUnit) {
	ULLONG nvalue = 0;
	bool flag = GetKey(canBases, key, nvalue, transUnit);
	value = (LLONG)(LLONG)nvalue;
	return flag;
}
template <> bool tdCL_CanTranslator::GetKey(const vector<CanBase> &canBases, const string &key, float& value, bool transUnit) {
	double nvalue = 0.0;
	bool flag = GetKey(canBases, key, nvalue, transUnit);
	value = (float)nvalue;
	return flag;
}

template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, double value, bool transUnit) {
	tdST_CanSignal canSignal;
	if (!getCanSignalByKey(key, canSignal)) return false;
	int index = getCanBaseIndexById(canSignal.MessageID, canBases);
	if (index < 0) {
		canBases.push_back(CanBase(canSignal.MessageID));
		index = canBases.size() - 1;
	}
	double unitScale;
	if (transUnit && getUnitScale(canSignal.Unit, unitScale)) value /= unitScale;
	if (canSignal.Maximum > canSignal.Minimum) {
		if (value > canSignal.Maximum) value = canSignal.Maximum;
		if (value < canSignal.Minimum) value = canSignal.Minimum;
	}
	ULLONG nvalue = (ULLONG)(LLONG)round((value - canSignal.Offset) / canSignal.Factor);
	if (!canSignal.ByteOrder) canBases[index].set_data(nvalue, canSignal.StartBit, canSignal.SignalSize);
	else canBases[index].set_data_motorola(nvalue, canSignal.StartBit, canSignal.SignalSize);
	return true;
}

template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, ULLONG value, bool transUnit) {
	tdST_CanSignal canSignal;
	if (!getCanSignalByKey(key, canSignal)) return false;
	int index = getCanBaseIndexById(canSignal.MessageID, canBases);
	if (index < 0) {
		canBases.push_back(CanBase(canSignal.MessageID));
		index = canBases.size() - 1;
	}
	double dvalue;
	if (!canSignal.ValueType) dvalue = (double)value;
	else dvalue = (double)(LLONG)value;
	if (canSignal.Factor != 1 || canSignal.Offset != (double)(LLONG)canSignal.Offset || canSignal.Unit != "")
		return SetKey(canBases, key, dvalue);
	ULLONG nvalue;
	if (!canSignal.ValueType) nvalue = value - (ULLONG)(LLONG)round(canSignal.Offset);
	else nvalue = (ULLONG)((LLONG)value - (LLONG)round(canSignal.Offset));
	if (canSignal.Maximum > canSignal.Minimum) {
		if (!canSignal.ValueType && nvalue > (ULLONG)round(canSignal.Maximum)) nvalue = (ULLONG)round(canSignal.Maximum);
		if (!canSignal.ValueType && nvalue < (ULLONG)round(canSignal.Minimum)) nvalue = (ULLONG)round(canSignal.Minimum);
		if (canSignal.ValueType && (LLONG)nvalue > (LLONG)round(canSignal.Maximum)) nvalue = (ULLONG)round(canSignal.Maximum);
		if (canSignal.ValueType && (LLONG)nvalue < (LLONG)round(canSignal.Minimum)) nvalue = (ULLONG)round(canSignal.Minimum);
	}	
	if (!canSignal.ByteOrder) canBases[index].set_data(nvalue, canSignal.StartBit, canSignal.SignalSize);
	else canBases[index].set_data_motorola(nvalue, canSignal.StartBit, canSignal.SignalSize);
	return true;
}

template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, bool value, bool transUnit) {
	return SetKey(canBases, key, (ULLONG)(LLONG)value, transUnit);
}
template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, INT8 value, bool transUnit) {
	return SetKey(canBases, key, (ULLONG)(LLONG)value, transUnit);
}
template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, UINT8 value, bool transUnit) {
	return SetKey(canBases, key, (ULLONG)(LLONG)value, transUnit);
}
template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, INT16 value, bool transUnit) {
	return SetKey(canBases, key, (ULLONG)(LLONG)value, transUnit);
}
template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, UINT16 value, bool transUnit) {
	return SetKey(canBases, key, (ULLONG)(LLONG)value, transUnit);
}
template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, INT32 value, bool transUnit) {
	return SetKey(canBases, key, (ULLONG)(LLONG)value, transUnit);
}
template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, UINT32 value, bool transUnit) {
	return SetKey(canBases, key, (ULLONG)(LLONG)value, transUnit);
}
template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, LLONG value, bool transUnit) {
	return SetKey(canBases, key, (ULLONG)(LLONG)value, transUnit);
}
template <> bool tdCL_CanTranslator::SetKey(vector<CanBase> &canBases, const string &key, float value, bool transUnit) {
	return SetKey(canBases, key, (double)value, transUnit);
}

bool tdCL_CanTranslator::getCanSignalByKey(const string &key, tdST_CanSignal &canSignal) const {
	if (KeySignalMap.find(key) == KeySignalMap.end()) return false;
	auto signal = KeySignalMap.find(key)->second;
	if (StrMapStSignal.find(signal) == StrMapStSignal.end()) return false;
	canSignal = StrMapStSignal.find(signal)->second;
	return true;
}

int tdCL_CanTranslator::getCanBaseIndexById(int id, const vector<CanBase> &canBases) const {
	int i = 0;
	for ( ; i < canBases.size(); ++i)
		if (canBases[i].get_id() == id) break;
	return i == canBases.size() ? -1 : i;
}

bool tdCL_CanTranslator::getUnitScale(const string &unit, double& unitScale) const {
	if (UnitScaleMap.find(unit) == UnitScaleMap.end()) return false;
	unitScale = UnitScaleMap.find(unit)->second;
	return true;
}

UINT8* tdCL_CanTranslator::GetMsgBytesByKey(vector<CanBase> &canBases,const string &key) {
			tdST_CanSignal canSignal;
	if (!getCanSignalByKey(key, canSignal)) return NULL;
	int index = getCanBaseIndexById(canSignal.MessageID, canBases);
	if (index < 0) return NULL;
	return canBases[index].get_bytes();
}

}