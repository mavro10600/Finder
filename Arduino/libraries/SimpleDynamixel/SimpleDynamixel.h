#ifndef _SIMPLESABER_H_INCLUDED
#define _SIMPLESAVER_H_INCLUDED

#include "DynamixelSerial.h"

class SimpleDynamixelClass {
public:
	SimpleDynamixelClass(DynamixelClass* dynamixelSerial, int min, int max, int range);
	SimpleDynamixelClass(DynamixelClass* dynamixelSerial, int ID, int min, int max, int range);
	SimpleDynamixelClass(DynamixelClass* dynamixelSerial, int ID, int dirPin, int min, int max, int range);
	void write(int value);																// Write an angle from -val->min to +val->max
	int read(); 																		// Returns current position as an angle between -val and val degrees
	void stop();																		// Sets center (sugar for setting an angle of 0)
	void begin();																		// Inits Dynamixel object and sets servo values, max torque, max current, et al...
private:
	int min;
	int max;
	int range;
	int value;
	DynamixelClass* dynamixelobj;														// Inheriting is not done because there may be one or more instances of DynamixelClass, let the user decide
	bool isDir : 1;
	int id : 3;
	int dirpin;
};
#endif
