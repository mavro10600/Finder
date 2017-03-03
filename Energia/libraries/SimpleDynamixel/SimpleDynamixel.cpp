#include "Arduino.h"

#include "SimpleDynamixel.h"

SimpleDynamixelClass::SimpleDynamixelClass(DynamixelClass* dynamixelSerial, int min, int max, int range) {
	isDir = false;
	dynamixelobj = dynamixelSerial;
	id = 1;

	value = 0;

	this->min = min;
	this->max = max;
	this->range = range;
}

SimpleDynamixelClass::SimpleDynamixelClass(DynamixelClass* dynamixelSerial, int ID, int min, int max, int range) {
	isDir = false;
	dynamixelobj = dynamixelSerial;
	id = ID;

	value = 0;

	this->min = min;
	this->max = max;
	this->range = range;
}

SimpleDynamixelClass::SimpleDynamixelClass(DynamixelClass* dynamixelSerial, int ID, int dirPin, int min, int max, int range) {
	isDir = true;
	dynamixelobj = dynamixelSerial;
	id = ID;

	dirpin = dirPin;
	value = 0;

	this->min = min;
	this->max = max;
	this->range = range;
}

void SimpleDynamixelClass::write(int value) {
	value = constrain(value, -abs(range), abs(range));

	this->value = value;

	if (range < 0) value = -value;

	value = map(value, -100, 100, min, max);

	// Make local copy of direction pin
	int orgdirpin = dynamixelobj->Direction_Pin;
	// Set
	if (isDir)
		dynamixelobj->setDirPin(dirpin);
	// Move
	dynamixelobj->move(id, value);
	// Reset
	if (isDir)
		dynamixelobj->setDirPin(orgdirpin);
}

int SimpleDynamixelClass::read() {
	return value;
}

void SimpleDynamixelClass::stop() {
	this->write(0);
}

void SimpleDynamixelClass::begin() {
	// Make local copy of direction pin
	int orgdirpin = dynamixelobj->Direction_Pin;
	// Begin comm with this dirpin at 1MHz
	if (isDir)
		dynamixelobj->setDirPin(dirpin);
	dynamixelobj->begin(1000000UL);
	// Set parameters of dynamixel
	dynamixelobj->setTempLimit(id, 80);
	dynamixelobj->setVoltageLimit(id, 65, 100);
	dynamixelobj->setMaxTorque(id, 512);
	dynamixelobj->ledStatus(id, ON);
	// Restore direction pin
	if (isDir)
		dynamixelobj->setDirPin(orgdirpin);
}
