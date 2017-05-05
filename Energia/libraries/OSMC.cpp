#include "Arduino.h"

#include "OSMC.h"

OSMCClass::OSMCClass(int pinDer, int pinIzq, int umbral, int range) {
	pinder = pinDer;
	pinizq = pinIzq;

	isDisable = false;

	this->umbral = umbral;
	this->range = range;

	value = 0;
}

OSMCClass::OSMCClass(int pinDer, int pinIzq, int pinDis, int umbral, int range) {
	pinder = pinDer;
	pinizq = pinIzq;

	isDisable = true;
	disablepin = pinDis;

	this->umbral = umbral;
	this->range = range;

	value = 0;
}

void OSMCClass::begin() {
	// ENSURE MOTOR STARTS OFF!!! ALSO ENABLED BY DEFAULT
	analogWrite(pinder, 0);
	analogWrite(pinizq, 0);
	if (isDisable) digitalWrite(disablepin, LOW);
}

void OSMCClass::write(int value) {
	value = constrain(value, -abs(range), abs(range));

	if (abs(value) <= umbral) value = 0;
	this->value = value;

	if (range < 0) value = -value;

	value = map(value, -100, 100, -255, 255);

	if (value > 0) {
		analogWrite(pinder, abs(value));
		analogWrite(pinizq, 0);
	}

	if (value == 0) {
		analogWrite(pinder, 0);
		analogWrite(pinizq, 0);
	}

	if (value < 0) {
		analogWrite(pinder, 0);
		analogWrite(pinizq, abs(value));
	}

	if (isDisable) digitalWrite(disablepin, LOW);

}

int OSMCClass::read() {
	return value;
}

void OSMCClass::stop() {
	this->write(0);
}

void OSMCClass::disable() {
	if (isDisable) digitalWrite(disablepin, HIGH);
}

void OSMCClass::enable() {
	if (isDisable) digitalWrite(disablepin, LOW);
}
