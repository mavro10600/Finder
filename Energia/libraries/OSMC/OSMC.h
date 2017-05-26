#ifndef _OSMC_H_INCLUDED
#define _OSMC_H_INCLUDED


class OSMCClass {
public:
	OSMCClass(int pinDer, int pinIzq, int umbral, int range);
	OSMCClass(int pinDer, int pinIzq, int pinDis, int umbral, int range);
	void begin();
	void write(int value);																					// Write a PWM angle from -127 to +127
	int read(); 																							// Returns current PWM as an angle between -127 and 127 degrees
	void stop();																							// Sets center (sugar for setting a PWM of 0)
	void disable();
	void enable();
private:
	int value;																							// PWM angle from -127 to +127
	bool isDisable;
	int disablepin;
	int pinder;
	int pinizq;
	int umbral;
	int range;
};
#endif
