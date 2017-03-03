#ifndef _AS5043_H_INCLUDED
#define _AS5043_H_INCLUDED

#include <stdio.h>

#define MAX_AS5043_DEVICES			4

class AS5043Class {
public:
	uint8_t maxdevices;
	unsigned int sensorsData[MAX_AS5043_DEVICES];
	uint8_t pincsn;							// ssPin
	AS5043Class();							// Hardware mode without setting pinCSn
	AS5043Class(uint8_t pinCSn);			// Hardware mode setting the pinCSn
	AS5043Class(uint8_t pinCLK, uint8_t pinDO, uint8_t pinProg);
	AS5043Class(uint8_t pinCLK, uint8_t pinDO, uint8_t pinProg, uint8_t pinCSn);
	void setPinCSn(uint8_t pinCSn);			// Set the pinCSn for all functions
	void begin();							// Starts the class
	void begin(uint8_t pinCSn);				// Starts the class setting pinCSn
	void end();								// Ends communication
	unsigned int read();					// Reads chain, returns first sensor read
	unsigned int get();						// Gets 16 bit stream of first sensor of last read
	unsigned int getAngle();				// Gets the angle of the first sensor of last read
	unsigned int getStatus();				// Gets the status of the first sensor of the last read
	unsigned int get(uint8_t index);		// Gets 16 bit stream of index sensor of last read
	unsigned int getAngle(uint8_t index);	// Gets the angle of the index sensor of last read
	unsigned int getStatus(uint8_t index);	// Gets the status of the index sensor of the last read
	unsigned int singleAlign();				// Performs a single alignment mode read with a single sensor and returns D9...D0 only
	void softProg(unsigned int data);		// Volatile programming of a single sensor, use only once per sensor, MSB sent first
	bool parityCheck(unsigned int data);	// Check parity bit of 16 bit data stream, returns true if ok
private:
	bool isSSI;								// isSSI -> hardware mode, else software mode
	uint8_t pinclk;							// The pins in the software mode
	uint8_t pindo;
	uint8_t pinprog;
	void initSSI();							// Start the SSI module
	void readSSI();							// Chained read with SSI
	void readBang();						// Chained read by software
	void pushBit(bool bit);					// Send a single bit to sensor, used by progCore
	void closeComm();						// Ensure sensor is left in OFF mode
	void setPins();							// Set all pinModes
	unsigned long atomSSI();				// Single SSI read, abstracts the hardware
	void setDataMode(uint8_t mode);
	void setClockDivider(uint8_t rate);
};
#endif
