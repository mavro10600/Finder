#include "Arduino.h"

#include "AS5043.h"

#define SPI_CLOCK_DIV4 		0x00
#define SPI_CLOCK_DIV16 	0x01
#define SPI_CLOCK_DIV64 	0x02
#define SPI_CLOCK_DIV128 	0x03
#define SPI_CLOCK_DIV2 		0x04
#define SPI_CLOCK_DIV8 		0x05
#define SPI_CLOCK_DIV32 	0x06
//#define SPI_CLOCK_DIV64 	0x07

#define SPI_MODE0 			0x00
#define SPI_MODE1 			0x04
#define SPI_MODE2 			0x08
#define SPI_MODE3 			0x0C

#define SPI_MODE_MASK 		0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK 		0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 	0x01  // SPI2X = bit 0 on SPSR

// *****************************************************************************

AS5043Class::AS5043Class(void) {
	isSSI = true;
	//pinclk = SCK;
	//pindo = MISO;
	//pinprog = MOSI;
	//pincsn = SS;
	maxdevices = MAX_AS5043_DEVICES;
}

AS5043Class::AS5043Class(uint8_t pinCSn) {
	isSSI = true;
	//pinclk = SCK;
	//pindo = MISO;
	//pinprog = MOSI;
	pincsn = pinCSn;
	maxdevices = MAX_AS5043_DEVICES;
}

AS5043Class::AS5043Class(uint8_t pinCLK, uint8_t pinDO, uint8_t pinProg) {
	isSSI = false;
	pinclk = pinCLK;
	pindo = pinDO;
	pinprog = pinProg;
	pincsn = -1;
	maxdevices = MAX_AS5043_DEVICES;
}

AS5043Class::AS5043Class(uint8_t pinCLK, uint8_t pinDO, uint8_t pinProg,
		uint8_t pinCSn) {
	isSSI = false;
	pinclk = pinCLK;
	pindo = pinDO;
	pinprog = pinProg;
	pincsn = pinCSn;
	maxdevices = MAX_AS5043_DEVICES;
}

void AS5043Class::setPinCSn(uint8_t pinCSn) {
	pincsn = pinCSn;
	pinMode(pincsn, OUTPUT);
	digitalWrite(pincsn, HIGH);
}

void AS5043Class::begin() {
	this->begin(pincsn);
}

void AS5043Class::begin(uint8_t pinCSn) {
	this->setPinCSn(pinCSn);

	if (isSSI)
		this->initSSI();
	else {
		pinMode(pinclk, OUTPUT);
		pinMode(pindo, INPUT);
		pinMode(pinprog, OUTPUT);
		digitalWrite(pinclk, HIGH);
		digitalWrite(pinprog, LOW);
	}
}

void AS5043Class::end() {
	digitalWrite(pincsn, HIGH);
//	if (isSSI)
		//SPCR &= ~_BV(SPE);
}

unsigned int AS5043Class::read() {
	if (isSSI)
		this->readSSI();
	else
		this->readBang();
	return sensorsData[0];
}

unsigned int AS5043Class::get() {
	return this->get(0);
}

unsigned int AS5043Class::get(uint8_t index) {
	return sensorsData[index];
}

unsigned int AS5043Class::getAngle() {
	return this->getAngle(0);
}

unsigned int AS5043Class::getAngle(uint8_t index) {
	unsigned int _singleData = sensorsData[index];
	return (_singleData >> 6);
}

unsigned int AS5043Class::getStatus() {
	return this->getStatus(0);
}

unsigned int AS5043Class::getStatus(uint8_t index) {
	unsigned int _singleData = sensorsData[index];
	return (_singleData & B00111111);
}

unsigned int AS5043Class::singleAlign() {
	this->end();
	this->setPins();

	// Entering alignment mode
	digitalWrite(pincsn, LOW);
	delayMicroseconds(4);
	digitalWrite(pincsn, HIGH);
	delayMicroseconds(4);
	digitalWrite(pinprog, HIGH);
	delayMicroseconds(4);
	digitalWrite(pincsn, LOW);
	delayMicroseconds(4);
	digitalWrite(pinprog, LOW);

	// Reading a single sensor data by software
	unsigned int data = 0;

	digitalWrite(pinclk, LOW);
	delayMicroseconds(1);
	for (uint8_t k = 0; k < 16; k++) {
		digitalWrite(pinclk, HIGH);
		delayMicroseconds(1);
		data = (data << 1) | (digitalRead(pindo) ? 0x0001 : 0x0000);
		digitalWrite(pinclk, LOW);
		delayMicroseconds(1);
	}
	this->closeComm();
	this->begin();

	return (data >> 6);
}

void AS5043Class::softProg(unsigned int data) {
	this->end();
	this->setPins();

	// Entering the programming mode
	digitalWrite(pinclk, LOW);
	delayMicroseconds(4);
	digitalWrite(pincsn, LOW);
	delayMicroseconds(4);
	digitalWrite(pinprog, HIGH);
	delayMicroseconds(4);
	digitalWrite(pincsn, HIGH);
	delayMicroseconds(4);

	// Start data write
	for (uint8_t k = 0; k < 16; k++)
		this->pushBit(((data & (0x0001 << (15 - k))) >> (15 - k)));

	// Finish the programming mode
	digitalWrite(pinprog, LOW);
	delayMicroseconds(4);
	digitalWrite(pincsn, LOW);
	delayMicroseconds(4);

	this->closeComm();
	this->begin();
}

bool AS5043Class::parityCheck(unsigned int data) {
	unsigned int parity = 0;
	unsigned int parity_bit = data & 0x0001;
	// We start with 1 because the bit 0 is the parity bit
	for (uint8_t k = 1; k < 16; k++)
		parity ^= ((data & (0x0001 << k)) >> k);
	return (parity_bit == parity);
}

// *****************************************************************************

void AS5043Class::initSSI() {
	// When the SS pin is set as OUTPUT, it can be used as
	// a general purpose output port (it doesn't influence
	// SPI operations).
	//pinMode(SS, OUTPUT);

	// Warning: if the SS pin ever becomes a LOW INPUT then SPI
	// automatically switches to Slave, so the data direction of
	// the SS pin MUST be kept as OUTPUT.
	//SPCR |= _BV(MSTR);
	//SPCR |= _BV(SPE);

	// Configure to 1 Mhz and MODE_2
	this->setClockDivider(SPI_CLOCK_DIV16);
	this->setDataMode(SPI_MODE2);

	// Set direction register for SCK and MOSI pin.
	// MISO pin automatically overrides to INPUT.
	// By doing this AFTER enabling SPI, we avoid accidentally
	// clocking in a single bit since the lines go directly
	// from "input" to SPI control.
	// http://code.google.com/p/arduino/issues/detail?id=888
	//pinMode(SCK, OUTPUT);

	// Ensure programming mode starts off (sending data is done without the module anyway)
	//digitalWrite(MOSI, LOW);
	//pinMode(MOSI, OUTPUT);
}

void AS5043Class::readSSI() {
	uint8_t base_bits = maxdevices * 16 + maxdevices;
	uint8_t required_reads = base_bits / 8 + base_bits % 8 == 0 ? 0 : 1;
	unsigned long chainData = 0;
	uint8_t index = 0;
	digitalWrite(pincsn, LOW);
	for (uint8_t k = 0; k < required_reads; k++) {
		// Extract data
		chainData = (chainData << 8) | (this->atomSSI() & 0x00FF);
		// Put data in container by singles
		if ((k + 1) * 8 >= (index + 1) * 17) {
			uint8_t shift_val = 7 - index % 8;
			sensorsData[index++] = (chainData >> shift_val) & 0xFFFF;
		}
	}
	digitalWrite(pincsn, HIGH);
}

void AS5043Class::readBang() {
	uint8_t base_bits = maxdevices * 16 + maxdevices - 1;
	unsigned long chainData = 0;
	uint8_t index = 0;
	digitalWrite(pincsn, LOW);
	delayMicroseconds(1);
	digitalWrite(pinclk, LOW);
	delayMicroseconds(1);
	for (uint8_t k = 0; k < base_bits; k++) {
		digitalWrite(pinclk, HIGH);
		delayMicroseconds(1);
		chainData = (chainData << 1) | (digitalRead(pindo) ? 0x0001 : 0x0000);
		// Put data in container by singles
		if (k + 1 >= (index + 1) * 16 + index)
			sensorsData[index++] = chainData & 0xFFFF;
		digitalWrite(pinclk, LOW);
		delayMicroseconds(1);
	}
	this->closeComm();
}

void AS5043Class::pushBit(bool bit) {
	digitalWrite(pinprog, bit);
	delayMicroseconds(1);
	digitalWrite(pinclk, HIGH);
	delayMicroseconds(2);
	digitalWrite(pinclk, LOW);
	delayMicroseconds(1);
}

void AS5043Class::closeComm() {
	// Ensure pincsn and pinclk are left HIGH and pinprog LOW
	digitalWrite(pinprog, LOW);
	delayMicroseconds(1);
	digitalWrite(pincsn, HIGH);
	delayMicroseconds(1);
	digitalWrite(pinclk, HIGH);
}

void AS5043Class::setPins() {
	pinMode(pincsn, OUTPUT);
	pinMode(pinclk, OUTPUT);
	pinMode(pindo, INPUT);
	pinMode(pinprog, OUTPUT);
}
/*
unsigned long AS5043Class::atomSSI() {
	// Write a null sequence and wait
	SPDR = 0x00;
	;
	while (!(SPSR & (1 << SPIF)))
		;
	// Read value
	unsigned long _rxData = SPDR;
	return _rxData;
}
*/
void AS5043Class::setDataMode(uint8_t mode) {
	//SPCR = (SPCR & ~SPI_MODE_MASK) | mode;
}

void AS5043Class::setClockDivider(uint8_t rate) {
	//SPCR = (SPCR & ~SPI_CLOCK_MASK) | (rate & SPI_CLOCK_MASK);
	//SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((rate >> 2) & SPI_2XCLOCK_MASK);
}
