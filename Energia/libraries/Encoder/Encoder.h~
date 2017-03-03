#ifndef _ENCODER_H_INCLUDED
#define _ENCODER_H_INCLUDED

#include <stdio.h>
#include <AS5043.h>

#define MODULE_RANGE		1024

/*
 * This encoder class serves as a general wrapper for reading encoders. Whether they are analog encoders (a pot, for example) or
 * a digital encoder (magnetic, absolute, 10 bits resolution), this class works. The idea is that, for the arm, each of the first
 * 4 DOF need an encoder to measure an output position. Now, the first encoder, the one for the z motion, uses a pot (as of
 * now), multi-turn, using a single "effective" turn between a minimal and maximal analog read values for the turn. Using an
 * absolute encoder would not change much in terms of a single turn, but a problem arises with the "jump" effect of the encoder,
 * when passing from 1023 to 0, or in reverse.
 *
 * The other degrees of freedom are easier to control within the turn. Simply, only the first DOF is able to ever turn endlessly,
 * the other degrees of freedom ALL use a magnetic absolute encoder with minimal and maximal angles WITHIN a single turn. The
 * second DOF can only go from -angle to +angle (a consistent mapping of -90 to +90) "degrees". The third DOF has similar restrictions,
 * and the fourth DOF too. Since all of them use magnetic absolute encoders, a "fix" in mapping values is done easily by just
 * physically manipulating the magnets. But these DOF are subject to the "jump" effect too, under certain circumstances.
 *
 * To solve this possible inconvenience, an abstraction is proposed. Each encoder is declared with a minimal and maximal lecture
 * value, that we are going to translate to an angle between -angle and +angle. If there is no jump between the minimal and maximal (this
 * is, the minimal is less than the maximal and we go from the minimal to the maximal through "increasing" values of the lecture), then
 * there is nothing to fix. But if this is not so, then a fix is needed. Basically:
 *
 * 		Let lecture - minimal and lecture - maximal be called the "differences". Then, in order to account for the jump, we want to
 * 		look at these differences modulo 1024 (this is, in terms of the operation %, taking both the positive AND negative possibilities).
 * 		This module taking DOES NOT by itself solves the problem of the jump, but a simple arrangement will do. Lets say that we get that
 *
 * 			lecture - minimal
 *
 * 		is negative, then (lecture - minimal + 1024) == (lecture - minimal) mod 1023, and also positive. We can then use this positive
 * 		remainder as our basis for calculations. This is, instead of using "absolute" values, we consider "cycles" of values, or cycles
 * 		of "remainders" when diving modulus some integer number. This simple operation solves the jump problem for when reporting the
 * 		position value. We simply look at the positive "congruent" value of the first difference.
 *
 * 		Now, lets suppose we read a value outside the range module 1024. This is, the positive remainder mod 1024 of lecture - minimal is
 * 		greater than the positive remainder of maximal - minimal. Do we report a value greater than +angle or less than -angle? We can do a
 * 		simple check, basically, we decide the "side" based on which value is "closer" (IN ABSOLUTE VALUE) to either the minimal or the
 * 		maximal. This is, we check now both values for BOTH differences, an look for the minimal magnitude of the remainder. This will give us
 * 		the "closer" side, so we use that side to report a value less than -angle OR greater than +angle.
 *
 * 	The logic is the same if taking decreasing values instead of increasing ones. Also, since this class tries to be simple, either
 * 	saturation or limitation is LEFT to outside objects or algorithms.
 *
 */

class EncoderClass {
public:
	/*
	 * The basic constructor types specify:
	 *
	 * anPIn - if the sensor is pure analog, use this pin for analogRead
	 * AS5043 - if the sensor is MGN ABS 10 BIT in SPI (hardware or software) mode read, use this object to perform read
	 * indexChain - The "number" of the sensor in the "chain". Use 0 if there is a single sensor per CS pin, at most MAX_DEVICES - 1  (currently 3)
	 * pinCSn - Serves to specify an CS pin to perform read, in case one is needed for this encoder
	 *
	 * min - minimal value of lecture, if not specified then 0
	 * max - maximal value of lecture, if not specified then 1023
	 * map_out - if > 0, use INCREASING values, if < 0, use DECREASING values, if 0 report lecture
	 * 		   the magnitude of map_out is used to specify "output angle" size. This is, if magnitude is 127, read goes from -127 to +127,
	 * 		   if magnitude is 511, read goes from -511 to +511, etc.
	 *
	 */
	EncoderClass(int anPin, int min, int max, int maxChange, int map_out);				// Also
	EncoderClass(AS5043Class* AS5043, int pinCSn, int map_out);							// Full turn, 0 => -abs_map_out, 1023 => +abs_map_out
	EncoderClass(AS5043Class* AS5043, int pinCSn, int min, int max, int map_out);
	EncoderClass(AS5043Class* AS5043, int pinCSn, int indexChain, int map_out);		// Full turn, 0 => -abs_map_out, 1023 => +abs_map_out
	EncoderClass(AS5043Class* AS5043, int pinCSn, int indexChain, int min, int max, int map_out);

	unsigned int read();					// Reads sensor and returns 10 bit ANGLE stream or the RAW analogRead
	int readAngle();						// Performs read and returns angle between -abs_map_out and +abs_map_out
	int readChange();						// Returns the change (fixed modulo 1023) between reads in terms of -abs_map_out to +abs_map_out
	unsigned int get();						// Gets last read 10 bit ANGLE stream or the RAW analogRead
	int getAngle();							// Returns angle between -abs_map_out and +abs_map_out from last read
	int getChange();						// Return the last change calculated between reads
											// If NOT using an AS5043 the next will return 0 or nothing
	void begin();							// If using the AS5043 object, performs initialization sequence
	void softProg(unsigned int data);		// If using the AS5043 object, perform a software program with data
	unsigned int singleAlign();				// If using the AS5043 object, return an alignment mode read first 10 bit RAW! Does not modifies lecture

	int debug_var;
	
private:
	int min;								// min of read from 0 to 1023
	int max;								// max of read from 0 to 1023
	int maxChange;
	int map_out;								// direction and sampling size specification

	bool isAnalog : 1;						// Is analog (the first constructor)?
	int pinanalog;						// pin to use in analog mode

	AS5043Class* as5043;					// Pointer to reading object in SPI mode
	int index : 3;						// index of sensor in chain
	int pincsn;							// Slave select pin of chain

	int lecture;							// The lecture value, RAW value
	int lastlecture;
	int change;

	int angle;
	int lastAngle;

	void getAtom();					// Calculate the angle after each read
};
#endif
