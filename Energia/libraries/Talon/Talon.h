#ifndef _TALON_H_INCLUDED
#define _TALON_H_INCLUDED

#include "Servo.h"

/* To simplify control operation, this differs from the vanilla Servo class in various ways:
 *
 * 	1.- Angle sent goes from -127 to + 127 "degrees" instead of 0 to 180
 * 	2.- There is no attach() and detach() methods, the pin must be specified in the constructor
 * 	3.- An special method, stop(), writes the 0 angle (just syntactic sugar)
 * 	4.- As with Servo, there is a min and max, but also an added center value (0 degrees in here, 90 degrees in standard Servo),
 * 		since center can differ from the min/max "real" center, angles are mapped properly:
 *
 * 			Say that min is 500 uS, max is 2500 uS, then the min/max "real" center is 1500 uS
 * 			But we can set the center value to 2000 uS (can be done with the Talon, because of hobby joystick restrictions)
 * 			Then -127 a 0 degrees will cover 500 usS to 2000 uS and 0 to 127 from 2000 uS to 2500 uS !!!
 * 			Inversely, reading the angle value maps to the "right" extremes
 *
 * 		Thus, while, like with Servo, the user can send values in raw uS, this is NOT recommended
 *
 * 	5.- An umbral value specifies the minimal write angle for nonzero output, to ease entry to brake mode
 *
 */

#define MIN_TALON_DEFAULT		1000
#define MAX_TALON_DEFAULT		2000
#define CENTER_TALON_DEFAULT	1500

class TalonClass {
public:
	TalonClass(int umbral, int range);									// Associates the Talon with a Servo attach pin
	TalonClass(int min, int max, int umbral, int range);				// Associates the Talon with a Servo attach pin and sets min and max values in uS (RECALCULATES CENTER!)
	TalonClass(int min, int max, int center, int umbral, int range);	// Also sets the center value
	void write(int value);																		// Sends angle to servo, if value > 544, calls writeMicroseconds
	void writeMicroseconds(int value); 															// Write pulse width in microseconds (directly, only check for min-max)
	int read(); 																				// Returns current pulse width as an angle between -127 and 127 degrees
	int readMicroseconds(); 																	// Returns current pulse width in microseconds for this servo
	void calibrate(int value);																	// Adds or decreases microseconds to the center value (to ensure stop works well)
	void stop();																				// Sets center (stops the talon)
	void attach(int servoPin);																				// attach needs to be done in runtime
private:
	Servo servo;																				// Servo object
	int min;																					// minimum in us, -127 degrees
	int max;																					// maximun in us, +127 degrees
	int center;																					// center in us, 0 degrees
	int umbral;
	int range;
};
#endif
