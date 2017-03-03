#ifndef _SIMPLESERVO_H_INCLUDED
#define _SIMPLESERVO_H_INCLUDED

#include "Servo.h"

/* To simplify control operation, this differs from the vanilla servo class in various ways:
 *
 * 	1.- Angle sent goes from -90 to + 90 degrees instead of 0 to 180
 * 	2.- There is no attach() and detach() methods, the pin must be specified in the constructor
 * 	3.- An special method, stop(), writes the 0 angle (just syntactic sugar)
 * 	4.- The range value limits the max angle value, to avoid extremes in bad servos
 *
 */

#define MIN_SERVO_DEFAULT		1000
#define MAX_SERVO_DEFAULT		2000

class SimpleServoClass {
public:
	SimpleServoClass(Servo *servoobj, int range);							// Associates the class with a Servo attach pin
	SimpleServoClass(Servo *servoobj, int min, int max, int range);		// Associates the class with a Servo attach pin and sets min and max values in uS
	void write(int value);									// Sends angle from -90 to 90 to servo, if value >= 544, calls writeMicroseconds
	void writeMicroseconds(int value); 						// Write pulse width in microseconds (directly, only check for min-max)
	int read(); 											// returns current pulse width as an angle between -90 and 90 degrees
	int readMicroseconds(); 								// returns current pulse width in microseconds for this servo
	void stop();											// Sets center (write 0)
	void attach(int servoPin);							// Attach needs to be done in runtime
private:
	Servo *servo;											// Servo object
	int min;												// minimum in us, -90 degrees
	int max;												// maximun in us, +90 degrees
	int range;
};
#endif
