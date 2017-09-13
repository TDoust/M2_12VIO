#ifndef _Debounce_h
#define _Debounce_h

#include <stdint.h>

uint32_t debounceDelay = 5;    // the debounce time; increase if the output flickers


typedef struct{
	uint32_t M2_Button;	// the button we are reading from
	volatile bool buttonState = HIGH;	// the current reading from the input pin
	volatile bool lastButtonState = HIGH;   // the previous reading from the input pin
	volatile bool reading = HIGH;	// holds the value read from the button
	uint8_t FlipFlop = 0;	// Toggles when button changes
	volatile bool Pressed = LOW;

	// the following variables are unsigned long's because the time, measured in miliseconds,
	// will quickly become a bigger number than can be stored in an int.
	volatile uint32_t lastDebounceTime = 0;  // the last time the output pin was toggled
}debounce_t;

#endif //_Debounce_h
