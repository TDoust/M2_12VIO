#ifndef _M2_Blink_h
#define _M2_Blink_h

#include <stdint.h>

#define Led1 DS2	// Red LED
#define Led2 DS3	// Yellow LED
#define Led3 DS4	// Yellow LED
#define Led4 DS5	// Yellow LED
#define Led5 DS6	// Green LED

#define Led_RGB_Red	RGB_RED
#define Led_RGB_Green RGB_GREEN
#define LED_RGB_Blue RGB_BLUE

// ***Change this value if the button is triggering for more than 1 button push*** //
uint32_t Debounce_Delay = 1;	// this is the number of milli seconds delay before checking the button again in Debounce

typedef struct{
	uint32_t M2_Button;                     // the button we are reading from
	volatile bool buttonState = true;       // the current reading from the input pin
	volatile bool lastButtonState = true;   // the previous reading from the input pin
	volatile bool reading = true;           // holds the value read from the button
	volatile uint8_t FlipFlop = 0;          // Toggles when button changes
	volatile bool Pressed = false;

	// the following variables are unsigned long's because the time, measured in miliseconds,
	// will quickly become a bigger number than can be stored in an int.
	volatile uint32_t lastDebounceTime = 0;  // the last time the output pin was toggled
}debounce_t;

#endif //_M2_Blink_h

