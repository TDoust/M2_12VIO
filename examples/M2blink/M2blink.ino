/*
Blink
Strobes the LEDS down the side of the M2 processor board repeatedly.
the strobing delay is a non blocking delay
also functioning as a HELLO WORLD type sketch.

along with demonstrating the 2 user buttons on the Macchina processor board
using interupts & button debouncing

This example code is in the public domain.
Author Tony Doust
modified 3/6/2017

*/

/*
Fade

the fade function delay of the RGB LED is a non blocking delay
Modified 3/6/2017
by Tony Doust

Modified 6 / 6 / 16 to demonstrate Macchina 2.0 RGB LEDs,
also functioning as a HELLO WORLD type sketch.

This example shows how to fade an LED on the RGB LED
using the analogWrite() function.


This example code is in the public domain.
*/

/*
M2I/O
to demonstrate the M2 I/O connect a LED with a series resistor between (J5 26 pin connector) pin2 (GPIO1 output) to ground pin26.
you can do the same for GPIO outputs GPIO2 & GPIO3 as these outputs are source pins.
if you wish to connect LEDS to GPIO pins GPIO4, GPIO5, GPIO6 these pins are SINK pins therfore you will need to connect the LED from J5 pin19 +12Vio
this pin will supply 12 volts monitored & controlled by the current limiting circuitry & the other side of the series resistor LED to the GPIOx 4, 5, 6
**** Ensure you observer correct polarity for the LED ***

This example code is in the public domain.
Author Tony Doust
modified 20/10/2017

*/

/*
M2I/O PWM
to demonstrate the M2 I/O PWM connect a LED with a series resistor to GPIO1 as above M2I/O example
M2 Buttons 1 & 2 will increment PWM frequency in multiples of 10Hz from a starting frequency of 10Hz with a Duty cycle of 50%
once you get above 30Hz you will not be able to distinguish the flasing of the LED by eye.

*/

#include <Arduino.h>
#include <pins_arduino.h>
#include <stdint.h>

#include "M2blink.h"
#include <M2_12VIO.h>


#define Strobe_Leds // **** demonstrate Strobing of the LEDs along the side of the M2 **** //

    // *** Uncomment the defines below to demonstrate the desired functions *** //
//#define M2IO_Leds   // **** Uncomment to demonstrate LEDs with a series resistor connected to M2 I/O Pins incrementing to the next pin **** //
//#define M2_PWM  // **** Uncomment to demonstrate setting up & using a GPIOx pin for PWM **** //  in this demo we are using pin GPIO1 with a series resistor connected LED

#ifdef M2_PWM
    #undef M2IO_Leds

    #define M2_PWM_Change_Frequency     // **** Uncomment to demonstrate PWM change frequency **** //

    #ifdef M2_PWM_Change_Frequency
        #undef M2_PWM_Previous_Selected
        #undef M2_PWM_Activate_Deactivate
    #else

        //#define M2_PWM_Previous_Selected    // **** Uncomment to demonstrate PWM how to Turn ON previously selected PWM **** //

        #ifdef M2_PWM_Previous_Selected
            #undef M2_PWM_Change_Frequency
            #undef M2_PWM_Activate_Deactivate
        #else

            //#define M2_PWM_Activate_Deactivate   // **** Uncomment to demonstrate PWM how to Turn ON previously selected PWM & also Activate/DeActivate PWM **** //

        #endif

    #endif

#else   /* M2_PWM has not been defined therfore ensure we disable all PWM functions & enable M2_Leds*/
    #undef M2_PWM_Change_Frequency
    #undef M2_PWM_Previous_Selected
    #undef M2_PWM_Activate_Deactivate
#endif


M2_12VIO M2IO;	//Constructor for the M2_12Vio library


// ************************************************************************************************************ //
//											Button Defines														//
// ************************************************************************************************************ //

debounce_t M2_Button1;
debounce_t M2_Button2;


uint32_t Button_endMillis;
uint32_t Button_startMillis;

// ************************************************************************************************************ //
//											Dimming Defines														//
// ************************************************************************************************************ //
#define Strobe_Delay 80	// Strobe delay between consectuive lights turning ON then OFF
#define Flash_Delay 500	// Flash delay between ON then OFF of the RED LED for warinings
#define Dimming_Delay 5	// Dimming delay between dimm steps
#define fadeAmount 5    // how many points to fade the LED by

bool Over_Current_Display_Latch = false;
uint32_t Brightness = 0;    // how bright the LED is

uint32_t Dimming_endMillis = 0;   /* used in loop timing for Dimming & Flashing Leds */

uint32_t Strobe_endMillis = 0;
uint8_t Strobe_Led = 10;

uint32_t Flash_endMillis = 0;

bool Dimming_Up = false;
bool Dimming_Down = false;

bool Led = 0;
bool Led5_OFF = false;
bool Led4_OFF = false;
bool Led3_OFF = false;
bool Led2_OFF = false;
bool Led1_OFF = false;
bool Led5_ON = false;
bool Led4_ON = false;
bool Led3_ON = false;
bool Led2_ON = false;
bool Led1_ON = false;

uint32_t Time_Delay_endMills = 0;
uint32_t Time_Delay_startMillis = 0;

float temp = 0;
uint32_t M2_IO_Pin = 1;
uint32_t M2_Analog_Pin = 1;

uint32_t Pin_Number = 2;

uint32_t Pwm_Frequency = 10;
uint32_t Pwm_Duty = 50;
bool Pwm_Active = false;


/*\brief Interrupt Service Routine for Button 1 on M2 Processor Board
*	Set the variable M2Button1.Pressed TRUE
*	to indicate button has been pressed
*\param NIL
*\return NIL
*/
void ISR_Button1(){
	M2_Button1.Pressed = true;
}

/*\brief Interrupt Service Routine for Button 2 on M2 Processor Board
*	Set the variable M2_Button2.Pressed TRUE
*	to indicate button has been pressed
*\param NIL
*\return NIL
*/
void ISR_Button2(){
	M2_Button2.Pressed = true;
}

void Strobe(){	// non blocking strobe delay of LEDS

	if(Led5_OFF){
		digitalWrite(Led5, HIGH);   // turn the LED OFF by making the output HIGH
		Led5_OFF = false;
	}
	if(Led4_OFF){
		digitalWrite(Led4, HIGH);   // turn the LED OFF by making the output HIGH
		Led4_OFF = false;
	}
	if(Led3_OFF){
		digitalWrite(Led3, HIGH);   // turn the LED OFF by making the output HIGH
		Led3_OFF = false;
	}
	if(Led2_OFF){
		digitalWrite(Led2, HIGH);   // turn the LED OFF by making the output HIGH
		Led2_OFF = false;
	}
	if(Led1_OFF){
		digitalWrite(Led1, HIGH);   // turn the LED OFF by making the output HIGH
		Led1_OFF = false;
	}
	if(Led5_ON){
		digitalWrite(Led5, LOW);    // turn the LED ON by making the output LOW
		Led5_ON = false;
	}
	if(Led4_ON){
		digitalWrite(Led4, LOW);    // turn the LED ON by making the output LOW
		Led4_ON = false;
	}
	if(Led3_ON){
		digitalWrite(Led3, LOW);    // turn the LED ON by making the output LOW
		Led3_ON = false;
	}
	if(Led2_ON){
		digitalWrite(Led2, LOW);   // turn the LED ON by making the output LOW
		Led2_ON = false;
	}
	if(Led1_ON){
		digitalWrite(Led1, LOW);    // turn the LED ON by making the output LOW
		Led1_ON = false;
	}
}

void Strobe_Off(){
	digitalWrite(Led5, HIGH);   // turn the LED off by making the output HIGH
	digitalWrite(Led4, HIGH);   // turn the LED off by making the output HIGH
	digitalWrite(Led3, HIGH);   // turn the LED off by making the output HIGH
	digitalWrite(Led2, HIGH);   // turn the LED off by making the output HIGH
	digitalWrite(Led1, HIGH);   // turn the LED off by making the output HIGH
}

void Flash_Red_Led(){   // Flash the RED led
	digitalWrite(Led1, Led ? HIGH : LOW);
	Led = Led ? 0 : 1;
}

bool Button_Debounce(debounce_t *Button){
	// read the state of the switch into a local variable:
	//Button->reading = digitalReadNonDue(Button->M2_Button);
	Button->reading = Button->Pressed;
	Button->Pressed = false;

	// check to see if you just pressed the button
	// (i.e. the input went from HIGH to LOW),  and you've waited
	// long enough since the last press to ignore any noise:

	// If the switch changed, due to noise or pressing:
	if(Button->reading != Button->lastButtonState){
		// reset the debouncing timer
		Button->lastDebounceTime = millis();
	}

	if((millis() - Button->lastDebounceTime) > Debounce_Delay){
		// whatever the reading is at, it's been there for longer
		// than the debounce delay, so take it as the actual current state:

		// if the button state has changed:
		if(Button->reading != Button->buttonState){

			// only toggle Pressed if the new button state is TRUE
			if(Button->buttonState == true){
				Button->Pressed = true;
			}
			Button->buttonState = Button->reading;
		}
	}

	// save the reading.  Next time through the loop,
	// it'll be the lastButtonState:
	Button->lastButtonState = Button->reading;

	return(Button->Pressed);
}

void setup() {
    digitalWrite(PS_BUCK, HIGH);    // Turn ON the Interface Circuit - 12V to 5V DC-DC Converter (Interface Schematic Sht 4)
	SerialUSB.begin(115200);
    delay(1000);
	SerialUSB.print("\nSetup Started");


	M2_Button1.M2_Button = Button1;
	M2_Button1.lastButtonState = true;
	M2_Button1.buttonState = true;
	M2_Button1.Pressed = false;
	M2_Button1.FlipFlop = 0;

	M2_Button2.M2_Button = Button2;
	M2_Button2.lastButtonState = true;
	M2_Button2.buttonState = true;
	M2_Button2.Pressed = false;
	M2_Button2.FlipFlop = 0;

	Strobe_Led = 10;
	Strobe_endMillis = millis();

	Dimming_endMillis = millis();   // Used for Dimming LEDS & Flashing Leds

	pinMode(Button1, INPUT);        // Set Button 1 as input
	digitalWrite(Button1, HIGH);
	pinMode(Button2, INPUT);        // Set Button 2 as input
	digitalWrite(Button2, HIGH);

	//attachInterrupt(Button1, ISR_Button1, FALLING);	// ISR Button 1
	attachInterrupt(digitalPinToInterrupt(Button1), ISR_Button1, FALLING);	// ISR Button 1
	//attachInterrupt(Button2, ISR_Button2, FALLING);	// ISR Button 2
	attachInterrupt(digitalPinToInterrupt(Button2), ISR_Button2, FALLING);	// ISR Button 2
	
	pinMode(Led1, OUTPUT);		// Set the Green LED PIN as an OUTPUT to enable flasing
	digitalWrite(Led1, HIGH);	// Turn the Green LED OFF by making the PIN HIGH
	pinMode(Led2, OUTPUT);		// Set the Orange LED PIN as an OUTPUT to enable flasing
	digitalWrite(Led2, HIGH);   // turn the Orange LED OFF by making the PIN HIGH
	pinMode(Led3, OUTPUT);		// Set the Orange LED PIN as an OUTPUT to enable flasing
	digitalWrite(Led3, HIGH);   // turn the Orange LED OFF by making the PIN HIGH
	pinMode(Led4, OUTPUT);		// Set the Orange LED PIN as an OUTPUT to enable flasing
	digitalWrite(Led4, HIGH);   // turn the Orange LED OFF by making the PIN HIGH
	pinMode(Led5, OUTPUT);		// Set the RED LED PIN as an OUTPUT to enable flasing or Flash ERROR Condition
	digitalWrite(Led5, HIGH);   // turn the RED LED OFF by making the PIN HIGH


	pinMode(Led_RGB_Red, OUTPUT);       // declare RED led pin to be an output:
	analogWrite(Led_RGB_Red, 255);      // Turn the LED OFF
	pinMode(Led_RGB_Green, OUTPUT);     // declare GREEN led pin to be an output:
	analogWrite(Led_RGB_Green, 255);    // Turn the LED OFF
	pinMode(LED_RGB_Blue, OUTPUT);      // declare BLUE led pin to be an output:
	analogWrite(LED_RGB_Blue, 255);     // Turn the LED OFF

	M2_Button1.Pressed = false;
	M2_Button2.Pressed = false;
    
    M2IO.Init_12VIO();  // Initialise the M2I/O library

	Over_Current_Trip = false;
    Over_Current_Display_Latch = false;
	Dimming_Up = true;
	Brightness = 0;
    M2_IO_Pin = 1;
    M2_Analog_Pin = 1;

    #ifdef M2_PWM
        M2IO.Setpin_12VIO(M2_IO_Pin, ON, SOURCE, PWM_PIN, Pwm_Frequency, Pwm_Duty);
        Pwm_Active = true;
    #endif
}

void loop() {
        // **** demonstrate Strobing of the LEDS along the side of the M2 **** //
	if(!Over_Current_Trip){	// non blocking LED strobe delay
		if((millis() - Strobe_endMillis) > Strobe_Delay){
			Strobe_endMillis = millis();
			switch(Strobe_Led){
				case 10:
					Led5_OFF = true;
					break;
				case 9:
					Led4_OFF = true;
					break;
				case 8:
					Led3_OFF = true;
					break;
				case 7:
					Led2_OFF = true;
					break;
				case 6:
					Led1_OFF = true;
					break;
				case 5:
					Led5_ON = true;
					break;
				case 4:
					Led4_ON = true;
					break;
				case 3:
					Led3_ON = true;
					break;
				case 2:
					Led2_ON = true;
					break;
				case 1:
					Led1_ON = true;
					break;
				default:
					break;
			}
			--Strobe_Led;
			if(Strobe_Led <= 0){
				Strobe_Led = 10;
			}
        #ifdef Strobe_Leds
            Strobe();
        #endif
		}
	} else{	// Over_Current = TRUE Disable the strobing & flash the RED LED. To Reset press Button 1
		if(Over_Current_Display_Latch){
			if((millis() - Flash_endMillis) > Flash_Delay){
				Flash_endMillis = millis();
				Flash_Red_Led();
			}
			if(M2_Button1.Pressed){
				Over_Current_Trip = false;
                Over_Current_Display_Latch = false;
				Flash_Red_Led();
				Strobe_Led = 10;
				Strobe_endMillis = millis();
			}
		} else{	// Display the Load AMPS on the first occurance only
            uint32_t Temp_Load = M2IO.Load_Amps();
			SerialUSB.print("\n**Over_Current Amps*** = ");
			SerialUSB.print(Temp_Load);
            SerialUSB.print("mA");
        #ifdef Strobe_Leds
            Strobe_Off();
        #endif
            Over_Current_Display_Latch = true;
		}
	}

	if(millis() - Dimming_endMillis > Dimming_Delay){	// RGB Leds Dimming
            // **** demonstrate analogWrite to change fading Brightness of RGB Leds **** //
        analogWrite(Led_RGB_Red, Brightness);	// set the brightness of RGB Blue Led pin: //
		analogWrite(Led_RGB_Green, Brightness);	// set the brightness of RGB Blue Led pin: //
		analogWrite(LED_RGB_Blue, Brightness);	// set the brightness of RGB Blue Led pin: //
		if(Dimming_Up){
			Brightness = Brightness + fadeAmount;	// change the brightness for next time through the loop: //
		}else{
			Brightness = Brightness - fadeAmount;	// change the brightness for next time through the loop: //
		}

		if(Dimming_Up && Brightness >= 4095){	// reverse the direction of the fading at the ends of the fade: //
			Brightness = 4095;
			Dimming_Up = false;
		}else if(!Dimming_Up && Brightness <= 0){
			Brightness = 0;
			Dimming_Up = true;
		}

		Dimming_endMillis = millis(); // Used for Dimming LEDS & Flashing Leds //
	}

	if(Time_Delay_endMills >= 4000){

        temp = M2IO.Supply_Volts();
        SerialUSB.print("\nVehicle Voltage = ");
        SerialUSB.print(temp / 1000);
        SerialUSB.print("V");

        temp = M2IO.Read_12VIO(M2_Analog_Pin);
        SerialUSB.print("\nAnalogue Input Voltage = ");
        SerialUSB.print(temp / 1000);
        SerialUSB.print("V");

        SerialUSB.print("\nAnalogue Button = ");
        if(M2IO.GetButton_12VIO(M2_Analog_Pin)){
            SerialUSB.print("ON");
        }else{
            SerialUSB.print("OFF");
        }


        temp = M2IO.Load_Amps();
        SerialUSB.print("\nM2 I/O Output Load = ");
        SerialUSB.print(temp);
        SerialUSB.print("mA");

        SerialUSB.print("\nChip Temperature ");
        SerialUSB.print(M2IO.Temperature());
        SerialUSB.print("C");

        SerialUSB.println();
		Time_Delay_startMillis = millis();
	}

	Time_Delay_endMills = millis() - Time_Delay_startMillis;    // Time delay between displaying readings
	Button_endMillis = millis() - Button_startMillis;

	if(M2_Button1.Pressed){
		Button_Debounce(&M2_Button1);
		M2_Button1.FlipFlop = !M2_Button1.FlipFlop;
		if(M2_Button1.FlipFlop){	// True
            #ifdef M2_PWM
                Pwm_Active = true;
                #ifdef M2_PWM_Activate_Deactivate
                    M2IO.Setpin_12VIO(M2_IO_Pin, ON, SOURCE, PWM_PIN, Pwm_Frequency, Pwm_Duty);
                #endif
                #ifdef M2_PWM_Previous_Selected
                    M2IO.Setpin_12VIO(M2_IO_Pin, ON);
                #endif
                #ifdef M2_PWM_Change_Frequency
                    if(Pwm_Active){
                        Pwm_Frequency = Pwm_Frequency - 10;
                        if(Pwm_Frequency <= 10){
                            Pwm_Frequency = 10;
                        }
                        SerialUSB.print("\nPwm Frequency = ");
                        SerialUSB.print(Pwm_Frequency);
                        SerialUSB.print("Hz");
                        M2IO.Change_Frequency_12VIO(M2_IO_Pin, Pwm_Frequency);
                    }
                #endif
            #endif
            #ifdef M2IO_Leds
                if(Pin_Number > 6){
                    Pin_Number = 0;
                }
                ++Pin_Number;   // Increment to the next output pin
            #endif
            SerialUSB.print("\nButton 1 ON");
		} else{	//False
        #ifdef M2_PWM
            #ifdef M2_PWM_Previous_Selected
                M2IO.Setpin_12VIO(M2_IO_Pin, OFF);
            #endif
        #endif
            SerialUSB.print("\nButton 1 OFF");
        }
	}

	if(M2_Button2.Pressed){
		Button_Debounce(&M2_Button2);
		M2_Button2.FlipFlop = !M2_Button2.FlipFlop;
		if(M2_Button2.FlipFlop){
            #ifdef M2_PWM
                Pwm_Active = true;
                #ifdef  M2_PWM_Change_Frequency
                    if(Pwm_Active){
                        Pwm_Frequency = Pwm_Frequency + 10;
                        if(Pwm_Frequency > Max_Hz){
                            Pwm_Frequency = Max_Hz;
                        }
                        SerialUSB.print("\nPwm Frequency = ");
                        SerialUSB.print(Pwm_Frequency);
                        SerialUSB.print("Hz");
                        M2IO.Change_Frequency_12VIO(M2_IO_Pin, Pwm_Frequency);
                    }
                #endif
            #endif
            #ifdef M2IO_Leds
                M2IO.Setpin_12VIO(Pin_Number, ON);  // Turn the output ON. Used for demonstrating I/O Pins with LEDS connected to M2 I/O pins
            #endif
            SerialUSB.print("\nButton 2 ON");
		} else{
            #ifdef M2IO_Leds
                M2IO.Setpin_12VIO(Pin_Number, OFF);  // Turn the output OFF. Used for demonstrating I/O Pins with LEDS connected to M2 I/O pins
            #endif
            #ifdef M2_PWM
                #ifdef M2_PWM_Activate_Deactivate
                M2IO.Setpin_12VIO(M2_IO_Pin, OFF, SOURCE, PWM_OFF, Pwm_Frequency, Pwm_Duty);
                #endif
                Pwm_Active = false;
            #endif
            SerialUSB.print("\nButton 2 OFF");
		}
	}
}
