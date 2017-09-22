/*
*  M2_12VIO.h for Macchina M2
*
*	Author:	Tony Doust
*	Date:	8/5/2017
*	Version: V0.0.1
*
* Short description:
*	Macchina 2.0 has 6 General Purpose 12 volt outputs 3xSOURCE & 3xSINK Outputs & 6 x 12 Volt Analogue Input Driver circuits.
*	The 12VIO.h Library is for control & Monitoring of the Macchina M2 general purpose 12V IO pins available on the 26 pin connector.
*	When Sourcing power from the M2 or (Sinking power from the M2 via +12VIO PIN 19) the 12V rail is monitored by an internal current monitor.
*	The goal here is to be able to dynamically measure how much power 1 or all external device are using AND ensure that the total power used from the OBD2
*	port does not exceed X amps and blow the car's OBD2 fuse or the main fuse (F1 2.6A) on the Macchina M2.
*
* Instance M2_12VIO::
* Functions:
*	Init_12VIO()				// Initialise the 12VIO & the M2 I/O power supply current monitoring
*	Setpin_12VIO(3, ON)			// turn output pin 3 ON
*	Setpin_12VIO(3, ON, SOURCE)	// sets output 3 to SOURCE 12 volts & turns output pin ON	"*** Macchina BETA Hardware ONLY ***"
                                // after setting the output pin to SOURCE or sink then use Setpin_12VIO(3, ON or OFF) to switch the pin ON or OFF
*	Setpin_12VIO(4, OFF)		// turn output pin 4 OFF
*	Setpin_12VIO(4, ON, SINK)	// sets output 4 to SINK 12 volts & turns output pin ON		"*** Macchina BETA Hardware ONLY ***"

*	Setpin_12VIO(3, PWM, 75)	// set pin 3 to PWM with a 75% duty cycle
*	Load_Amps()					// returns the total load currently being drawn from the M2 +12io line
                                // if there has been a overload condition Load_Amps will return the load drawn at the time of the overload condition
*	Supply_Volts()				// returns the battery volts of the vehicle the M2 is plugged into
*	Read_12VIO(IO_Pin)			// reads the Analogue pin & returns the scaled value as mVolts
*	Getpin_12VIO(IO_Pin)		// read the analogue pin as a digital input to enable the analogue input to be used as a button input
*	Temperature()				// returns the internal temperature of the SAM DUE processor chip
*

Private Functions
*	Enable_12VIO_Monitor(ON)	// Turn ON or OFF (12Vio_EN pin) thus Enabling or Disabling All 12VIO Outputs together (i.e. this can be considered as a Master ON/OFF switch)
*
*
*	!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*				Important Note:
*	**** WARNING Not suitable for use with Macchina M2 Hardware Beta version ****
*	If using the M2 BETA hardware with this software the six SOURCE pins will only be used.
*	if you enable the sink pins be aware that you could inadvertantely cause a short across the output
*	by having the SOURCE & sink pins for the same output turned on at the same time.
*
*	It is possible to create a direct short on the I/O Mosfets & damage your Macchina BETA M2 I/O Board
*	by not controlling the 6 general purpose 12V I/O pins correctly.
*
*	!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*
*	This library is especially important since it abstracts the lower-level drivers and ensures that the overload circuit is controlled correctly
*
*
*  Arduino IDE 1.8
*
*/

/*  Copyright (c) 2016 Tony Doust
*
*  Permission is hereby granted, free of charge, to any person obtaining
*  a copy of this software and associated documentation files (the
*  "Software"), to deal in the Software without restriction, including
*  without limitation the rights to use, copy, modify, merge, publish,
*  distribute, sublicense, and/or sell copies of the Software, and to
*  permit persons to whom the Software is furnished to do so, subject to
*  the following conditions:
*
*  The above copyright notice and this permission notice shall be included
*  in all copies or substantial portions of the Software.
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
*  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
*  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
*  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
*  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef _M2_12VIO_H
#define _M2_12VIO_H

#include <Arduino.h>
#include <pins_arduino.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


    enum Pin_Mode{
        OFF, ON 
    };

    enum Source_Mode{
        SOURCE, SINK
    };

    enum Pwm_Mode{
        PWM_PIN, PWM_ALT
    };

    enum Output_Pin{
        Output_Pin_UnUsed, Output_1, Output_2, Output_3, Output_4, Output_5, Output_6
    };

    enum Input_Pin{
        Input_Pin_UnUsed, Input_1, Input_2, Input_3, Input_4, Input_5, Input_6
    };

    // ************************************************************************************************************ //
    //												I/O Pin Defines													//
    // ************************************************************************************************************ //
    /* I/O Port 1 */
#define GPIO1A GPIO1
#ifdef MACCHINA_M2_BETA	// M2 Beta legacy Hardware Sink Input Pins
#define GPIO1B GPIO1_B
#else
#define GPIO1B GPIO1
#endif
#define Analog1 ANALOG_1	/* Analog Input 1 */

    /* I/O Port 2 */
#define GPIO2A GPIO2
#ifdef MACCHINA_M2_BETA	// M2 Beta legacy Hardware Sink Input Pins
#define GPIO2B GPIO2_B
#else
#define GPIO2B GPIO2
#endif
#define Analog2 ANALOG_2	/* Analog Input 2 */

    /* I/O Port 3 */
#define GPIO3A GPIO3
#ifdef MACCHINA_M2_BETA	// M2 Beta legacy Hardware Sink Input Pins
#define GPIO3B GPIO3_B
#else
#define GPIO3B GPIO3
#endif
#define Analog3 ANALOG_3	/* Analog Input 3 */

    /* I/O Port 4 */
#define GPIO4A GPIO4
#ifdef MACCHINA_M2_BETA	// M2 Beta legacy Hardware Sink Input Pins
#define GPIO4B GPIO4_B
#else
#define GPIO4B GPIO4
#endif
#define Analog4 ANALOG_4	/* Analog Input 4 */

    /* I/O Port 5 */
#define GPIO5A GPIO5
#ifdef MACCHINA_M2_BETA	// M2 Beta legacy Hardware Sink Input Pins
#define GPIO5B GPIO5_B
#else
#define GPIO5B GPIO5
#endif
#define Analog5 ANALOG_5	/* Analog Input 5 */

    /* I/O Port 6 */
#define GPIO6A GPIO6
#ifdef MACCHINA_M2_BETA	// M2 Beta legacy Hardware Sink Input Pins
#define GPIO6B GPIO6_B
#else
#define GPIO6B GPIO6
#endif
#define Analog6 ANALOG_6	/* Analog Input 6 */

#define Cpu_Temp_Pin CPU_TEMP	// Processor Chip Temperature



	// M2 I/O Class Definition
class M2_12VIO{
    private:
       /*------------------------------------------------------------------------------------------------------------------
        *                                                     Definitions
        *------------------------------------------------------------------------------------------------------------------*/
		// ************************************************************************************************************ //
		//                          M2 Design Voltages & Currents for Analogue Calcs                                    //
		// ************************************************************************************************************ //
		/* *** Note This will Blow the main Fuse F1 *** Maximum 12Volt Current draw (Allowed) supplied through Q6 Mosfet */
		#define Max_Current_M2 3.733            // mA The Maximum Design Current through Q6 Mosfet & measured across R18 by ICU2 for 12vio

		#define ADCVREF 3.0                     // mV VCC Vref Voltage to processor for Analogue Calculations
        #define Analog_Vref 3.0                 // Analogue Reference Voltage
        #define Vehicle_Vref 3.0                // Vehicle Reference Voltage

        #define Design_Current 2.262            // mA  Design Maximum Current 2.262A for IC U2 ZXCT1109 for a 2.0V output across R25
		#define Design_Volts 2.000				// mV Design Voltage output from U2 & Input to (I SENSE) Analogue Input for comparison
		#define Max_Current_Per_Channel .166    // mA Maximum Current Draw of 1 12VIO output pin = (166mAmps)

		#define Max_Vehicle_Volts 16.63         // 18.30mV Maximum Vehicle Voltage used for voltage divider input to processor pin (PA6, pin 82, AD3) for vehicle voltage 18.3V
		#define Max_Analog_Volts 16.63		    // Maximum Analogue Voltage used for voltage divider for Analogue Inputs ANA_1-ANA_6
        #define Number_Analog_Samples 16        // Number of Analogue samples to take

		// ************************************************************************************************************ //
		//                      Vehicle Voltage, 12V Current Sensing & 12VPower Enable Pin Defines                      //
		// ************************************************************************************************************ //
		#define IO_Enable I_SENSE_EN                // 12VIo_Enable PIN ((75) PIO_PC24)
		#define Interupt_Over_Current I_SENSE_INT   // Interupt Over_Current Input PIN ((76) PIO_PD1) to generate an Interupt
		#define Dac_Compare I_SENSE_DAC             // Analogue Output PIN ((95) PIO_PB16) to U3A Comparator for Over Current Compare
		#define Supply_Amps I_SENSE                 // Analogue Input PIN ((93) PIO_PB17) Amps being used from Output of U2

		// **** Pin Description assignment needs to be checked **** //
		#define	Voltage_Sense V_SENSE               // Analogue Input pin(92) ADC3 PA6X1_AD3 read the Vehicles 12Volt supply across voltage divider resistor network R34/R48 to the M2 from the Vehicle

		// ************************************************************************************************************ //
		//										Error Number Defines													//
		// ************************************************************************************************************ //
		#define Err_Ok 0	/* Operation Successful */
		#define Err_Pin_Range 1	/* PIN Out of Range */
		#define Err_Mode_Range 2	/* Mode Out of Range */
		#define Err_Mode_Selection 3	/* SOURCE or SINK has NOT been selected */
		#define Err_Mode_Selection_On 4	/* SOURCE or SINK Pin selected is incorrect prior to Turning Pin ON */
		#define Err_Mode_Selection_Off 5	/* SOURCE or SINK Pin selected is incorrect prior to Turning Pin OFF */
		#define Err_Mode_Conflict 6	/* SOURCE or SINK previously set does not match current Operation*/
		#define Err_Mode_ON 7	/* Pin already turned ON while trying to change Modes SOURCE to Sink or viceversa */
		#define Err_Init 8	/* Pin mode has not been selected prior to turning ON or OFF */
        #define Err_Duty_Range 9  /* Duty cycle not in range */


    public:

		uint16_t Init_12VIO();
		uint16_t Reset_Current_Limit();
		uint32_t Load_Amps();
		uint32_t Supply_Volts();
		uint32_t Read_12VIO(uint32_t IO_Pin);
        uint16_t Setpin_12VIO(uint32_t IO_Pin, uint8_t Pin_Mode);
        uint16_t Setpin_12VIO(uint32_t IO_Pin, uint8_t Pin_Mode, uint8_t Source_Mode);
        uint16_t Setpin_12VIO(uint32_t IO_Pin, uint8_t Pin_Mode, uint8_t Source_Mode, uint8_t Pwm_Mode, uint8_t Duty);
        uint16_t InitButton_12VIO(uint32_t IO_Pin);
        bool GetButton_12VIO(uint32_t IO_Pin);
		float Temperature();

    private:

		uint8_t Enable_12VIO_Monitor(uint8_t mode);

	}; // end M2 I/O Class Definition

extern int16_t Setpin_Error;
extern volatile uint32_t M2_Amps;
extern volatile bool Over_Current_Trip;
extern void ISR_Over_Current();



#ifdef __cplusplus
}
#endif	//end __cplusplus

#endif /* _M2_12VIO_H */
