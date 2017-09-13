# M2_12VIO Macchina M2 I/O Library


*  M2_12VIO.h Library for the Macchina M2

	Author:	Tony Doust
	Date:	8/5/2017
	Version: V1.0

# Short description:
	Macchina 2.0 has 6 General Purpose 12 volt outputs 3xSOURCE & 3xSINK Outputs & 6 x 12Volt Analogue Input Driver
	circuits.
	The 12VIO.h Library is for control & Monitoring of the Macchina M2 general purpose 12V IO pins available on the
	26 pin connector.

	When Sourcing power from the M2 or (Sinking power from the M2 via +12VIO on the 26 pin connector PIN 19)
	the 12V rail is monitored by an internal current monitor circuit via I_SENSE with feed back via I_SENSE_DAC.

	The goal here is to be able to dynamically measure how much power 1 or all currently ON external device are using
	AND ensure that the total power used from the OBD2 port does not exceed X amps and blow the car's OBD2 fuse or
	the main fuse (F1 2.6A) on the Macchina M2.

* Instance M2_12VIO::
* Functions:
*	Init_12VIO()				// Initialise the 12VIO & the M2 I/O power supply current monitoring
*	Setpin_12VIO(3, ON)			// turn output pin 3 ON
*	Setpin_12VIO(3, ON, SOURCE)	// sets output 3 to Source 12 volts & turns output pin ON	"*** Macchina BETA Hardware ONLY ***"
								// after setting the output pin to source or sink then use Setpin_12VIO(3, ON or OFF) to switch the pin ON or OFF
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


   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# Important Note:
	**** WARNING Not suitable for use with Macchina M2 Hardware Beta version ****
	If using the M2 BETA hardware with this software the six source pins will only be used.
	if you enable the sink pins by uncommenting the #define M2_Beta on line 42 in the file variant.h.
	Be aware that you could inadvertantely cause a short across the output +12vio supply to O volt line
	by having the source & sink pins for the same output turned on at the same time.

	"It is possible to create a direct short on the I/O Mosfets & damage your Macchina BETA M2 I/O Board
	by not controlling the 6 general purpose 12V I/O pins correctly."

   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	This library is especially important since it abstracts the lower-level drivers and ensures that the overload
	circuit is controlled correctly


  Arduino IDE 1.8


