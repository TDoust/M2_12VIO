# M2_12VIO Macchina M2 I/O Library

# !!! WARNING to M2 Beta Hardware users !!!
	The only difference between the M2Beta version & the Production version hardware that this library interacts with are the 12VIO SINK pins.
	It is recommended that the end user selects Macchina M2 in the boards manager & not Macchina M2 (Beta).
	By selecting Macchina M2 in the Board Manager this will avoid possible damage to the hardware. 


*  M2_12VIO.h Library for the Macchina M2

	Author:	Tony Doust
	Date:	8/5/2017
	Version: V0.1.2
		Revision History
			24/10/2017
			25/11/2017 Added optional support to work in unison with M2RET ADC DMA & M2I/O

# Short description:
	Macchina 2.0 has 6 General Purpose 12 volt outputs 3xSOURCE & 3xSINK Outputs & 6 x 12Volt Analogue Input Driver
	circuits.
	The 12VIO.h Library is for control & Monitoring of the Macchina M2 general purpose 12V IO pins available on the
	26 pin connector.

	When Sourcing power from the M2 or (Sinking power from the M2 via +12VIO on PIN 19 of the 26 pin connector)
	the 12V rail is monitored by the hardware current monitor circuit via I_SENSE with feed back from the
	SAM processor chip DAC output I_SENSE_DAC to the hardware monitoring circuit.

	The goal here is to be able to dynamically measure how much power 1 or all currently ON external devices are using
	& ensure that the total power used from the OBD2 port does not exceed X amps and blow the car's OBD2 fuse or
	the main fuse (F1 2.6A) on the Macchina M2.

* Instance M2_12VIO::
* Functions:
*	Init_12VIO()	Initialise the 12VIO & the M2 I/O power supply current monitoring

*	Setpin_12VIO(3, ON)	turn output pin 3 ON. This function will turn ON the Digital configured pins & also turn ON the existing
configured PWM pin if it has been enable for this pin.

*	Setpin_12VIO(3, ON, SOURCE) sets output 3 to SOURCE 12 volts & turns the output pin ON.
After setting the output pin the user can use Setpin_12VIO(3, OFF) to turn the output OFF or ON

*	Setpin_12VIO(4, OFF)	turn output pin 4 OFF. This function will also turn off the PWM if it has been enable for this pin

*	Setpin_12VIO(4, ON, SINK)	sets output 4 to SINK 12 volts & turns the output pin ON.
After setting the output pin the user can use Setpin_12VIO(4, OFF) to turn the output OFF or ON

*	Setpin_12VIO(1, ON, SOURCE, PWM_PIN, 100, 50)
set pin 1, ON, as 'SOURCE', & PWM_Pin mode, with a Frequency of 100Hz, & a Duty cycle of 50% duty & start the PWM for pin 1.
After setting the pin the user can use Setpin_12VIO(1, OFF) to turn the output OFF or On,
(ON = restarting the PWM if configured as PWM with the previous Frequency & Duty cycle that had been set when first calling the function)

*	Setpin_12VIO(1, OFF, SOURCE, PWM_OFF, 100, 50)
set pin '1', 'OFF', as 'SOURCE', & 'PWM_OFF' mode, with a Frequency of 100Hz, & a Duty cycle of 50% duty & stop the PWM for pin 1.
The setting 'PWM_OFF' Disables the PWM for this pin so we can revert back to using the pin as a normal digital pin

*	Change_Frequency_12VIO(IO_Pin, Frequency)
change the frequency of an already running PWM pin that has been set via the Setpin_12VIO(1, ON, SOURCE, PWM_PIN, 100, 50) function.
This function will automatically adjust the existing duty to maintain the current duty that had been set via the function Setpin_12VIO(1, ON, SOURCE, PWM_PIN, 100, 50)

*	Change_Duty_12VIO(IO_Pin, Duty)
change the Duty of an already running PWM pin that has been set via the Setpin_12VIO(1, ON, SOURCE, PWM_PIN, 100, 50) function
no changes will be made to the currently running frequency

*	Load_Amps() returns the total load currently being drawn from the M2 +12io line.
If there has been a overload condition calling Load_Amps() will return the AMPS drawn at the time of the overload condition

*	Supply_Volts()
returns the battery volts of the vehicle the M2 is plugged into

*	Read_12VIO(IO_Pin) reads the Analogue pin & returns the scaled value as mVolts


*	GetButton_12VIO(IO_Pin) reads the analogue pin as a button if ON or OFF

*	Temperature() returns the internal temperature of the SAM DUE processor chip
*

 Private Functions
*	Enable_12VIO_Monitor(ON)
 turn ON or OFF (12Vio_EN pin) thus Enabling or Disabling All 12VIO Outputs together.
 (i.e. this can be considered as a Master ON/OFF switch)

*	Frequency_12VIO(IO_Pin)
 calculates the frequency from the variable stored for this I/O pin to be used in the PWM functions

*	Duty_12VIO(IO_Pin)
 calculates the Duty cycle from the variable stored for this I/O pin to be used in the PWM functions


   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# Important Note:
	**** WARNING Not suitable for use with Macchina M2 Hardware Beta version ****
	If using the M2 BETA hardware & selecting Macchina M2 in the Board manager then this software will use the six source pins only.
	If you select Macchina M2 (Beta) in the board manager this will enable the SINK pins in variant.h.
	Be aware that you could inadvertantely cause a short across the output +12vio supply to O volt line
	by having the SOURCE & SINK pins for the same output turned on at the same time.
	As an added precaution a number of lines have been commented out that could cause this issue.
	If the end user needs to use the SINK pins then remove the // from the commented functions but be careful & test carefully.
	!!! The Author accepts no responsibility for an damage that could be caused to the Hardware !!! 

	"It is possible to create a direct short on the I/O Mosfets & damage your Macchina BETA M2 I/O Board
	by not controlling the 6 general purpose 12V I/O pins correctly."

   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	This library is especially important since it abstracts the lower-level drivers and ensures that the overload
	circuit is controlled correctly


  Arduino IDE 1.8


