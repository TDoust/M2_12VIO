# M2_12VIO Macchina M2 I/O Library


*  M2_IO.ino for the Macchina M2

	Author:	Tony Doust
	Date:	8/5/2017
	Version: V0.1.1
		Revision History
			24/10/2017

# Short description:
	Macchina 2.0 has 6 General Purpose 12 volt outputs 3xSOURCE & 3xSINK Outputs & 6 x 12Volt Analogue Input Driver
	circuits.
The example file M2_IO.ino & M2_IO.h demonstrates how to use the functions from the M2_12VIO library 
	When Sourcing power from the M2 or (Sinking power from the M2 via +12VIO on PIN 19 of the 26 pin connector)
	the 12V rail is monitored by hardware current monitor circuit via I_SENSE
	with feed back from the SAM processor chip DAC output I_SENSE_DAC to the hardware monitoring circuit.

	The goal here is to be able to dynamically measure how much power 1 or all currently ON external device are using
	& ensure that the total power used from the OBD2 port does not exceed X amps and blow the car's OBD2 fuse or
	the main fuse (F1 2.6A) on the Macchina M2.

The functions
	Supply_Volts(), Read_12VIO(IO_Pin) return uncalibarated milliVolt values & for most users these values will be close enough.
	The reason for needing to carry out calibrartion is due to variances in resistor values & other component tolerances.
	Should the end user require more accurate values then calibration will be required.
	The calibration is carried out by appling a known value say 12VDC to the input & measure this with a good quality voltmeter
	& comparing this value to the uncalibrated value output from the M2 via the SerialUSB.print() function to the console. Once these 2 values are obtained
	then go to the M2_12VIO::Init_12VIO() function line 301 "Calibrated_Vehicle_Volts_Scaler" & line 306 "Calibrated_Analog_IO_Scaler"
	& change the values Measured Value & Displayed Value. Once these values have been changed then go to the appropriate functions
	M2_12VIO::Supply_Volts() line 450 & change the UnCalibrated_Vehicle_Volts_Scaler to Calibrated_Vehicle_Volts_Scaler for the Vehicle Voltage.
	Alternatively remove the comment // from line 450 & then comment line 449.
	M2_12VIO::Read_12VIO(uint32_t IO_Pin) line 472 & change the UnCalibrated_Analog_IO_Scaler to Calibrated_Analog_IO_Scaler.
	Alternatively remove the comment // from line 473 & then comment line 472.

# External Testing Components Required:
	The Examples sketches were tested by installing LEDS with suitably sized current limiting resistors in series with the LED connected from the 26pin SOURCE
	pin output to 0Volt pin on the 26 pin connector for the 3 SOURCE pins & LEDS with current limiting resistors connected from the 26pin 12Vio supply to the SINK pin.
	For External Button inputs a push button was connected from the 26pin 12Vio supply to a analogue input to simulate an external button.
	For Analogue inputs a varaiable voltage was connected to one of the Analogue inputs simulating an external Analogue input.
	The 2 User Buttons Button1 & Button2 on board the M2 hardware were used for selecting diferent functions. 
	Enjoy the library & i hope it is of use to everyone.
	Tony

* Instance M2_12VIO::
* Functions:
*	Init_12VIO()	Initialise the 12VIO & the M2 I/O power supply current monitoring

*	Setpin_12VIO(3, ON)	turn output pin 3 ON
*	Setpin_12VIO(3, ON, SOURCE)
sets output 3 to SOURCE 12 volts & turns output pin ON.
After setting the output pin the user can use Setpin_12VIO(3, OFF) to turn the output OFF or ON

*	Setpin_12VIO(4, OFF)	turn output pin 4 OFF
*	Setpin_12VIO(4, ON, SINK)
sets output 4 to SINK 12 volts & turns the output pin ON.
After setting the output pin the user can use Setpin_12VIO(4, OFF) to turn the output OFF or ON

*	Setpin_12VIO(1, ON, SOURCE, PWM_PIN, 100, 50)
set pin 1, ON, as 'SOURCE', & PWM_Pin mode, with a Frequency of 100Hz, & a Duty cycle of 50% duty & start the PWM for pin 1.
After setting the pin the user can use Setpin_12VIO(1, OFF) to turn the output OFF or On,
(ON = restarting the PWM if configured as PWM with the previous Frequency & Duty cycle that had been set when first calling the function)

*	Setpin_12VIO(1, OFF, SOURCE, PWM_OFF, 100, 50)
set pin '1', 'OFF', as 'SOURCE', & 'PWM_OFF' mode, with a Frequency of 100Hz, & a Duty cycle of 50% duty & stop the PWM for pin 1.
the settings above PWM_OFF Disables the PWM for this pin so we can revert back to using the pin as a normal digital pin

*  Change_Frequency_12VIO(IO_Pin, Frequency)
change the frequency of an already running PWM pin that has been set via the Setpin_12VIO(1, ON, SOURCE, PWM_PIN, 100, 50) function.
this function will adjust the existing duty to maintain the current duty that had been set via the Setpin_12VIO(1, ON, SOURCE, PWM_PIN, 100, 50) function

*  Change_Duty_12VIO(IO_Pin, Duty)
change the Duty of an already running PWM pin that has been set via the Setpin_12VIO(1, ON, SOURCE, PWM_PIN, 100, 50) function
no changes will be made to the currently running frequency

*	Load_Amps()
returns the total load currently being drawn from the M2 +12io line.
if there has been a overload condition calling Load_Amps() will return the AMPS drawn at the time of the overload condition

*	Supply_Volts()
returns the battery volts of the vehicle the M2 is plugged into

*	Read_12VIO(IO_Pin)
reads the Analogue pin & returns the scaled value as mVolts

*	InitButton_12VIO(IO_Pin)
Initialise the Analog_Pin as a pseduo digital input
*	GetButton_12VIO(IO_Pin)
reads the analogue pin as a button if ON or OFF

*	Temperature()	returns the internal temperature of the SAM DUE processor chip
*

 Private Functions
 *	Enable_12VIO_Monitor(ON)
 Turn ON or OFF (12Vio_EN pin) thus Enabling or Disabling All 12VIO Outputs together.
 (i.e. this can be considered as a Master ON/OFF switch)
  *  Frequency_12VIO(IO_Pin)
 Calculates the frequency from the variable stored for this I/O pin to be used in the PWM functions
 *  Duty_12VIO(IO_Pin)
 Calculates the Duty cycle from the variable stored for this I/O pin to be used in the PWM functions


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


