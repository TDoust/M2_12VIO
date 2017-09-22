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
                                // after setting the output pin to SOURCE or SINK then use Setpin_12VIO(3, ON or OFF) to switch the pin ON or OFF
*	Setpin_12VIO(4, OFF)		// turn output pin 4 OFF
*	Setpin_12VIO(4, ON, SINK)	// sets output 4 to SINK 12 volts & turns output pin ON		"*** Macchina BETA Hardware ONLY ***"

*	Setpin_12VIO(3, PWM_PIN, 75)// set pin 3 to PWM_PIN with a 75% duty cycle
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
*   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*                                Important Note:
*	**** WARNING Not suitable for use with Macchina M2 Hardware Beta version ****
*	If using the M2 BETA hardware with this software the six SOURCE pins will only be used.
*	if you enable the SINK pins be aware that you could inadvertantely cause a short across the output
*	by having the SOURCE & SINK pins for the same output turned on at the same time.
*
*	"It is possible to create a direct short on the I/O Mosfets & damage your Macchina BETA M2 I/O Board
*	by not controlling the 6 general purpose 12V I/O pins correctly."
*
*   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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
*
*/

#include <stdint.h>
#include "M2_12VIO.h"

#ifdef __cplusplus
extern "C" {
#endif

// ************************************************************************************************************ //
//												Analogue Setup													//
// ************************************************************************************************************ //
uint32_t Analog_Resolution = 0;                 // Analogue Resolution number of bits to use(0 = 0-4095 bits) (1 = 0-1023 bits)
uint16_t Max_Number_Analogue_Bits = 4095;

uint32_t Amps_Scaler = 0;                       // Amps scaler used to calculate M2 Load Current
uint32_t UnCalibrated_Vehicle_Volts_Scaler = 0; // UnCalibrated Vehicle voltage for use in reading Analogue input of Vehicle_Voltage
uint32_t Calibrated_Vehicle_Volts_Scaler = 0;   // Calibrated Vehicle voltage for use in reading Analogue input of Vehicle_Voltage
uint32_t UnCalibrated_Analog_IO_Scaler = 0;     // UnCalibrated Analogue Scaler for use in reading Analogue IO inputs
uint32_t Calibrated_Analog_IO_Scaler = 0;       // Calibrated Analogue Scaler for use in reading Analogue IO inputs


// ************************************************************************************************************ //
//												Array Tables													//
// ************************************************************************************************************ //

uint32_t PortC_Output_Status_Reg = 0;   // holds a Copy of PortC output status register digitalWrite(x, HIGH) for checking if OUTPUTS ar either ON or OFF
uint32_t PortC_Pin_Status_Reg = 0;      // holds a copy of the PortC PinMode i.e. pinMode(x, OUTPUT)


/* M2_Pin[1-6][0]Holds the current settings of the Digital Output Pins[1-6] i.e. OFF, ON, SOURCE, SINK, PWM_PIN, PWM_ALT
*Pin BIT assignments
*MSB---LSB
* 00000000
* | | | | | | | |_1=OFF
* | | | | | | |__1=ON
* | | | | | |___1=SOURCE (Mode)
* | | | | |____1=SINK (Mode)
* | | | |_____1=PWM_PIN (Mode) "** In this mode the output pin will PWM the output as a SOURCE or SINK dependent on bits 2 or 3 **"
* | | |______1=PWM_ALT (Mode) "** In this mode the output will alternate between SOURCE then SINK & viceversa **" MACCHINA_M2_BETA only
* | |_______Unused
* |________Unused

{  [1-6][0] = Pin bit assignment(OFF, ON, etc),\
// [1-6][1] = SOURCE or SINK Pin I/O Table other than Beta Version. Beta Version SOURCE Pin,\
// [1-6][2] = GPIOx Port C bit Mask pin ON,\
// [1-6][3] = SINK Pin I/O Table (Beta Version),\
// [1-6][4] = GPIOx_EN Port C bit Mask pin ON }

*/
uint32_t M2_Pin[6][6] = {
    { 0, GPIO1A, 0B1000/*0x8*/, GPIO1B, 0B100/*0x4*/, 0 },
    { 0, GPIO2A, 0B100000/*0x20*/, GPIO2B, 0B10000/*0x10*/, 0 },
    { 0, GPIO3A, 0B10000000/*0x80*/, GPIO3B, 0B1000000/*0x40*/, 0 },
    { 0, GPIO4A, 0B1000000000/*0x200*/, GPIO4B, 0B100000000/*0x100*/, 0 },
    { 0, GPIO5A, 0B100000000000000000000/*0x100000*/, GPIO5B, 0B1000000000000000000000/*0x200000*/, 0 },
    { 0, GPIO6A, 0B10000000000000000000/*0x80000*/, GPIO6B,  0B10000000000000000000000/*0x400000*/, 0 },
};


uint32_t Analog_Pin[] = { Analog1, Analog2, Analog3, Analog4, Analog5, Analog6 };

// ************************************************************************************************************ //
//											Over_Current Variables												//
// ************************************************************************************************************ //

volatile uint32_t Over_Current_Analogue_Bits = 0;   // The Over current analogue bits read just before ISense shutdown & used when retreiving via Get_Load_Amps()

volatile bool Over_Current_Trip = false;            // flag to hold overcurrent trip False=Healthy True=Over Current Condition
volatile bool Overload_Latch = false;               // flag to enable the returning of the overload AMPS only once
volatile uint32_t M2_Amps;                          // Holds the Total mAmps i.e.(3000 = 3Amps) drawn by the M2 in an overload condition when retrieved using function Get_Load_Amps()

// ************************************************************************************************************ //

int16_t Setpin_Error = 0;

/*
*Analog Bit Look up table
*Analog_Bit_Table[0] holds the Analog bits for the IDLE current of U2 for the M2 Interface Card
*Analog_Bit_Table[1..6] hold the Analog bits for the Max Allowed Current for combined output steps Ouput Pins 1..6
*/
uint16_t Analog_Bit_Table[7];

/******************************************************************************************************************/
/*                                             M2_12VIO Functions                                                 */
/******************************************************************************************************************/

/*
*\brief		This "Interupt Service Routine" is called when an OverCurrent Condition is detected
*
*	Variable uint32_t Over_Current_Analogue_Bits will hold the Maximum mAMPS in BITs from the Analogue read before Disabling all outputs
*	this Analogue bit value is converted to mAmps by calling function Load_Amps() which will return the mAmps Value that caused the overload condition
*	Variable bool Over_Current_Trip will be set (TRUE)
*	Variable bool Overload_Latch will be set (TRUE)
*	DS2 RED LED will be set to flash at .5sec intervals to indicate an Overload condition
*	User buttons 1 or 2 could be used to reset the overload condition
*	Its up to the end user to remove the OverLoad condition before calling Reset_Current_Limit(). After the overload has been removed from the output pin or pins
*
*\param NIL
\return NIL
*/
void ISR_Over_Current(){	// handle pin change interrupt for PIO_PD1 Over Current Input from LM393 Comparator here
    Over_Current_Analogue_Bits = adc_get_channel_value(ADC, adc_channel_num_t (g_APinDescription[Supply_Amps].ulADCChannelNumber));	/* Read the Analogue Bits from the I_Sense Pin for later amps calculation */
    digitalWrite(IO_Enable, LOW);   // There appears to have been an OverCurrent condition therfore we Turn the 12Vio_EN pin OFF Disablng all 12Vio output pins
	Over_Current_Trip = true;       // Set the flag to indicate an OverCurrent condition has occured
	Overload_Latch = true;
}


/**
* \brief
*	Initialise the 12VIO for use
*	Set the Analog Resolution to 12bits to enable resolution (0 to 4095)bits
*	Initialise the Analog_Bit_Table[] Array for use
*	Clear out the M2_Pin[0-5][0] Array for M2 Output Pin Status
*	Set up the M2 Current sensing & Enable
*	Set all GPIOx output pins to OUTPUT & turn the OUTPUTS OFF
*	Set the output pin 12Vio_EN to output & turn ON (Enabling all 12VIO)
*
* \param NIL
* \return uint16_t Setpin_Error ("Success = 0" or "Error = 1 Failure to Initialise" )
*/
uint16_t M2_12VIO::Init_12VIO(){
	Setpin_Error = 0;
	Overload_Latch = false;

    //******************************************************************************************************//
    //										Initialise the GPIO output pins									//
    //******************************************************************************************************//
#ifdef MACCHINA_M2_BETA
    for(uint8_t i = 0; i < 6; i++){
        M2_Pin[i][0] = 1;		// Clear out the I/O Status Array & set to 1 all outputs OFF
        M2_Pin[i][5] = 0;		// Clear out the PwmDuty & set to 0
        pinMode(M2_Pin[i][1], OUTPUT);
        digitalWrite(M2_Pin[i][1], LOW); // Set the SOURCE outputs OFF=LOW
        //Not yet Implemented & or Fully Tested
        /*
        pinMode(M2_Pin[i][3], OUTPUT);
        digitalWrite(M2_Pin[i][3], HIGH); // Set the SINK outputs OFF=HIGH
        */
    }
#else
    for(uint8_t i = 0; i < 3; i++){	// Initialise the ouput pins GPIO to OUTPUT & Turn OFF
        M2_Pin[i][0] = 1;                   // Clear out the I/O Status Array & set to 1 all outputs OFF
        M2_Pin[i][5] = 0;                   // Clear out the PwmDuty & set to 0
        pinMode(M2_Pin[i][1], OUTPUT);
        digitalWrite(M2_Pin[i][1], LOW);    // Set the SOURCE outputs OFF=LOW
        bitSet(M2_Pin[i][0], 2);            // Set the SOURCE Flag

        M2_Pin[i + 3][0] = 1;                 // Clear out the I/O Status Array & set to 1 all outputs OFF
        M2_Pin[i + 3][5] = 0;                   // Clear out the PwmDuty & set to 0
        pinMode(M2_Pin[i + 3][1], OUTPUT);
        digitalWrite(M2_Pin[i + 3][1], HIGH); // Set the SINK outputs OFF=HIGH
        bitSet(M2_Pin[i + 3][0], 3);        // Set the SINK Flag
    }
#endif

    // ************************************************************************************************************ //
    //												Analogue Setup													//
    // ************************************************************************************************************ //

    ADC->ADC_ACR |= 0x10u;	// set the tson bit to Enable the temperature sensor

    ADC->ADC_CHER = 1 << 15u;	// Enable the Analogue Temperature channel AD15

    // Setup the software defaults used in analogRead() to 12 bit resolution
    analogReference(AR_DEFAULT);    // this set the default analogue reference voltage

    Analog_Resolution = ADC->ADC_MR & 0x10; // Check is processor is set for Hardware 10 or 12 bit resolution 1=10bit 0=12bit (Default = 12bit)
    if(Analog_Resolution == 1){
        //ADC->ADC_MR |= 0x10;            // this set hardware 10 bit analogue resolution
        analogWriteResolution(10u);     // set analogue write resolution to 10bit resolution
        analogReadResolution(10u);      // set analogue read resolution to 10 bits resolution
        Max_Number_Analogue_Bits = 1023;	/* **Dont Change** Hardware Analogue Resolution Bit range 10 bits */
    } else{
        analogWriteResolution(12u);		// set analogue write resolution to 12bit resolution
        analogReadResolution(12u);		// set analogue read resolution to 12 bits resolution
        Max_Number_Analogue_Bits = 4095;	/* **Dont Change** Hardware Analogue Resolution Bit range 12 bits */
    }

    Amps_Scaler = round(((((ADCVREF*1000) / Max_Number_Analogue_Bits) * (Design_Volts * 1000)) / (Design_Current * 1000)) * 65536);
    //  3000 / 4095 = 72.71108506308505 * 2000 = 145422.1701261701 / 2262 = 64.28919987894346 * 65536 = 4213257.003266439

    UnCalibrated_Vehicle_Volts_Scaler = round((((Max_Vehicle_Volts - Vehicle_Vref) / Vehicle_Vref) * 65536) / Number_Analog_Samples); // low reading 10.95V
    // (16.63 - 3.0) / 3.0 = 4.543333333333333 * 65536 = 297751.8933333333
    Calibrated_Vehicle_Volts_Scaler = round((12.10 / 13.46) * UnCalibrated_Vehicle_Volts_Scaler);
    // Calibrated_Vehicle_Volts_Scaler = (Measured Value / Displayed Value) * UnCalibrated_Vehicle_Volts_Scaler

    UnCalibrated_Analog_IO_Scaler = round((((Max_Analog_Volts - Analog_Vref) / Analog_Vref) * 65536) / Number_Analog_Samples);
    // (16.63 - 3.0) / 3.0 = 4.543333333333333 * 65536 = 297751.8933333333 / 16 = 18609.49333333333
    Calibrated_Analog_IO_Scaler = round((10.04/10.21) * UnCalibrated_Analog_IO_Scaler);
    // Calibrated_Analog_IO_Scaler = round((Measured Value / Displayed Value) * UnCalibrated_Analog_IO_Scaler);

    //******************************************************************************************************//
	//										Arrays Lookup Table Setup										//
	//******************************************************************************************************//
    pinMode(IO_Enable, OUTPUT);		// Set 12Vio_EN to Output

    digitalWrite(IO_Enable, HIGH);	// Turn the 12Vio_EN pin ON to enable calculating the offset voltage for the current sensing (LOW = OFF HIGH = ON)
    delay(10);
    ADC->ADC_CR = 0x2;      // set ADC_CR bit 1 Start the ADC Conversion.
    delay(10);  // delay to allow the processor analogue circuitry to settle
    M2_Amps = adc_get_channel_value(ADC, adc_channel_num_t(g_APinDescription[Supply_Amps].ulADCChannelNumber)); // Do dummy read to get around the DUE BUG
    M2_Amps = 0;

    //SerialUSB.print("\nOffset =");
    for(uint8_t i = 0; i < 16; ++i){ // Calculate the standing current (idle no load current output of U2) & store in Analog_Bit_Table[0] for use in calculating the 6 output current limits
        ADC->ADC_CR = 0x2;      // set ADC_CR bit 1 Start the ADC Conversion.
        delay(10);  // delay to allow the processor analogue circuitry to settle
        while((ADC->ADC_ISR & (1 << g_APinDescription[Supply_Amps].ulADCChannelNumber)) == 0);  // Wait for end of analogue conversion
        M2_Amps = M2_Amps + adc_get_channel_value(ADC, adc_channel_num_t(g_APinDescription[Supply_Amps].ulADCChannelNumber)); // Read the standing voltage output from U2 with no load
        //SerialUSB.print(" ");
        //SerialUSB.print(M2_Amps);
    }
    digitalWrite(IO_Enable, LOW);       // Turn the 12Vio_EN pin OFF to finish setting up the current limiting (LOW = OFF HIGH = ON)
    Analog_Bit_Table[0] = M2_Amps >> 4; // Right shift 4 place i.e. divide by 16
    M2_Amps = 0;

    for(uint8_t i = 1; i <= 6; i++){ // (((Design_Current * 1000) - (((Design_Current * 1000) / "2730 bits calculated (Analog_Bit_Table[0] + (Analog_Bit_Table[1] * 6)"  maximum bits for the design current) * Analog_Bit_Table[0]bits)) / 6)  each output pin = 411 bits = 266.2236083986747 mA
        Analog_Bit_Table[i] = round(Analog_Bit_Table[0] + (((((Max_Number_Analogue_Bits / (Analog_Vref *1000)) * (Design_Volts * 1000)) - Analog_Bit_Table[0]) /6)* i));	// [1]=675, [2]=1086 etc. Set the Analog Bits Max Amps for each Output Pin multiple
    }
    SerialUSB.print("\nMaximum load per M2 Digital Output = ");
    SerialUSB.print(((Analog_Bit_Table[1] - Analog_Bit_Table[0])* Amps_Scaler) >> 16);
    SerialUSB.print(" mAmps");
    SerialUSB.print("\nMaximum Total load 6 x M2 outputs = ");
    SerialUSB.print(float((((Analog_Bit_Table[1] * 6) - Analog_Bit_Table[0])* Amps_Scaler) >> 16) /1000);
    SerialUSB.print(" Amps\n");

/*
    // *** Debug ***
    SerialUSB.print("\nI_Sense Bit Table");
    for(uint8_t i = 0; i <= 6; i++){
        SerialUSB.print("\nTable ");
        SerialUSB.print(i);
        SerialUSB.print(" = ");
        SerialUSB.print(Analog_Bit_Table[i]);
    }
*/

	//******************************************************************************************************//
	//									Current Sense Monitoring for U2 & U3A								//
	//******************************************************************************************************//

    pinMode(Interupt_Over_Current, INPUT_PULLUP);		/* Set OverCurrent to Input & enable pullup resistors. Used for Interupt Input using Pinchange Interupt */


	digitalWrite(IO_Enable, HIGH);	/* Turn the 12Vio_EN pin ON Enablng all 12Vio output pins (LOW = OFF HIGH = ON) */

	analogWrite(Dac_Compare, Analog_Bit_Table[0]);	/* Set the DAC1 Analog output Default to Analog_Bit_Table[0] standing idle current as a safety measure to generate an interupt if current is being drawn because of a stuck Mosfet */

    // ***** I_Sense Analogue input Pin is used for monitoring the M2 Current being used from Output of U2 function M2_12VIO::Load_Amps() ***** //

	attachInterrupt(digitalPinToInterrupt(Interupt_Over_Current), ISR_Over_Current, FALLING);	/* Over_Current Pin PD1 Pin 76 */

    Reset_Current_Limit();  // Turn on current monitoring

    ADC->ADC_MR |= 0x80;    // set ADC_MR bit 7 FREERUN ADC to free running
    ADC->ADC_CR |= 0x2;      // set ADC_CR bit 1 Start the ADC Conversion.

	return(Setpin_Error);
}


/**
* \brief
*	Reset the Over_Current_Trip & Overload_Latch flags & turn ON the current monitoring system
* \param NIL
* \return uint16_t Setpin_Error ("Success = 0" or "Error = 2" (Mode Out of Range)
*/
uint16_t M2_12VIO::Reset_Current_Limit(){
	M2_Amps = 0;
	Over_Current_Trip = false;
	Overload_Latch = true;
	return(M2_12VIO::Enable_12VIO_Monitor(ON));
}


/*
*\brief
*	Reads the Total load AMPS of the M2 & returns the value as uint32_t milliAmps
*	If there has been an overload condition the Over_Current flag is set & the Load Amps
*	prior to the trip will be returned
*	milliAmps returned i.e.(3000 = 3.0Amps)
*
*\param nil
*\return uint32_t Load_Amps "The current Load Amps are calculated & returned Overload Amps are stored in the variable M2_Amps in milliAmps"
*/
uint32_t M2_12VIO::Load_Amps(){
	M2_Amps = 0;
	if(Over_Current_Trip && Overload_Latch){	// Over_Current_Trip & Overload_Latch TRUE dont read the analogue pin use the value already read from the ISR_Over_Current
		Overload_Latch = false;
        M2_Amps = Over_Current_Analogue_Bits;	// M2_Amps calculated prior to Over_Current shutdown  in mAmps
	} else{	// read from the analogue pin
        M2_Amps = adc_get_channel_value (ADC, adc_channel_num_t (g_APinDescription[Supply_Amps].ulADCChannelNumber)); // Do dummy read to get around the DUE BUG
        M2_Amps = 0;
        for(uint8_t i = 0; i < 16; i++){
            while((ADC->ADC_ISR & (1 << g_APinDescription[Supply_Amps].ulADCChannelNumber)) == 0);  // Wait for end of analogue conversion
            M2_Amps = M2_Amps + adc_get_channel_value(ADC, adc_channel_num_t(g_APinDescription[Supply_Amps].ulADCChannelNumber));
        }
        M2_Amps = M2_Amps >> 4; // Divide the results by 16
    }

    if(M2_Amps <= Analog_Bit_Table[0]){
        M2_Amps = Analog_Bit_Table[0];
    }
    return(((M2_Amps - Analog_Bit_Table[0]) * Amps_Scaler) >> 16); // Return mAmps
}


/*
*\brief
*	Read the Vehicle supply voltage to the M2 & return the scaled vehicle voltage in Volts
*	
*\param NIL
*\return uint32_t Scaled Supply Voltage in milliVolts i.e. 1620
*/
uint32_t M2_12VIO::Supply_Volts(){
	uint32_t Temp = 0;

    Temp = adc_get_channel_value(ADC, adc_channel_num_t (g_APinDescription[Voltage_Sense].ulADCChannelNumber)); // Do dummy read to get around the DUE BUG
    Temp = 0;
    for(uint8_t i = 0; i < Number_Analog_Samples; i++){
        while((ADC->ADC_ISR & (1 << g_APinDescription[Voltage_Sense].ulADCChannelNumber)) == 0);  // Wait for end of analogue conversion
        Temp = Temp + adc_get_channel_value(ADC, adc_channel_num_t(g_APinDescription[Voltage_Sense].ulADCChannelNumber));
    }
    return((Temp * UnCalibrated_Vehicle_Volts_Scaler)>>16);
}


/**
*\brief
*\param Read_12VIO(uint32_t IO_Pin) IO_Pin 1-6
*
*	Read the Value from the Analogue Input & return the scaled value
*
*\return uint32_t Scaled Analogue Value in milliVolts i.e. 1620
*/
uint32_t M2_12VIO::Read_12VIO(uint32_t IO_Pin){
    uint32_t Temp = 0;

    if((IO_Pin > 0) && (IO_Pin <= 6)){
        Temp = adc_get_channel_value(ADC, adc_channel_num_t(g_APinDescription[Analog_Pin[IO_Pin - 1]].ulADCChannelNumber)); // Do dummy read to get around the DUE BUG
        Temp = 0;

        for(uint8_t i = 0; i < Number_Analog_Samples; i++){
            while((ADC->ADC_ISR & (1 << g_APinDescription[Analog_Pin[IO_Pin - 1]].ulADCChannelNumber)) == 0);  // Wait for end of analogue conversion
            Temp = Temp + adc_get_channel_value(ADC, adc_channel_num_t(g_APinDescription[Analog_Pin[IO_Pin - 1]].ulADCChannelNumber));
        }
        return((Temp * Calibrated_Analog_IO_Scaler)>>16);
    }
    return(0);
}


/**
\brief Setpin_12VIO(uint8_t IO_pin, uint8_t Pin_Mode)
\param IO_pin (in the range Output_Pin 1 to 6)
\param Pin_Mode (OFF=0, ON=1)
*
\brief
*	Set the M2 Digital Output to ON or OFF
*	Checks if the IO_Pin being SET is in the correct pin range (1-6)
*	Checks if the Pin_Mode is in the correct pin mode range (0-1)

enum Pin_Mode
0=OFF
1=ON
*
*\return Global Variable uint16_t Setpin_Error ("Success = 0" or "Error = Number")
*/
uint16_t M2_12VIO::Setpin_12VIO(uint32_t IO_Pin, uint8_t Pin_Mode){
    Setpin_Error = 0;
    if((IO_Pin > 0) && (IO_Pin <= 6)){	// Test if IO_Pin in range
        if(Pin_Mode <= 1){	// Test if Mode is in range
            switch(Pin_Mode){
                case OFF:{       // Mode 0 OFF
                        bitClear(M2_Pin[IO_Pin - 1][0], 1);	// Reset the ON Flag
                        bitSet(M2_Pin[IO_Pin - 1][0], 0);	// Set the OFF Flag
                        delay(5);   // delay to prevent false triggering of overload interrupt
                    #ifdef MACCHINA_M2_BETA
                        // TODO finish Beta PWM Alternate
                        if((M2_Pin[IO_Pin - 1][0] & 0x30)){ //PWM_PIN or PWM_ALT pwm mode
                            if((M2_Pin[IO_Pin - 1][0] & 0x20)){ // PWM_PIN mode
                                if((M2_Pin[IO_Pin - 1][0] & 0x04)){ // SOURCE PWM_PIN mode
                                    analogWrite(M2_Pin[IO_Pin - 1][1], OFF); // Turn OFF the SOURCE pin
                                } else{ // SINK PWM_PIN mode
                                    //Not yet Implemented & or Fully Tested
                                    /*
                                    analogWrite(M2_Pin[IO_Pin - 1][3], OFF); // Turn OFF the SINK pin
                                    */
                                }
                            } else{ // PWM_ALT pwm mode
                                //Not yet Implemented & or Fully Tested
                                /*
                                analogWrite(M2_Pin[IO_Pin - 1][1], OFF); // Turn OFF the SOURCE pin
                                analogWrite(M2_Pin[IO_Pin - 1][3], 255); // Turn OFF the SINK pin
                                */
                            }
                        } else{ // Not PWM
                            if(M2_Pin[IO_Pin - 1][0] & 0x20){ // PWM_PIN mode
                                digitalWrite(M2_Pin[IO_Pin - 1][1], LOW);     // Turn the SOURCE OUTPUT Pin OFF
                                //Not yet Implemented & or Fully Tested
                                /*
                                digitalWrite(M2_Pin[IO_Pin - 1][3], HIGH);    // Turn the SINK OUTPUT Pin OFF
                                */
                            } else{
                                if((M2_Pin[IO_Pin - 1][0] & 4)){  // SOURCE mode
                                    digitalWrite(M2_Pin[IO_Pin - 1][1], LOW);     // Turn the SOURCE OUTPUT Pin OFF
                                } else if((M2_Pin[IO_Pin - 1][0] & 8)){  // SINK mode
                                    //Not yet Implemented & or Fully Tested
                                    /*
                                    digitalWrite(M2_Pin[IO_Pin - 1][3], HIGH);    // Turn the SINK OUTPUT Pin OFF
                                    */
                                }
                            }
                        }
                    #else
                        if((M2_Pin[IO_Pin - 1][0] & 0x30)){ // PWM_PIN or PWM_ALT pwm mode
                            if((M2_Pin[IO_Pin - 1][0] & 0x20)){ // PWM_PIN mode
                                if((M2_Pin[IO_Pin - 1][0] & 0x04)){ // SOURCE PWM_PIN mode
                                    analogWrite(M2_Pin[IO_Pin - 1][1], OFF); // Turn OFF the SOURCE pin
                                } else{ // SINK PWM_PIN mode
                                    analogWrite(M2_Pin[IO_Pin - 1][3], OFF); // Turn OFF the SINK pin
                                }
                            } else{ // PWM_ALT pwm mode
                                //Not Implemented reserved for possible future use/hardware revision
                                /*
                                //analogWrite(M2_Pin[IO_Pin - 1][1], OFF); // Turn OFF the SOURCE pin
                                //analogWrite(M2_Pin[IO_Pin - 1][3], 255); // Turn OFF the SINK pin
                                */
                            }
                        } else{  // Not PWM_PIN or PWM_ALT pwm mode
                            if(IO_Pin >= 1 && IO_Pin <= 3){
                                digitalWrite(M2_Pin[IO_Pin - 1][1], LOW);     // Turn the OUTPUT Pin OFF Output pins 1-3
                            } else{
                                digitalWrite(M2_Pin[IO_Pin - 1][1], HIGH);    // Turn the OUTPUT Pin OFF Output pins 4-6
                            }
                        }
                    #endif
                        analogWrite(Dac_Compare, Analog_Bit_Table[IO_Pin]); //adjust the DAC output to reflect the number of outputs turned OFF
                        //SerialUSB.print("\nOutput Pin OFF ");
                    }
                    break;
                case ON:{        // Mode 1 ON
                        analogWrite(Dac_Compare, Analog_Bit_Table[IO_Pin]); //adjust the DAC output to reflect the number of outputs turned ON
                        bitClear(M2_Pin[IO_Pin - 1][0], 0);	// Reset the OFF Flag
                        bitSet(M2_Pin[IO_Pin - 1][0], 1);	// Set the ON Flag
                        delay(5);   // delay to prevent false triggering of overload interrupt
                    #ifdef MACCHINA_M2_BETA
                        if((M2_Pin[IO_Pin - 1][0] & 0x30)){ //PWM_PIN or PWM_ALT pwm mode
                            if((M2_Pin[IO_Pin - 1][0] & 0x20)){ // PWM_PIN mode
                                if((M2_Pin[IO_Pin - 1][0] & 0x04)){ // SOURCE PWM_PIN mode
                                    analogWrite(M2_Pin[IO_Pin - 1][1], map(M2_Pin[IO_Pin - 1][5], 1, 100, 1, 255)); // Turn on the SOURCE pin
                                } else{ // SINK PWM_PIN mode
                                    //Not yet Implemented & or Fully Tested
                                    /*
                                    analogWrite(M2_Pin[IO_Pin - 1][3], map(M2_Pin[IO_Pin - 1][5], 1, 100, 1, 255)); // Turn on the SINK pin
                                    */
                                }
                            } else{ // PWM_ALT pwm mode
                                //Not yet Implemented & or Fully Tested
                                /*
                                analogWrite(M2_Pin[IO_Pin - 1][1], map(M2_Pin[IO_Pin - 1][5], 1, 100, 1, 255)); // Turn on the SOURCE pin
                                analogWrite(M2_Pin[IO_Pin - 1][3], map(M2_Pin[IO_Pin - 1][5], 1, 100, 255, 1)); // Turn on the SINK pin
                                */
                            }
                        }else{  // Not PWM_PIN or PWM_ALT pwm mode
                            if((M2_Pin[IO_Pin - 1][0] & 4)){  // SOURCE mode
                                digitalWrite(M2_Pin[IO_Pin - 1][1], HIGH);     // Turn the SOURCE OUTPUT Pin ON
                            } else if((M2_Pin[IO_Pin - 1][0] & 8)){
                                //Not yet Implemented & or Fully Tested
                                /*
                                digitalWrite(M2_Pin[IO_Pin - 1][3], LOW);    // Turn the SINK OUTPUT Pin ON
                                */
                            }
                        }
                    #else
                        if((M2_Pin[IO_Pin - 1][0] & 0x30)){ // PWM_PIN or PWM_ALT pwm mode
                            if((M2_Pin[IO_Pin - 1][0] & 0x20)){ // PWM_PIN mode
                                if((M2_Pin[IO_Pin - 1][0] & 0x04)){ // SOURCE PWM_PIN mode
                                    analogWrite(M2_Pin[IO_Pin - 1][1], map(M2_Pin[IO_Pin - 1][5], 1, 100, 1, 255)); // Turn on the SOURCE pin
                                } else{ // SINK PWM_PIN mode
                                    analogWrite(M2_Pin[IO_Pin - 1][3], map(M2_Pin[IO_Pin - 1][5], 1, 100, 1, 255)); // Turn on the SINK pin
                                }
                            } else{ // PWM_ALT pwm mode
                                //Not Implemented reserved for possible future use/hardware revision
                                /*
                                //analogWrite(M2_Pin[IO_Pin - 1][1], map(M2_Pin[IO_Pin - 1][5], 1, 100, 1, 255)); // Turn on the SOURCE pin
                                //analogWrite(M2_Pin[IO_Pin - 1][3], map(M2_Pin[IO_Pin - 1][5], 1, 100, 255, 1)); // Turn on the SINK pin
                                */
                            }
                        } else{  // Not PWM_PIN or PWM_ALT pwm mode
                            if(IO_Pin >= 1 && IO_Pin <= 3){
                                digitalWrite(M2_Pin[IO_Pin - 1][1], HIGH);	// Turn the OUTPUT Pin ON Output pins 1-3
                            } else{
                                digitalWrite(M2_Pin[IO_Pin - 1][1], LOW);     // Turn the OUTPUT Pin ON Output pins 4-6
                            }
                        }
                    #endif
                        //SerialUSB.print("\nOutput Pin ON ");
                    }
                    break;
                default:{   // Pin_Mode not in range //
                        Setpin_Error = Err_Mode_Range;
                    }
                    break;
            }
        } else{
            Setpin_Error = Err_Mode_Range;
        }
    } else{ // Pin_Mode not in range //
        Setpin_Error = Err_Pin_Range;
    }
    return(Setpin_Error);
}


/*
\brief Setpin_12VIO(uint8_t IO_pin, uint8_t Pin_Mode, uint8_t Source_Mode)
*
\param uint8_t IO_pin (in the range for Output_Pin 1 to 6)
\param uint8_t Pin_Mode (OFF=0, ON=1)
\param uint8_t Source_Mode ( SOURCE=0, SINK=1)
*
\brief
*	Checks if the IO_Pin being SET is in the correct pin range (1-6)
*	Checks if Pin_Mode, Source_Mode is in the correct range (0-1)
*	Set the M2 Digital Output to ON or OFF
*	Set the M2 Digital Output to SOURCE or SINK etc.+

enum Pin_Mode
0=OFF
1=ON
enum Source_Mode
0=SOURCE
1=SINK
*
*\return Global Variable uint16_t Setpin_Error ("Success = 0" or "Error = Number")
*/
uint16_t M2_12VIO::Setpin_12VIO(uint32_t IO_Pin, uint8_t Pin_Mode, uint8_t Source_Mode){
    Setpin_Error = 0;
    if((IO_Pin > 0) && (IO_Pin <= 6)){	// Test if IO_Pin in range
        if(Pin_Mode <= 1 || Source_Mode <= 1){	// Test if Mode is in range
            switch(Source_Mode){
                case SOURCE:{	// Mode 2 SOURCE
                    #ifdef MACCHINA_M2_BETA	// M2 Beta legacy Hardware SINK Output Pins defined in variant.h
                        if(M2_Pin[IO_Pin - 1][0] & 0x02){	// Output is Turned ON therfore turn OFF before changing mode
                            M2_12VIO::Setpin_12VIO(IO_Pin, Pin_Mode);
                        }
                        bitClear(M2_Pin[IO_Pin - 1][0], 3);	// Reset the SINK Flag
                        bitSet(M2_Pin[IO_Pin - 1][0], 2);	// Set the SOURCE Flag
                        M2_12VIO::Setpin_12VIO(IO_Pin, Pin_Mode);
                    #else
                        bitClear(M2_Pin[IO_Pin - 1][0], 3);	// Reset the SINK Flag
                        bitSet(M2_Pin[IO_Pin - 1][0], 2);	// Set the SOURCE Flag
                                                            // At this stage nothing more to do reserved for later possible use
                    #endif  // M2 Beta endif
                    }
                    break;
                case SINK:{      // Mode 3 SINK
                    #ifdef MACCHINA_M2_BETA	// M2 Beta legacy Hardware SINK Output Pins defined in variant.h
                        if(M2_Pin[IO_Pin - 1][0] & 0x02){	// Output is Turned ON therfore turn OFF before changing mode
                            M2_12VIO::Setpin_12VIO(IO_Pin, Pin_Mode);
                        }
                        bitClear(M2_Pin[IO_Pin - 1][0], 2);	// Reset the SOURCE Flag
                        bitSet(M2_Pin[IO_Pin - 1][0], 3);	// Set the SINK Flag
                        M2_12VIO::Setpin_12VIO(IO_Pin, Pin_Mode);
                    #else
                        bitClear(M2_Pin[IO_Pin - 1][0], 2);	// Reset the SOURCE Flag
                        bitSet(M2_Pin[IO_Pin - 1][0], 3);	// Set the SINK Flag
                                                            // At this stage nothing more to do reserved for later possible use
                    #endif  // M2 Beta endif
                    }
                    break;
                default:{   // SOURCE_Mode not in range //
                        Setpin_Error = Err_Mode_Range;
                    }
                    break;
            }
            if(!(M2_Pin[IO_Pin - 1][0] & 0x30)){  // if Not PWM_PIN or PWM_ALT pwm carry on else return to PWM caller
                switch(Pin_Mode){
                    case OFF:       // Mode 0 OFF
                    case ON:        // Mode 1 ON
                        M2_12VIO::Setpin_12VIO(IO_Pin, Pin_Mode);
                        break;
                    default:{   // SOURCE Mode not in range //
                            Setpin_Error = Err_Mode_Range;
                        }
                        break;
                }
            }
        } else{
            Setpin_Error = Err_Mode_Range;
        }
    } else{ // Pin_Mode not in range //
        Setpin_Error = Err_Pin_Range;
    }
    return(Setpin_Error);
}


/*
\brief Setpin_12VIO(uint8_t IO_pin, uint8_t Pin_Mode, uint8_t Source_Mode, uint8_t Pwm_Mode, uint16_t Duty)
*
\param uint8_t IO_pin (in the range for Output_Pin 1 to 6)
\param uint8_t Pin_Mode (OFF=0, ON=1)
\param uint8_t Source_Mode ( SOURCE=0, SINK=1)
\param uint8_t Pwm_Mode (PWM_PIN=0, PWM_ALT=1)
\param uint16_t Duty (Duty=1-255)
*
\brief
*	Checks if the IO_Pin being SET is in the correct pin range (1-6)
*	Checks if Pin_Mode, Source_Mode, PWM_Mode is in the correct range (0-1)
*	Set the M2 Digital Output to ON or OFF
*	Set the M2 Digital Output to SOURCE or SINK etc.+
*	Set the PWM duty cycle 1-100

enum Pin_Mode
0=OFF
1=ON
enum Source_Mode
0=SOURCE
1=SINK
enum Pwm_Mode
0=PWM_PIN
1=PWM_ALT
*
*\return Global Variable uint16_t Setpin_Error ("Success = 0" or "Error = Number")
*/
uint16_t M2_12VIO::Setpin_12VIO(uint32_t IO_Pin, uint8_t Pin_Mode, uint8_t Source_Mode, uint8_t Pwm_Mode, uint8_t Duty){
    Setpin_Error = 0;
    if((IO_Pin > 0) && (IO_Pin <= 6)){	// Test if IO_Pin in range
        if(Pin_Mode <= 1 || Source_Mode <= 1 || Pwm_Mode <= 1){	// Test if Mode is in range
            if(Duty > 0 && Duty <= 100){    // Test if duty cycle in range
                switch(Pwm_Mode){
                    case PWM_PIN:{   // Mode 4 PWM_PIN
                            bitClear(M2_Pin[IO_Pin - 1][0], 5);	// Reset the PWM_ALT Flag
                            bitSet(M2_Pin[IO_Pin - 1][0], 4);	// Set the PWM_PIN Flag
                            M2_Pin[IO_Pin - 1][5] = Duty;
                        }
                        break;
                    case PWM_ALT:{   // Mode 5 PWM_ALT only for use with Beta hardware
                        #ifdef MACCHINA_M2_BETA	// M2 Beta legacy Hardware SINK Output Pins defined in variant.h
                            bitClear(M2_Pin[IO_Pin - 1][0], 4);	// Reset the PWM_PIN Flag
                            bitSet(M2_Pin[IO_Pin - 1][0], 5);	// Set the PWM_ALT Flag
                            M2_Pin[IO_Pin - 1][5] = Duty;
                        #endif  // M2 Beta endif
                        }
                        break;
                    default:{   // PWM_PIN_Pin not in range //
                            Setpin_Error = Err_Mode_Range;
                        }
                        break;
                }
                switch(Source_Mode){
                    case SOURCE:	// Mode 0 SOURCE
                    case SINK:      // Mode 1 SINK
                        M2_12VIO::Setpin_12VIO(IO_Pin, Pin_Mode, Source_Mode);
                        break;
                    default:{   // SOURCE Mode not in range //
                            Setpin_Error = Err_Mode_Range;
                        }
                        break;
                }
                switch(Pin_Mode){
                    case OFF:       // Mode 0 OFF
                    case ON:        // Mode 1 ON
                        M2_12VIO::Setpin_12VIO(IO_Pin, Pin_Mode);
                        break;
                    default:{   // SOURCE Mode not in range //
                            Setpin_Error = Err_Mode_Range;
                        }
                        break;
                }
            } else{ // Pin_Mode not in range //
                Setpin_Error = Err_Duty_Range;
            }
        } else{
            Setpin_Error = Err_Mode_Range;
        }
    } else{
        Setpin_Error = Err_Pin_Range;
    }
    return(Setpin_Error);
}


/*
\brief InitButton_12VIO(uint32_t IO_Pin)
*
\param (uint32_t Analog_Pin(in the range 1-6))
*
*	Initialise the Analog_Pin as a pseduo digital input
*	Initialise the Debounce registers for the Analogue Input pin in the range 1-6 corresponding to Analogue Input pins 1-6
*
*\return Global Variable uint16_t Setpin_Error ("Success = 0" or "Error = Number")
*/
uint16_t M2_12VIO::InitButton_12VIO(uint32_t IO_Pin){
    Setpin_Error = 0;
    if((IO_Pin >= 1) && (IO_Pin <= 6)){	// Test if Analogue IO_Pin in range
        if(!PMC->PMC_PCSR0 & g_APinDescription[Analog_Pin[IO_Pin - 1]].ulPeripheralId){ // Test if the peripheral clock is enabled
            pmc_enable_periph_clk(g_APinDescription[Analog_Pin[IO_Pin - 1]].ulPeripheralId);  // Enable Peripherial Clock PIOx
        }
        g_APinDescription[Analog_Pin[IO_Pin - 1]].pPort->PIO_IFER = g_APinDescription[Analog_Pin[IO_Pin - 1]].ulPin; // Input Filter Enable Register
        g_APinDescription[Analog_Pin[IO_Pin - 1]].pPort->PIO_DIFSR = g_APinDescription[Analog_Pin[IO_Pin - 1]].ulPin; // Debouncing Input Filter Select Register
        g_APinDescription[Analog_Pin[IO_Pin - 1]].pPort->PIO_SCDR |= 0x02; // Slow Clock Divider Register
    }
    return(Setpin_Error);
}


/*
\brief Getpin_12VIO(uint32_t IO_Pin)
*
\param (uint32_t Analog_Pin(in the range 1-6))
*	call with an Analog_Pin in the range 1-6 corresponding to Analogue Input pins 1-6
*	This function can be used as a pseudo Button digital input external to the M2
*	with power supplied from the 26pin connector +12Vio
*
\return bool TRUE or FALSE (TRUE = input pin ON) (FALSE = input pin OFF)
*/
bool M2_12VIO::GetButton_12VIO(uint32_t IO_Pin){
    bool Return_Event = false;
    if((IO_Pin > 0) && (IO_Pin <= 6)){
        uint32_t Temp = 0;
        uint32_t Threshold = 200;
        while((ADC->ADC_ISR & (1 << g_APinDescription[Analog_Pin[IO_Pin - 1]].ulADCChannelNumber)) == 0);  // Wait for end of analogue conversion
        Temp =  adc_get_channel_value(ADC, adc_channel_num_t(g_APinDescription[Analog_Pin[IO_Pin - 1]].ulADCChannelNumber));

        if(Max_Number_Analogue_Bits == 4095){   // Threshold values are the minimum for a 5 Volt input on the analogue pin
            Threshold = 1000;
        } else{
            Threshold = 200;
        }

        if(Temp >= Threshold){
            Return_Event = true;
        } else{
            Return_Event = false;
        }
    }
    return(Return_Event);
}


/*
*\brief
*	!!!! Note Calling the Temperature function will add typically 50uA @ 3V & this could be an issue in low power Mode !!!!
*	!!!! Note when calling the temperature this could disable pin PB27 affecting signal SWC_M0 CAN mode 0 selection !!!!
*	Get the temperature from the processor chip & return the value in Deg C
*\param NIL
*\retrun uint32_t Temp in Deg C (+-15degC)
*/
float M2_12VIO::Temperature(){
	float Processor_Temp = 0;
	float Temperature_Scaler = ADCVREF/Max_Number_Analogue_Bits;
    float OutputVoltage = 0.8;
    float Sensitivity = 0.00265;
	unsigned int Fixed_Temperature = 27;
    Processor_Temp = adc_get_channel_value(ADC, ADC_TEMPERATURE_SENSOR); // do a dummy read first incase the temperature has not been read in some time to get around the DUE BUG
    Processor_Temp = 0;

    while((ADC->ADC_ISR & 1 << ADC_TEMPERATURE_SENSOR) == 0);	// Wait for end of analogue conversion of temperature sensor
    Processor_Temp = Fixed_Temperature + (((Temperature_Scaler * adc_get_channel_value(ADC, ADC_TEMPERATURE_SENSOR)) - OutputVoltage) / Sensitivity);	// Read the value
	return(Processor_Temp);
}


    // **** Private Functions **** //
/**
*\brief
*	Turn (ON or OFF) the 12Vio_EN pin thus Enabling or Disabling all 12Vio output pins simultanously
*
*\param mode Either (0 or OFF = Disable , 1 or ON = Enable)
*\return uint16_t Setpin_Error ("Success = 0" or "Error = 2" (Mode Out of Range)
*/
uint8_t M2_12VIO::Enable_12VIO_Monitor(uint8_t mode){
	Setpin_Error = 0;
	bool setmode = LOW;
	if(mode > 1){
		Setpin_Error = Err_Mode_Range;
	} else{
		if(mode == 0){
            setmode = LOW; //Turn OFF the I/O enable line
        }else{
            setmode = HIGH; //Turn ON the I/O enable line
        }
		digitalWrite(IO_Enable, setmode);	/* Turn the 12Vio_EN pin (ON or 1) = Enable all Outputs, (OFF or 0) = Disable all 12Vio output pins */
	}
	return(Setpin_Error);
}


#ifdef __cplusplus
}
#endif	// end __cplusplus
