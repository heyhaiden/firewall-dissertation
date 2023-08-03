#include <math.h>
//********************************************************************************************************************
//***********************************************Version 1.0**********************************************************
//*******Please refer to the Document titled: Watermark sensor reading circuit prior to next step*********************
//************Documentation available at : www.irrometer.com/200ss.html ******************************************
// Version 1.0 created 5/13/2021 by Jeremy Sullivan, Irrometer Co Inc.*****************************************************
// This program will run one loop ONLY and display the values and stop, to refresh the values add delay at the end and/or run in a loop or press "  RESET Button" to refresh.
// Code tested on Arduino UNO R3
// The code is written without the use of any functions to show the sequential flow of code in a simplified format from top to bottom. 
// Purpose of this code is to demonstrate valid WM reading code, circuitry and excitation using a voltage divider and "psuedo-ac" method
// This program uses a modified form of Shocks 1998 calibration equation.

// Sensor to be energized by digital pin 12 or digital pin 8, alternating between HIGH and LOW states

//This is a simplified version of the MEGA_MUX code in the same ZIP. The MEGA_MUX code is an example which uses multiplexers, a temp sensor, and a calibration resistor to read multiple sensors as accurately as possible.
//As a simplified example, this version reads one sensor only and assumes a default temperature of 24C.

//NOTE: the 0.09 excitation time may not be sufficient depending on circuit design, cable lengths, voltage, etc. Increase if necessary to get accurate readings, do not exceed 0.2
//NOTE: this code assumes a 10 bit ADC. If using 12 bit, replaced the 1024 in the voltage conversions to 4096

#define num_of_read 3 // number of iterations, each is actually two reads of the sensor (both directions)
const int Rx = 10000;  //fixed resistor attached in series to the sensor and ground...the same value repeated for all WM and Temp Sensor. 
const long open_resistance=35000, short_resistance=200, short_CB=240, open_CB=255, TempC=24; 
int i, j=0, WM1_CB=0, SupplyV=5.0; // Assuming 5V output, this can be measured and replaced with an exact value if required

double SenVWM1=0, SenVWM2=0, ARead_A1=0, ARead_A2=0 ;

void setup() 
{
	// initialize serial communications at 9600 bps:
	Serial.begin(9600); 
  
	// initialize the pins, 8 and 12 randomly chosen. In the voltage divider circuit example in figure 1(www.irrometer.com/200ss.html), pin 12 is the "Output Pin" and pin 8 is the "GND".
	// if the direction is reversed, the WM1_Resistance A and B formulas would have to be swapped.
	
	pinMode(8, OUTPUT);
	pinMode(12, OUTPUT);
	//set both low
	digitalWrite(8, LOW);
	digitalWrite(12, LOW);

	delay(100);   // time in milliseconds, wait 0.1 minute to make sure the OUTPUT is assigned
}

void loop()
{   
    while(j==0)
    {
    
    ARead_A1=0;
	ARead_A2=0;
        
//**********READ THE WM1 SENSOR**************
    // first take reading through path A
 
	for (i=0; i<num_of_read; i++)   //the num_of_read initialized above, controls the number of read successive read loops that is averaged. 
	{

		digitalWrite(12, HIGH);   //Set pin 12 as Vs
		delay(0.09); //wait 90 micro seconds and take sensor read
		ARead_A1+=analogRead(A1);   // read the analog pin
		digitalWrite(12, LOW);      //set the excitation voltage to OFF/LOW
		
		delay(100); //0.1 second wait before moving to next channel or switching MUX
		
		// Now lets swap polarity and take path B

		digitalWrite(8, HIGH); //Set pin 8 as Vs
		delay(0.09); //wait 90 micro seconds and take sensor read
		ARead_A2+=analogRead(A1);   // read the analog pin
		digitalWrite(8, LOW);      //set the excitation voltage to OFF/LOW
	
	}
	
	SenVWM1=((ARead_A1/1024)*SupplyV) / (num_of_read); //get the average of the readings and convert to volts
	SenVWM2=((ARead_A2/1024)*SupplyV) / (num_of_read); //get the average of the readings and convert to volts

	double WM1_ResistanceA = (Rx*(SupplyV-SenVWM1)/SenVWM1); //do the voltage divider math, using the Rx variable representing the known resistor
	double WM1_ResistanceB = Rx*SenVWM2/(SupplyV-SenVWM2);
	double WM1_Resistance = (WM1_ResistanceA+WM1_ResistanceB) / 2;
	
    delay(100); //0.1 second wait before moving to next channel or switching MUX

//*****************CONVERSION OF RESISTANCE TO kPa************************************

    //convert WM1 Reading to Centibars or KiloPascal
	if (WM1_Resistance>550.00) {
		
		if (WM1_Resistance>8000.00) {
			
			WM1_CB=-2.246-5.239*(WM1_Resistance/1000.00)*(1+.018*(TempC-24.00))-.06756*(WM1_Resistance/1000.00)*(WM1_Resistance/1000.00)*((1.00+0.018*(TempC-24.00))*(1.00+0.018*(TempC-24.00))); 
			
		} else if (WM1_Resistance>1000.00) {

			WM1_CB=(-3.213*(WM1_Resistance/1000.00)-4.093)/(1-0.009733*(WM1_Resistance/1000.00)-0.01205*(TempC)) ;

		} else {

			WM1_CB=((WM1_Resistance/1000.00)*23.156-12.736)*(1.00+0.018*(TempC-24.00));
		}
		
	} else {

		if(WM1_Resistance>300.00)  {
			WM1_CB=0.00;
		}
		
		if(WM1_Resistance<300.00 && WM1_Resistance>=short_resistance) {
			
			WM1_CB=short_CB; //240 is a fault code for sensor terminal short
		}
	}
	
	if(WM1_Resistance>=open_resistance) {
		
		WM1_CB=open_CB; //255 is a fault code for open circuit or sensor not present 
	}

    Serial.print("Temperature(C)= ");
    Serial.print(abs(TempC));
    Serial.print("\n");
    Serial.print("WM1 Resistance(Ohms)= ");
    Serial.print(WM1_Resistance);
    Serial.print("\n");
	
    Serial.print("\n");
    Serial.print("WM1(CB)= ");
    Serial.print(abs(WM1_CB));
    Serial.print("\n");

//****************END CONVERSION BLOCK********************************************************

    delay(200000);

   }
}    
