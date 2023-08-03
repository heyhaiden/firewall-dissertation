#define num_of_read 3 // number of iterations, each is actually two reads of the sensor (both directions)

double SenVWM=0, ARead_A1=0, SoilWaterPotential = 0, Resistance = 0;
const double R = 5000.0; // fixed internal resistor value, need to verify
const double Vcc = 3.3;  // supply voltage

void setup() 
{
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
}

void loop()
{   
    ARead_A1=0;

    //**********READ THE WATERMARK SENSOR**************
    for (int i=0; i<num_of_read; i++) {
        ARead_A1+=analogRead(A1);
        delay(100); //0.1 second wait before moving to next reading
    }

    SenVWM=((ARead_A1/1024.0)*3.3) / num_of_read; //get the average of the readings and convert to volts

    // Calculate Soil Water Potential
    SoilWaterPotential = (SenVWM - 0.6) / -0.04;

    // Calculate Resistance assuming it follows typical voltage to resistance conversion
    Resistance = (Vcc / SenVWM - 1) * R;

    //Print readings
    Serial.print("Moisture Voltage (V) = ");
    Serial.println(SenVWM, 3);
    Serial.print("Soil Water Potential (kPa) = ");
    Serial.println(SoilWaterPotential, 3);
    Serial.print("Resistance (Ohms) = ");
    Serial.println(Resistance, 3);
    Serial.println("");

    delay(10000); // wait 2 seconds before next loop
}
