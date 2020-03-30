#include <SPI.h>
#include "AD9910.h"


//Define pin mappings:
#define CSPIN 14                        // DDS chip select pin. Digital input (active low). Bringing this pin low enables detection of serial clock edges.
#define OSKPIN 15                       // DDS Output Shift Keying. Digital input.
#define PS0PIN 5                        // DDS PROFILE[0] pin. Profile Select Pins. Digital input. Use these pins to select one of eight profiles for the DDS.
#define PS1PIN 6                        // DDS PROFILE[1] pin. Profile Select Pins. Digital input. Use these pins to select one of eight profiles for the DDS.
#define PS2PIN 7                        // DDS PROFILE[2] pin. Profile Select Pins. Digital input. Use these pins to select one of eight profiles for the DDS.
#define IO_UPDATEPIN  9                 // DDS I/O_UPDATE pin. Digital input. A high on this pin transfers the contents of the buffers to the internal registers.
#define RESETPIN 10                     // DDS MASTER_RESET pin. Digital input. Clears all memory elements and sets registers to default values.
#define TRIGGERIN 12                    // Pin to trigger the Arduino
#define TRIGGEROUT 13                   // Pin to trigger events with the Arduino or to trigger the a

int divider=25;                         // System clock is ref clk * divider
int ref_clk=40000000;                   //Reference clock is 40 MHz

//Declare the DDS object:
AD9910 DDS(CSPIN, RESETPIN, IO_UPDATEPIN, PS0PIN, PS1PIN, PS2PIN, OSKPIN);


void setup() {
  delay(100);
  SPI.begin();
  SPI.setClockDivider(4);
  SPI.setDataMode(SPI_MODE0,CSPIN);
  SPI.setBitOrder(MSBFIRST);
  
  Serial.begin(115200);
  
  delay(10);
  
  DDS.initialize(ref_clk,divider);
  DDS.setFreqAmp(10000000,1.0,0);
  DDS.setFreqAmp(1000000,0.9,1);
  DDS.setFreqAmp(10000000,0.8,2);
  DDS.setFreqAmp(1000000,0.7,3);
  DDS.setFreqAmp(10000000,0.6,4);
  DDS.setFreqAmp(1000000,0.5,5);
  DDS.setFreqAmp(10000000,0.4,6);
  DDS.setFreqAmp(1000000,0,7);
  DDS.setProfile(0);

  delay (10);
  pinMode(TRIGGERIN, INPUT);
  pinMode(TRIGGEROUT, OUTPUT);
  digitalWrite(TRIGGEROUT, LOW);
}

void loop() {
  digitalWrite(TRIGGEROUT, HIGH);
  // put your main code here, to run repeatedly:
  DDS.setProfileFast(0);
  digitalWrite(TRIGGEROUT, LOW);
  //delay(500);
  DDS.setProfileFast(1);
  DDS.setProfileFast(2);
  DDS.setProfileFast(3);
  DDS.setProfileFast(4);
  DDS.setProfileFast(5);
  DDS.setProfileFast(6);
  DDS.setProfileFast(7);
  
  
  
}
