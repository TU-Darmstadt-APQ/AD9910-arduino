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
#define TRIGGERIN 28                    // Pin to trigger the Arduino
#define TRIGGEROUT 13                   // Pin to trigger events with the Arduino

int divider=25;                         // System clock is ref clk * divider
int ref_clk=40000000;                   //Reference clock is 40 MHz
int delay_us=1000;
int delay_ms=1000;


//Declare the DDS object:
AD9910 DDS(CSPIN, RESETPIN, IO_UPDATEPIN, PS0PIN, PS1PIN, PS2PIN, OSKPIN);

typedef struct{
  uint32_t freq;
  double scaledAmp;
  uint8_t profile;
} profile_t;

// Define an 2D array with frequency, amplitude, profile:
profile_t data_array[1000];

void setup() {
  delay(100);
  SPI.begin();
  SPI.setClockDivider(4);
  SPI.setDataMode(SPI_MODE0,CSPIN);
  SPI.setBitOrder(MSBFIRST);
  
  Serial.begin(115200);
  
  delay(10);
  
  DDS.initialize(ref_clk,divider);
  DDS.setFreq(1000000,0);
  DDS.setAmp(0.0,0);
  DDS.setProfile(0);

  delay (10);
  pinMode(TRIGGERIN, INPUT);
  pinMode(TRIGGEROUT, OUTPUT);
  digitalWrite(TRIGGEROUT, LOW);


  

  for (int n=0; n<1000; n++){
    data_array[n].freq = 100000*n;
    data_array[n].scaledAmp = 0.1*n;
    data_array[n].profile = 0;
    }
  
}

void loop() {
  
  for (int i=0; i<10; i++){
    digitalWrite(TRIGGEROUT, HIGH);
    digitalWrite(TRIGGEROUT, LOW);
    DDS.setFreqAmp(data_array[i].freq,data_array[i].scaledAmp,data_array[i].profile);
    delay(delay_ms);
  }
  
  //DDS.setAmp(0.01,0);
  //delay(delay_ms);
  //DDS.setFreqAmp(1000000,1.0,0);
  //DDS.setAmp(1.0,0);
  //delay(delay_ms);
  //
  //
  //delay(delay_ms);
  //digitalWrite(TRIGGEROUT, LOW);
  //DDS.setAmp(0.2,0);
  //delay(delay_ms);
  //digitalWrite(TRIGGEROUT, HIGH);
  //DDS.setFreq(1000000,0);
  //delayMicroseconds(delay_us);
  //digitalWrite(TRIGGEROUT, LOW);
  //DDS.setAmp(1.0,0);
  //delayMicroseconds(delay_us);
}
