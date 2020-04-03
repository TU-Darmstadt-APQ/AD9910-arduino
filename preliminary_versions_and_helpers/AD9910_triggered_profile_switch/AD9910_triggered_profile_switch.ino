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
int delay_ms=10;

volatile int state=0;  //Variablen, die von Iterruptroutinen angefasst werden, sollten "volatile" deklariert werden
volatile unsigned long lastTime=0;  //Zeitpunkt der letzten steigenden Flanke, nötig zum Entprellen
volatile bool state_bool = 0;

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
  DDS.setFreqAmp(1000000,0.3,7);
  DDS.setProfile(0);

  delay (10);
  pinMode(TRIGGERIN, INPUT);
  pinMode(TRIGGEROUT, OUTPUT);
  digitalWrite(TRIGGEROUT, LOW);

  attachInterrupt(digitalPinToInterrupt(TRIGGERIN), triggered, RISING);
}

void loop() {
  DDS.setProfileFast(state);
  //digitalWrite(TRIGGEROUT, HIGH);
  //digitalWrite(TRIGGEROUT, LOW);
  //DDS.setFreqAmp(10000000,1.0,0);
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

//void triggered(){                   //state hochzählen, falls mindestens 50ms seit der letzten Flanke vergangen sind
//  unsigned long now = millis();
//  if (now - lastTime > 50){        //entprellen
//    state++;
//    state_bool = !state_bool;
//    if (state > 7){
//      state=0;
//    }
//  }
//  lastTime=now;
//}
void triggered(){                   //state hochzählen
  state++;
  if (state > 7){
    state=0;
  }
}
