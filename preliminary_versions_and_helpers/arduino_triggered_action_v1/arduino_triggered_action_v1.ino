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
#define TRIGGERIN 28   //Pin28=D3, 35=C3// Pin to trigger the Arduino
#define TRIGGEROUT 13  //Pin13=B27      // Pin to trigger events with the Arduino

int divider=25;                         // System clock is ref clk * divider
int ref_clk=40000000;                   //Reference clock is 40 MHz
int delay_us=1000;
int delay_ms=10;

bool transitionToManual = false;
bool dataTransmissionFinished = true;
int arrWriteIndex = 0;
int arrReadIndex = 20;

const int lengthArr = 20001;

//2D array with frequency, amplitude, profile:
typedef struct{
  uint32_t freq;
  unsigned long AmpSF;
  uint8_t profile=0;
} profile_t;

uint32_t AD9910_PDW_array[lengthArr];



void setup() {
  delay(100);
  SPI.begin();
  SPI.setClockDivider(4);
  SPI.setDataMode(SPI_MODE0,CSPIN);
  SPI.setBitOrder(MSBFIRST);
  
  Serial.begin(115200);

  // initializing array elements 
  for (int i = 0; i < lengthArr ; i++) { 
      AD9910_PDW_array[i] = 0x000FF1FE; 
  } 
  
  delay (10);
  pinMode(TRIGGERIN, INPUT);
  pinMode(TRIGGEROUT, OUTPUT);
  digitalWrite(TRIGGEROUT, LOW);

  int PP_Pins[] = {33,34,35,36,37,38,39,40,44,45,46,47,48,49,50,51};
        
  for(int a = 0; a < (sizeof(PP_Pins)/sizeof(PP_Pins[0])); a++) {
      pinMode(PP_Pins[a], OUTPUT);
      digitalWrite(PP_Pins[a], LOW); 
  }
  
  // Mask the output for the corresponding pins; BE CAREFUL: Interrupts can destroy this!
  //PIOC->PIO_OWER = 0x000ff1fe; // Enable output writing for all PP_Pins as HEX: 0x000FF1FE
  //PIOC->PIO_OWDR = 0xfff00e01; // Disable output writing for all other Pins

  pinMode(41, OUTPUT);
  digitalWrite(41,LOW);

  //Use Edge detection of Atmel SAM3X8E at Arduino Pin28( Port D.3):
  PIOD->PIO_AIMER = PIO_AIMER_P3; //Essentially important to not interrupt on falling edge, I don't know why.
  PIOD->PIO_ESR = PIO_ESR_P3;
  PIOD->PIO_REHLSR = PIO_REHLSR_P3; // The interrupt source is a Rising Edge
  PIOD->PIO_IER = PIO_IER_P3;
}

void loop() {
  PIOD->PIO_CODR = PIO_CODR_P3;  // Be sure to start with a low level for output pin
  //Assumed working principle: PIOD is a pointer to Port D (all D-Pins).
  //PIO_CODR is a 32bit register which is set 1 for Pin3 meaning it will set Pin3 low as it sets all high-entries to low
  //Due to the dereference operator -> Port D is set as PIO_CODR -> Pin3 is set low. 
  while (true) {
    //wait until there is a rising edge on Pin 3:
    if ((PIOD->PIO_ISR & PIO_ISR_P3) == PIO_ISR_P3) {

    // Toggle pin 13
    //PIOB->PIO_SODR = PIO_SODR_P27;  // low to high by using xor operation between ...?
    //PIOB->PIO_CODR = PIO_CODR_P27;
    //Toggle Parallel Port:
    //PIOC->PIO_SODR = 0x000ff1fe;
    //PIOC->PIO_CODR = 0x000ff1fe;
    //Set frequency via ParallelPort:
    setPPFreqFast(AD9910_PDW_array[arrWriteIndex]);
    //Set ParallelPort low:
    PIOC->PIO_CODR = 0x000ff1fe;
    
    arrWriteIndex++;
      //Stop execution when we are at the end of the array
      //arrReadIndex has to be reduced by one as during data transmission it is increased +1 after last received dataset.
      if (arrWriteIndex > arrReadIndex-1){
        transitionToManual = true;
        dataTransmissionFinished = false;
        arrWriteIndex = 0;
      }
    //PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // high to low
    
    }
  }
  
}

//inline void setPPFreqFast(uint32_t port_data_word) __attribute__((always_inline));

void setPPFreqFast(uint32_t port_data_word){
  // Set Trigger for delay
  //PIOB -> PIO_SODR = PIO_SODR_P27;
  //PIOB -> PIO_CODR = PIO_CODR_P27;
  //Set parallel Port C:
  PIOC->PIO_SODR = PIO_SODR_P9;
  PIOC->PIO_ODSR = port_data_word;
  //REG_PIOC_ODSR = port_data_word;
  PIOC->PIO_CODR = PIO_CODR_P9;
}
