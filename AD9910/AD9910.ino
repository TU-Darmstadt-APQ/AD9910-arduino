//Define imported modules:
#include <SPI.h>
#include "AD9910.h"


//Define pin mappings:
#define CSPIN 14                        // DDS chip select pin. Digital input (active low). Bringing this pin low enables detection of serial clock edges.
#define OSKPIN 15                       // DDS Output Shift Keying. Digital input.
#define PS0PIN 25                       // DDS PROFILE[0] pin. Profile Select Pins. Digital input. Use these pins to select one of eight profiles for the DDS.
#define PS1PIN 26                       // DDS PROFILE[1] pin. Profile Select Pins. Digital input. Use these pins to select one of eight profiles for the DDS.
#define PS2PIN 27                       // DDS PROFILE[2] pin. Profile Select Pins. Digital input. Use these pins to select one of eight profiles for the DDS.
#define IO_UPDATEPIN  11 //was9         // DDS I/O_UPDATE pin. Digital input. A high on this pin transfers the contents of the buffers to the internal registers.
#define RESETPIN 12  //was10            // DDS MASTER_RESET pin. Digital input. Clears all memory elements and sets registers to default values.
#define TRIGGERIN 28                    // Pin to trigger the Arduino
#define TRIGGEROUT 13                   // Pin to trigger events with the Arduino
#define F0PIN 30                        // Pin to select ParallelPort Mode
#define F1PIN 32                        // Pin to select ParallelPort Mode

/*
//Pins for ramp generation:
#define DROVER 19                       // Input Pin to detect end of ramp
#define DRCTL 18                        // Pin to control Slope polarity
#define DRHOLD 17                       // Pin to hold the ramp generator
#define IO_RESET 16                       // Pin to reset Serial Communication
 */

//Definitions for DDS:
int divider=25;                         // System clock is ref clk * divider
int ref_clk=40000000;                   // Reference clock is 40 MHz
const double RESOLUTION  = 4294967296.0;
int FM_gain = 0xf;
bool oskEnable = false;
bool parallel_programming = false;

//Declare the DDS object:
AD9910 DDS(CSPIN, RESETPIN, IO_UPDATEPIN, PS0PIN, PS1PIN, PS2PIN, OSKPIN, F0PIN, F1PIN);

//Define data type to save frequency, amplitude, profile:
typedef struct{
  uint32_t freq;
  unsigned long AmpSF;
  uint8_t profile=0;
} profile_t;

const int LENGTHARR = 20001;

// Define the data array depending on the Mode used; CHOOSE ONLY ONE!:
uint32_t AD9910_PDW_array[LENGTHARR];
uint32_t frequency;
//profile_t AD9910_data_array[LENGTHARR];



//Definitions for data transmission
const byte BUFFERSIZE = 18;                 //number of characters that can be saved per transmission cycle
const char STARTMARKER = '[';
const char ENDMARKER = ']';
const char STARTMARKERSEQUENCE = '<';
const char ENDMARKERSEQUENCE = '>';
char inputBuffer[BUFFERSIZE];               //array to save received message
int arrWriteIndex = 0;                      //index transmitted with profile data to store in correct position of data_array
int arrReadIndex = 0;                       //index transmitted with profile data to read from correct position of data_array
byte bytesReceived = 0;                     //index counting the received bytes; Reading data from PC stops when bytesReceived == BUFFERSIZE
bool readInProgress = false;                
bool sequenceReadInProgress = false;
bool newDataFromPC = false;
bool dataTransmissionFinished = false;

//Definitions for Manual und Buffered Mode:
const char MANUALMODEMARKER = 'M';
const char BUFFEREDMODEMARKER = 'B';
bool transitionToBuffered = true;
bool transitionToManual = false;
bool manualMode = false;
bool bufferedMode = false;



// Define setup-Function:
void setup() {
  //Begin SPI-Communication with AD9910
  delay(100);
  SPI.begin();
  SPI.setClockDivider(4);
  SPI.setDataMode(SPI_MODE0,CSPIN);
  SPI.setBitOrder(MSBFIRST);
  
  // Begin serial connection with PC:
  Serial.begin(115200);
  // Here transfer of DDS settings can be included...
  //tell the PC we are ready
  Serial.println("<Arduino is ready>");
  
  delay(10);

  //Initialize DDS:
  DDS.initialize(ref_clk,divider, FM_gain, oskEnable, parallel_programming);
  //Set Frequency and Amplitude for ParallelPort Frequency Modification
//  DDS.setFTWRegister(2000000);
//  DDS.setPPFreq(1000000);
//  DDS.setOSKAmp(1.0);
  
  //Set Freq, Amp, Phase for Profile Mode:
  DDS.setFreq(30000000,0);
  DDS.setAmp(1,0);
  DDS.setPhase(0,0);
//  DDS.setFreq(20000000,1);
//  DDS.setAmp(1,1);
//  DDS.setPhase(180,1);
  DDS.setProfile(0);

  //Set Trigger Connections:
//  delay (10);
//  pinMode(TRIGGERIN, INPUT);
//  pinMode(TRIGGEROUT, OUTPUT);
//  digitalWrite(TRIGGEROUT, LOW);
//
//  //Initialize Edge detection of Atmel SAM3X8E at Arduino Pin28( Port D.3):
//  PIOD->PIO_AIMER = PIO_AIMER_P3; //Essentially important to not interrupt on falling edge, I don't know why.
//  PIOD->PIO_ESR = PIO_ESR_P3;
//  PIOD->PIO_REHLSR = PIO_REHLSR_P3; // The interrupt source is a Rising Edge
//  PIOD->PIO_IER = PIO_IER_P3;
}

//Inline function needs to be places here for correct compiling:
inline void setFrequencyTriggeredFast() __attribute__((always_inline));

void setFrequencyTriggeredFast() {
    //Takes approx 400ns:
    if ((PIOD->PIO_ISR & PIO_ISR_P3) == PIO_ISR_P3) {

    // Toggle pin 13
    //PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // low to high by using xor operation between ...?
   
    //Setting Frequency via ParallelPort (as Inline Function):
    DDS.setPPFreqFast(AD9910_PDW_array[arrWriteIndex]);
    
        
    arrWriteIndex++;
      //Stop execution when we are at the end of the array
      //arrReadIndex has to be reduced by one as during data transmission it is increased +1 after last received dataset.
      if (arrWriteIndex > arrReadIndex-1){
        transitionToManual = true;
        //dataTransmissionFinished = false;  //Comment out for test purposes with labscript at home
        arrWriteIndex = 0;
      }
    //PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // high to low
    }
    
}

// Program Loop for Testing:

//void loop() {
//// Toggle pin 13
//  PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // low to high
//  DDS.setProfile(0);
//  PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // high to low
//  delay(1000);
//  PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // low to high
//  DDS.setProfile(1);
//  PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // high to low
//  delay(1000);
//}

// Program Loop for testing SPI connection:

void loop() {
//  DDS.setFreq(10000000,0);
//  DDS.setAmp(1,0);
//  DDS.setPhase(0,0);
//  DDS.setFreq(20000000,1);
//  DDS.setAmp(1,1);
//  DDS.setPhase(180,1);
//  DDS.setProfile(0);
  delay(10);
}
