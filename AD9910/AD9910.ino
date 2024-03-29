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
bool parallel_programming = true;

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
double amplitude;
//profile_t AD9910_data_array[LENGTHARR];

//sweepiung frequencies
const int freq_len = 1000;
int freqs[freq_len];
int freq_iterator;

uint32_t pdw_20, pdw_10;


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
const char AMPLITUDEMODEMARKER = 'A';
bool transitionToBuffered = true;
bool transitionToManual = false;
bool manualMode = false;
bool bufferedMode = false;
bool amplitudeMode = false;



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
  DDS.setFTWRegister(2000000);
  DDS.setPPFreq(110000000);
  //DDS.setOSKAmp(0.97);
  
  //Set Freq, Amp, Phase for Profile Mode:
//  DDS.setFreq(100000000,0);
  DDS.setAmp(1,0); //This is the Amplitude necessary to optimally drive the AOD? TODO:Check 23.01.24
//  DDS.setPhase(0,0);
//  DDS.setFreq(20000000,1);
//  DDS.setAmp(1,1);
//  DDS.setPhase(180,1);
//  DDS.setProfile(0);

  //Set Trigger Connections:
  delay (10);
  pinMode(TRIGGERIN, INPUT);
  pinMode(TRIGGEROUT, OUTPUT);
  digitalWrite(TRIGGEROUT, LOW);
  digitalWrite(TRIGGEROUT, HIGH);
  digitalWrite(TRIGGEROUT, LOW);
  digitalWrite(TRIGGEROUT, HIGH);
  digitalWrite(TRIGGEROUT, LOW);

  //Initialize Edge detection of Atmel SAM3X8E at Arduino Pin28( Port D.3):
  PIOD->PIO_AIMER = PIO_AIMER_P3; //Essentially important to not interrupt on falling edge, I don't know why.
  PIOD->PIO_ESR = PIO_ESR_P3;
  PIOD->PIO_REHLSR = PIO_REHLSR_P3; // The interrupt source is a Rising Edge
  PIOD->PIO_IER = PIO_IER_P3;

//  //Array for sweeping frequencies
//  
//  int freq_start = 100000000;
//  int freq_end = 300000000;
//  int freq_step = 5000000;
//  
//  for(int i = 0; i < freq_len; i++){
//    freqs[i] = (freq_start + i*freq_step) % freq_end;
//  }
//
//  pdw_20 = DDS.transformToPDW(50000000);
//  pdw_10 = DDS.transformToPDW(210000000);
}

//Inline function needs to be places here for correct compiling:
inline void setFrequencyTriggeredFast() __attribute__((always_inline));

void setFrequencyTriggeredFast() {
    //Takes approx 400ns:
    if ((PIOD->PIO_ISR & PIO_ISR_P3) == PIO_ISR_P3) {

    // Toggle pin 13
    PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // low to high by using xor operation between ...?
   
    //Setting Frequency via ParallelPort (as Inline Function):
    DDS.setPPFreqFast(AD9910_PDW_array[arrWriteIndex]);
    
        
    arrWriteIndex++;
      //Stop execution when we are at the end of the array
      //arrReadIndex has to be reduced by one as during data transmission it is increased +1 after last received dataset.
      if (arrWriteIndex >= arrReadIndex-1){
        transitionToManual = true;
        dataTransmissionFinished = false;  //Comment out for test purposes with labscript at home
        arrWriteIndex = 0;
      }
    PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // high to low
    }
    
}

// Program Loop for Testing:

//void loop() {
//// Toggle pin 13
//  PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // low to high
//  //DDS.setProfile(0);
//  DDS.setPPFreqFast(pdw_20);
//  PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // high to low
//  delay(2000);
////  PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // low to high
////  //DDS.setProfile(1);
////  DDS.setPPFreqFast(pdw_10);
////  PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // high to low
////  delay(2000);
//
////sweeping
////DDS.setPPFreq(freqs[freq_iterator % freq_len]);
////freq_iterator++;
////delay(1000);
//}


// Labscript Program loop:
void loop() {
  //Wait for new instruction starting with correct MARKER:
  if(Serial.available() > 0) {
    char x = Serial.read();
    if (x==MANUALMODEMARKER) {
      manualMode=true;
      bufferedMode=false;
    } else if (x==BUFFEREDMODEMARKER) {
      manualMode=false;
      bufferedMode=true;
      transitionToBuffered=true;
    } else if (x==AMPLITUDEMODEMARKER){
      //this is not really a mode but makes implementation easier
      manualMode=false;
      bufferedMode=false;
      amplitudeMode=true;
    }
  }
  
  // Start buffered Mode:
  while (bufferedMode == true) {
    
    //transition_to_buffered:
    while (transitionToBuffered == true) { //loop not necessary, left for conceptual reasons
      getDataFromPC(AD9910_PDW_array);
      replyToPC();
    }
    
    //Shot ongoing:
    while (dataTransmissionFinished == true) {
      setFrequencyTriggeredFast();
    }
    
    //transition_to_manual:
    while (transitionToManual == true) {
      sendFinishtoPC();
      transitionToManual = false;
      bufferedMode=false;
      manualMode=false;
    }
  }
  
  //Start manual Mode:
  while (manualMode == true) {
    getDataFromPC(AD9910_PDW_array);
    replyToPC();
    
    //Set frequency after all data has been received:
    if (dataTransmissionFinished == true) {
      DDS.setPPFreqFast(AD9910_PDW_array[0]);
      dataTransmissionFinished = false;
      bufferedMode=false; 
      manualMode=false;
    }
  }

  //Start amplitude Mode:
  while(amplitudeMode == true) {
    
    getAmpFromPC(amplitude);
    replyToPC();
    //Set amplitude after all data has been received:
    if (dataTransmissionFinished == true) {
      DDS.setAmp(amplitude);
      //Serial.print(amp);
      dataTransmissionFinished = false;
      amplitudeMode=false;
      }
  }
  
}

//This function is necessary for transmission of Frequency, Amplitude (and Phase):
// THIS VERSION IS CURRENTLY NOT USED
//void getDataFromPC(profile_t data_array[]) {
//  const char STARTMARKER = '[';
//  const char ENDMARKER = ']';
//  const char STARTMARKERSEQUENCE = '<';
//  const char ENDMARKERSEQUENCE = '>';
//  
//  // receive data from PC and save it into inputBuffer  
//  if(Serial.available() > 0) {
//
//    char x = Serial.read();
//      if (x == ENDMARKER) {
//        newDataFromPC = true;
//      }
//
//      if (readInProgress == true) {
//        // the order of these IF clauses is significant
//          
//        if (x == ENDMARKERSEQUENCE) {
//          sequenceReadInProgress = false;
//          inputBuffer[bytesReceived] = 0;
//          parseData(data_array,arrReadIndex);
//          arrReadIndex +=1;
//        }
//        
//        if(sequenceReadInProgress) {
//          inputBuffer[bytesReceived] = x;
//          bytesReceived ++;
//          if (bytesReceived == BUFFERSIZE) {
//            bytesReceived = BUFFERSIZE - 1;
//          }
//        }
//    
//        if (x == STARTMARKERSEQUENCE) { 
//          bytesReceived = 0; 
//          sequenceReadInProgress = true;
//        }
//      }
//
//      if (x == STARTMARKER) {
//        readInProgress = true;
//        arrReadIndex=0;
//      }
//        
//  }
//}

// Get frequency data from PC:
void getDataFromPC(uint32_t data_array[]) {
  
  // receive data from PC and save it into inputBuffer  
  if(Serial.available() > 0) {

    char x = Serial.read();
    // the order of the IF clauses is important!    
      if (x == ENDMARKER) {
        newDataFromPC = true;
        readInProgress = false;
      }

      if (readInProgress == true) {
        
        if (x == ENDMARKERSEQUENCE) {
          sequenceReadInProgress = false;
          inputBuffer[bytesReceived] = 0;
          frequency = atol(inputBuffer);     // convert inputBuffer string to an frequency integer 
          data_array[arrReadIndex] = DDS.transformToPDW(frequency); //Convert frequency integer to ParallelPort Data Word
          arrReadIndex +=1;
        }
        
        if(sequenceReadInProgress) {
          inputBuffer[bytesReceived] = x;
          bytesReceived ++;
          if (bytesReceived == BUFFERSIZE) {
            bytesReceived = BUFFERSIZE - 1;
          }
        }
    
        if (x == STARTMARKERSEQUENCE) { 
          bytesReceived = 0; 
          sequenceReadInProgress = true;
        }
      }

      if (x == STARTMARKER) {
        readInProgress = true;
        arrReadIndex=0;
      }
        
  }
}

// This function is only used for getting the amplitude from the pc
void getAmpFromPC(double& amplitude){
  
   // receive data from PC and save it into inputBuffer  
  if(Serial.available() > 0) {

    char x = Serial.read();
    // the order of the IF clauses is important!    
      if (x == ENDMARKER) {
        newDataFromPC = true;
        readInProgress = false;
      }

      if (readInProgress == true) {
        
        if (x == ENDMARKERSEQUENCE) {
          sequenceReadInProgress = false;
          inputBuffer[bytesReceived] = 0;
          amplitude = atof(inputBuffer);     // convert inputBuffer string to a double
        }
        
        if(sequenceReadInProgress) {
          inputBuffer[bytesReceived] = x;
          bytesReceived ++;
          if (bytesReceived == BUFFERSIZE) {
            bytesReceived = BUFFERSIZE - 1;
          }
        }
    
        if (x == STARTMARKERSEQUENCE) { 
          bytesReceived = 0; 
          sequenceReadInProgress = true;
        }
      }

      if (x == STARTMARKER) {
        readInProgress = true;
        arrReadIndex=0;
      }
        
  }
}

// This function is only necessary when using slowly setting Frequency and Amplitude
void parseData(profile_t data_array[], int index ) {
  // split the data into its parts
    
  char * strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(inputBuffer,","); // get the first part - the string
  data_array[index].freq = atol(strtokIndx);     // convert this part to an integer
  
  strtokIndx = strtok(NULL, ","); 
  data_array[index].AmpSF = atol(strtokIndx);     // convert this part to a float
  
}



void replyToPC() {
  if (newDataFromPC) {
    newDataFromPC = false;
    Serial.print("<");
    Serial.print("Profiles received: ");
    Serial.print(arrReadIndex);
    Serial.println(">");
    transitionToBuffered  = false;
    dataTransmissionFinished = true;
  }
}

void sendFinishtoPC() {
  Serial.print("<");
  Serial.print("Shot finished");
  Serial.println(">");
}

// This function is only necessary when using slowly setting Frequency and Amplitude
//void setProfileTriggeredFast() {
//  PIOD->PIO_CODR = PIO_CODR_P3;  // Be sure to start with a low level for Trigger pin
//  //Assumed working principle: PIOD is a pointer to Port D (all D-Pins).
//  //PIO_CODR is a 32bit register which is set 1 for Pin3 meaning it will set Pin3 low as it sets all high-entries to low
//  //Due to the dereference operator -> Port D is set as PIO_CODR -> Pin3 is set low. 
//    // wait until pin 28 is high:
//    while ((PIOD->PIO_PDSR & PIO_PDSR_P3) == 0);
//    // PIO_PDSR_P3 is a data status register for Pin3 (bit 3 is high)
//    // PIOD->PIO_PDSR is the current value of the data status register of Port D
//    // PDSR is high if Input is high
//    // If they are both 1 the AND-operator & makes the result 1 and we exit the while loop
//
//    // Toggle pin 13
//    PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // low to high by using xor operation between ...?
//    // PIO_ODSR_P27 is high at 27th bit.
//    // New value of PIO_ODSR of Port B (PIOB->PIO_ODSR) is the xor-result of old PIOB->PIO_ODSR and PIO_ODSR_P27
//    
//    //Insert here the function to run:
//    //Currently slow Profile setting via SPI; Setting the profile takes up to 200us, until the function is processed it takes up to 1,2ms!
//    //DDS.setFreqAmpSF(AD9910_data_array[arrWriteIndex].freq, AD9910_data_array[arrWriteIndex].AmpSF, AD9910_data_array[arrWriteIndex].profile);
//
//
//    arrWriteIndex++;
//      //Stop execution when we are at the end of the array
//      //arrReadIndex has to be reduced by one as during data transmission it is increased +1 after last received dataset.
//      if (arrWriteIndex > arrReadIndex-1){
//        transitionToManual = true;
//        dataTransmissionFinished = false;
//      }
//    PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // high to low
//    // Add a delay if necessary to wait for the end of the thunder
//    // and avoid toggling once more ??
//}
