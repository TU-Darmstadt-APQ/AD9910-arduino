//Define imported modules:
#include <SPI.h>
#include "AD9910.h"


//Define pin mappings:
#define CSPIN 14                        // DDS chip select pin. Digital input (active low). Bringing this pin low enables detection of serial clock edges.
#define OSKPIN 15                       // DDS Output Shift Keying. Digital input.
#define PS0PIN 5                        // DDS PROFILE[0] pin. Profile Select Pins. Digital input. Use these pins to select one of eight profiles for the DDS.
#define PS1PIN 6                        // DDS PROFILE[1] pin. Profile Select Pins. Digital input. Use these pins to select one of eight profiles for the DDS.
#define PS2PIN 7                        // DDS PROFILE[2] pin. Profile Select Pins. Digital input. Use these pins to select one of eight profiles for the DDS.
#define IO_UPDATEPIN  11 //was9         // DDS I/O_UPDATE pin. Digital input. A high on this pin transfers the contents of the buffers to the internal registers.
#define RESETPIN 12  //was10            // DDS MASTER_RESET pin. Digital input. Clears all memory elements and sets registers to default values.
#define TRIGGERIN 28                    // Pin to trigger the Arduino
#define TRIGGEROUT 13                   // Pin to trigger events with the Arduino

//Definitions for DDS:
int divider=25;                         // System clock is ref clk * divider
int ref_clk=40000000;                   //Reference clock is 40 MHz
double RESOLUTION  = 4294967296.0;
int FM_gain = 0xf;
bool oskEnable = true;
bool parallel_programming = true;
int delay_us=1000;
int delay_ms=10;
const int lengthArr = 20001;

//2D array with frequency, amplitude, profile:
typedef struct{
  uint32_t freq;
  unsigned long AmpSF;
  uint8_t profile=0;
} profile_t;

//Declare the DDS object:
AD9910 DDS(CSPIN, RESETPIN, IO_UPDATEPIN, PS0PIN, PS1PIN, PS2PIN, OSKPIN);

// Define the data array depending on the Mode used; CHOOSE ONLY ONE!:
uint32_t AD9910_PDW_array[lengthArr];
uint32_t frequency;
//profile_t AD9910_data_array[lengthArr];



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
  
  Serial.begin(115200);
  
  delay(10);

  //Initialize DDS:
  DDS.initialize(ref_clk,divider, FM_gain, oskEnable, parallel_programming);
  //DDS.setFreq(1000000,0);
  DDS.setFTWRegister(2000000);
  DDS.setPPFreq(1000000);
  DDS.setOSKAmp(0.8);
  //DDS.setAmp(0.5,0);
  //DDS.setProfile(0);

  //Set Trigger Connections:
  delay (10);
  pinMode(TRIGGERIN, INPUT);
  pinMode(TRIGGEROUT, OUTPUT);
  digitalWrite(TRIGGEROUT, LOW);

  //Initialize communication with PC:
  // wait for USB serial port to be connected - wait for pc program to open the serial port
  //SerialUSB.begin(115200);    // Initialize Native USB port
  //while(!SerialUSB);

  // normal serial connection
  Serial.begin(115200);
  //tell the PC we are ready
  Serial.println("<Arduino is ready>");

  //Use interrupts (takes 4us to switch profile):
  //attachInterrupt(digitalPinToInterrupt(TRIGGERIN), triggered, RISING);
}

void loop() {
  if(Serial.available() > 0) {
    char x = Serial.read();
    if (x==MANUALMODEMARKER) {
      manualMode=true;
      bufferedMode=false;
    } else if (x==BUFFEREDMODEMARKER) {
      manualMode=false;
      bufferedMode=true;
      transitionToBuffered=true;
    }
  }
//  delayMicroseconds(2);
//  PIOB -> PIO_SODR = PIO_SODR_P27;
//  delayMicroseconds(2);
//  PIOB -> PIO_CODR = PIO_CODR_P27;

  
  while (bufferedMode == true) {
    //transition_to_buffered:
    
    while (transitionToBuffered == true) { //loop not necessary, left for conceptual reasons
      getDataFromPC(AD9910_PDW_array);
      replyToPC();
      
    }
    
    while (dataTransmissionFinished == true) {
      setFrequencyTriggeredFast();
//      delayMicroseconds(3);
//      PIOB -> PIO_SODR = PIO_SODR_P27;
//      delayMicroseconds(3);
//      PIOB -> PIO_CODR = PIO_CODR_P27;
    }
    
    //transition_to_manual:
    while (transitionToManual == true) {
      sendFinishtoPC();
      transitionToManual = false;
      bufferedMode=false;
      manualMode=false;
//      delayMicroseconds(4);
//      PIOB -> PIO_SODR = PIO_SODR_P27;
//      delayMicroseconds(4);
//      PIOB -> PIO_CODR = PIO_CODR_P27;
    }
  }

  while (manualMode == true) {
      getDataFromPC(AD9910_PDW_array);
      replyToPC();
    
    
    if (dataTransmissionFinished == true) {
      DDS.setPPFreqFast(AD9910_PDW_array[0]);
      dataTransmissionFinished = false;
      //bufferedMode=false;
      manualMode=false;
    }
  }
  
}

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

void getDataFromPC(uint32_t data_array[]) {
  
  // receive data from PC and save it into inputBuffer  
  if(Serial.available() > 0) {

    char x = Serial.read();
      if (x == ENDMARKER) {
        newDataFromPC = true;
        readInProgress = false;
      }

      if (readInProgress == true) {
        // the order of these IF clauses is significant
          
        if (x == ENDMARKERSEQUENCE) {
          sequenceReadInProgress = false;
          inputBuffer[bytesReceived] = 0;
          frequency = atol(inputBuffer);     // convert inputBuffer string to an frequency integer 
          data_array[arrReadIndex] = DDS.transformToPDW(frequency); 
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

//void parseData(profile_t data_array[], int index ) {
//  // split the data into its parts
//    
//  char * strtokIndx; // this is used by strtok() as an index
//  
//  strtokIndx = strtok(inputBuffer,","); // get the first part - the string
//  data_array[index].freq = atol(strtokIndx);     // convert this part to an integer
//  
//  strtokIndx = strtok(NULL, ","); 
//  data_array[index].AmpSF = atol(strtokIndx);     // convert this part to a float
//  
//}



void replyToPC() {
  if (newDataFromPC) {
    newDataFromPC = false;
    Serial.print("<");
    Serial.print("Profiles recieved: ");
    Serial.print(arrReadIndex);
    Serial.print(",");
    Serial.print(AD9910_PDW_array[0]);
    Serial.print(",");
    Serial.print(AD9910_PDW_array[1]);
    Serial.print(",");
    Serial.print(frequency);
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

void setFrequencyTriggeredFast() {
  PIOD->PIO_CODR = PIO_CODR_P3;  // Be sure to start with a low level for Trigger pin
  //Assumed working principle: PIOD is a pointer to Port D (all D-Pins).
  //PIO_CODR is a 32bit register which is set 1 for Pin3 meaning it will set Pin3 low as it sets all high-entries to low
  //Due to the dereference operator -> Port D is set as PIO_CODR -> Pin3 is set low. 
    // wait until pin 28 is high:
    while ((PIOD->PIO_PDSR & PIO_PDSR_P3) == 0);
    // PIO_PDSR_P3 is a data status register for Pin3 (bit 3 is high)
    // PIOD->PIO_PDSR is the current value of the data status register of Port D
    // PDSR is high if Input is high
    // If they are both 1 the AND-operator & makes the result 1 and we exit the while loop

    // Toggle pin 13
    //PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // low to high by using xor operation between ...?
    // PIO_ODSR_P27 is high at 27th bit.
    // New value of PIO_ODSR of Port B (PIOB->PIO_ODSR) is the xor-result of old PIOB->PIO_ODSR and PIO_ODSR_P27
    
    //Insert here the function to run:
    //Currently Setting Frequency via ParallelPort:
    DDS.setPPFreqFast(AD9910_PDW_array[arrWriteIndex]);
    //delayMicroseconds(1);
        
    arrWriteIndex++;
    //Stop execution when we are at the end of the array
    //arrReadIndex has to be reduced by one as during data transmission it is increased +1 after last received dataset.
    if (arrWriteIndex > arrReadIndex-1){
      transitionToManual = true;
      dataTransmissionFinished = false;
      arrWriteIndex = 0;
    }
    //PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // high to low
    //Avoid toggling once more if the pulse is to long:
    //((~PIOD->PIO_PDSR) & PIO_PDSR_P3) is only one if P3 goes low.
    while (((~PIOD->PIO_PDSR) & PIO_PDSR_P3) == 0);
    
}
