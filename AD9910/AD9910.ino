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
int FM_gain = 0xf;
bool oskEnable = true;
int delay_us=1000;
int delay_ms=10;


//Declare the DDS object:
AD9910 DDS(CSPIN, RESETPIN, IO_UPDATEPIN, PS0PIN, PS1PIN, PS2PIN, OSKPIN);


// Define an 2D array with frequency, amplitude, profile:
typedef struct{
  uint32_t freq;
  unsigned long AmpSF;
  uint8_t profile=0;
} profile_t;

const int lengthDataArr = 10;
profile_t AD9910_data_array[lengthDataArr];

const int lengthFreqArr = 10;
uint32_t AD9910_freq_array[lengthFreqArr];


//Definitions for data transmission
  const byte bufferSize = 18;                 //number of characters that can be saved per transmission cycle
  char inputBuffer[bufferSize];               //array to save received message
  int arrWriteIndex = 0;                      //index transmitted with profile data to store in correct position of data_array
  int arrReadIndex = 0;                       //index transmitted with profile data to read from correct position of data_array
  byte bytesReceived = 0;                     //index counting the received bytes; Reading data from PC stops when bytesReceived == bufferSize
  bool readInProgress = false;                
  bool sequenceReadInProgress = false;
  bool newDataFromPC = false;
  bool transitionToBuffered = true;
  bool transitionToManual = false;
  bool shotRunning = false;
  bool programManual = false;


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
  DDS.initialize(ref_clk,divider, FM_gain, oskEnable);
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
  //transition_to_buffered:
  while (transitionToBuffered == true) {
    getDataFromPC(AD9910_data_array );
    replyToPC(AD9910_data_array);
  }
  
  while (shotRunning == true) {
    setProfileTriggeredFast();
  }
  
  //transition_to_manual:
  while (transitionToManual == true) {
    sendFinishtoPC();
  }

  while (programManual == true) {
    transitionToBuffered = true;
    programManual == false;
  }
  
}

void getDataFromPC(profile_t data_array[]) {
  const char startMarker = '[';
  const char endMarker = ']';
  const char startMarkerSequence = '<';
  const char endMarkerSequence = '>';
  
  
    // receive data from PC and save it into inputBuffer
    
  if(Serial.available() > 0) {

    char x = Serial.read();
      if (x == endMarker) {
        newDataFromPC = true;
      }

      if (readInProgress == true) {
        // the order of these IF clauses is significant
          
        if (x == endMarkerSequence) {
          sequenceReadInProgress = false;
          inputBuffer[bytesReceived] = 0;
          parseData(data_array,arrReadIndex);
          arrReadIndex +=1;
        }
        
        if(sequenceReadInProgress) {
          inputBuffer[bytesReceived] = x;
          bytesReceived ++;
          if (bytesReceived == bufferSize) {
            bytesReceived = bufferSize - 1;
          }
        }
    
        if (x == startMarkerSequence) { 
          bytesReceived = 0; 
          sequenceReadInProgress = true;
        }
      }

      if (x == startMarker) {
        readInProgress = true;
        arrReadIndex=0;
      }
        
  }
}

void parseData(profile_t data_array[], int index ) {
  // split the data into its parts
    
  char * strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(inputBuffer,","); // get the first part - the string
  data_array[index].freq = atol(strtokIndx);     // convert this part to an integer
  
  strtokIndx = strtok(NULL, ","); 
  data_array[index].AmpSF = atol(strtokIndx);     // convert this part to a float
  
}

void replyToPC(profile_t data_array[]) {
  if (newDataFromPC) {
    newDataFromPC = false;
    Serial.print("<");
    Serial.print("Profiles recieved: ");
    Serial.print(arrReadIndex);
    Serial.println(">");
    transitionToBuffered  = false;
    shotRunning = true;
  }
}

void sendFinishtoPC() {
  Serial.print("<");
  Serial.print("Shot finished");
  Serial.println(">");
  transitionToManual = false;
  programManual = true;
}

void setProfileTriggeredFast() {
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
    PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // low to high by using xor operation between ...?
    // PIO_ODSR_P27 is high at 27th bit.
    // New value of PIO_ODSR of Port B (PIOB->PIO_ODSR) is the xor-result of old PIOB->PIO_ODSR and PIO_ODSR_P27
    
    //Insert here the function to run:
    //Currently slow Profile setting via SPI; Setting the profile takes up to 200us, until the function is processed it takes up to 1,2ms!
    DDS.setFreqAmpSF(AD9910_data_array[arrWriteIndex].freq, AD9910_data_array[arrWriteIndex].AmpSF, AD9910_data_array[arrWriteIndex].profile);
    DDS.setPPFreqFast(0xa07a);
    
    
    arrWriteIndex++;
      //Stop execution when we are at the end of the array
      //arrReadIndex has to be reduced by one as during data transmission it is increased +1 after last received dataset.
      if (arrWriteIndex > arrReadIndex-1){
        transitionToManual = true;
        shotRunning = false;
      }
    PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // high to low
    // Add a delay if necessary to wait for the end of the thunder
    // and avoid toggling once more ??
}

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
    PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // low to high by using xor operation between ...?
    // PIO_ODSR_P27 is high at 27th bit.
    // New value of PIO_ODSR of Port B (PIOB->PIO_ODSR) is the xor-result of old PIOB->PIO_ODSR and PIO_ODSR_P27
    
    //Insert here the function to run:
    //Currently Setting Frequency via ParallelPort:
    DDS.setPPFreqFast(AD9910_freq_array[arrWriteIndex]);
        
    arrWriteIndex++;
      //Stop execution when we are at the end of the array
      //arrReadIndex has to be reduced by one as during data transmission it is increased +1 after last received dataset.
      if (arrWriteIndex > arrReadIndex-1){
        transitionToManual = true;
        shotRunning = false;
      }
    PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // high to low
    // Add a delay if necessary to wait for the end of the thunder
    // and avoid toggling once more ??
}
