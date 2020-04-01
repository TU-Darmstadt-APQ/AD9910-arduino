//Define imported modules:
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

//Definitions for data transmission
const byte bufferSize = 18;             //number of characters that can be saved per transmission cycle
char inputBuffer[bufferSize];           //array to save received message
const char startMarkerSequence = '<';
const char endMarkerSequence = '>';
const char startMarker = '[';
const char endMarker = ']';
int arrWriteIndex = 0;                      //index transmitted with profile data to store in correct position of data_array
int arrReadIndex = 0;
byte bytesReceived = 0;
boolean readInProgress = false;
boolean sequenceReadInProgress = false;
boolean newDataFromPC = false;
char array_size[10];
boolean transitionToBuffered = false;
boolean transitionToManual = false;
boolean shotRunning = false;
boolean programManual = false;

//Definitions for DDS:
int divider=25;                         // System clock is ref clk * divider
int ref_clk=40000000;                   //Reference clock is 40 MHz
int delay_us=1000;
int delay_ms=1000;

//Declare the DDS object:
AD9910 DDS(CSPIN, RESETPIN, IO_UPDATEPIN, PS0PIN, PS1PIN, PS2PIN, OSKPIN);

typedef struct{
  uint32_t freq;
  unsigned long AmpSF;
  uint8_t profile=0;
} profile_t;

// Define an 2D array with frequency, amplitude, profile:
const int lengthArr = 10;
profile_t data_array[lengthArr];

// Define setup-Function:
void setup() {
  //Begin SPI-Communication with AD9910
  delay(100);
  SPI.begin();
  SPI.setClockDivider(4);
  SPI.setDataMode(SPI_MODE0,CSPIN);
  SPI.setBitOrder(MSBFIRST);
  
  delay(10);

  //Initialize DDS:
  DDS.initialize(ref_clk,divider);
  DDS.setFreq(1000000,0);
  DDS.setAmp(0.0,0);
  DDS.setProfile(0);

  //Set Trigger Connections:
  delay (10);
  pinMode(TRIGGERIN, INPUT);
  pinMode(TRIGGEROUT, OUTPUT);
  digitalWrite(TRIGGEROUT, LOW);

  //Initialize communication with PC:
  // wait for USB serial port to be connected - wait for pc program to open the serial port
  //SerialUSB.begin(115200);    // Initialize Native USB port
  //while(!SerialUSB);

  transitionToBuffered = true;
  // normal serial connection
  Serial.begin(115200);
  //tell the PC we are ready
  Serial.println("<Arduino is ready>");

  //Use interrupts (takes 4us to switch profile):
  //attachInterrupt(digitalPinToInterrupt(TRIGGERIN), triggered, RISING);
}

void loop() {
  //SerialUSB.println("hello");
  //delay(1000);
  
  //transition_to_buffered:
  while (transitionToBuffered == true) {
    getDataFromPC();
    //delay(10000);
    replyToPC();
  }
  //delay(10);
  //digitalWrite(TRIGGEROUT, HIGH);
  //digitalWrite(TRIGGEROUT, LOW);
  
  while (shotRunning == true) {
    //digitalWrite(TRIGGEROUT, HIGH);
    //delay(1);
    //digitalWrite(TRIGGEROUT, LOW);
    //PIOC->PIO_CODR = PIO_CODR_P3;
    setProfileTriggeredFast();
//    for (int i=0; i<2; i++){
//      DDS.setFreqAmpSF(data_array[i].freq, data_array[i].AmpSF, data_array[i].profile);
//      delay(1000);
//    }  
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

void getDataFromPC() {

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
          parseData();
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

void parseData() {

    // split the data into its parts
    
  char * strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(inputBuffer,","); // get the first part - the string
  data_array[arrReadIndex].freq = atol(strtokIndx);     // convert this part to an integer
  
  strtokIndx = strtok(NULL, ","); 
  data_array[arrReadIndex].AmpSF = atol(strtokIndx);     // convert this part to a float

}

void replyToPC() {
  
  if (newDataFromPC) {
    newDataFromPC = false;
    Serial.print("<");
    Serial.print("Profiles recieved: ");
    Serial.print(arrReadIndex);
    Serial.print("; ");
    Serial.print(data_array[arrReadIndex-1].freq);
    Serial.print("; ");
    Serial.print(data_array[arrReadIndex-1].AmpSF);
    Serial.print("; ");
    Serial.print(data_array[arrReadIndex-1].profile);
    Serial.println(">");
    transitionToBuffered  = false;
    shotRunning = true;
  }
//  delay(10000);
//  
//  delay(1);
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
  //while (shotRunning==true) {
    // wait until pin 28 is high
    while ((PIOD->PIO_PDSR & PIO_PDSR_P3) == 0);
    // PIO_PDSR_P3 is a data status register for Pin3 (bit 3 is high)
    // PIOD->PIO_PDSR is the current value of the data status register of Port D
    // PDSR is high if Input is high
    // If they are both 1 the AND-operator & makes the result 1 and we exit the while loop

    // Toggle pin 13
    PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // low to high by using xor operation between ...?
    // PIO_ODSR_P27 is high at 27th bit.
    // Does new value of PIO_ODSR of Port B (PIOB->PIO_ODSR) is the xor-result of old PIOB->PIO_ODSR and PIO_ODSR_P27

    //Insert here the function to run:
    //Currently slow Profile setting via SPI
    DDS.setFreqAmpSF(data_array[arrWriteIndex].freq, data_array[arrWriteIndex].AmpSF, data_array[arrWriteIndex].profile);
    arrWriteIndex++;
      //Stop execution when we are at the end of the array
      //arrReadIndex has to be reduced by one as during data transmission it is increased +1 after last received dataset.
      if (arrWriteIndex > arrReadIndex-1){
        transitionToManual = true;
        shotRunning = false;
      }
    PIOB->PIO_ODSR ^= PIO_ODSR_P27;  // high to low
    //delay(1000);
    // Add a delay if necessary to wait for the end of the thunder
    // and avoid toggling once more ??
  //}
}
