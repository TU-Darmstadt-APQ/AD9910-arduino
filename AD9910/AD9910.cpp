/*
   AD9910.cpp - AD9910 DDS communication library
   Modified from AD9910 by N. Pisenti and D. Barker, JQI, 2020
   Based on AD9914 by Ben Reschovsky, 2016, JQI
   Lars Kohfahl, TU Darmstadt
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Arduino.h"
#include "SPI.h"
#include "AD9910.h"
#include <math.h>

/* CONSTRUCTOR */

// Constructor function; initializes communication pinouts
AD9910::AD9910(int ssPin, int resetPin, int updatePin, int ps0, int ps1, int ps2, int osk, int f0, int f1) // reset = master reset
{
  RESOLUTION  = 4294967296.0;
  _ssPin = ssPin;
  _resetPin = resetPin;
  _updatePin = updatePin;
  _ps0 = ps0;
  _ps1 = ps1;
  _ps2 = ps2;
  _osk = osk;
  _fancy = 1; // flag to keep track of extra functionality
  _f0=f0;
  _f1=f1;
  _txEnable = 53;   //Set TxEnable Pin to digital pin 53; this is fixed.
}

// alternate constructor function only using profile 0; initializes communication pinouts
AD9910::AD9910(int ssPin, int resetPin, int updatePin, int ps0) // reset = master reset
{
  RESOLUTION  = 4294967296.0;
  _ssPin = ssPin;
  _resetPin = resetPin;
  _updatePin = updatePin;
  _ps0 = ps0;
  _fancy = 0; // flag to keep track of extra functionality
}

/* PUBLIC CLASS FUNCTIONS */

// initialize(refClk, divider) - initializes DDS with reference freq, divider
void AD9910::initialize(unsigned long ref, uint8_t divider, uint8_t FM_gain, bool OSKon, bool parallel_programming ){
  //Define some internal functions:
  _parallel_programming = parallel_programming;
  _refClk = ref*divider;
  _FM_gain = FM_gain;
  
  // sets up the pinmodes for output
  pinMode(_ssPin, OUTPUT);
  pinMode(_resetPin, OUTPUT);
  pinMode(_updatePin, OUTPUT);
  pinMode(_ps0, OUTPUT);
  if (_fancy == 1){
    pinMode(_ps1, OUTPUT);
    pinMode(_ps2, OUTPUT);
    pinMode(_osk, OUTPUT);
  }
  
  if (_parallel_programming == true) {
    //Set pinmodes for Parallel Programming Pins (Port C, Pins 33-40 and 44-51 which is )
    int PP_Pins[] = {33,34,35,36,37,38,39,40,44,45,46,47,48,49,50,51};
        
    for(int a = 0; a < (sizeof(PP_Pins)/sizeof(PP_Pins[0])); a++) {
        pinMode(PP_Pins[a], OUTPUT);
        digitalWrite(PP_Pins[a], LOW); 
    }
    pinMode(_f0, OUTPUT);
    pinMode(_f1, OUTPUT);
    pinMode(_txEnable, OUTPUT);
    // Configure Parallel Port Destination to frequency:
    digitalWrite(_f0, LOW);
    digitalWrite(_f1, HIGH);
    // Mask the output for the corresponding pins; BE CAREFUL: Interrupts can destroy this!
    REG_PIOC_OWER = 0x000ff1fe; // Enable output writing for all PP_Pins as HEX: 0x000FF1FE
    REG_PIOC_OWDR = 0xfff00e01; // Disable output writing for all other Pins
  } 
  //else {
  // set pinmodes for profile pins and mask for PortD pins 25-27
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  REG_PIOD_OWER = 0x00000007; //Enable output writing
  REG_PIOD_OWDR = 0xfffffff8; //Disable output writing
  //}
  // defaults for pin logic levels
  digitalWrite(_ssPin, HIGH);
  digitalWrite(_resetPin, LOW);
  digitalWrite(_updatePin, LOW);
  digitalWrite(_ps0, LOW);
  if (_fancy == 1){
    digitalWrite(_ps1, LOW);
    digitalWrite(_ps2, LOW);
    if (OSKon == true) {
      digitalWrite(_osk, HIGH);
    } else {
      digitalWrite(_osk, LOW);
    }
  }

  AD9910::reset();

  delay(1);

  reg_t _cfr1;
  _cfr1.addr = 0x00;
  _cfr1.data.bytes[0] = 0x00;
  if (OSKon == true){
    _cfr1.data.bytes[1] = 0x02;               // Enable Output shift keying in manual mode. Amplitude Scale Factor set via Register 9.
    _cfr1.data.bytes[2] = 0x80;               // Enable external Control.
    //_cfr1.data.bytes[1] = 0x00;
   // _cfr1.data.bytes[2] = 0x00;
  } else {
    _cfr1.data.bytes[1] = 0x00;
    _cfr1.data.bytes[2] = 0x00;
  }
  _cfr1.data.bytes[3] = 0x00;
  
  reg_t _cfr2;
  _cfr2.addr = 0x01;
  if (_parallel_programming == true){
    _cfr2.data.bytes[0] = 0x70 + 0x0f;  //disable Sync timing validation (default); enable Parallel data port; set FM gain to maximum;
    //_cfr2.data.bytes[0] = 0x20;
  } else {
    _cfr2.data.bytes[0] = 0x20 ;  //disable Sync timing validation (default)
  }
  _cfr2.data.bytes[1] = 0x0a;
  _cfr2.data.bytes[2] = 0x00;  // sync_clk pin disabled; not used
  if (OSKon == true) {
    _cfr2.data.bytes[3] = 0x00;  // enable ASF from single tone profiles
  } else {
    _cfr2.data.bytes[3] = 0x01;  // enable ASF from single tone profiles
  }
  
  reg_t _cfr3;
  _cfr3.addr = 0x02;
  _cfr3.data.bytes[0] = divider << 1; // pll divider
  if (divider == 0){
    _cfr3.data.bytes[1] = 0x40;    // bypass pll
    _cfr3.data.bytes[3] = 0x07;
  } else {
    _cfr3.data.bytes[1] = 0x41;    // enable PLL
    _cfr3.data.bytes[3] = 0x05;
  }
  _cfr3.data.bytes[2] = 0x3F;

  reg_t _auxdac;
  _auxdac.addr = 0x03;
  _auxdac.data.bytes[0] = 0xFF;

  writeRegister(_cfr1);
  writeRegister(_cfr2);
  writeRegister(_cfr3);
  writeRegister(_auxdac);
  update();

  delay(1);
  _activeProfile = 0;           // Set default profile to 0

}

// reset() - takes no arguments; resets DDS
void AD9910::reset(){
  digitalWrite(_resetPin, HIGH);
  delay(1);
  digitalWrite(_resetPin, LOW);
}

// update() - sends a logic pulse to IO UPDATE pin on DDS; updates all Registers; 2us faster than using digitalWrite()
void AD9910::update(){
  PIOD->PIO_SODR = PIO_SODR_P7;
  delay(1);
  PIOD->PIO_CODR = PIO_CODR_P7;
}

// setProfile(profile) -- Activates a profile by setting correcponsing profile pins high/low
void AD9910::setProfile(uint8_t profile) {
  _activeProfile = profile;
  digitalWrite(_ps0, bitRead(profile,0));
  digitalWrite(_ps1, bitRead(profile,1));
  digitalWrite(_ps2, bitRead(profile,2));
}

// setProfileFast(profile) -- Activates a profile by setting correcponsing profile pins high/low
//Fast way using Register entries Port D.0 to D.2 (equivalent to Arduino Pins 33-35)
void AD9910::setProfileFast(uint8_t profile) {
  _activeProfile = profile;
  REG_PIOD_ODSR = profile;
}

// setFreq(freq) -- writes freq to DDS board, in FTW0
void AD9910::setFreq(uint32_t freq, uint8_t profile){
  if (profile > 7) {
    return; //invalid profile, return without doing anything
  }
  // set _freq and _ftw variables
  _freq[profile] = freq;
  _ftw[profile] = round(freq * RESOLUTION / _refClk) ;

  AD9910::writeProfile(profile);
}

// Function setFTW -- accepts 32-bit frequency tuning word ftw;
//      updates instance variables for FTW and Frequency, and writes ftw to DDS.
void AD9910::setFTW(unsigned long ftw, byte profile){
    if (profile > 7) {
        return; //invalid profile, return without doing anything
    }

    // set freqency and ftw variables
    _ftw[profile] = ftw;
    _freq[profile] = ftw * _refClk / RESOLUTION;

    AD9910::writeProfile(profile);
}

void AD9910::setAmp(double scaledAmp, byte profile){
   // Use a scaledAmplitude between 0 and 1
   if (profile > 7) {
        return; //invalid profile, return without doing anything
   }

   _scaledAmp[profile] = scaledAmp;
   _asf[profile] = round(scaledAmp*16384.0);  // 14-bit DAC
   _scaledAmpdB[profile] = 20.0*log10(_asf[profile]/16384.0);

   if (_asf[profile] >= 16384) {
      _asf[profile]=16383; //write max value
   } else if (scaledAmp < 0) {
      _asf[profile]=0; //write min value
   }
   AD9910::writeProfile(profile);
}

void AD9910::setAmpSF(unsigned long AmpSF, byte profile){
   // Use the amplitude scale factor between 0 and 16383 (2^14-1)
   if (profile > 7) {
        return; //invalid profile, return without doing anything
   }

   _asf[profile] = AmpSF;  // 14-bit DAC
   _scaledAmpdB[profile] = 20.0*log10(_asf[profile]/16384.0);

   if (_asf[profile] >= 16384) {
      _asf[profile]=16383; //write max value
   } else if (_asf[profile] < 0) {
      _asf[profile]=0; //write min value
   }
   AD9910::writeProfile(profile);
}

void AD9910::setAmpdB(double scaledAmpdB, byte profile){
  // Use the amplitude given in dB; value must be smaller than 0
  if (profile > 7) {
        return; //invalid profile, return without doing anything
   }

   if (scaledAmpdB > 0) {
       return; //only valid for attenuation, so dB should be less than 0, return without doing anything
   }

   _scaledAmpdB[profile] = scaledAmpdB;
   _asf[profile] = round(pow(10,scaledAmpdB/20.0)*16384.0);
   _scaledAmp[profile] = _asf[profile]/16384.0;

   if (_asf[profile] >= 16384) {
      _asf[profile]=16383; //write max value
   }

   AD9910::writeProfile(profile);
}

// setFreqAmp(freq) -- writes freq and amp to DDS board
void AD9910::setFreqAmp(uint32_t freq, double scaledAmp, uint8_t profile){
   if (profile > 7) {
     return; //invalid profile, return without doing anything
   }
   // set _freq and _ftw variables
   _freq[profile] = freq;
   _ftw[profile] = round(freq * RESOLUTION / _refClk) ;

   _scaledAmp[profile] = scaledAmp;
   _asf[profile] = round(scaledAmp*16384.0);  // 14-bit DAC
   _scaledAmpdB[profile] = 20.0*log10(_asf[profile]/16384.0);

   if (_asf[profile] >= 16384) {
      _asf[profile]=16383; //write max value
   } else if (scaledAmp < 0) {
      _asf[profile]=0; //write min value
   }
   AD9910::writeProfile(profile);
}

void AD9910::setFreqAmpSF(uint32_t freq, unsigned long AmpSF, uint8_t profile){
   if (profile > 7) {
     return; //invalid profile, return without doing anything
   }
   // set _freq and _ftw variables
   _freq[profile] = freq;
   _ftw[profile] = round(freq * RESOLUTION / _refClk) ;

   _asf[profile] = AmpSF;  // 14-bit DAC
   _scaledAmpdB[profile] = 20.0*log10(_asf[profile]/16384.0);

   if (_asf[profile] >= 16384) {
      _asf[profile]=16383; //write max value
   } else if (_asf[profile] < 0) {
      _asf[profile]=0; //write min value
   }
   AD9910::writeProfile(profile);
}

//This function was moved to the header for inlining:

//void AD9910::setPPFreqFast(uint32_t port_data_word){
//  // Set Trigger for delay
//  PIOB -> PIO_SODR = PIO_SODR_P27;
//  PIOB -> PIO_CODR = PIO_CODR_P27;
//  //Set parallel Port C:
//  PIOB->PIO_SODR = PIO_SODR_P14;
//  REG_PIOC_ODSR = port_data_word;
//  PIOB->PIO_CODR = PIO_CODR_P14;
//}

void AD9910::setPPFreq(uint32_t freq){
  
  // Calculate frequency tuning word:
  /*
  _FTW = round(freq * RESOLUTION / _refClk) ;
  _fdw = (_FTW >> _FM_gain)& 0xffff;
  _port_data_word_lower = (_fdw & 0xff) <<1;
  _port_data_word_upper = (_fdw & 0xff00) << 4;
  _port_data_word = _port_data_word_lower | _port_data_word_upper;
  */
  _port_data_word = transformToPDW(freq);

  //Set parallel Port C:
  PIOB->PIO_SODR = PIO_SODR_P14;
  REG_PIOC_ODSR = _port_data_word;
  PIOB->PIO_CODR = PIO_CODR_P14;
}

// Transforms the frequency given in Hz to the Port Data Word send to Port C (Parallel Port) 
// The D0-D7 of the AD9910 board are connected to C12-C19 of the Due
// The D8-15 of the AD9910 board are connected to C8-C1 of the Due
// the bits are shifted accordingly
uint32_t AD9910::transformToPDW(uint32_t freq) {  
     
  _FTW = round(freq * RESOLUTION / _refClk) ;
  //Serial.print("FTW: \n");
  //prntBits(_FTW, 32);
  _fdw = (_FTW >> _FM_gain)& 0xffff;
  //Serial.print("_fdw: \n");
  //prntBits(_fdw, 32);
  _port_data_word_upper = (_fdw & 0xff) <<12;
  //Serial.print("upper: \n");
  //prntBits(_port_data_word_upper, 32);
  //this line would be clearer as two lines, see the comment below the line
  _port_data_word_lower = reverse_byte((_fdw & 0xff00) >> 8) << 1;
  //unsigned long reversed_lower = reverse_byte((_fdw & 0xff00) >> 8) << 8;
  //_port_data_word_lower = reversed_lower >> 7;
  //Serial.print("lower: \n");
  //prntBits(_port_data_word_lower, 32);
  _port_data_word = _port_data_word_lower | _port_data_word_upper;
  //Serial.print("pdw: \n");
  //prntBits(_port_data_word, 32);
  return _port_data_word; 
}

//since println doesnt display leading zeros this func is useful
void AD9910::prntBits(unsigned long b, int n_bits)
{
  for(int i = n_bits-1; i >= 0; i--)
    Serial.print(bitRead(b,i));
  Serial.println();  
}

//this method reverses the first 8 bits in the byte. 
//from https://www.avrfreaks.net/forum/optimal-byte-flipping?name=PNphpBB2&file=viewtopic&t=41613
unsigned long AD9910::reverse_byte(unsigned long byte_arg) {
  
  unsigned long _new = 0x00;
  _new = (byte_arg >> 7 & 0x1 ) 
         | (byte_arg >> 5 & 0x2 ) 
         | (byte_arg >> 3 & 0x4 ) 
         | (byte_arg >> 1 & 0x8 )
         | (byte_arg << 1 & 0x10) 
         | (byte_arg << 3 & 0x20) 
         | (byte_arg << 5 & 0x40) 
         | (byte_arg << 7 & 0x80);
  
  return _new;
}
//
//unsigned char AD9910::reverse_byte(unsigned char x)
//{
//    static const unsigned char table[] = {
//        0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
//        0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
//        0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
//        0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
//        0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
//        0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
//        0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
//        0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
//        0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
//        0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
//        0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
//        0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
//        0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
//        0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
//        0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
//        0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
//        0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
//        0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
//        0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
//        0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
//        0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
//        0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
//        0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
//        0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
//        0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
//        0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
//        0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
//        0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
//        0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
//        0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
//        0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
//        0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff,
//    };
//    return table[x];
//}

// Set phase as value between 0 and 360 degrees:
void AD9910::setPhase(double phase, byte profile) {
  if (profile > 7) {
    return; //invalid profile, return without doing anything
  }

  _phase[profile] = phase;
  _pow[profile] = round((fmod(phase,360.0)/360)*65536);

  AD9910::writeProfile(profile);
}

void AD9910::setPOW(unsigned long Pow, byte profile) {
  if (profile > 7) {
    return; //invalid profile, return without doing anything
  }

  _pow[profile] = Pow;
  _phase[profile] = 360*(Pow/65536);

  AD9910::writeProfile(profile);
}


void AD9910::setOSKAmp(double scaledAmp){

  _ASF = round(scaledAmp*16383.0) << 2;
  
  reg_t ASF_reg;
  ASF_reg.addr = 0x09;
  ASF_reg.data.bytes[0] = _ASF & 0xff ;  //disable Sync timing validation (default); enable Parallel data port; set FM gain to maximum;
  ASF_reg.data.bytes[1] = ((_ASF & 0xff00) >> 8);
  ASF_reg.data.bytes[2] = 0x00;  
  ASF_reg.data.bytes[3] = 0x00;  

  writeRegister(ASF_reg);
  update();
}

void AD9910::setFTWRegister(uint32_t freq){

  //_FTW = round(freq * RESOLUTION / _refClk) ;
  _FTW = 0xffffffff;
  
  reg_t FTW_reg;
  FTW_reg.addr = 0x07;
  FTW_reg.data.bytes[0] = _FTW & 0xff ;  //disable Sync timing validation (default); enable Parallel data port; set FM gain to maximum;
  FTW_reg.data.bytes[1] = ((_FTW & 0xff00) >> 8);
  FTW_reg.data.bytes[2] = ((_FTW & 0xff0000) >> 16);  
  FTW_reg.data.bytes[3] = ((_FTW & 0xff000000) >> 24);

  writeRegister(FTW_reg);
  update();
}


/////////////////////////////////////////////////////
///// Get-Functions:

// getFreq() - returns current frequency
unsigned long AD9910::getFreq(byte profile){
    return _freq[profile];
}

// getFTW() -- returns current FTW
unsigned long AD9910::getFTW(byte profile){
    return _ftw[profile];
}

//Gets current amplitude
double AD9910::getAmp(byte profile){
  return _scaledAmp[profile];
}

// Gets current amplitude in dB
double AD9910::getAmpdB(byte profile){
  return _scaledAmpdB[profile];
}

//Gets current amplitude scale factor
unsigned long AD9910::getASF(byte profile){
  return _asf[profile];
}

byte AD9910::getProfile() {
  return _activeProfile;
}
/*

//enable OSK
void AD9910::enableOSK(){
  //write 0x00, byte 8 high
  _OSKon = true;
  byte registerInfo[] = {0x00, 4};
  byte data[] = {0x00, 0x01, 0x01, 0x08};
  AD9910::writeRegister(registerInfo, data);
  AD9910::update();
}

//disable OSK
void AD9910::disableOSK(){
  //write 0x00, byte 8 low
  _OSKon = false;
  byte registerInfo[] = {0x00, 4};
  byte data[] = {0x00, 0x01, 0x00, 0x08};
  AD9910::writeRegister(registerInfo, data);
  AD9910::update();
}

//return bool indicating if OSK mode is activated
bool AD9910::getOSKMode() {
  return _OSKon;
}




*/

// Writes SPI to particular register.
//      registerInfo is a 2-element array which contains [register, number of bytes]
void AD9910::writeRegister(reg_t payload){
  SPI.beginTransaction(SPISettings(CLOCKSPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(_ssPin, LOW);
  SPI.transfer(payload.addr);
  // MSB
  for (int i = payload.bytes; i > 0; i--){
    SPI.transfer(payload.data.bytes[i-1]);
  }
  digitalWrite(_ssPin, HIGH);
  SPI.endTransaction();
}

//void AD9910::getRegister(byte _register) {
//  //SDO must be connected which is not the case on our AD9910 board
//}

/* PRIVATE CLASS FUNCTIONS */
void AD9910::writeProfile(byte profile) {
   reg_t payload;
   payload.bytes = 8;
   payload.addr = 0x0E + profile;
   //Set frequency block:
   payload.data.block[0] = _ftw[profile];
   // Set amplitude/phase block:
   payload.data.block[1] = (_asf[profile] << 16 ) | _pow[profile]; // Set phase to 0x0000
   
   // actually writes to register
   //AD9910::writeRegister(payload);
   writeRegister(payload);
   update();
}
