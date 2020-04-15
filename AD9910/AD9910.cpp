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
   // Use a scaledAmplitude between 0 and 1
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
  _FTW = round(freq * RESOLUTION / _refClk) ;
  _fdw = (_FTW >> _FM_gain)& 0xffff;
  _port_data_word_lower = (_fdw & 0xff) <<1;
  _port_data_word_upper = (_fdw & 0xff00) << 4;
  _port_data_word = _port_data_word_lower | _port_data_word_upper;

  //Set parallel Port C:
  PIOB->PIO_SODR = PIO_SODR_P14;
  REG_PIOC_ODSR = _port_data_word;
  PIOB->PIO_CODR = PIO_CODR_P14;
}

// Transforms the frequency given in Hz to the Port Data Word send to Port C (Parallel Port) 
// As Pins 1-8 and 12-19 of Port c is used, the lower and upper 8 bits have to be shifted accordingly 
uint32_t AD9910::transformToPDW(uint32_t freq) {  
     
  _FTW = round(freq * RESOLUTION / _refClk) ;
  _fdw = (_FTW >> _FM_gain)& 0xffff; 
  _port_data_word_lower = (_fdw & 0xff) <<1; 
  _port_data_word_upper = (_fdw & 0xff00) << 4; 
  _port_data_word = _port_data_word_lower | _port_data_word_upper; 
  return _port_data_word; 
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
   payload.data.block[1] = (_asf[profile] << 16 ) | 0x0000; // Set phase to 0x0000
   
   // actually writes to register
   //AD9910::writeRegister(payload);
   writeRegister(payload);
   update();
}
