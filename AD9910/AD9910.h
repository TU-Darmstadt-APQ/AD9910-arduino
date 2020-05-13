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
   aunsigned long with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef AD9910_h
#define AD9910_h

#include "Arduino.h"

#define CLOCKSPEED 1000000

class AD9910
{
  typedef union {
    uint8_t bytes[8] = {0};
    uint32_t block[2];
  } data_t;

  typedef struct {
    data_t data;
    uint8_t addr;
    uint8_t bytes = 4;  // number of bytes to write
  } reg_t;

  public:
    // Constructor function.
    AD9910(int ssPin, int resetPin, int updatePin, int ps0, int ps1, int ps2, int osk, int f0, int f1);
    AD9910(int ssPin, int resetPin, int updatePin, int ps0);
    // Initialize with refIn frequency, and clock multiplier value
    void initialize(unsigned long ref, uint8_t mult, uint8_t FM_gain, bool oskEnable, bool parallel_programming);
    // Reset the DDS
    void reset();
    // Update to load newly written settings
    void update();
    
    // Sets frequency, amplitude, phase
    void setFreq(unsigned long freq, uint8_t profile = 0);
    // Sets frequency tuning word
    void setFTW(unsigned long ftw, uint8_t profile = 0);
    //Sets scaled amplitude
    void setAmp(double scaledAmp, uint8_t profile = 0);
    void setAmpSF(unsigned long AmpSF, byte profile);
    void setAmpdB(double scaledAmpdB, uint8_t profile = 0);
    //Set Frequency and amplitude with one writing process
    void setFreqAmp(uint32_t freq, double scaledAmp, uint8_t profile);
    void setFreqAmpSF(uint32_t freq, unsigned long AmpSF, uint8_t profile);
    // Sets profile used
    void setProfile(uint8_t profile = 0);
    void setProfileFast(uint8_t profile = 0);
    // Sets phase
    void setPhase(double phase, byte profile);
    void setPOW(unsigned long Pow, byte profile);


    //Get Frequency
    unsigned long getFreq(uint8_t profile = 0);
    // Gets current frequency tuning word
    unsigned long getFTW(uint8_t profile = 0);
    // Gets current amplitude
    double getAmp(uint8_t profile = 0);
    // Gets current amplitude in dB
    double getAmpdB(uint8_t profile = 0);
    // Gets current amplitude scale factor
    unsigned long getASF(uint8_t profile = 0);

    //Parallel Programming:
    void setPPFreq(uint32_t freq);
    inline void setPPFreqFast(uint32_t port_data_word) __attribute__((always_inline));
    //void setPPFreqFast(uint32_t port_data_word);
    // Set Amplitude by OSK:
    void setOSKAmp(double scaledAmp);
    // Set FTW Register:
    void setFTWRegister(uint32_t freq);
    // Function to transform frequency to ParallelPort data word:
    uint32_t transformToPDW(uint32_t freq);

    //RAM programming:
    void enableRamFreq();
    void programRAM(uint32_t data_array[], byte profile, uint16_t start_addr, uint16_t end_addr, uint16_t step_rate, byte RAM_mode, byte no_dwell, byte zero_cross);
    
    /*  *********************** to implement later ***************
    // places DDS in linear sweep mode
    //void linearSweep(unsigned long, unsigned long, unsigned long, byte, unsigned long, byte);
    //enable profile mode
    void enableProfileMode();
    //disable profile mode
    void disableProfileMode();
    //enable OSK
    void enableOSK();
    //disable OSK
    void disableOSK();
    //Get profile mode status
    bool getProfileSelectMode();
    //Get OSK mode status
    bool getOSKMode();
    //enable the Sync Clck output
    void enableSyncClck();
    //disable the Sync Clck output
    void disableSyncClck();
    //Change active profile mode:
*/
    //Get currently active profile
    uint8_t getProfile();

    //Write Register:
    void writeRegister(reg_t payload);

  private:
    // Instance variables that hold pinout mapping
    // from arduino to DDS pins.
    int _ssPin, _resetPin, _updatePin, _ps0, _ps1, _ps2, _f0, _f1, _txEnable, _osk, _fancy, _FM_gain;
    // Instance variables for arrays in profile mode: frequency _freq, frequency tuning word _ftw, amplitude scale factor _asf,
    // reference clock frequency _refClk, amplitude scale factor _ASF, frequency tuning word _FTW,
    // frequency data word _fdw and different parts of the port data word
    unsigned long _freq[8], _ftw[8], _refClk, _asf[8], _pow[8], _ASF, _FTW, _fdw, _port_data_word_lower, _port_data_word_upper, _port_data_word;
    //Instance variables for amplitude and phase arrays in profile mode: 
    double _scaledAmp[8], _scaledAmpdB[8], _phase[8];
    //Instance variable for active profile:
    uint8_t _activeProfile;
    // Instance variables to keep track of the DDS mode:
    bool _OSKon, _parallel_programming;
    // write Freq/Amp/Phase to profile:
    void writeProfile(byte profile);
    // DDS frequency resolution
    double RESOLUTION;// = 4294967296; // sets resolution to 2^32 = 32 bits. Using type double to avoid confusion with integer division...
};

//Inlined_functions:
void AD9910::setPPFreqFast(uint32_t port_data_word){
  // Set Trigger for delay
  //PIOB -> PIO_SODR = PIO_SODR_P27;
  //PIOB -> PIO_CODR = PIO_CODR_P27;
  //Set parallel Port C using TxEnable as Gate(Pin 53):
  PIOB->PIO_SODR = PIO_SODR_P14;
  PIOC->PIO_ODSR = port_data_word;
  PIOB->PIO_CODR = PIO_CODR_P14;
}


#endif
