//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino sketch for the GMLAB D9X
// An open source Do-It-Yourself advanced drawbar controller with 9 drawbars, buttons and knobs for controlling
// virtual tonewheel organs via MIDI CC messages. Mapped to control GSi VB3-II, but compatible with similar products.
// Could be easily remapped for other instruments, or modified to send other types of Midi messages.
// Code by Guido Scognamiglio - www.GenuineSoundware.com
// Project homepage: www.gmlab.it
// GitHub page: https://github.com/ZioGuido
// Last update: March 2019
// 
// Runs on Arduino Leonardo or compatible boards (Atmel ATmega32U4)
// This sketch requires external libraries:
// - MIDI
// - MIDIUSB
// - Adafruit MCP23017
// - MillsTimer
//
// Reads 16 analog inputs (9 drawbars + 6 potentiometers + 1 expression pedal) using 2x CD4051 analog multiplexers.
// Buttons and static LEDs are attached to two MCP23017 I2C I/O Expanders.
// One PWM LED is directly attached to the Atmel.
//


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// USER DEFINITION OF THE MIDI MAP
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define EXP_PEDAL_CC 11

int DrawbarCCMap[3][9] = 
{
  { 12, 13, 14, 15, 16, 17, 18, 19, 20 }, // Upper
  { 21, 22, 23, 24, 25, 26, 27, 28, 29 }, // Lower
  { 33, 35,  0,  0,  0,  0,  0,  0,  0 }  // Pedalboard
};

enum PotentiometerCCMap
{
  kMidiCC_Volume          =  7,
  kMidiCC_Overdrive       = 76,  
  kMidiCC_Reverb          = 91,  
  kMidiCC_KeyClick        = 75,
  kMidiCC_EqBass          =  8,  
  kMidiCC_EqTreble        = 10,  
};

enum ButtonCCMap 
{
  kMidiCC_VibratoType     = 73,
  kMidiCC_VibratoUpper    = 31,  
  kMidiCC_VibratoLower    = 30,  
  kMidiCC_PercussionOn    = 66,  
  kMidiCC_PercussionSoft  = 70,  
  kMidiCC_PercussionFast  = 71,  
  kMidiCC_PercussionThird = 72,  
  kMidiCC_RotaryOnOff     = 85,  
  kMidiCC_RotarySlowFast  =  1,  
  kMidiCC_RotaryRunStop   = 68,  
};

// The CC values sent for each of the 6 C/V positions
int CVvalues[6] = { 0, 27, 52, 78, 102, 127 };


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// YOU DON'T NEED TO MODIFY ANYTHING ELSE BELOW THIS LINE
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// I/O PIN DEFINITIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Peripherals directly attached to the Arduino board
#define CD4051_A          15
#define CD4051_B          14
#define CD4051_C          16
#define CD4051_AIN1       A1 // 19
#define CD4051_AIN2       A0 // 18
#define PWM_LED_SPEED     10

// Buttons and LEDs attached to MCP expanders in pin order
#define BTN_VIB_ON    0
#define LED_VIB_ON    1
#define LED_VIB_V3    2
#define LED_VIB_C2    3
#define LED_VIB_V2    4
#define BTN_VIB_TYP   5
#define LED_VIB_C1    6
#define LED_VIB_V1    7
#define LED_VIB_C3    8
#define BTN_RUNSTOP   9
#define LED_RUNSTOP  10
#define BTN_PRC_ON   11
#define LED_PRC_ON   12
#define BTN_PRC_HR   13
#define LED_PRC_HR   14
#define BTN_PRC_FS   15
#define LED_PRC_FS   16
#define BTN_PRC_SF   17
#define LED_PRC_SF   18
#define BTN_DRB_SEL  19
#define BTN_SHIFT    20
#define LED_SEL_UP   21
#define LED_SEL_LO   22
#define LED_SEL_PD   23
#define BTN_SLOWFAST 24


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// External libraries
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <MIDIUSB.h>
#include <MIDI.h>
MIDI_CREATE_DEFAULT_INSTANCE();
#include <EEPROM.h>
#include <Wire.h> // Needed to access the I2C bus
#include "Adafruit_MCP23017.h" // This is needed for the two I/O Expanders
Adafruit_MCP23017 mcp0;
Adafruit_MCP23017 mcp1;
#include "MillisTimer.h"  // This library creates a timer with millisecond precision
MillisTimer ButtonTimer;  // This is used for checking the buttons
MillisTimer LedAnimTimer; // This is used for animating the "speed" PWM-driven LED


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define DEADBAND 8
int adc_counter = 0;
int drawbar_set = 0; // range 0 ~ 2 for the three sets
int vc_position = 0; // range 0 ~ 5 for the 6 positions
int rotary_speed = 1; // 0 = stop, 1 = slow, 2 = fast
int PotCCMap[6] = { kMidiCC_EqBass, kMidiCC_EqTreble,  kMidiCC_Volume, kMidiCC_Overdrive, kMidiCC_Reverb, kMidiCC_KeyClick };
int prev_val[16] = { -1, -1, -1, -1, -1, -1, -1, -1,  -1, -1, -1, -1, -1, -1, -1, -1 };
int ButtonPinNumbers[10] = { BTN_VIB_ON, BTN_VIB_TYP, BTN_RUNSTOP, BTN_PRC_ON, BTN_PRC_SF, BTN_PRC_FS, BTN_PRC_HR, BTN_DRB_SEL, BTN_SHIFT, BTN_SLOWFAST };
int btn_prev_status[10] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
int SHIFT = 0;
int PWM_Ramp = 0;
int PWM_RampInc = 1;

// This const array contains precalculated values of a sinusoidal waveform
const unsigned char sine_tbl[128] = { 134, 140, 147, 153, 159, 165, 171, 177, 182, 188, 193, 199, 204, 209, 213, 218, 222, 226, 230, 234, 237, 240, 243, 245, 248, 250, 251, 253, 254, 254, 255, 255, 255, 254, 254, 253, 251, 250, 248, 245, 243, 240, 237, 234, 230, 226, 222, 218, 213, 209, 204, 199, 193, 188, 182, 177, 171, 165, 159, 153, 147, 140, 134, 128, 122, 116, 109, 103, 97, 91, 85, 79, 74, 68, 63, 57, 52, 47, 43, 38, 34, 30, 26, 22, 19, 16, 13, 11, 8, 6, 5, 3, 3, 3, 2, 2, 2, 3, 3, 3, 5, 6, 8, 11, 13, 16, 19, 22, 26, 30, 34, 38, 43, 47, 52, 57, 63, 68, 74, 79, 85, 91, 97, 103, 109, 116, 122, 128, };

enum EEPROM_MemoryLocations
{
  EEPROM_DRAWBAR_SET = 16,
  EEPROM_VIBRATO_UPPER,
  EEPROM_VIBRATO_LOWER,
  EEPROM_VIBRATO_SEL,
  EEPROM_PERC_ON,
  EEPROM_PERC_SOFT,
  EEPROM_PERC_FAST,
  EEPROM_PERC_THIRD,
  EEPROM_SPEED_RUNSTOP,
  EEPROM_SPEED_SLOWFAST,
  EEPROM_ROTARY_ONOFF,
};

enum ButtonNumbers
{
  kBtnNum_VibratoOn, 
  kBtnNum_VibratoType,
  kBtnNum_RunStop,
  kBtnNum_PercussionOn,
  kBtnNum_PercussionSoft,
  kBtnNum_PercussionFast,
  kBtnNum_PercussionThird,
  kBtnNum_ManualSelect,
  kBtnNum_SHIFT,
  kBtnNum_SlowFast,
};

// This structure holds the latching status of each button. 
// This could have been done using a single int variable and setting each bit with bitshift operators,
// but why? This way it's more readable and easy to understand. Plus, we have plenty of memory for our application :) 
struct LatchesStruct
{
  bool VibratoUpper;
  bool VibratoLower;
  bool RotaryOnOff;
  bool RotarySlowFast;
  bool RotaryRunStop;
  bool PercussionOn;
  bool PercussionSoft;
  bool PercussionFast;
  bool PercussionThird;
} Latches;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS...
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// This helper function maps the pins to the appropriate MCP expander and sets the required modes
void pinModeMCP(int pin, int mode)
{
  Adafruit_MCP23017 *mcp = pin > 15 ? &mcp1 : &mcp0;
  int mcp_pin = pin > 15 ? pin - 16 : pin;
  
  switch (mode)
  {
  case OUTPUT:
    mcp->pinMode(mcp_pin, OUTPUT);
    break;
    
  case INPUT:
    mcp->pinMode(mcp_pin, INPUT);
    break;

  case INPUT_PULLUP:
    mcp->pinMode(mcp_pin, INPUT);
    mcp->pullUp(mcp_pin, HIGH);
    break;
  }
}

void SendMidiCC(int channel, int num, int value)
{
  // Don't send if CC number is zero
  if (num == 0) return;
  
  midiEventPacket_t CC = {0x0B, 0xB0 | channel, num, value};
  MidiUSB.sendMIDI(CC);
  MidiUSB.flush();

  MIDI.sendControlChange(num, value, channel + 1); // Midi lib wants channels 1~16
}

// Helper function used to set the status of LEDs attached to the two MCP expanders
void SetLed(int ID, int value)
{
  if (ID > 15) 
    mcp1.digitalWrite(ID - 16, value);
  else
    mcp0.digitalWrite(ID, value);
}

// This function selects which of the 6 C/V LEDs must be lit according to the current selection
void SetVC_LEDs()
{
  SetLed(LED_VIB_V1, vc_position == 0);
  SetLed(LED_VIB_C1, vc_position == 1);
  SetLed(LED_VIB_V2, vc_position == 2);
  SetLed(LED_VIB_C2, vc_position == 3);
  SetLed(LED_VIB_V3, vc_position == 4);
  SetLed(LED_VIB_C3, vc_position == 5);
}

// Same as above, but for the three drawbar set LEDs
void SetDbSet_LEDs()
{
  SetLed(LED_SEL_UP, drawbar_set == 0);
  SetLed(LED_SEL_LO, drawbar_set == 1);
  SetLed(LED_SEL_PD, drawbar_set == 2);

  // Restore VibratoOn LED when switching drawbar set
  SetLed(LED_VIB_ON, drawbar_set == 0 ? Latches.VibratoUpper : Latches.VibratoLower);
}

// This function is called by the timer for animating the SPEED PWM-driven LED
void DoSpeedLED()
{
  // Keep this LED off if the Rotary is set to Bypass (OFF)
  if (Latches.RotaryOnOff == 0) 
  {
    analogWrite(PWM_LED_SPEED, 0);
    return;
  }
  
  // Vary the PWM incrementer according to the selected speed
  switch (rotary_speed)
  {
  case 0: // STOP
    if (--PWM_RampInc < 1) PWM_RampInc = 0;
    break;

  case 1: // SLOW
    if (--PWM_RampInc <= 5) PWM_RampInc = 5;
    break;

  case 2: // FAST
    if (++PWM_RampInc >= 50) PWM_RampInc = 50;
    break;  
  }
  
  // Now increment the PWM ramp to read from the sine wavetable and set the PWM value
  PWM_Ramp += PWM_RampInc;
  if (PWM_Ramp > 127) PWM_Ramp = 0;
  analogWrite(PWM_LED_SPEED, sine_tbl[PWM_Ramp]);
}

// This is called by the main loop for reading all analog inputs attached to the two CD4051 multiplexers
// and send the MIDI CC messages accordingly
void DoPot(int d, int value)
{
  // Get difference from current and previous value
  int diff = abs(value - prev_val[d]);
  
  // Exit this function if the new value is within the deadband
  if (diff <= DEADBAND) return;
  
  // Store new value
  prev_val[d] = value;    

  // Get the 7 bit value
  int val7bit = value >> 3;
  
  // Send Midi 
  if (d < 6) // It's a potentiometer
    SendMidiCC(0, PotCCMap[d], val7bit);
  else if (d == 6) // The expression pedal
    SendMidiCC(0, EXP_PEDAL_CC, val7bit);
  else // Drawbars
    SendMidiCC(0, DrawbarCCMap[drawbar_set][d - 7], val7bit); // Start from AIN n.7 = Drawbar 0
}

// This function checks whether a button changes its status and takes action accordingly
void DoButton(int btn, int status)
{
  if (btn_prev_status[btn] == status) return;
  btn_prev_status[btn] = status;

  // Things to do when a button is depressed
  if (status == LOW)
  {
    // Things to do when the button SHIFT is held depressed
    if (SHIFT)
    {
      if (btn == kBtnNum_RunStop)
      {
        Latches.RotaryOnOff = !Latches.RotaryOnOff;
        SendMidiCC(0, kMidiCC_RotaryOnOff, Latches.RotaryOnOff ? 127 : 0);
        EEPROM.write(EEPROM_ROTARY_ONOFF, Latches.RotaryOnOff);
      }
      // Skip the rest of the function
      return; 
    }

    switch (btn)
    {
    case kBtnNum_VibratoOn:
      if (drawbar_set == 0)
      {
        Latches.VibratoUpper = !Latches.VibratoUpper;
        SendMidiCC(0, kMidiCC_VibratoUpper, Latches.VibratoUpper ? 127 : 0);
        SetLed(LED_VIB_ON, Latches.VibratoUpper);
        EEPROM.write(EEPROM_VIBRATO_UPPER, Latches.VibratoUpper);
      } else {
        Latches.VibratoLower = !Latches.VibratoLower;
        SendMidiCC(0, kMidiCC_VibratoLower, Latches.VibratoLower ? 127 : 0);
        SetLed(LED_VIB_ON, Latches.VibratoLower);
        EEPROM.write(EEPROM_VIBRATO_LOWER, Latches.VibratoLower);
      }
      break;
      
    case kBtnNum_VibratoType:
      if (++vc_position > 5) vc_position = 0;
      SendMidiCC(0, kMidiCC_VibratoType, CVvalues[vc_position]);
      SetVC_LEDs();
      EEPROM.write(EEPROM_VIBRATO_SEL, vc_position);
      break;

    case kBtnNum_RunStop:
      Latches.RotaryRunStop = !Latches.RotaryRunStop;
      SendMidiCC(0, kMidiCC_RotaryRunStop, Latches.RotaryRunStop ? 127 : 0);
      SetLed(LED_RUNSTOP, Latches.RotaryRunStop);
      if (Latches.RotaryRunStop)
      {
        rotary_speed = 0;
        SendMidiCC(0, kMidiCC_RotarySlowFast, 64);
      } else {
        rotary_speed = Latches.RotarySlowFast + 1;
        SendMidiCC(0, kMidiCC_RotarySlowFast, Latches.RotarySlowFast ? 127 : 0);
      }
      EEPROM.write(EEPROM_SPEED_RUNSTOP, Latches.RotaryRunStop);
      DoSpeedLED();
      break;
      
    case kBtnNum_PercussionOn:
      Latches.PercussionOn = !Latches.PercussionOn;
      SendMidiCC(0, kMidiCC_PercussionOn, Latches.PercussionOn ? 127 : 0);
      SetLed(LED_PRC_ON, Latches.PercussionOn);
      EEPROM.write(EEPROM_PERC_ON, Latches.PercussionOn);
      break;
      
    case kBtnNum_PercussionSoft:
      Latches.PercussionSoft = !Latches.PercussionSoft;
      SendMidiCC(0, kMidiCC_PercussionSoft, Latches.PercussionSoft ? 127 : 0);
      SetLed(LED_PRC_SF, Latches.PercussionSoft);
      EEPROM.write(EEPROM_PERC_SOFT, Latches.PercussionSoft);
      break;
      
    case kBtnNum_PercussionFast:
      Latches.PercussionFast = !Latches.PercussionFast;
      SendMidiCC(0, kMidiCC_PercussionFast, Latches.PercussionFast ? 127 : 0);
      SetLed(LED_PRC_FS, Latches.PercussionFast);
      EEPROM.write(EEPROM_PERC_FAST, Latches.PercussionFast);
      break;
      
    case kBtnNum_PercussionThird:
      Latches.PercussionThird = !Latches.PercussionThird;
      SendMidiCC(0, kMidiCC_PercussionThird, Latches.PercussionThird ? 127 : 0);
      SetLed(LED_PRC_HR, Latches.PercussionThird);
      EEPROM.write(EEPROM_PERC_THIRD, Latches.PercussionThird);
      break;
      
    case kBtnNum_ManualSelect:
      if (++drawbar_set > 2) drawbar_set = 0;
      SetDbSet_LEDs();
      EEPROM.write(EEPROM_DRAWBAR_SET, drawbar_set);
      break;
      
    case kBtnNum_SHIFT:
      SHIFT = 1;
      break;
      
    case kBtnNum_SlowFast:
      if (Latches.RotaryRunStop) // If brake is engaged
      {
        // Disengage the brake..
        Latches.RotaryRunStop = 0;
        SendMidiCC(0, kMidiCC_RotaryRunStop, Latches.RotaryRunStop ? 127 : 0); 
        SetLed(LED_RUNSTOP, Latches.RotaryRunStop);
        EEPROM.write(EEPROM_SPEED_RUNSTOP, Latches.RotaryRunStop);
      }
      // ...and switch speed
      Latches.RotarySlowFast = !Latches.RotarySlowFast;
      rotary_speed = Latches.RotarySlowFast + 1; // set to 1 for slow and 2 for fast
      SendMidiCC(0, kMidiCC_RotarySlowFast, Latches.RotarySlowFast ? 127 : 0);
      EEPROM.write(EEPROM_SPEED_SLOWFAST, Latches.RotarySlowFast);
      DoSpeedLED();
      break;
    }  
  }

  // Things to do when a button is released
  else
  {
    if (btn == kBtnNum_SHIFT)
      SHIFT = 0;
  }
}

// This is called by the timer to read all button statuses
void CheckButtons()
{
  for (int b=0; b<10; ++b)
  {
    int ButtonPin = ButtonPinNumbers[b];
    if (ButtonPin < 16)
    {
      DoButton(b, mcp0.digitalRead(ButtonPin));
    } else {
      DoButton(b, mcp1.digitalRead(ButtonPin - 16));
    }
  }
}

// Called once at boot time to set all I/Os and initialize variables and other stuff
void setup()
{
  // Define pins for the two CD4051 analog multiplexers
  pinMode(CD4051_A, OUTPUT); // A
  pinMode(CD4051_B, OUTPUT); // B
  pinMode(CD4051_C, OUTPUT); // C

  // This is the PWM output for the Slow/Fast LED
  pinMode(PWM_LED_SPEED, OUTPUT);

  // Initialize the two MCP23017 expanders on the I2C bus
  mcp0.begin(0);
  mcp1.begin(1);
  
  // Set all buttons in INPUT mode and activate the internal 100K pull-up resistors
  pinModeMCP(BTN_VIB_ON,    INPUT_PULLUP);
  pinModeMCP(BTN_VIB_TYP,   INPUT_PULLUP);
  pinModeMCP(BTN_RUNSTOP,   INPUT_PULLUP);
  pinModeMCP(BTN_PRC_ON,    INPUT_PULLUP);
  pinModeMCP(BTN_PRC_HR,    INPUT_PULLUP);
  pinModeMCP(BTN_PRC_FS,    INPUT_PULLUP);
  pinModeMCP(BTN_PRC_SF,    INPUT_PULLUP);
  pinModeMCP(BTN_DRB_SEL,   INPUT_PULLUP);
  pinModeMCP(BTN_SHIFT,     INPUT_PULLUP);
  pinModeMCP(BTN_SLOWFAST,  INPUT_PULLUP);
  
  // Set all LEDs in OUTPUT mode
  pinModeMCP(LED_SEL_UP, OUTPUT);
  pinModeMCP(LED_SEL_LO, OUTPUT);
  pinModeMCP(LED_SEL_PD, OUTPUT);
  pinModeMCP(LED_VIB_V1, OUTPUT);
  pinModeMCP(LED_VIB_C1, OUTPUT);
  pinModeMCP(LED_VIB_V2, OUTPUT);
  pinModeMCP(LED_VIB_C2, OUTPUT);
  pinModeMCP(LED_VIB_V3, OUTPUT);
  pinModeMCP(LED_VIB_C3, OUTPUT);
  pinModeMCP(LED_VIB_ON, OUTPUT);
  pinModeMCP(LED_RUNSTOP, OUTPUT);
  pinModeMCP(LED_PRC_ON, OUTPUT);
  pinModeMCP(LED_PRC_HR, OUTPUT);
  pinModeMCP(LED_PRC_FS, OUTPUT);
  pinModeMCP(LED_PRC_SF, OUTPUT);

  // Restore last settings from EEPROM
  drawbar_set = EEPROM.read(EEPROM_DRAWBAR_SET);
  vc_position = EEPROM.read(EEPROM_VIBRATO_SEL);
  Latches.VibratoUpper    = EEPROM.read(EEPROM_VIBRATO_UPPER);
  Latches.VibratoLower    = EEPROM.read(EEPROM_VIBRATO_LOWER);
  Latches.PercussionOn    = EEPROM.read(EEPROM_PERC_ON);
  Latches.PercussionSoft  = EEPROM.read(EEPROM_PERC_SOFT);
  Latches.PercussionFast  = EEPROM.read(EEPROM_PERC_FAST);
  Latches.PercussionThird = EEPROM.read(EEPROM_PERC_THIRD);
  Latches.RotaryRunStop   = EEPROM.read(EEPROM_SPEED_RUNSTOP);
  Latches.RotarySlowFast  = EEPROM.read(EEPROM_SPEED_SLOWFAST);
  Latches.RotaryOnOff     = EEPROM.read(EEPROM_ROTARY_ONOFF);
  rotary_speed = Latches.RotaryRunStop ? 0 : (Latches.RotarySlowFast ? 2 : 1);

  // Restore LEDs
  SetLed(LED_RUNSTOP, Latches.RotaryRunStop);
  SetLed(LED_PRC_ON, Latches.PercussionOn);
  SetLed(LED_PRC_SF, Latches.PercussionSoft);
  SetLed(LED_PRC_FS, Latches.PercussionFast);
  SetLed(LED_PRC_HR, Latches.PercussionThird);
  SetDbSet_LEDs();
  SetVC_LEDs();
  DoSpeedLED();
  
  // Set Timers
  ButtonTimer.setInterval(5);
  ButtonTimer.expiredHandler(CheckButtons);
  ButtonTimer.start();

  LedAnimTimer.setInterval(50);
  LedAnimTimer.expiredHandler(DoSpeedLED);
  LedAnimTimer.start();

  // Initialize serial MIDI
  MIDI.begin(MIDI_CHANNEL_OMNI);  
}

// The main loop
void loop()
{
  // Increment counter for the two 4051
  if (++adc_counter > 15) adc_counter = 0;

  // Set 4051 BCD selection ports
  digitalWrite(CD4051_A, (adc_counter & 7) & 1);
  digitalWrite(CD4051_B, (adc_counter & 7) >> 1 & 1);
  digitalWrite(CD4051_C, (adc_counter & 7) >> 2 & 1);

  // Read the input from the selected ADC port:
  DoPot(adc_counter, analogRead(adc_counter > 7 ? CD4051_AIN2 : CD4051_AIN1));

  // Run Timers
  ButtonTimer.run();
  LedAnimTimer.run();
}
