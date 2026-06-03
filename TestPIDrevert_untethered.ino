// PID Servo on qNimble Device
// Author: Prahlad Iyengar (prahlad.iyengar@gtri.gatech.edu)

/*
NOTES
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 The qNimble uses an Arduino, and therefore interface with the base code must be done through the Arduino IDE. However, once the desired functions have been programmed,
 the protocol for interfacing with the device is a simple serial COM line. This serial line is specially labeled for the Quarto board, and is called qCommand.

 The qNimble has 4 ADC channels and 4 DAC channels, so the user can perform servoing for 4 separate systems at maximum. However, it is generally desirable to track the
 error differential between the input and setpoint while servoing, which requires the use of 2 DAC channels per servo. Thus, the default configuration of this PID ctrl
 is to servo on 2 channels, outputting the error signals from each servo onto 2 of the DACs while outputting normally on the other 2.
 Sampling Note: the max sample rate is 1 MSPS for the ADC, or one sample every microsecond (us)... so for sampling on 4 channels, the fastest we can sample on each is every 4us.
 If we configure fewer ADC channels, we can improve the bandwidth

 The remaining ADC channels can optionally be configured to feed in the setpoint, if this is desired. Note, however, that this will reduce the servo bandwidth. If it is
 possible to adjust the setpoints programatically instead, which is supported by serial command interface, then the bandwidth is doubled. THe qNimble's Arduino samples at
 1 MSPS across all 4 of its ADC channels, so the faster version allows for a servo step every 2 us, whereas the slower version samples each ADC every 4 us.

 To prevent headache, use the following conventions:
     - Perform servo on ADC/DAC channels 1 and 2 only --> preference goes to channel 1 if performing only one servo
     - Read error from servo on DAC channels 3 and 4 respectively
     - If feeding a setpoint, feed into ADC channels 3 and 4 respectively
nah man i fixed it to this ->

All 4 channels operate independently:
- Each channel reads error from its corresponding ADC (ADC1→Ch1, ADC2→Ch2, etc.)
- Each channel outputs to its corresponding DAC (DAC1→Ch1, DAC2→Ch2, etc.)
- Setpoints are digital only (set via serial commands)
- No cross-channel dependencies


 If a change is made to the configuration (e.g., setpoint feeding is turned on when previously off, one servo is removed, etc.), use the serial input function ADCconfig.
 This calls setConfig(), which then sets up the configuration as desired to optimize bandwidth. Note that this will temporarily halt servoing.

 An offset can be added to the output of each channel, e.g. to ensure that the power out of the servo is enough to cause AOM modulation / diffraction mode splitting for
 servo monitoring. Additionally, the output of each channel can be swept according to a cosine curve (provided parameters), or even simply copied from one of the ADCs.

 The PID channel can be in one of several possible states:
 (1) SERVO  --> active servoing (can be indicated either by triggers or by pins)
 (2) HOLD   --> inactive servo, holds the previous integrator value
 (3) OFFSET --> disabled servo, outputs the offset voltage (including when the output is swept)
 (4) TRACK  --> disabled servo, outputs the error of PID channel Y = (X+2) mod 4
 (5) FEED   --> disabled servo, feeds the input of ADC X to PID Y as a setpoint

 Notice that states TRACK and FEED are not mutually exclusive, i.e., they can occur simultaneously. Otherwise, states are mutually exclusive.

 When evaluating the servo conditions, the following conditional demonstrates the hierarchy of decision:
 
          bool isServoX = !feed_set(X + 2 (mod 4)) && !track_error(X + 2 (mod 4)) && enableX && !holdX;

 isServo tells us whether servoing should be active. It is NOT active when:

 (1) feeding setpoint to the neighboring PID since the corresponding ADC will be busy --> returns FALSE to servo
 (2) tracking error on the nieghboring PID since the correspondiong DAC will be busy --> returns FALSE to servo
 (3) servo is not enabled --> returns TRUE to servo
 (4) holding is ON --> returns FALSE to servo
 
 The decision hierarchy flows from left to right, top to bottom. PID channel X involves at minimum ADC X for reading and DAC X for PID control.
 Thus, letting Y = (X+2) mod 4:

 (1) the first check is whether ADC X is occupied with feeding the setpoint for PID Y. 
 (2) the second check is whether DAC X is occupied with the differential error from PID Y. Note that if BOTH are being done for PID Y, ONLY PID X is decommissioned
          *** The above 2 top level checks are equivalent in hierarchy *** 
 (3) the third check is whether servoing is at all enabled on the PID channel. This is set programatically, but is autoset to FALSE if either of the above two checks return TRUE
 (4) the fourth check is whether to HOLD the output (integrator) on ADC X, controlled by input PIN X. This is likewise autoset to FALSE if any of the above three checks are TRUE

 If isServo returns TRUE, then all four checks were passed, and servoing is ACTIVE. Otherwise, we evaluate back UP the hierarchy to see which case to consider:

 (A) (if) isServo is TRUE --> STATE:SERVO
         subcheck: (if) feed_setX is TRUE --> setX = inY (note that PIN Y+4 should be ON for inY to populate) --> check is redundant with case (C) in PID Y
     (else if) [track_errorY is TRUE || feed_setY is TRUE]*:
        FIRST: turn OFF lower hierchial conditionals (holdX, enableX, track_errorX, feed_setX)
        (B) (if) track_errorY is TRUE --> output errY on DAC X. STATE:TRACK 
        (C) (if) feed_setY is TRUE --> setY = inX. STATE:FEED
 (D) (else if) enableX is FALSE --> output offsetX to DAC X & turn holdX OFF (this is NOT the hold condition) STATE:OFFSET
 (E) (else) enableX is TRUE && holdX is TRUE --> STATE:HOLD (effectively pass**, so no need to check this conditional)

  * Note that within this realization, both TRACK and FEED can occur simultaneously. The nested structure ensures that this is evaluated before OFFSET and HOLD
  ** In practical application, the HOLD state is the default (the DAC will hold the last value it was fed). So there is no need to run anything on the conditional enableX.

  MEMORY
  In case of power cycling, the Quarto stores the gain constants and ADC configuration in its non-volatile memory (NVM), which is known as EEPROM on Arduinos. The total space
  is 96 kB, and the readNVM() function outputs a uint16_t, so the input address must be EVEN. If it's odd, the readNVM() function wll round it down. Thus readNVM(15) = readNVM(14).
  
  The NVM is arranged in 768 pages of 128 bytes each. Writing is done page-by-page, so an entire page must be rewritten during rewrites, or else information is lost. To mitigate
  this, since there is not much data to store between reboots, each PID gain constant and setpoint is stored on its OWN PAGE in the NVM. The allocation is as follows:

    - Pages 00-03 store the proportional gain constants 1-4, respectively                   (double)
    - Pages 04-07 store the integral gain constants 1-4, respectively                       (double)
    - Pages 08-11 store the differential gain constants 1-4, respectively                   (double)
    - Pages 12-15 store setpoints 1-4, respectively                                         (double)
    - Pages 16-19 store the offsets 1-4, respectively                                       (double)
    - Page 20 stores the current configuration as a uint8_t variable config                 (uint8_t)
    - Pages 25-28 store the servo enable states of channels 1-4, respectively               (int)
    - Pages 29-32 store the error tracking status of channels 1-4, respectively             (bool)
    - Pages 33-36 store the setpoint feeding status of channels 1-4, respectively           (bool)
    - Pages 37-40 store the sweeping status of channels 1-4, respectively                   (bool)
    - Pages 41-44 store the sweep amplitude of channels 1-4, respectively                   (double)
    - Pages 45-48 store the sweep period of channels 1-4, respectively                      (double)
    - Pages 49-52 store the sweep phases of channels 1-4, respectively                      (double)
    - Pages 53-56 store the sweep types of channels 1-4, respectively                       (int)
    - Pages 57-60 store the input ranges of the ADCs 1-4, respectively                      (adc_scale_t) --> ENUM
    - Pages 61-64 store the minimum output rails of DACs 1-4, respectively                  (double)
    - Pages 65-68 store the maximum output rails of DACs 1-4, respectively                  (double)
    - Pages 69-72 store the locking precision of servos 1-4, respectively                   (double)
    - Page 73 stores the default default hold logic value                                   (uint8_t)

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

#include "qCommand.h"
#include <math.h>
#define M_PI 3.14159265358979323846

// ============================================================================
// SINE LOOKUP TABLE (256 samples for one complete period)
// ============================================================================
const float SINE_TABLE[256] = {
  0.000000, 0.024541, 0.049068, 0.073565, 0.098017, 0.122411, 0.146730, 0.170962,
  0.195090, 0.219101, 0.242980, 0.266713, 0.290285, 0.313682, 0.336890, 0.359895,
  0.382683, 0.405241, 0.427555, 0.449611, 0.471397, 0.492898, 0.514103, 0.534998,
  0.555570, 0.575808, 0.595699, 0.615232, 0.634393, 0.653173, 0.671559, 0.689541,
  0.707107, 0.724247, 0.740951, 0.757209, 0.773010, 0.788346, 0.803208, 0.817585,
  0.831470, 0.844854, 0.857729, 0.870087, 0.881921, 0.893224, 0.903989, 0.914210,
  0.923880, 0.932993, 0.941544, 0.949528, 0.956940, 0.963776, 0.970031, 0.975702,
  0.980785, 0.985278, 0.989177, 0.992480, 0.995185, 0.997290, 0.998795, 0.999699,
  1.000000, 0.999699, 0.998795, 0.997290, 0.995185, 0.992480, 0.989177, 0.985278,
  0.980785, 0.975702, 0.970031, 0.963776, 0.956940, 0.949528, 0.941544, 0.932993,
  0.923880, 0.914210, 0.903989, 0.893224, 0.881921, 0.870087, 0.857729, 0.844854,
  0.831470, 0.817585, 0.803208, 0.788346, 0.773010, 0.757209, 0.740951, 0.724247,
  0.707107, 0.689541, 0.671559, 0.653173, 0.634393, 0.615232, 0.595699, 0.575808,
  0.555570, 0.534998, 0.514103, 0.492898, 0.471397, 0.449611, 0.427555, 0.405241,
  0.382683, 0.359895, 0.336890, 0.313682, 0.290285, 0.266713, 0.242980, 0.219101,
  0.195090, 0.170962, 0.146730, 0.122411, 0.098017, 0.073565, 0.049068, 0.024541,
  0.000000, -0.024541, -0.049068, -0.073565, -0.098017, -0.122411, -0.146730, -0.170962,
  -0.195090, -0.219101, -0.242980, -0.266713, -0.290285, -0.313682, -0.336890, -0.359895,
  -0.382683, -0.405241, -0.427555, -0.449611, -0.471397, -0.492898, -0.514103, -0.534998,
  -0.555570, -0.575808, -0.595699, -0.615232, -0.634393, -0.653173, -0.671559, -0.689541,
  -0.707107, -0.724247, -0.740951, -0.757209, -0.773010, -0.788346, -0.803208, -0.817585,
  -0.831470, -0.844854, -0.857729, -0.870087, -0.881921, -0.893224, -0.903989, -0.914210,
  -0.923880, -0.932993, -0.941544, -0.949528, -0.956940, -0.963776, -0.970031, -0.975702,
  -0.980785, -0.985278, -0.989177, -0.992480, -0.995185, -0.997290, -0.998795, -0.999699,
  -1.000000, -0.999699, -0.998795, -0.997290, -0.995185, -0.992480, -0.989177, -0.985278,
  -0.980785, -0.975702, -0.970031, -0.963776, -0.956940, -0.949528, -0.941544, -0.932993,
  -0.923880, -0.914210, -0.903989, -0.893224, -0.881921, -0.870087, -0.857729, -0.844854,
  -0.831470, -0.817585, -0.803208, -0.788346, -0.773010, -0.757209, -0.740951, -0.724247,
  -0.707107, -0.689541, -0.671559, -0.653173, -0.634393, -0.615232, -0.595699, -0.575808,
  -0.555570, -0.534998, -0.514103, -0.492898, -0.471397, -0.449611, -0.427555, -0.405241,
  -0.382683, -0.359895, -0.336890, -0.313682, -0.290285, -0.266713, -0.242980, -0.219101,
  -0.195090, -0.170962, -0.146730, -0.122411, -0.098017, -0.073565, -0.049068, -0.024541
};


qCommand qC; //for serial interface

////////////////////////////////////////////////////////////////////////////////////
// FORWARD DECLARATIONS             ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
// ISR callback functions
void getADC1(void);
void getADC2(void);
void getADC3(void);
void getADC4(void);


// Timing measurement functions
void timing_stats(qCommand& qC, Stream& S);
void reset_timing(qCommand& qC, Stream& S);


uint64_t isr1_cycles_sum = 0;
uint64_t isr2_cycles_sum = 0;
uint64_t isr3_cycles_sum = 0;
uint64_t isr4_cycles_sum = 0;
// ← END OF FORWARD DECLARATIONS ↑

////////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES                 ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

// =========================
// Output modes
// =========================
enum OutMode { 
  MODE_SERVO = 0,
  MODE_SWEEP = 1,
  MODE_SQUARE = 2
};

OutMode out_mode = MODE_SERVO;   // default: your existing PID/sweep

// =========================
// Square-wave / PWM config
// =========================

IntervalTimer SQ_Timer;
const float sq_update_us = 1.0f; // common step for all channels

struct SquareConfig {
  bool   enabled;      // is square mode enabled on this channel?
  OutMode mode;        // current output mode for this channel

  // square wave parameters
  float  freq_Hz;      // square wave frequency
  float  duty;         // 0..1
  float  amp_scale;    // Vpp scaling factor (controlled by Vpp loop)

  // Vpp control
  float  vpp_set;      // desired Vpp
  float  vpp_gain;     // gain for amplitude control
  float  meas_min;
  float  meas_max;
  uint32_t meas_count;
  uint32_t meas_window_samples;

  // timing state
  uint32_t count;      // step counter within square-wave generator
  uint32_t steps_per_period;

  //value 
  float value;
};

SquareConfig sq[4];    // sq[0] for channel 1, sq[1] for channel 2, etc.
int sq_mode_int[4];

// =========================
// NVM layout for square/Vpp
// =========================
const int SQ_BASE_PAGE      = 74;    // first free page after your existing 0..73
const int SQ_PAGES_PER_CH   = 7;     // pages per channel for config


// VERSION NUMBER
String version = "v1.2";

// ISR entry counters for diagnostics
volatile uint32_t isr1_entry_count = 0;
volatile uint32_t isr2_entry_count = 0;
volatile uint32_t isr3_entry_count = 0;
volatile uint32_t isr4_entry_count = 0;
// ← END ↑

// Initialize LED
bool purple = false;

// Default servo values (do not write to these)
double default_p = 0;
double default_i = 0;
double default_d = 0;
double default_s = 0;

double default_offset = 0;
double default_a = 0;
double default_T = 0;
double default_phi = 0;

int default_enable = 0;
bool default_track = 0;
bool default_sweep = 0;
bool default_feed = 0;

int default_swp_tp = 1;

uint8_t default_config = 0x01;

adc_scale_t default_adc_range = BIPOLAR_10V;
float default_rail_max = 10;
float default_rail_min = -10;
float default_lock_precision = 0.0001;

uint8_t default_default_hold = 1; // the default hold logic value - can be flipped with the "flip_hold" method

// Servo Setup
double t_res = 2.0; // sample period on each channel in us... 1 MSPS maximum across all 4 channels

uint8_t config = 0x05; // current ADC configuration

double offset1 = 1.0; // NOTE: offsetX is the constant value which is ALWAYS added to the output of DAC channel X (e.g., if we want to ensure a minimum AOM modulation)
double offset2 = 1.0; // to disable, simply set to 0. This can be programatically swept using the sweepOffset() function. This is NOT the value which stays on the
double offset3 = 1.0; // channel in the HOLD situation -- that will be the value previously calculated by the PID, keeping a constant integrator. The held value will
double offset4 = 1.0; // already have the offset added to it, so there is no need to continue adding it.

// Servo I/O channel values for monitoring
double in1 = 0;
double in2 = 0;
double in3 = 0;
double in4 = 0;

double out1 = 0;
double out2 = 0;
double out3 = 0;
double out4 = 0;

static double err1 = 0;
static double err2 = 0;
static double err3 = 0;
static double err4 = 0;

// Servo extreme parameters
adc_scale_t adc1_range = BIPOLAR_10V;
adc_scale_t adc2_range = BIPOLAR_10V;
adc_scale_t adc3_range = BIPOLAR_10V;
adc_scale_t adc4_range = BIPOLAR_10V;

double rail1_min = -10; // DAC min rail values
double rail2_min = -10;
double rail3_min = -10;
double rail4_min = -10;

double rail1_max = 10; // DAC max rail values
double rail2_max = 10;
double rail3_max = 10;
double rail4_max = 10;

double lock1_precision = 0.0001; // error bound to qualify as "locked"
double lock2_precision = 0.0001;
double lock3_precision = 0.0001;
double lock4_precision = 0.0001;

// Servo Enable + Hold Booleans
int enable1 = 0; // servo enable is set programatically to allow servoing to occur on the channel
int enable2 = 0;
int enable3 = 0;
int enable4 = 0;

uint8_t default_hold = 1;

uint8_t hold1 = default_hold; // Hold Booleans are set by digital input pins 1-4
uint8_t hold2 = default_hold;
uint8_t hold3 = default_hold;
uint8_t hold4 = default_hold;

// Status booleans for the servo lock --> these are NOT stored in memory, but instead are actively overwritten during servo
bool isLock1 = false;
bool isLock2 = false;
bool isLock3 = false;
bool isLock4 = false;

bool railed1 = false;
bool railed2 = false;
bool railed3 = false;
bool railed4 = false;

// Commenting out to untether channels from each other - mg
// Analog tracking and feeding
// Track error differentials
// bool track_error1 = false;
// bool track_error2 = false; // Error on ADC channel X will appear on DAC channel X + 2 (mod 4)
// bool track_error3 = false;
// bool track_error4 = false;

// Feed setpoints
// bool feed_set1 = false;
// bool feed_set2 = false; // The setpoint on ADC channel X will be read in from ADC channel X + 2 (mod 4)
// bool feed_set3 = false;
// bool feed_set4 = false; 

// SWEEP CONFIG
////////////////////////////////////////////////////////////////////////////////////

// Whether to sweep: bool
bool sweep1 = false; // To sweep the output on DAC channel X, set sweepX to TRUE
bool sweep2 = false;
bool sweep3= false;
bool sweep4 = false;

// What kind of sweep
int swp_typ1 = -1; // 0 for sinusoid, 1 for sawtooth, 2 for triangle, -1 for NONE
int swp_typ2 = -1;
int swp_typ3 = -1;
int swp_typ4 = -1;

// Sweep Parameters: common to all, so watch out for that

double amp1 = 0;
double amp2 = 0;
double amp3 = 0;
double amp4 = 0;

double pd1 = 0;
double pd2 = 0;
double pd3 = 0;
double pd4 = 0;

// constexpr double pi() { return std::atan(1) * 4 } // Expression for pi --> alternatively, reference as M_PI

double phi1 = 0;
double phi2 = 0;
double phi3 = 0;
double phi4 = 0;

// Track time over sweep
double t1 = 0;
double t2 = 0;
double t3 = 0;
double t4 = 0;

// Reset Integral Triggers
// Use these to clear integrator value
int reset1 = 0;
int reset2 = 0;
int reset3 = 0;
int reset4 = 0;


////////////////////////////////////////////////////////////////////////////////////
// TIMING MEASUREMENT VARIABLES     ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
// Timing statistics variables
uint32_t isr1_cycles_min = 0xFFFFFFFF;
uint32_t isr1_cycles_max = 0;
uint32_t isr1_count = 0;


uint32_t isr2_cycles_min = 0xFFFFFFFF;
uint32_t isr2_cycles_max = 0;
uint32_t isr2_count = 0;


uint32_t isr3_cycles_min = 0xFFFFFFFF;
uint32_t isr3_cycles_max = 0;
uint32_t isr3_count = 0;


uint32_t isr4_cycles_min = 0xFFFFFFFF;
uint32_t isr4_cycles_max = 0;
uint32_t isr4_count = 0;


// PID Gain Constants (initialization)

// Proportional gain
double p1 = 0;
double p2 = 0;
double p3 = 0;
double p4 = 0;

// Integral gain
double i1 = 0;
double i2 = 0;
double i3 = 0;
double i4 = 0;

// Setpoint
double s1 = 0;
double s2 = 0;
double s3 = 0;
double s4 = 0;

// Differential gain
double d1 = 0;
double d2 = 0;
double d3 = 0;
double d4 = 0;

////////////////////////////////////////////////////////////////////////////////////
// INITIALIZATION               ////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

void setup(void) {
  setDebugWord(0x11110001);
  Serial.begin(115200); //change baud rate based on how often we want to read out values from serial

  ///////////////
  // ENABLE ARM CYCLE COUNTER FOR TIMING MEASUREMENT
  ///////////////
  ARM_DEMCR |= ARM_DEMCR_TRCENA;          // Enable trace
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA; // Enable cycle counter
  //

  ///////////////
  // DIGITAL I/O
  ///////////////

  // Use digital I/O pins 1-4 instead of triggers to activate our servos - that way we can have them all off or have them all simultaneously run
  pinMode(1, INPUT_PULLDOWN); // servo trigger for DAC channel 1 only if enable1 is ON -- pull-up clamps floating values to the logical HIGH (such that the floating hold defaults to default_hold); can be changed by flip_hold()
  pinMode(2, INPUT_PULLDOWN); // servo trigger for DAC channel 2 only if enable2 is ON
  pinMode(3, INPUT_PULLDOWN); // servo trigger for DAC channel 3 only if enable3 is ON
  pinMode(4, INPUT_PULLDOWN); // servo trigger for DAC channel 4 only if enable4 is ON

  // Use digital I/O pins 5-8 as the outputs to pins 1-4 above. This will allow the HOLD state to be implemented entirely natively rather than rely on external equipment
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  setDebugWord(0x11110002);

  // Initialize sq[] baseline before reading NVM
  // Overwrite with values from NVM (if present)
  readSquareFromNVM();
  initSquareDefaultsIfNeeded();


  // start shared square wave timer once
  SQ_Timer.begin(updateSquares, sq_update_us);

    ///////////////
  // SQUARE WAVE
  ///////////////

  for (int ch = 0; ch < 4; ch++) {
    sq[ch].enabled   = false;
    sq[ch].mode      = MODE_SERVO;
    sq_mode_int[ch]  = 0;
    sq[ch].freq_Hz   = 2000.0f;
    sq[ch].duty      = 0.5f;
    sq[ch].amp_scale = 1.0f;

    sq[ch].vpp_set   = 5.0f;
    sq[ch].vpp_gain  = 0.05f;
    sq[ch].meas_min  =  1e9f;
    sq[ch].meas_max  = -1e9f;
    sq[ch].meas_count = 0;
    sq[ch].meas_window_samples = 500; //1 period, 2 kHz: T = 1/2000 = 500 us

    sq[ch].count = 0;
    sq[ch].steps_per_period = 0;

    sq[ch].value = 0.0f;
  }


  ///////////////
  // READ MEMORY
  ///////////////

  // Set the PID gain constants based on the ones stored in memory from the last run, in case of reboot/reflash/power cycling
  // Rewriting a gain constant or boolean value will also rewrite it in memory
  // Note that there are 768 pages, each page length is 128 bytes, and there are a total of 98,304 bytes of storage (96 kB), so the index runs from 0 to 98,304

  // Read in the proportional gain constants
  readNVMblock(&p1, sizeof(p1), 0 * 128);
  readNVMblock(&p2, sizeof(p2), 1 * 128);
  readNVMblock(&p3, sizeof(p3), 2 * 128);
  readNVMblock(&p4, sizeof(p4), 3 * 128);

  setDebugWord(0x11110003);

  // Read in the integral gain constants
  readNVMblock(&i1, sizeof(i1), 4 * 128);
  readNVMblock(&i2, sizeof(i2), 5 * 128);
  readNVMblock(&i3, sizeof(i3), 6 * 128);
  readNVMblock(&i4, sizeof(i4), 7 * 128);

  setDebugWord(0x11110004);

  // Read in the differential gain constants
  readNVMblock(&d1, sizeof(d1), 8 * 128);
  readNVMblock(&d2, sizeof(d2), 9 * 128);
  readNVMblock(&d3, sizeof(d3), 10 * 128);
  readNVMblock(&d4, sizeof(d4), 11 * 128);

  setDebugWord(0x11110005);

  // Read in the setpointss
  readNVMblock(&s1, sizeof(s1), 12 * 128);
  readNVMblock(&s2, sizeof(s2), 13 * 128);
  readNVMblock(&s3, sizeof(s3), 14 * 128);
  readNVMblock(&s4, sizeof(s4), 15 * 128);

  setDebugWord(0x11110006);

  // Read in the channel voltage offsets
  readNVMblock(&offset1, sizeof(offset1), 16 * 128);
  readNVMblock(&offset2, sizeof(offset2), 17 * 128);
  readNVMblock(&offset3, sizeof(offset3), 18 * 128);
  readNVMblock(&offset4, sizeof(offset4), 19 * 128);

  setDebugWord(0x11110007);

  // Read in the ADC configuration setting
  readNVMblock(&config, sizeof(config), 20 * 128);
  setConfig(config);

  setDebugWord(0x11110008);

  // Read in the enable, track, feed, and sweep boolean status on each channel
  readNVMblock(&enable1, sizeof(enable1), 25 * 128);
  readNVMblock(&enable2, sizeof(enable2), 26 * 128);
  readNVMblock(&enable3, sizeof(enable3), 27 * 128);
  readNVMblock(&enable4, sizeof(enable4), 28 * 128);

  //setDebugWord(0x11110009);

  //readNVMblock(&track_error1, sizeof(track_error1), 29 * 128);
  //readNVMblock(&track_error2, sizeof(track_error2), 30 * 128);
  //readNVMblock(&track_error3, sizeof(track_error3), 31 * 128);
  //readNVMblock(&track_error4, sizeof(track_error4), 32 * 128); -mg

  //setDebugWord(0x1111000A);

  //readNVMblock(&feed_set1, sizeof(feed_set1), 33 * 128);
  //readNVMblock(&feed_set2, sizeof(feed_set2), 34 * 128);
  //readNVMblock(&feed_set3, sizeof(feed_set3), 35 * 128);
  //readNVMblock(&feed_set4, sizeof(feed_set4), 36 * 128); -mg

  setDebugWord(0x1111000B);

  readNVMblock(&sweep1, sizeof(sweep1), 37 * 128);
  readNVMblock(&sweep2, sizeof(sweep2), 38 * 128);
  readNVMblock(&sweep3, sizeof(sweep3), 39 * 128);
  readNVMblock(&sweep4, sizeof(sweep4), 40 * 128);

  setDebugWord(0x1111000C);

  // Read in the sweeping parameters
  readNVMblock(&amp1, sizeof(amp1), 41 * 128);
  readNVMblock(&amp2, sizeof(amp2), 42 * 128);
  readNVMblock(&amp3, sizeof(amp3), 43 * 128);
  readNVMblock(&amp4, sizeof(amp4), 44 * 128);

  setDebugWord(0x1111000D);

  readNVMblock(&pd1, sizeof(pd1), 45 * 128);
  readNVMblock(&pd2, sizeof(pd2), 46 * 128);
  readNVMblock(&pd3, sizeof(pd3), 47 * 128);
  readNVMblock(&pd4, sizeof(pd4), 48 * 128);

  setDebugWord(0x1111000E);

  readNVMblock(&phi1, sizeof(phi1), 49 * 128);
  readNVMblock(&phi2, sizeof(phi2), 50 * 128);
  readNVMblock(&phi3, sizeof(phi3), 51 * 128);
  readNVMblock(&phi4, sizeof(phi4), 52 * 128);

  setDebugWord(0x1111000F);

  readNVMblock(&swp_typ1, sizeof(swp_typ1), 53 * 128);
  readNVMblock(&swp_typ2, sizeof(swp_typ2), 54 * 128);
  readNVMblock(&swp_typ3, sizeof(swp_typ3), 55 * 128);
  readNVMblock(&swp_typ4, sizeof(swp_typ4), 56 * 128);

  setDebugWord(0x11110010);

  readNVMblock(&adc1_range, sizeof(adc1_range), 57 * 128);
  readNVMblock(&adc2_range, sizeof(adc2_range), 58 * 128);
  readNVMblock(&adc3_range, sizeof(adc3_range), 59 * 128);
  readNVMblock(&adc4_range, sizeof(adc4_range), 60 * 128);

  setDebugWord(0x11110011);

  readNVMblock(&rail1_min, sizeof(rail1_min), 61 * 128);
  readNVMblock(&rail2_min, sizeof(rail2_min), 62 * 128);
  readNVMblock(&rail3_min, sizeof(rail3_min), 63 * 128);
  readNVMblock(&rail4_min, sizeof(rail4_min), 64 * 128);

  setDebugWord(0x11110012);

  readNVMblock(&rail1_max, sizeof(rail1_max), 65 * 128);
  readNVMblock(&rail2_max, sizeof(rail2_max), 66 * 128);
  readNVMblock(&rail3_max, sizeof(rail3_max), 67 * 128);
  readNVMblock(&rail4_max, sizeof(rail4_max), 68 * 128);

  setDebugWord(0x11110013);

  readNVMblock(&lock1_precision, sizeof(lock1_precision), 69 * 128);
  readNVMblock(&lock2_precision, sizeof(lock2_precision), 70 * 128);
  readNVMblock(&lock3_precision, sizeof(lock3_precision), 71 * 128);
  readNVMblock(&lock4_precision, sizeof(lock4_precision), 72 * 128);

  setDebugWord(0x11110014);

  readNVMblock(&default_hold, sizeof(default_hold), 73 * 128);


  ///////////////
  // SET METHODS
  ///////////////

  setDebugWord(0x11110015);

  // Integral Resets
  qC.assignVariable("reset1",&reset1); //example: to reset the integral on channel 2, type "reset2 1"
  qC.assignVariable("reset2",&reset2);
  qC.assignVariable("reset3",&reset3);
  qC.assignVariable("reset4",&reset4);

  // READ-ONLY Servo I/O
  qC.assignVariable("input1", &in1); //input to channel
  qC.assignVariable("input2", &in2);
  qC.assignVariable("input3", &in3);
  qC.assignVariable("input4", &in4);

  qC.assignVariable("output1", &out1); //output from channel
  qC.assignVariable("output2", &out2);
  qC.assignVariable("output3", &out3);
  qC.assignVariable("output4", &out4);

  qC.assignVariable("error1", &err1); //error differential between input and setpoint
  qC.assignVariable("error2", &err2);
  qC.assignVariable("error3", &err3);
  qC.assignVariable("error4", &err4);

  //Square wave variables
  for (int ch = 0; ch < 4; ch++) {

    qC.assignVariable("enabled" + ch, &sq[ch].enabled);
    qC.assignVariable("frequency" + ch, &sq[ch].freq_Hz);
    qC.assignVariable("duty" + ch, &sq[ch].duty);
    qC.assignVariable("scaledamplitude" + ch, &sq[ch].amp_scale);

    String name = "mode" + String(ch);  // e.g. "mode0", "mode1"
    qC.assignVariable(name.c_str(), &sq_mode_int[ch]);

    qC.assignVariable("vppset" + ch, &sq[ch].vpp_set);
    qC.assignVariable("vppgain" + ch, &sq[ch].vpp_gain);
    qC.assignVariable("measuredminvoltage" + ch, &sq[ch].meas_min);
    qC.assignVariable("measuredmaxvoltage" + ch, &sq[ch].meas_max);
    qC.assignVariable("measuredcount" + ch, &sq[ch].meas_count);
    qC.assignVariable("measuredsamples" + ch, &sq[ch].meas_window_samples);

    qC.assignVariable("idealcount" + ch, &sq[ch].count);
    qC.assignVariable("stepsperperiod" + ch, &sq[ch].steps_per_period);

    qC.assignVariable("value" + ch, &sq[ch].value);
    
    //qC.assignVariable(&sq[ch].duty,      sizeof(sq[ch].duty),      (base + 1) * 128);
    //qC.assignVariable(&sq[ch].vpp_set,   sizeof(sq[ch].vpp_set),   (base + 2) * 128);
    //qC.assignVariable(&sq[ch].amp_scale, sizeof(sq[ch].amp_scale), (base + 3) * 128);
  }

  // COMMANDS
  setDebugWord(0x11110016);
  // Interact with serial monitor
  qC.addCommand("hello",&hello);
  qC.addCommand("hi", &hello);
  qC.addCommand("help", &help);
  qC.addCommand("SN", &get_SN);
  qC.addCommand("ver", &getVersion);

  // qC.addCommand("start_adcs", &start_adcs);
  qC.addCommand("isr_count", &isr_count);  // ← ADD THIS

  //square wave functions
  qC.addCommand("toggle", &toggleSquareWave);
  qC.addCommand("freq",&setFrequencySquare);
  qC.addCommand("duty",&setDutyCycleSquare);


  // Reading on ADC channels
  qC.addCommand("servos", &read_servos);
  qC.addCommand("holds", &holds);
  qC.addCommand("get",&get_gain);
  qC.addCommand("input", &readADC);
  qC.addCommand("output", &getOutput);
  qC.addCommand("error", &getError);
  qC.addCommand("flip", &flip_hold);
  qC.addCommand("locks", &locks);
  
  
  // Writing to IO channels
  qC.addCommand("stop", &write_val);
  qC.addCommand("start", &unwrite_val);
  qC.addCommand("zero", &zero);
  qC.addCommand("force", &force);
  qC.addCommand("check", &check_hold);

  // Status updates
  qC.addCommand("railed", &isRailed);
  qC.addCommand("rails", &setRails);
  qC.addCommand("extremes", &getRails);
  qC.addCommand("locked", &isLocked);
  qC.addCommand("precision", &setLockPrecision);
  qC.addCommand("howPrecise", &getLockPrecision);
  qC.addCommand("test_timing", &test_timing);

  // Setting PID and sweep parameters
  qC.addCommand("prop", &setProp);
  qC.addCommand("int", &setInt);
  qC.addCommand("diff", &setDiff);
  qC.addCommand("set", &setSetpt);

  qC.addCommand("amp", &setAmplitude);
  qC.addCommand("pd", &setPeriod);
  qC.addCommand("phi", &setPhase);
  qC.addCommand("offset", &setOffset); // default value on channel, added no matter what mode

  // Specify inputs and outputs for PID channels
  qC.addCommand("enable", &enable); // channel enable/disable
  // qC.addCommand("track", &track); // track error / stop tracking error of this channel -mg
  //qC.addCommand("feed", &feed);  // feed setpoint / stop feeding setpoint to this channel -mg
  //qC.addCommand("sweep", &sweep);  // sweep output / stop sweeping output of this channel -mg
  qC.addCommand("sweepType", &sweepType); // set the sweep type (sinusoid, sawtooth, or triangle)
  qC.addCommand("getSweep", &get_sweep); // return the sweep parameters
  qC.addCommand("is", &isTogg);

  // ADC configuration
  qC.addCommand("setConfig", &ADCconfig);
  qC.addCommand("drop", &dropADC);
  qC.addCommand("config", &get_config);
  qC.addCommand("res", &res);
  qC.addCommand("range", &setRange);

  // Memory
  qC.addCommand("erase", &eraseNVM);
  qC.addCommand("reveal", &revealMemory);
  qC.addCommand("nvmdump", dumpNVM);

  // Timing measurement
  qC.addCommand("timing", &timing_stats);
  qC.addCommand("reset_timing", &reset_timing);
  qC.addCommand("save", &save_params);
  qC.addCommand("rescue", &save_params);

  // ← END OF ADDITION ↑
  // Default (accident)
  qC.addCommand("", accident);

}





////////////////////////////////////////////////////////////////////////////////////
// TIMING //////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

void test_timing(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFFFFFF);
  
  float cpu_mhz = F_CPU / 1000000.0f;
  
  S.printf("Channel 1 ISR Timing (%u samples):\n", isr1_count);
  S.printf("Min: %.2f us\n", isr1_cycles_min / cpu_mhz);
  if (isr1_count > 0) {
    S.printf("Avg: %.2f us\n", (isr1_cycles_sum / (float)isr1_count) / cpu_mhz);
  } else {
    S.printf("Avg: 0.00 us\n");
  }
  S.printf("Max: %.2f us\n", isr1_cycles_max / cpu_mhz);
  S.println("  ");
  
  S.printf("Channel 2 ISR Timing (%u samples):\n", isr2_count);
  S.printf("Min: %.2f us\n", isr2_cycles_min / cpu_mhz);
  if (isr2_count > 0) {
    S.printf("Avg: %.2f us\n", (isr2_cycles_sum / (float)isr2_count) / cpu_mhz);
  } else {
    S.printf("Avg: 0.00 us\n");
  }
  S.printf("Max: %.2f us\n", isr2_cycles_max / cpu_mhz);
  S.println("  ");
  
  S.printf("Channel 3 ISR Timing (%u samples):\n", isr3_count);
  S.printf("Min: %.2f us\n", isr3_cycles_min / cpu_mhz);
  if (isr3_count > 0) {
    S.printf("Avg: %.2f us\n", (isr3_cycles_sum / (float)isr3_count) / cpu_mhz);
  } else {
    S.printf("Avg: 0.00 us\n");
  }
  S.printf("Max: %.2f us\n", isr3_cycles_max / cpu_mhz);
  S.println("  ");
  
  S.printf("Channel 4 ISR Timing (%u samples):\n", isr4_count);
  S.printf("Min: %.2f us\n", isr4_cycles_min / cpu_mhz);
  if (isr4_count > 0) {
    S.printf("Avg: %.2f us\n", (isr4_cycles_sum / (float)isr4_count) / cpu_mhz);
  } else {
    S.printf("Avg: 0.00 us\n");
  }
  S.printf("Max: %.2f us\n", isr4_cycles_max / cpu_mhz);
  S.println("  ");
  
  S.printf("Budget (t_res): %.2f us\n", t_res);
}


////////////////////////////////////////////////////////////////////////////////////
// LED CONFIG               ////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

void loop(void) {

  setDebugWord(0xFFFF0001);
  static unsigned long lastrun = 0;
  qC.readSerial(Serial);

  for (int ch = 0; ch < 4; ch++) {
  syncSqEnumFromModeInt(ch);
  }

  // We should probably get rid of this atp
  if (millis() > lastrun) {
    lastrun = millis() + 500; //ms
    purple = !purple;
    if (purple) {
      // toggleLEDBlue();
      // toggleLEDRed();
      // int running = 0;
    } else {
      // toggleLEDGreen();
      // toggleLEDRed();
      // int running = 1;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////
// TEST FUNCTIONS FOR SERIAL INTERFACE          ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

void hello(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFF0002);
  if (qC.next() == NULL) {
    S.println("Hello.");
  } else {
    S.printf("Hello %s, it is nice to meet you.",qC.current());    
  }
}

void help(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFF0003);
  // type "help" in the serial cmd line to view all defined commands for this stream
    S.println("Available commands are:");
    qC.printAvailableCommands(S);
}

// ← ADD THIS NEW FUNCTION HERE ↓
//void start_adcs(qCommand& qC, Stream& S) {
//  startADCs();
//  S.println("ADCs started");
//}
void get_SN(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFF0004);
  S.printf("%i-%i\n", getSerialNumber(), getHardwareID());
}
// ← ADD THIS ↓
void isr_count(qCommand& qC, Stream& S) {
  S.printf("ISR entry counts:\n");
  S.printf("  Channel 1: %u\n", isr1_entry_count);
  S.printf("  Channel 2: %u\n", isr2_entry_count);
  S.printf("  Channel 3: %u\n", isr3_entry_count);
  S.printf("  Channel 4: %u\n", isr4_entry_count);
}
// ← END ↑
void getVersion(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFF0005);
  S.println(version);
}

void accident(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFF0006);
  // if something is accidentally sent
  S.println("You didn't type a command!");
}

///////////////////////////////////////////
// Read/Write to pins to trigger servo
///////////////////////////////////////////

void write_val(qCommand& qC, Stream& S) { //debugging method intended to investigate integrator hold on channel X by triggering input pin X with input pin (X + 4)
  // Turn output pin(s) [5,6,7,8] to default_hold
  // serial command is stop X (for pin X)
  setDebugWord(0xFFFF0007);
  int val;
  if (default_hold == 1) {
    val = HIGH;
  } else {
    val = LOW;
  }
  // Corrected Null
  char* chanStr = qC.next();
  if (chanStr != NULL) {  // ← Check pointer, not integer!
    int pinOut = atoi(chanStr) + 4;
    digitalWrite(pinOut, val);
    S.printf("Wrote %i to PIN %i\n", default_hold, pinOut);
  } else {
    digitalWrite(5, val);
    digitalWrite(6, val);
    digitalWrite(7, val);
    digitalWrite(8, val);
    S.printf("Wrote %i to all PINs\n", default_hold);
  }
}

void unwrite_val(qCommand& qC, Stream& S) { //debugging method intended to investigate integrator hold on channel X by triggering input pin X with input pin (X + 4)
  // Turn output pin(s) [5,6,7,8] to !default_hold
  // serial command is start X (for pin X)
  setDebugWord(0xFFFF0008);
  int val;
  if (default_hold == 1) {
    val = LOW;
  } else {
    val = HIGH;
  }
  
  char* chanStr = qC.next();
  if (chanStr != NULL) {  // ← Check pointer, not integer!
    int pinOut = atoi(chanStr) + 4;
    digitalWrite(pinOut, val);
    S.printf("Wrote %i to PIN %i\n", !default_hold, pinOut);
  } else {
    digitalWrite(5, val);
    digitalWrite(6, val);
    digitalWrite(7, val);
    digitalWrite(8, val);
    S.printf("Wrote %i to all PINs\n", !default_hold);
  }
}

void force(qCommand& qC, Stream& S) {
  // Force output on DAC
  // Won't work if either servo or auxiliary tracking are also occuring, of course
  setDebugWord(0xFFFF0009);
  int chan = atoi(qC.next());
  double val = atof(qC.next());
  writeDAC(chan, val);
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
// COMMAND STREAM FUNCTIONS                    /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////


// TO RUN: type [COMMAND][space][ARG1][space][ARG2]...


////////////////////////////////////////////////////////////////////////////////////
// PARAMETER SETUP AND READINGS             ////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

// Note: the functions below read from and write to memory. Be careful with data types, etc.

// ADC configuration

void setConfig(uint8_t channels) {
    // Set or reset the ADC configuration to optiomize bandwidth
    // NOTE: The input (channels) will be a (hex) integer which is read as an 8-bit binary value. This byte serves as a mask to determine which bits to turn on.
    // For example... Let channels = 11, which is 0x1A in hex or 00001011 in binary. Then 0x07 & 0x01 = (00001011) & (00000001) = 1, so channel 1 is ON
    //                                                                                    0x1A & 0x02 = (00001011) & (00000010) = 1, so channel 2 is ON
    //                                                                                    0x1A & 0x04 = (00001011) & (00000100) = 0, so channel 3 is OFF
    //                                                                                    0x1A & 0x08 = (00001011) & (00001000) = 1, so channel 4 is ON
    // Each of them is active for 8 inputs, inactive for 8 inputs. Turn ALL ON with input 15, ALL OFF with input 0.
    setDebugWord(0x11FF1000);

    uint8_t config_mask = channels; // preserve input for the masking later
    // Find number of active channels based on input
    int numActive = 0;
    while (config_mask) { // config_mask will deprecate by a bit each iteration (for at most 4 iterations)
      numActive += config_mask & 1;
      config_mask = config_mask >> 1; // bit shift
    }
    t_res = 1.0 * numActive; // resolution time in us, stored as global variable
    double t_div = t_res / numActive; // divide time evenly between active ADC channels
    

    // Configure ADCs desired

    // configureADC(Channel Number, Time Between Measurements (us), Time Delay (us), Voltage Range (V), callback function)
    // Trigger Note: callback function will only execute PID control output on DAC channel X when BOTH bool enableX and NOT input pin X return 1
    //                  ELSE if enableX is OFF then GET function will write the default (null) voltage value to the channel
    //                  ELSE if enableX is ON and NOT input pin X is OFF (i.e. input pin X is ON) then GET function will not overwrite and channel will perform integrator HOLD
  
    // Sampling Note: to get one sample per us, we set the Time Delay to 0, 1, 2, 3 us respectively such that they each happen 1 us apart and the cycle repeats every 4 us

    double order = 0.0;
    // ADC 1
    if ((int)channels & 0x1) { // 00000001 - ADC 1 is ACTIVE for inputs 1, 3, 5, 7, 9, 11, 13, 15
      configureADC(1,t_res, order * t_div, adc1_range, getADC1); // ADC1 will always be 1st in order (if used)
      order += 1.0;
      setDebugWord(0x11FF1001);
    }
    // ADC 2
    if ((int)channels & 0x2) { // 00000010 - ADC 2 is ACTIVE for inputs 2, 3, 6, 7, 10, 11, 14, 15
      configureADC(2,t_res, order * t_div, adc2_range, getADC2);
      order += 1.0;
      setDebugWord(0x11FF1002);
    }
    // ADC 3
    if ((int)channels & 0x4) { // 00000100 - ADC 3 is ACTIVE for inputs 4, 5, 6, 7, 12, 13, 14, 15
      configureADC(3,t_res, order * t_div, adc3_range, getADC3);
      order += 1.0;
      setDebugWord(0x11FF1003);
    }
    // ADC 4
    if ((int)channels & 0x8) { // 00001000 - ADC 4 is ACTIVE for inputs 8, 9, 10, 11, 12, 13, 14, 15
      configureADC(4,t_res, order * t_div, adc4_range, getADC4); // ADC4 will always be last in order (if used)
      setDebugWord(0x11FF1004);
    }
    setDebugWord(0x11FF1005);
    writeNVMpages(&channels, sizeof(channels), 20); // to remember configuration
    setDebugWord(0x11FF1006);
}

void ADCconfig(qCommand& qC, Stream& S) {
  // Take in a four digits and convert it to an unsigned 8-bit integer for configuration
  // referenced as "setconfig _ _ _ _" with 0 ==> OFF and 1 ==> ON
  setDebugWord(0xFFFF1000);
  uint8_t newConfig = 0x00;
  if (atoi(qC.next()) == 1) {
    newConfig += 1;
  }
  if (atoi(qC.next()) == 1) {
    newConfig += 2;
  }
  if (atoi(qC.next()) == 1) {
    newConfig += 4;
  }
  if (atoi(qC.next()) == 1) {
    newConfig += 8;
  }
  setConfig(newConfig);
  config = newConfig;
  setDebugWord(0xFFFF1001);
  S.printf("Config mask %u\n", config);
}

void setRange(qCommand& qC, Stream& S) {
  // change the input voltage range of the ADC... valid options are: +/- {1.25V, 2.5V, 5V, 10V}
  setDebugWord(0xFFFF000A);
  int chan = atoi(qC.next());
  float abs_max = atof(qC.next());

  int numActive = 0;
  float order = 0;
  uint8_t current_config = config;
    while (current_config) { // config will deprecate by a bit each iteration (for at most 4 iterations)
      numActive += current_config & 1;
      current_config = current_config >> 1; // bit shift
    }
    double t_div = t_res / numActive; // divide time evenly between active ADC channels

  float order_step = 4.0 / numActive;
  adc_scale_t bp_range; // bipolar range -- adc_scale_t is an ENUM type
  
  if (abs_max == 1.25) {
          bp_range = BIPOLAR_1250mV;
      } else if (abs_max == 2.5) {
          bp_range = BIPOLAR_2500mV;
      } else if (abs_max == 5.0) {
          bp_range = BIPOLAR_5V;
      } else if (abs_max == 10.0) {
          bp_range = BIPOLAR_10V;
      } else {
        bp_range = BIPOLAR_10V; // default behavior
  }
  
  setDebugWord(0xFFFF1002);
  switch (chan) {
    case 1:
      configureADC(1, t_res, order * t_div, bp_range, getADC1);
      break;
    case 2:
      if ((int)config & 0x1) {
        order += order_step;
      } else {order = 0;}
      configureADC(2,t_res, order * t_div, bp_range, getADC2);
      break;
    case 3:
      if ((int)config & 0x1 && (int)config & 0x2) {
        order += 2 * order_step;
      } else if ((int)config & 0x1 || (int)config & 0x2){
        order += order_step;
      } else {order = 0;}
      configureADC(3,t_res, order * t_div, bp_range, getADC3);
      break;
    case 4:
      if ((int)config & 0x1 && (int)config & 0x2 && (int)config & 0x4) {
        order += 3 * order_step;
      } else if ( ((int)config & 0x1 && (int)config & 0x2) || ((int)config & 0x1 && (int)config & 0x4) || ((int)config & 0x2 && (int)config & 0x4) ){
        order += 2 * order_step;
      } else if ((int)config & 0x1 || (int)config & 0x2 || (int)config & 0x4) {
        order += order_step;
      } else {order = 0;}
      configureADC(4,t_res, order * t_div, bp_range, getADC4);
      break;
  }
  setDebugWord(0xFFFF000B);
  writeNVMpages(&bp_range, sizeof(bp_range), (uint16_t) (chan + 56)); // store the bp_range in memory
  S.printf("ADC %i input range is now %f V\n", chan, abs_max);
}

void res(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFF000C);
  // print ADC resolution - referenced as "res"
  S.printf("%f\n", t_res);
}

void dropADC(qCommand& qC, Stream& S) {
  // Drop an ADC channel while servoing - should not interrupt servo process
  // Referenced as "drop [channel #]"
  setDebugWord(0xFFFF000D);
  // Disable channel
  int channel = atoi(qC.next());
  disableADC(channel);
  setDebugWord(0xFFFF1003);
  // Remove channel from the config in memory
  uint8_t config_drop = pow(2, channel - 1);
  if (config & config_drop) {
    config = config - config_drop;
    S.printf("Current config mask: %u", config);
    writeNVMpages(&config, sizeof(config), 20); // to remember configuration
  }
}

void get_config(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFF000E);
  // Return the configuration state to serial window
  uint8_t currentConfig = config; // from local variables (dynamic memory)
  // uint8_t currentConfig; // from NVM memory
  // readNVMblock(&currentConfig, sizeof(currentConfig), 20 * 128);
  S.println(currentConfig);
  S.printf("%d, %d, %d, %d\n", bool(config & 0x00000001), bool(config & 0x00000002), bool(config & 0x00000004), bool(config & 0x00000008));
}

// Servo + Hold status

void read_servos(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFF000F);
  // Read out state of servos from input pins [1,2,3,4]
  S.printf("%d, %d, %d, %d\n", (enable1 && !bool(hold1) && bool(config & 0x00000001)), (enable2 && !bool(hold2) && bool(config & 0x00000002)), (enable3 && !bool(hold3) && bool(config & 0x00000004)), (enable4 && !bool(hold4) && bool(config & 0x00000008)));
  if (enable1 && !hold1 && (config & 0x00000001)) {
    S.print("Servoing on channel 1\n");
  }
  if (enable2 && !hold2 && (config & 0x00000002)) {
    S.print("Servoing on channel 2\n");
  }
  if (enable3 && !hold3 && (config & 0x00000004)) {
    S.print("Servoing on channel 3\n");
  }
  if (enable4 && !hold4 && (config & 0x00000008)) {
    S.print("Servoing on channel 4\n");
  }
  if ((!enable1 || bool(hold1) || !(config & 0x00000001)) && (!enable2 || bool(hold2) || !(config & 0x00000002)) && (!enable3 || bool(hold3) || !(config & 0x00000004)) && (!enable4 || bool(hold4) || !(config & 0x00000008))) {
    S.print("Not servoing currently\n"); // All channels are currently being held (sequence) OR servoing is currently disabled (user) OR the ADCs are not even configured to read anything to begin with (user/config)
  }
}

void holds(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFF0010);
  // referenced as "holds"
  S.printf("%i, %i, %i, %i\n", hold1, hold2, hold3, hold4);
  // S.printf("%i, %i, %i, %i\n", !digitalRead(1), !digitalRead(2), !digitalRead(3), !digitalRead(4));
  // S.printf("default hold: %i\n", default_hold);
}

void locks(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFF0011);
  // referenced as "locks"
  S.printf("%i, %i, %i, %i\n", isLock1, isLock2, isLock3, isLock4);
}

void flip_hold(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFF0012);
  // referenced as "flip"
  if (default_hold == 0) {
      default_hold = 1;
   } else {
      default_hold = 0;
   }
   writeNVMpages(&default_hold, sizeof(default_hold), 73);
   S.printf("default hold logic level is now %i\n", default_hold);
}

void setRails(qCommand& qC, Stream& S) {
  // referenced as "rails"
  // set or reset the rails for the ADC specified (minimum and maximum voltages)
  setDebugWord(0xFFFF0013);
  int chan = atoi(qC.next());
  double min = atof(qC.next());
  double max = atof(qC.next());

  switch (chan) {
    case 1:
      rail1_min = min;
      rail1_max = max;
      break;
    case 2:
      rail2_min = min;
      rail2_max = max;
      break;
    case 3:
      rail3_min = min;
      rail3_max = max;
      break;
    case 4:
      rail4_min = min;
      rail4_max = max;
      break;
  }
  writeNVMpages(&min, sizeof(min), 60 + chan);
  writeNVMpages(&max, sizeof(max), 64 + chan);

  S.printf("Rails on channel %i are now %f and %f\n", chan, min, max);
}

//Allows us to set the rails to be bipolar (cause I'm too lazy to have 2 inputs if theyre gonna be the exact same but opposite in sign)

void setRailsBipolar(qCommand& qC, Stream& S) {
  // referenced as "rails"
  // set or reset the rails for the ADC specified (minimum and maximum voltages)
  setDebugWord(0xFFFF0013);

  char* arg1 = qC.next();
  char* arg2 = qC.next();
  
  if (!arg1 || !arg2) {
    S.println("Syntax: brails <chan> <rail>");
  }

  int chan = atoi(qC.next());
  double rail = atof(qC.next());

  switch (chan) {
    case 1:
      rail1_min = -rail;
      rail1_max = rail;
      break;
    case 2:
      rail2_min = -rail;
      rail2_max = rail;
      break;
    case 3:
      rail3_min = -rail;
      rail3_max = rail;
      break;
    case 4:
      rail4_min = -rail;
      rail4_max = rail;
      break;
  }
  writeNVMpages(&rail, sizeof(rail), 60 + chan);

  S.printf("Rails on channel %i are now %f and %f\n", chan, -rail, rail);
}

void getRails(qCommand& qC, Stream& S) {
  // referenced as "extremes"
  setDebugWord(0xFFFF0014);
  int chan = atoi(qC.next());
  double mini = 0;
  double maxi = 0;

  switch (chan) {
    case 1:
      mini = rail1_min;
      maxi = rail1_max;
      break;
    case 2:
      mini = rail2_min;
      maxi = rail2_max;
      break;
    case 3:
      mini = rail3_min;
      maxi = rail3_max;
      break;
    case 4:
      mini = rail4_min;
      maxi = rail4_max;
      break;
  }
  S.printf("%f %f\n", mini, maxi);
}

// Status updates for passive servo monitoring

void isRailed(qCommand& qC, Stream& S) {
  // referenced as "railed"
  setDebugWord(0xFFFF0015);
  int channel = atoi(qC.next());
  bool whether = false;
  switch (channel) {
    case 1:
      whether = railed1;
      break;
    case 2:
      whether = railed2;
      break;
    case 3:
      whether = railed3;
      break;
    case 4:
      whether = railed4;
      break;
  }
  S.printf("%i\n", whether);
}

void isLocked(qCommand& qC, Stream& S) {
  // referenced as "locked"
  setDebugWord(0xFFFF0016);
  int channel = atoi(qC.next());
  bool whether = false;
  switch (channel) {
    case 1:
      whether = isLock1;
      break;
    case 2:
      whether = isLock2;
      break;
    case 3:
      whether = isLock3;
      break;
    case 4:
      whether = isLock4;
      break;
  }
  S.printf("%i\n", whether);
}

void setLockPrecision(qCommand& qC, Stream& S) {
  // referenced as "precision"
  setDebugWord(0xFFFF0017);
  int chan = atoi(qC.next());
  double prec = atof(qC.next());

  switch (chan) {
    case 1:
      lock1_precision = prec;
      break;
    case 2:
      lock2_precision = prec;
      break;
    case 3:
      lock3_precision = prec;
      break;
    case 4:
      lock4_precision = prec;
      break;
  }
  writeNVMpages(&prec, sizeof(prec), 68 + chan);
  S.printf("Channel %i will lock within %f V of setpoint\n", chan, prec);
}

void getLockPrecision(qCommand& qC, Stream& S) {
  // referenced as "howPrecise"
  setDebugWord(0xFFFF0018);
  int chan = atoi(qC.next());
  double prec = -1.0;

  switch (chan) {
    case 1:
      prec = lock1_precision;
      break;
    case 2:
      prec = lock2_precision;
      break;
    case 3:
      prec = lock3_precision;
      break;
    case 4:
      prec = lock4_precision;
      break;
  }
  S.printf("%f\n", prec);
}

// Setting PID and sweep parameters

void setProp(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFF2001);
  // referenced as "prop"
  int channel = atoi(qC.next());
  double p = atof(qC.next());
  switch (channel) {
    case 1:
      p1 = p;
      break;
    case 2:
      p2 = p;
      break;
    case 3:
      p3 = p;
      break;
    case 4:
      p4 = p;
      break;
  }
  writeNVMpages(&p, sizeof(p), channel - 1);
  S.printf("Proportional gain %i changed to %f\n", channel, p);
}


////////////////////////////////////////////////////////////////////////////////////
// SQUARE WAVE CONFIG //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

void computeSquareStepsForChan(int ch) {
  if (sq[ch].freq_Hz <= 0.0f) {
    sq[ch].steps_per_period = 0;
    return;
  }
  float period_us = 1e6f / sq[ch].freq_Hz;
  sq[ch].steps_per_period = (uint32_t)(period_us / sq_update_us + 0.5f);
}

void readSquareFromNVM() {
  for (int ch = 0; ch < 4; ch++) {
    int base = SQ_BASE_PAGE + ch * SQ_PAGES_PER_CH;

    // Only configuration fields go in NVM
    readNVMblock(&sq[ch].enabled,   sizeof(sq[ch].enabled),   (base + 0) * 128);
    readNVMblock(&sq[ch].mode,      sizeof(sq[ch].mode),      (base + 1) * 128);
    readNVMblock(&sq[ch].freq_Hz,   sizeof(sq[ch].freq_Hz),   (base + 2) * 128);
    readNVMblock(&sq[ch].duty,      sizeof(sq[ch].duty),      (base + 3) * 128);
    readNVMblock(&sq[ch].vpp_set,   sizeof(sq[ch].vpp_set),   (base + 4) * 128);
    readNVMblock(&sq[ch].amp_scale, sizeof(sq[ch].amp_scale), (base + 5) * 128);
    readNVMblock(&sq[ch].vpp_gain,  sizeof(sq[ch].vpp_gain),  (base + 6) * 128);

    // runtime-only fields are initialized in RAM:
    sq[ch].meas_min            =  1e9f;
    sq[ch].meas_max            = -1e9f;
    sq[ch].meas_count          = 0;
    sq[ch].meas_window_samples = 500;  // covers many periods at kHz freqs
    sq[ch].count               = 0;
    sq[ch].steps_per_period    = 0;
    sq[ch].value               = 0.0f;
  }
}

void initSquareDefaultsIfNeeded() {
  for (int ch = 0; ch < 4; ch++) {
    // If NVM was empty/uninitialized, give safe defaults
    if (!isfinite(sq[ch].freq_Hz) || sq[ch].freq_Hz <= 0.0f) {
      sq[ch].freq_Hz = 2000.0f;          // 2 kHz
    }
    if (!isfinite(sq[ch].duty) || sq[ch].duty < 0.0f || sq[ch].duty > 1.0f) {
      sq[ch].duty = 0.5f;                // 50%
    }
    if (!isfinite(sq[ch].vpp_set) || sq[ch].vpp_set <= 0.0f) {
      sq[ch].vpp_set = 5.0f;             // 5 Vpp target
    }
    if (!isfinite(sq[ch].amp_scale) || sq[ch].amp_scale <= 0.0f) {
      sq[ch].amp_scale = 1.0f;
    }
    if (!isfinite(sq[ch].vpp_gain)) {
      sq[ch].vpp_gain = 0.05f;
    }

    // If mode in NVM is garbage, default to SERVO
    if (sq[ch].mode != MODE_SERVO &&
        sq[ch].mode != MODE_SWEEP &&
        sq[ch].mode != MODE_SQUARE) {
      sq[ch].mode = MODE_SERVO;
    }
  }
}

void syncSqModeIntFromEnum(int ch) {
  switch (sq[ch].mode) {
    case MODE_SERVO:  sq_mode_int[ch] = 0; break;
    case MODE_SWEEP:  sq_mode_int[ch] = 1; break;
    case MODE_SQUARE: sq_mode_int[ch] = 2; break;
    default:          sq_mode_int[ch] = 0; sq[ch].mode = MODE_SERVO; break;
  }
}

void syncSqEnumFromModeInt(int ch) {
  switch (sq_mode_int[ch]) {
    case 0: sq[ch].mode = MODE_SERVO;  break;
    case 1: sq[ch].mode = MODE_SWEEP;  break;
    case 2: sq[ch].mode = MODE_SQUARE; break;
    default: sq_mode_int[ch] = 0; sq[ch].mode = MODE_SERVO; break;
  }
}

void updateSquares(){
  extern double rail1_min, rail1_max;
  extern double rail2_min, rail2_max;
  extern double rail3_min, rail3_max;
  extern double rail4_min, rail4_max;

  for (int ch = 0; ch < 4; ch++) {
    if (!sq[ch].enabled || sq[ch].mode != MODE_SQUARE)
      continue;

    if (sq[ch].steps_per_period == 0)
      continue;

    uint32_t N  = sq[ch].steps_per_period;
    uint32_t Nh = (uint32_t)(sq[ch].duty * N);
    uint32_t idx = sq[ch].count % N;
    sq[ch].count++;

    float rail_min, rail_max;
    switch (ch) {
      case 0: rail_min = rail1_min; rail_max = rail1_max; break;
      case 1: rail_min = rail2_min; rail_max = rail2_max; break;
      case 2: rail_min = rail3_min; rail_max = rail3_max; break;
      case 3: rail_min = rail4_min; rail_max = rail4_max; break;
    }

    float v_high = rail_max * sq[ch].amp_scale;
    float v_low  = rail_min * sq[ch].amp_scale;

    // store the channel's value to be used later in the getADCx (don't want to have conflicting ISRs)
    sq[ch].value = (idx < Nh) ? v_high : v_low;
  }
}

void toggleSquareWave(qCommand &qC, Stream& S){
  char* arg1 = qC.next();
  char* arg2 = qC.next();
  if (!arg1 || !arg2){
    S.println("Syntax: toggle <ADC input> <0|1>");
    return;
  }

  int channel = atoi(arg1);
  int enabled = atoi(arg2);

  if (channel < 1 || channel > 4) {
    S.println("Channel must be between 1-4");
    return;
  }
  
  int idx = channel - 1; //indexing
  sq[idx].enabled = (enabled != 0);
  sq[idx].mode    = sq[idx].enabled ? MODE_SQUARE : MODE_SERVO;

  syncSqModeIntFromEnum(idx); //keep as a variable for the qC GUI

  // store enabled/mode to NVM
  int base = SQ_BASE_PAGE + idx * SQ_PAGES_PER_CH;
  writeNVMpages(&sq[idx].enabled, sizeof(sq[idx].enabled), base + 0);
  writeNVMpages(&sq[idx].mode,    sizeof(sq[idx].mode),    base + 1);

   S.printf("CH%d square-wave mode: %s\n", channel, sq[idx].enabled ? "ENABLED" : "DISABLED");

}

void setFrequencySquare(qCommand&qC, Stream& S){
  // parse chan, freq, duty, validate...

  char* arg1 = qC.next();
  char* arg2 = qC.next();

  if (!arg1 || !arg2){
    S.println("Syntax: freq <channel> <frequency>");
    return;
  }

  int channel = atoi(arg1);
  double freq = atof(arg2);

  if (channel < 1 || channel > 4) {
    S.println("Channel must be 1–4");
    return;
  }

  int idx = channel - 1;
  sq[idx].freq_Hz = freq;

  // Recompute timing for this channel
  computeSquareStepsForChan(idx);

  // Optionally save freq/duty to NVM
  int base = SQ_BASE_PAGE + idx * SQ_PAGES_PER_CH;
  writeNVMpages(&sq[idx].freq_Hz, sizeof(sq[idx].freq_Hz), base + 2);

  S.printf("CH%d square: freq=%.2f Hz, duty=%.3f\n", channel, freq);
}

void setDutyCycleSquare(qCommand&qC, Stream& S){
  char* arg1 = qC.next();
  char* arg2 = qC.next();
  if (!arg1 || !arg2) {
    S.println("Syntax: duty <chan> <duty_0_1>");
    return;
  }

  int channel  = atoi(arg1);
  float duty = atof(arg2);
  if (channel < 1 || channel > 4) {
    S.println("Channel must be 1–4");
    return;
  }
  if (duty < 0.0f) duty = 0.0f;
  if (duty > 1.0f) duty = 1.0f;

  int idx = channel - 1;
  sq[idx].duty = duty;

  // save to NVM
  int base = SQ_BASE_PAGE + idx * SQ_PAGES_PER_CH;
  writeNVMpages(&sq[idx].duty, sizeof(sq[idx].duty), base + 3);

  S.printf("CH%d square duty set to %.3f\n", channel, duty);
}

void setInt(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFF2002);
  
  // ===== GET CHANNEL WITH NULL CHECK =====
  char* chanStr = qC.next();
  if (chanStr == NULL) {
    S.println("Usage: int <channel> <value>");
    S.println("Example: int 1 0.1");
    return;
  }
  int channel = atoi(chanStr);
  
  // ===== GET VALUE WITH NULL CHECK =====
  char* valueStr = qC.next();
  if (valueStr == NULL) {
    S.println("Usage: int <channel> <value>");
    S.println("Example: int 1 0.1");
    return;
  }
  double i = atof(valueStr);
  
  // ===== VALIDATE CHANNEL =====
  if (channel < 1 || channel > 4) {
    S.println("Error: Channel must be 1-4");
    return;
  }
  
  // ===== VALIDATE VALUE =====
  if (isnan(i) || isinf(i)) {
    S.println("Error: Invalid value");
    return;
  }
  
  // ===== SET VALUE WITH INTERRUPT PROTECTION =====
  noInterrupts();
  switch (channel) {
    case 1:
      i1 = i;
      break;
    case 2:
      i2 = i;
      break;
    case 3:
      i3 = i;
      break;
    case 4:
      i4 = i;
      break;
  }
  interrupts();
  
  // ===== SAVE TO NVM AND CONFIRM =====
  writeNVMpages(&i, sizeof(i), channel + channel + 3);
  S.printf("Integral gain %i changed to %f /us\n", channel, i);
}

void setDiff(qCommand& qC, Stream& S) {
  // referenced as "diff"
  setDebugWord(0xFFFF2003);
  int channel = atoi(qC.next());
  double d = atof(qC.next());
  switch (channel) {
    case 1: 
      d1 = d;
      
      break;
    case 2:
      d2 = d;
      break;
    case 3:
      d3 = d;
      break;
    case 4:
      d4 = d;
      break;
  }
  writeNVMpages(&d, sizeof(d), channel + 7);
  S.printf("Differential gain %i changed to %f us\n", channel, d);
}

//TODO: modify setpoint range for rails

void setSetpt(qCommand& qC, Stream& S) {
  // referenced as "set <"
  setDebugWord(0xFFFF2004);
  
  char* arg1 = qC.next();
  char* arg2 = qC.next();

  if(!arg1 || !arg2){
    S.println("Syntax: set <channel> <voltage setpoint>");
    return;
  } //sees if the arguments match the syntax of the command (prevent a nullptr error)

  int channel = atoi(arg1);
  double s = atof(arg2);
  switch (channel) {
    case 1: 
      s1 = s;
      break;
    case 2:
      s2 = s;
      break;
    case 3:
      s3 = s;
      break;
    case 4:
      s4 = s;
      break;
  }
  writeNVMpages(&s, sizeof(s), channel + 11);
  S.printf("Setpoint %i changed to %f V\n", channel, s);
}

void setAmplitude(qCommand& qC, Stream& S) {
  // referenced as "amp"
  setDebugWord(0xFFFF2005);
  int channel = atoi(qC.next());
  double a = atof(qC.next());
  switch (channel) {
    case 1: 
      amp1 = a;
      break;
    case 2:
      amp2 = a;
      break;
    case 3:
      amp3 = a;
      break;
    case 4:
      amp4 = a;
      break;
  }
  writeNVMpages(&a, sizeof(a), channel + 40);
  S.printf("Sweep amplitude %i changed to %f V\n", channel, a);
}

void setPeriod(qCommand& qC, Stream& S) {
  // referenced as "pd"
  setDebugWord(0xFFFF2006);
  int channel = atoi(qC.next());
  double t = atof(qC.next());
  switch (channel) {
    case 1: 
      pd1 = t;
      break;
    case 2:
      pd2 = t;
      break;
    case 3:
      pd3 = t;
      break;
    case 4:
      pd4 = t;
      break;
  }
  writeNVMpages(&t, sizeof(t), channel + 44);
  S.printf("Sweep period %i changed to %f us\n", channel, t);
}

void setPhase(qCommand& qC, Stream& S) {
  // referenced as "phi"
  setDebugWord(0xFFFF2007);
  int channel = atoi(qC.next());
  double phi = atof(qC.next());
  switch (channel) {
    case 1: 
      phi1 = phi;
      break;
    case 2:
      phi2 = phi;
      break;
    case 3:
      phi3 = phi;
      break;
    case 4:
      phi4 = phi;
      break;
  }
  writeNVMpages(&phi, sizeof(phi), channel + 48);
  S.printf("Sweep phase offset %i changed to %f us\n", channel, phi);
}

void setOffset(qCommand& qC, Stream& S) {
  // referenced as "offset"
  setDebugWord(0xFFFF2008);
  int channel = atoi(qC.next());
  double off = atof(qC.next());
  switch (channel) {
    case 1: 
      offset1 = off;
      break;
    case 2:
      offset2 = off;
      break;
    case 3:
      offset3 = off;
      break;
    case 4:
      offset4 = off;
      break;
  }
  writeNVMpages(&off, sizeof(off), channel + 15);
  S.printf("Channel offset %i changed to %f V\n", channel, off);
}

// State transitions

void enable(qCommand& qC, Stream& S) {
  // referenced as "enable"
  setDebugWord(0xFFFF2009);
  int channel = atoi(qC.next());
  int state = atoi(qC.next());
  switch (channel) {
    case 1:
      enable1 = state;
      break;
    case 2:
      enable2 = state;
      break;
    case 3:
      enable3 = state;
      break;
    case 4:
      enable4 = state;
     break;
  }
  writeNVMpages(&enable1, sizeof(enable1), channel + 24);
  S.printf("Servo enable states: %i %i %i %i\n", enable1, enable2, enable3, enable4);
}

//void track(qCommand& qC, Stream& S) {
  // referenced as "track"
//  setDebugWord(0xFFFF200A);
//  int channel = atoi(qC.next());
//  int state = atoi(qC.next());
//  switch (channel) {
//    case 1:
//      track_error1 = state;
//      break;
//    case 2:
//      track_error2 = state;
//      break;
//    case 3:
//      track_error3 = state;
//      break;
//    case 4:
//      track_error4 = state;
//      break;
//  }
//  writeNVMpages(&state, sizeof(state), channel + 28);
//  S.printf("Tracking error states: %i %i %i %i\n", track_error1, track_error2, track_error3, track_error4);
//} -mg

//void feed(qCommand& qC, Stream& S) {
  // referenced as "feed"
//  setDebugWord(0xFFFF200B);
//  int channel = atoi(qC.next());
//  int state = atoi(qC.next());
//  switch (channel) {
//    case 1:
//      feed_set1 = state;
//      break;
//    case 2:
//      feed_set2 = state;
//      break;
//    case 3:
//      feed_set3 = state;
//      break;
//    case 4:
//      feed_set4 = state;
//      break;
//  }
//  writeNVMpages(&state, sizeof(state), channel + 32);
//  S.printf("Feeding setpoints to servos: %i %i %i %i\n", feed_set1, feed_set2, feed_set3, feed_set4);
//} -mg

void sweep(qCommand& qC, Stream& S) {
  // referenced as "sweep"
  setDebugWord(0xFFFF200C);
  int channel = atoi(qC.next());
  int state = atoi(qC.next());
  switch (channel) {
    case 1:
      sweep1 = state;
      break;
    case 2:
      sweep2 = state;
      break;
    case 3:
      sweep3 = state;
      break;
    case 4:
      sweep4 = state;
      break;
  }
  writeNVMpages(&state, sizeof(state), channel + 36);
  S.printf("Sweeping on DACs: %i %i %i %i\n", sweep1, sweep2, sweep3, sweep4);
}

// Querying booleans and getting parameters

void isTogg(qCommand& qC, Stream& S) {
  // referenced as "is"
  // getting toggle-able states
  setDebugWord(0xFFFF0019);
  int cmd = atoi(qC.next());
  
    if (cmd == 0) {
      S.printf("enabled servo: %i %i %i %i\n", int(enable1), int(enable2), int(enable3), int(enable4));
    } else if (cmd == 1) {
      S.printf("sweep on DAC: %i %i %i %i\n", int(sweep1), int(sweep2), int(sweep3), int(sweep4));
    } else if (cmd == 2) {
      //S.printf("error outputs from servo: %i %i %i %i\n", int(track_error1), int(track_error2), int(track_error3), int(track_error4)); -mg
      S.println("error outputs: REMOVED (channels now independent)");

    } else if (cmd == 3) {
     S.println("setpoint feed: REMOVED (digital setpoints only)");

    } else {
      S.printf("not a valid toggle\n");
    }
}

void get_gain(qCommand& qC, Stream& S) {
  //referenced as "get"
  setDebugWord(0xFFFF200D);
  double p = -1.0;
  double i = -1.0;
  double d = -1.0;
  double s = -1.0;
  
  char* chanStr = qC.next();
  if (chanStr != NULL) {  // ← Check pointer, not integer!
    int chan = atoi(chanStr);
    switch(chan) {
      case 1:
        p = p1; i = i1; d = d1; s = s1;
        break;
      case 2:
        p = p2; i = i2; d = d2; s = s2;
        break;
      case 3:
        p = p3; i = i3; d = d3; s = s3;
        break;
      case 4:
        p = p4; i = i4; d = d4; s = s4;
        break;
    }
    S.printf("Channel %i: prop = %f, int = %f, diff = %f, setpt = %f\n", chan, p, i, d, s);
  } else {
    S.println("Undefined: Specify channel");
  }
}
  // return std::make_tuple(p, i, d, s)



void readADC(qCommand& qC, Stream& S) {
  //Read the voltage value on the specified ADC channel, i.e., current value X being compared to setpt S
  //referenced as "input X" 
  setDebugWord(0xFFFF1004);
  double voltage = 0.0;
  
  char* chanStr = qC.next();
  if (chanStr != NULL) {  // ← Check pointer, not integer!
    int channel = atoi(chanStr);
    
    if (channel == 1) {
      voltage = in1;
    } else if (channel == 2) {
      voltage = in2;
    } else if (channel == 3) {
      voltage = in3;
    } else if (channel == 4) {
      voltage = in4;
    }
    S.printf("Channel %i Voltage: %f\n", channel, voltage);
  } else {
    S.println("Undefined: Specify channel");
  }
}

void getOutput(qCommand& qC, Stream& S) {
  // referenced as "output X"
  setDebugWord(0xFFFF1005);
  double vOut = 0.0;
  
  char* chanStr = qC.next();
  if (chanStr != NULL) {  // ← Check pointer, not integer!
    int channel = atoi(chanStr);
    
    if (channel == 1) {
      vOut = out1 - offset1;
    } else if (channel == 2) {
      vOut = out2 - offset2;
    } else if (channel == 3) {
      vOut = out3 - offset3;
    } else if (channel == 4) {
      vOut = out4 - offset4;
    }
    S.printf("Channel %i Output: %f\n", channel, vOut);
  } else {
    S.println("Undefined: Specify channel");
  }
}

void getError(qCommand& qC, Stream& S) {
  // referenced as "error X"
  setDebugWord(0xFFFF200E);
  double vOut = 0.0;
  
  char* chanStr = qC.next();
  if (chanStr != NULL) {  // ← Check pointer, not integer!
    int channel = atoi(chanStr);
    
    if (channel == 1) {
      vOut = err1;
    } else if (channel == 2) {
      vOut = err2;
    } else if (channel == 3) {
      vOut = err3;
    } else if (channel == 4) {
      vOut = err4;
    }
    S.printf("Channel %i Error: %f\n", channel, vOut);
  } else {
    S.println("Undefined: Specify channel");
  }
}

void forceOutput(int channel, double voltage) {
  setDebugWord(0xFFFF001A);
  writeDAC(channel, voltage);
}

void sweepType(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFF200F);
  int channel = atoi(qC.next());
  int tp = atoi(qC.next());

  if (tp < 0 || tp > 2) {
    S.println("Uncrecognized sweep type");
  }
  else {
    switch (channel) {
      case 1:
        swp_typ1 = tp; 
        break;
      case 2:
        swp_typ2 = tp;
        break;
      case 3:
        swp_typ3 = tp;
        break;
      case 4:
        swp_typ4 = tp;
        break;
      }
    }
  writeNVMpages(&tp, sizeof(tp), channel + 52);

  switch (tp) {
    case 0:
      S.printf("Sweep type on channel %i changed to sinusoid\n", channel);
      break;
    case 1:
      S.printf("Sweep type on channel %i changed to sawtooth\n", channel);
      break;
    case 2:
      S.printf("Sweep type on channel %i changed to triangle\n", channel);
      break;
    default:
      S.printf("Sweep type on channel %i unrecognized: %i\n", channel, tp);
      break;
  }
}

void get_sweep(qCommand& qC, Stream& S) {
  // referenced as getSweep
  // return the sweep parameters for the specified channel
  setDebugWord(0xFFFF2010);
  int channel = atoi(qC.next());

  switch (channel) {
    case 1:
        S.printf("Sweep Channel 1: amp = %f, period = %f, phase = %f, offset = %f, type = %i\n", amp1, pd1, phi1, offset1, swp_typ1);
        break;
    case 2:
        S.printf("Sweep Channel 2: amp = %f, period = %f, phase = %f, offset = %f, type = %i\n", amp2, pd2, phi2, offset2, swp_typ2);
        break;
    case 3:
        S.printf("Sweep Channel 3: amp = %f, period = %f, phase = %f, offset = %f, type = %i\n", amp3, pd3, phi3, offset3, swp_typ3);
        break;
    case 4:
        S.printf("Sweep Channel 4: amp = %f, period = %f, phase = %f, offset = %f, type = %i\n", amp4, pd4, phi4, offset4, swp_typ4);
        break;
  }
}

void zero(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFF1006);
  //set all DACs to 0 - stop all servo output
  //set all output pins to LOW - stop manual trigger
  zeroDACs();
  for (int i = 5; i < 9; i++)
  digitalWrite(i, LOW);
}

void revealMemory(qCommand& qC, Stream& S) {
  // see what is written at some arbitrary page in memory
  setDebugWord(0x1F1F1F1F);
  int pg = atoi(qC.next());

  if ( (pg < 20) || ( (40 < pg) && (pg < 53) ) ||( (60 < pg) && (pg < 73) ) ) { // doubles
    double buff_d;
    readNVMblock(&buff_d, sizeof(buff_d), pg * 128);
    S.printf("%f\n", buff_d);
  } else if ( ( (19 < pg) && (pg < 29) ) || ( (52 < pg) && (pg < 57) ) || pg == 73) { //ints
    int buff_i;
    readNVMblock(&buff_i, sizeof(buff_i), pg * 128);
    S.printf("%i\n", buff_i);    
  } else if ( (28 < pg) && (pg < 41) ) { // bools
    bool buff_b;
    readNVMblock(&buff_b, sizeof(buff_b), pg * 128);
    S.printf("%d\n", buff_b);
  } else if ( (56 < pg) && (pg < 61) ) { // adc_scale_t ENUMs
    adc_scale_t buff_e;
    readNVMblock(&buff_e, sizeof(buff_e), pg * 128);
    S.printf("%i\n", buff_e);    
  } else {S.printf("There is nothing stored in that page in memory\n");}
}

void eraseNVM(qCommand& qC, Stream& S) {
  // reset NVM to default values
  setDebugWord(0x2F2F2F2F);
  for (int i = 0; i < 4; i++) {
    writeNVMpages(&default_p, sizeof(default_p), i);
  }

  for (int i = 4; i < 8; i++) {
    writeNVMpages(&default_i, sizeof(default_i), i);
  }

  for (int i = 8; i < 12; i++) {
    writeNVMpages(&default_d, sizeof(default_d), i);
  }
  
  for (int i = 12; i < 16; i++) {
    writeNVMpages(&default_s, sizeof(default_s), i);
  }

  for (int i = 16; i < 20; i++) {
    writeNVMpages(&default_offset, sizeof(default_offset), i);
  }

  writeNVMpages(&default_config, sizeof(config), 20);

  for (int i = 25; i < 29; i++) {
    writeNVMpages(&default_enable, sizeof(default_enable), i);
  }

  for (int i = 29; i < 33; i++) {
    writeNVMpages(&default_track, sizeof(default_track), i);
  }
  
  for (int i = 33; i < 37; i++) {
    writeNVMpages(&default_feed, sizeof(default_feed), i);
  }
  
  for (int i = 37; i < 41; i++) {
    writeNVMpages(&default_sweep, sizeof(default_sweep), i);
  }
  
  for (int i = 41; i < 44; i++) {
    writeNVMpages(&default_a, sizeof(default_a), i);
  }

  for (int i = 45; i < 49; i++) {
    writeNVMpages(&default_T, sizeof(default_T), i);
  }

  for (int i = 49; i < 53; i++) {
    writeNVMpages(&default_phi, sizeof(default_phi), i);
  }
    
  for (int i = 53; i < 57; i++) {
    writeNVMpages(&default_swp_tp, sizeof(default_swp_tp), i);
  }
      
  for (int i = 57; i < 61; i++) {
    writeNVMpages(&default_adc_range, sizeof(default_adc_range), i);
  }

  for (int i = 61; i < 65; i++) {
    writeNVMpages(&default_rail_min, sizeof(default_rail_min), i);
  }

  for (int i = 65; i < 69; i++) {
    writeNVMpages(&default_rail_max, sizeof(default_rail_max), i);
  }

  for (int i = 69; i < 73; i++) {
    writeNVMpages(&default_lock_precision, sizeof(default_rail_max), i);
  }

  S.printf("Non-volatile memory erase complete\n");
}

void check_hold(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFF001B);
  S.printf("%d\n", digitalRead(atoi(qC.next())));
  
}

  // Get which channel to report (or all if no argument)
  int chan = 0;
void timing_stats(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFFFFFF);
  
  float cpu_mhz = F_CPU / 1000000.0f;
  int chan = 0;
  
  if (qC.next() != NULL) {
    chan = atoi(qC.current());
  }
  
  if (chan == 0 || chan == 1) {
    S.printf("Channel 1 ISR Timing (%u samples):\n", isr1_count);
    S.printf("Min: %.2f us\n", isr1_cycles_min / cpu_mhz);
    if (isr1_count > 0) {
      S.printf("Avg: %.2f us\n", (isr1_cycles_sum / (float)isr1_count) / cpu_mhz);
    } else {
      S.printf("Avg: 0.00 us\n");
    }
    S.printf("Max: %.2f us\n", isr1_cycles_max / cpu_mhz);
    S.println("  ");
  }
  
  if (chan == 0 || chan == 2) {
    S.printf("Channel 2 ISR Timing (%u samples):\n", isr2_count);
    S.printf("Min: %.2f us\n", isr2_cycles_min / cpu_mhz);
    if (isr2_count > 0) {
      S.printf("Avg: %.2f us\n", (isr2_cycles_sum / (float)isr2_count) / cpu_mhz);
    } else {
      S.printf("Avg: 0.00 us\n");
    }
    S.printf("Max: %.2f us\n", isr2_cycles_max / cpu_mhz);
    S.println("  ");
  }
  
  if (chan == 0 || chan == 3) {
    S.printf("Channel 3 ISR Timing (%u samples):\n", isr3_count);
    S.printf("Min: %.2f us\n", isr3_cycles_min / cpu_mhz);
    if (isr3_count > 0) {
      S.printf("Avg: %.2f us\n", (isr3_cycles_sum / (float)isr3_count) / cpu_mhz);
    } else {
      S.printf("Avg: 0.00 us\n");
    }
    S.printf("Max: %.2f us\n", isr3_cycles_max / cpu_mhz);
    S.println("  ");
  }
  
  if (chan == 0 || chan == 4) {
    S.printf("Channel 4 ISR Timing (%u samples):\n", isr4_count);
    S.printf("Min: %.2f us\n", isr4_cycles_min / cpu_mhz);
    if (isr4_count > 0) {
      S.printf("Avg: %.2f us\n", (isr4_cycles_sum / (float)isr4_count) / cpu_mhz);
    } else {
      S.printf("Avg: 0.00 us\n");
    }
    S.printf("Max: %.2f us\n", isr4_cycles_max / cpu_mhz);
    S.println("  ");
  }
  
  S.printf("Budget (t_res): %.2f us\n", t_res);
}

void reset_timing(qCommand& qC, Stream& S) {
  setDebugWord(0xFFFFFFFE);
  
  // Reset channel 1
  isr1_cycles_min = 0xFFFFFFFF;
  isr1_cycles_max = 0;
  isr1_cycles_sum = 0;
  isr1_count = 0;
  
  // Reset channel 2
  isr2_cycles_min = 0xFFFFFFFF;
  isr2_cycles_max = 0;
  isr2_cycles_sum = 0;
  isr2_count = 0;
  
  // Reset channel 3
  isr3_cycles_min = 0xFFFFFFFF;
  isr3_cycles_max = 0;
  isr3_cycles_sum = 0;
  isr3_count = 0;
  
  // Reset channel 4
  isr4_cycles_min = 0xFFFFFFFF;
  isr4_cycles_max = 0;
  isr4_cycles_sum = 0;
  isr4_count = 0;
  
  S.println("Timing statistics reset");
}
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
// PID CONTROL            //////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////


// Initialize integral and previous error values on channel 1
static double integral1 = 0;
static double prev_err1 = 0;

void getADC1(void) {
  // ============================================================
  // TIMING START - MUST BE FIRST
  // ============================================================
  isr1_entry_count++;
  uint32_t start_cycles = ARM_DWT_CYCCNT;
  
  setDebugWord(0xFFFF2100);
  in1 = readADC1_from_ISR();

  //Tryna keep square wave value separate from the PID/sweep output (out1)
  float dac1_value;

  //  if (feed_set1) {
  //    s1 = in3;
  //  } -mg

//If we are in square signal mode: set the input equal to the min/max of the square wave every time
// add to count
  if (sq[0].enabled && sq[0].mode == MODE_SQUARE) {
    if (in1 < sq[0].meas_min) sq[0].meas_min = in1; //set the measured min and max voltage for structure equal to the ADC input reading
    if (in1 > sq[0].meas_max) sq[0].meas_max = in1;
    sq[0].meas_count++;

    if (sq[0].meas_count >= sq[0].meas_window_samples) { //once we hit the max number of samples, we determine the peak-to-peak voltage + thus our error
      float vpp_meas = sq[0].meas_max - sq[0].meas_min;
      float err      = sq[0].vpp_set - vpp_meas;

      sq[0].amp_scale += sq[0].vpp_gain * err;

      if (sq[0].amp_scale < 0.1f) sq[0].amp_scale = 0.1f; //clamping our amplitude scale so we don't use unreasonably high/low numbers
      if (sq[0].amp_scale > 1.0f) sq[0].amp_scale = 1.0f;

      sq[0].meas_min   =  1e9f;
      sq[0].meas_max   = -1e9f;
      sq[0].meas_count = 0;
    }
  }

  prev_err1 = err1;
  err1 = in1 - s1;
  
  setDebugWord(0xFFFF2101);
  if (default_hold == 0) {
    hold1 = !digitalRead(1);
  } else {
    hold1 = digitalRead(1);
  }
  
  setDebugWord(0xFFFF2102);

  // bool isServo1 = !feed_set3 && !track_error3 && enable1 && !bool(hold1); - mg
  bool isServo1 = enable1 && !bool(hold1);
  if (isServo1) {
    setLED(0,1,0);
    
    //if (track_error1) {
    //  out3 = err1;
    //  writeDAC(3, err1);
    //} -mg
    
    if (abs(err1) <= lock1_precision) {
      isLock1 = true;
      digitalWrite(5, HIGH);
    } else {
      isLock1 = false;
      digitalWrite(5, LOW);
    }
    
    double proportional1 = err1 * p1;
    
    if (reset1) {
      integral1 = 0;
      reset1 = 0;
    }
    
    integral1 += err1 * i1 * (t_res);
    
    if (integral1 > rail1_max) {
      integral1 = rail1_max;
    } else if (integral1 < rail1_min) {
      integral1 = rail1_min;
    }
    
    double differential1 = (err1 - prev_err1) * d1 / (t_res);
    out1 = proportional1 + integral1 + differential1;
  }
  //else if (track_error3 || feed_set3) {
  //  setLED(1,0,0);
  //  enable1 = 0;
  //  hold1 = !default_hold;
  //  track_error1 = 0;
  //  feed_set1 = 0;
  //  
  //  if (track_error3) {
  //    out1 = err3;
  //    amp1 = 0;
  //    pd1 = 0;
  //    phi1 = 0;
  //    offset1 = 0;
  //  }
  //  if (feed_set3) {
  //    s3 = in1;
  //    out1 = 0;
  //  }
  //} -mg

  else if (!enable1) {
    setLED(0,0,1);
    out1 = 0;
    reset1 = 1;
    hold1 = !default_hold;
  } else {
    setLED(1,1,0);
    out1 = -out1 + offset1;
    sweep1 = false;
  }
  
  // Add sweep (if desired)
  if (sweep1 && pd1 > 0 && !bool(hold1)) {
    if (pd1 < t_res * 10.0) {
      sweep1 = false;
    } else {
      switch (swp_typ1) {
        case 0: // sinusoid
          {
            float phase_time = t1 - phi1;
            if (phase_time < 0) phase_time += pd1;
            
            float table_pos = (phase_time / pd1) * 256.0f;
            uint32_t index1 = (uint32_t)table_pos % 256;
            uint32_t index2 = (index1 + 1) % 256;
            float frac = table_pos - floorf(table_pos);
            
            float sine_val = SINE_TABLE[index1] * (1.0f - frac) + SINE_TABLE[index2] * frac;
            out1 = out1 + amp1 * sine_val;
            break;
          }
        case 1: // sawtooth
          {
            float phase = (t1 - phi1) / pd1;
            phase = phase - floorf(phase);
            out1 = out1 + amp1 * (2.0f * phase - 1.0f);
            break;
          }
        case 2: // triangle
          {
            float phase = (t1 + phi1) / pd1;
            phase = phase - floorf(phase);
            out1 = out1 + amp1 * (4.0f * fabsf(phase - 0.5f) - 1.0f);
            break;
          }
        default:
          break;
      }
      
      t1 += t_res * 1e-6f;
      if (t1 >= pd1) t1 = 0;
    }
  } else {
    t1 = 0;
  }
  
  out1 = -out1 + offset1;
  
  if (out1 > rail1_max) {
    setLED(0,1,1);
    out1 = rail1_max;
    railed1 = true;
  } else if (out1 < rail1_min) {
    setLED(1,0,1);
    out1 = rail1_min;
    railed1 = true;
  } else {
    railed1 = false;
  }
  
  //Keeping square wave separate from PID/Sweep
  if (sq[0].enabled && sq[0].mode == MODE_SQUARE) {
    dac1_value = sq[0].value;          // square-wave path
  } else {
    dac1_value = (float)out1;          // existing PID/sweep path
  }
  writeDAC(1, dac1_value);
  
  // ============================================================
  // TIMING END - MUST BE LAST
  // ============================================================
  // TIMING END - SIMPLIFIED
  uint32_t end_cycles = ARM_DWT_CYCCNT;
  uint32_t elapsed = end_cycles - start_cycles;
  
  if (elapsed < isr1_cycles_min) isr1_cycles_min = elapsed;
  if (elapsed > isr1_cycles_max) isr1_cycles_max = elapsed;
  isr1_cycles_sum += elapsed;
  isr1_count++;
}


// Initialize integral and previous error values on channel 2
static double integral2 = 0;
static double prev_err2 = 0;

void getADC2(void) {
  // ============================================================
  // TIMING START - MUST BE FIRST
  // ============================================================
  isr2_entry_count++;
  uint32_t start_cycles = ARM_DWT_CYCCNT;
  
  setDebugWord(0xFFFF2200);
  in2 = readADC2_from_ISR();
  
    //Tryna keep square wave value separate from the PID/sweep output (out1)
  float dac2_value;

  //  if (feed_set1) {
  //    s1 = in3;
  //  } -mg

//If we are in square signal mode: set the input equal to the min/max of the square wave every time
// add to count
  if (sq[1].enabled && sq[1].mode == MODE_SQUARE) {
    if (in1 < sq[1].meas_min) sq[1].meas_min = in1; //set the measured min and max voltage for structure equal to the ADC input reading
    if (in1 > sq[1].meas_max) sq[1].meas_max = in1;
    sq[1].meas_count++;

    if (sq[1].meas_count >= sq[1].meas_window_samples) { //once we hit the max number of samples, we determine the peak-to-peak voltage + thus our error
      float vpp_meas = sq[1].meas_max - sq[1].meas_min;
      float err      = sq[1].vpp_set - vpp_meas;

      sq[1].amp_scale += sq[1].vpp_gain * err;

      if (sq[1].amp_scale < 0.1f) sq[1].amp_scale = 0.1f; //clamping our amplitude scale so we don't use unreasonably high/low numbers
      if (sq[1].amp_scale > 1.0f) sq[1].amp_scale = 1.0f;

      sq[1].meas_min   =  1e9f;
      sq[1].meas_max   = -1e9f;
      sq[1].meas_count = 0;
    }
  }

  //if (feed_set2) {
  s2 = in2;
  //}
  
  prev_err2 = err2;
  err2 = in2 - s2;
  
  setDebugWord(0xFFFF2201);
  if (default_hold == 0) {
    hold2 = !digitalRead(2);
  } else {
    hold2 = digitalRead(2);
  }
  
  setDebugWord(0xFFFF2202);
  // bool isServo2 = !feed_set4 && !track_error4 && enable2 && !bool(hold2); -mg
  bool isServo2 = enable2 && !bool(hold2);
  
  if (isServo2) {
    setLED(0,1,0);

    //if (track_error2) {
    //  out4 = err2;
    //  writeDAC(4, err2);
    //}
    
    if (abs(err2) <= lock2_precision) {
      isLock2 = true;
      digitalWrite(6, HIGH);
    } else {
      isLock2 = false;
      digitalWrite(6, LOW);
    }
    
    double proportional2 = err2 * p2;
    
    if (reset2) {
      integral2 = 0;
      reset2 = 0;
    }
    
    integral2 += i2 * err2 * (t_res);
    
    if (integral2 > 10) {
      integral2 = 10;
    } else if (integral2 < -10) {
      integral2 = -10;
    }
    
    double differential2 = (err2 - prev_err2) * d2 / (t_res);
    out2 = proportional2 + integral2 + differential2;
  }
  //else if (track_error4 || feed_set4) {
  //  setLED(1,0,0);
  //  enable2 = 0;
  //  hold2 = !default_hold;
  //  track_error2 = 0;
  //  feed_set2 = 0;
  //  
  //  if (track_error4) {
  //    out2 = err4;
  //    amp2 = 0;
  //    pd2 = 0;
  //    phi2 = 0;
  //    offset2 = 0;
  //  }
  //  if (feed_set4) {
  //    s4 = in2;
  //    out2 = 0;
  //  }
  //} -mg

  else if (!enable2) {
    setLED(0,0,1);
    out2 = 0;
    reset2 = 1;
    hold2 = !default_hold;
  } else {
    setLED(1,1,0);
    out2 = -out2 + offset2;
    sweep2 = false;
  }
  
  setDebugWord(0xFFFF2203);
  
  // Add sweep (if desired)
  if (sweep2 && pd2 > 0 && !bool(hold2)) {
    if (pd2 < t_res * 10.0) {
      sweep2 = false;
    } else {
      switch (swp_typ2) {
        case 0: // sinusoid
          {
            float phase_time = t2 - phi2;
            if (phase_time < 0) phase_time += pd2;
            
            float table_pos = (phase_time / pd2) * 256.0f;
            uint32_t index1 = (uint32_t)table_pos % 256;
            uint32_t index2 = (index1 + 1) % 256;
            float frac = table_pos - floorf(table_pos);
            
            float sine_val = SINE_TABLE[index1] * (1.0f - frac) + SINE_TABLE[index2] * frac;
            out2 = out2 + amp2 * sine_val;
            break;
          }
        case 1: // sawtooth
          {
            float phase = (t2 - phi2) / pd2;
            phase = phase - floorf(phase);
            out2 = out2 + amp2 * (2.0f * phase - 1.0f);
            break;
          }
        case 2: // triangle
          {
            float phase = (t2 + phi2) / pd2;
            phase = phase - floorf(phase);
            out2 = out2 + amp2 * (4.0f * fabsf(phase - 0.5f) - 1.0f);
            break;
          }
        default:
          break;
      }
      
      t2 = t2 + (t_res * 1e-6f);
    }
  } else {
    t2 = 0;
  }
  
  out2 = -out2 + offset2;
  
  if (out2 > rail2_max) {
    out2 = rail2_max;
    setLED(0,1,1);
    railed2 = true;
  } else if (out2 < rail2_min) {
    setLED(1,0,1);
    out2 = rail2_min;
    railed2 = true;
  } else {
    railed2 = false;
  }
  
  //Keeping square wave separate from PID/Sweep
  if (sq[1].enabled && sq[1].mode == MODE_SQUARE) {
    dac2_value = sq[1].value;          // square-wave path
  } else {
    dac2_value = (float)out1;          // existing PID/sweep path
  }
  writeDAC(2, dac2_value);
  
  // ============================================================
  // TIMING END - MUST BE LAST
  // ============================================================
    // TIMING END - SIMPLIFIED
  uint32_t end_cycles = ARM_DWT_CYCCNT;
  uint32_t elapsed = end_cycles - start_cycles;
  
  if (elapsed < isr2_cycles_min) isr2_cycles_min = elapsed;
  if (elapsed > isr2_cycles_max) isr2_cycles_max = elapsed;
  isr2_cycles_sum += elapsed;
  isr2_count++;
}




// Initialize integral and previous error values on channel 3
static double integral3 = 0;
static double prev_err3 = 0;

void getADC3(void) {
  // ============================================================
  // TIMING START - MUST BE FIRST
  // ============================================================
  isr3_entry_count++;
  uint32_t start_cycles = ARM_DWT_CYCCNT;
  
  setDebugWord(0xFFFF2300);
  in3 = readADC3_from_ISR();

  float dac3_value;
  
//  if (feed_set3) {
//    s3 = in1;
//  } -mg
  
  //If we are in square signal mode: set the input equal to the min/max of the square wave every time
  // add to count
  if (sq[2].enabled && sq[2].mode == MODE_SQUARE) {
    if (in1 < sq[2].meas_min) sq[2].meas_min = in1; //set the measured min and max voltage for structure equal to the ADC input reading
    if (in1 > sq[2].meas_max) sq[2].meas_max = in1;
    sq[1].meas_count++;

    if (sq[2].meas_count >= sq[2].meas_window_samples) { //once we hit the max number of samples, we determine the peak-to-peak voltage + thus our error
      float vpp_meas = sq[2].meas_max - sq[2].meas_min;
      float err      = sq[2].vpp_set - vpp_meas;

      sq[2].amp_scale += sq[2].vpp_gain * err;

      if (sq[2].amp_scale < 0.1f) sq[2].amp_scale = 0.1f; //clamping our amplitude scale so we don't use unreasonably high/low numbers
      if (sq[2].amp_scale > 1.0f) sq[2].amp_scale = 1.0f;

      sq[2].meas_min   =  1e9f;
      sq[2].meas_max   = -1e9f;
      sq[2].meas_count = 0;
    }
  }

  prev_err3 = err3;
  err3 = in3 - s3;
  
  setDebugWord(0xFFFF2301);
  if (default_hold == 0) {
    hold3 = !digitalRead(3);
  } else {
    hold3 = digitalRead(3);
  }
  
  setDebugWord(0xFFFF2302);
  //bool isServo3 = !feed_set1 && !track_error1 && enable3 && !bool(hold3); -mg
  bool isServo3 = enable3 && !bool(hold3);

  if (isServo3) {
    setLED(0,1,0);
    //if (track_error3) {
    //  out1 = err3;
    //  writeDAC(1, err3);
    //} -mg
    
    if (abs(err3) <= lock3_precision) {
      isLock3 = true;
      digitalWrite(7, HIGH);
    } else {
      isLock3 = false;
      digitalWrite(7, LOW);
    }
    
    double proportional3 = err3 * p3;
    
    if (reset3) {
      integral3 = 0;
      reset3 = 0;
    }
    
    integral3 += i3 * err3 * (t_res);
    
    if (integral3 > 10) {
      integral3 = 10;
    } else if (integral3 < -10) {
      integral3 = -10;
    }
    
    double differential3 = (err3 - prev_err3) * d3 / (t_res);
    out3 = proportional3 + integral3 + differential3;
  }
//  else if (track_error1 || feed_set1) {
//    setLED(1,0,0);
//    enable3 = 0;
//    hold3 = !default_hold;
//    track_error3 = 0;
//    feed_set3 = 0;
//    
//    if (track_error1) {
//      out3 = err1;
//      amp3 = 0;
//      pd3 = 0;
//      phi3 = 0;
//      offset3 = 0;
//    }
//    if (feed_set1) {
//      s1 = in3;
//      out3 = 0;
 //   }
//  }
  else if (!enable3) {
    out3 = 0;
    reset3 = 1;
    hold3 = !default_hold;
  } else {
    setLED(1,1,0);
    out3 = -out3 + offset3;
    sweep3 = false;
  }
  
  setDebugWord(0xFFFF2303);
  
  // Add sweep (if desired)
  if (sweep3 && pd3 > 0 && !bool(hold3)) {
    if (pd3 < t_res * 10.0) {
      sweep3 = false;
    } else {
      switch (swp_typ3) {
        case 0: // sinusoid
          {
            float phase_time = t3 - phi3;
            if (phase_time < 0) phase_time += pd3;
            
            float table_pos = (phase_time / pd3) * 256.0f;
            uint32_t index1 = (uint32_t)table_pos % 256;
            uint32_t index2 = (index1 + 1) % 256;
            float frac = table_pos - floorf(table_pos);
            
            float sine_val = SINE_TABLE[index1] * (1.0f - frac) + SINE_TABLE[index2] * frac;
            out3 = out3 + amp3 * sine_val;
            break;
          }
        case 1: // sawtooth
          {
            float phase = (t3 - phi3) / pd3;
            phase = phase - floorf(phase);
            out3 = out3 + amp3 * (2.0f * phase - 1.0f);
            break;
          }
        case 2: // triangle
          {
            float phase = (t3 + phi3) / pd3;
            phase = phase - floorf(phase);
            out3 = out3 + amp3 * (4.0f * fabsf(phase - 0.5f) - 1.0f);
            break;
          }
        default:
          break;
      }
      
      t3 = t3 + (t_res * 1e-6f);
    }
  } else {
    t3 = 0;
  }
  
  out3 = -out3 + offset3;
  
  if (out3 > rail3_max) {
    setLED(0,1,1);
    out3 = rail3_max;
    railed3 = true;
  } else if (out3 < rail3_min) {
    setLED(1,0,1);
    out3 = rail3_min;
    railed3 = true;
  } else {
    railed3 = false;
  }
  
  //Keeping square wave separate from PID/Sweep
  if (sq[2].enabled && sq[2].mode == MODE_SQUARE) {
    dac3_value = sq[2].value;          // square-wave path
  } else {
    dac3_value = (float)out1;          // existing PID/sweep path
  }
  writeDAC(3, dac3_value);
  
  // ============================================================
  // TIMING END - MUST BE LAST
  // ============================================================
   // TIMING END - SIMPLIFIED
  uint32_t end_cycles = ARM_DWT_CYCCNT;
  uint32_t elapsed = end_cycles - start_cycles;
  
  if (elapsed < isr3_cycles_min) isr3_cycles_min = elapsed;
  if (elapsed > isr3_cycles_max) isr3_cycles_max = elapsed;
  isr3_cycles_sum += elapsed;
  isr3_count++;
}



// Initialize integral and previous error values on channel 4
static double integral4 = 0;
static double prev_err4 = 0;

void getADC4(void) {
  // ============================================================
  // TIMING START - MUST BE FIRST
  // ============================================================
  isr4_entry_count++;
  uint32_t start_cycles = ARM_DWT_CYCCNT;
  
  setDebugWord(0xFFFF2400);
  in4 = readADC4_from_ISR();
  
  float dac4_value;

//  if (feed_set4) {
//    s4 = in2;
//  }
  if (sq[3].enabled && sq[3].mode == MODE_SQUARE) {
    if (in1 < sq[3].meas_min) sq[3].meas_min = in1; //set the measured min and max voltage for structure equal to the ADC input reading
    if (in1 > sq[3].meas_max) sq[3].meas_max = in1;
    sq[1].meas_count++;

    if (sq[3].meas_count >= sq[3].meas_window_samples) { //once we hit the max number of samples, we determine the peak-to-peak voltage + thus our error
      float vpp_meas = sq[3].meas_max - sq[3].meas_min;
      float err      = sq[3].vpp_set - vpp_meas;

      sq[3].amp_scale += sq[3].vpp_gain * err;

      if (sq[3].amp_scale < 0.1f) sq[3].amp_scale = 0.1f; //clamping our amplitude scale so we don't use unreasonably high/low numbers
      if (sq[3].amp_scale > 1.0f) sq[3].amp_scale = 1.0f;

      sq[3].meas_min   =  1e9f;
      sq[3].meas_max   = -1e9f;
      sq[3].meas_count = 0;
    }
  }

  prev_err4 = err4;
  err4 = in4 - s4;
  
  setDebugWord(0xFFFF2401);
  if (default_hold == 0) {
    hold4 = !digitalRead(4);
  } else {
    hold4 = digitalRead(4);
  }
  
  setDebugWord(0xFFFF2402);
  // bool isServo4 = !feed_set2 && !track_error2 && enable4 && !bool(hold4);
  bool isServo4 = enable4 && !bool(hold4);
  
  if (isServo4) {
    setLED(0,1,0);
//    if (track_error4) {
//      out2 = err4;
//      writeDAC(2, err4);
//    } -mg
    
    if (abs(err4) <= lock4_precision) {
      isLock4 = true;
      digitalWrite(8, HIGH);
    } else {
      isLock4 = false;
      digitalWrite(8, LOW);
    }
    
    double proportional4 = err4 * p4;
    
    if (reset4) {
      integral4 = 0;
      reset4 = 0;
    }
    
    integral4 += i4 * err4 * (t_res);
    
    if (integral4 > 10) {
      integral4 = 10;
    } else if (integral4 < -10) {
      integral4 = -10;
    }
    
    double differential4 = (err4 - prev_err4) * d4 / (t_res);
    out4 = proportional4 + integral4 + differential4;
  }
//  else if (track_error2 || feed_set2) {
//    setLED(1,0,0);
//    enable4 = 0;
//    hold4 = !default_hold;
//    track_error4 = 0;
//    feed_set4 = 0;
//    
//    if (track_error2) {
//      out4 = err2;
//      amp4 = 0;
//      pd4 = 0;
//      phi4 = 0;
//      offset4 = 0;
//    }
//    if (feed_set2) {
//      s2 = in4;
//      out4 = 0;
//    }
//  }

  else if (!enable4) {
    out4 = 0;
    reset4 = 1;
    hold4 = !default_hold;
  } else {
    setLED(1,1,0);
    out4 = -out4 + offset4;
    sweep4 = false;
  }
  
  setDebugWord(0xFFFF2403);
  
  // Add sweep (if desired)
  if (sweep4 && pd4 > 0 && !bool(hold4)) {
    if (pd4 < t_res * 10.0) {
      sweep4 = false;
    } else {
      switch (swp_typ4) {
        case 0: // sinusoid
          {
            float phase_time = t4 - phi4;
            if (phase_time < 0) phase_time += pd4;
            
            float table_pos = (phase_time / pd4) * 256.0f;
            uint32_t index1 = (uint32_t)table_pos % 256;
            uint32_t index2 = (index1 + 1) % 256;
            float frac = table_pos - floorf(table_pos);
            
            float sine_val = SINE_TABLE[index1] * (1.0f - frac) + SINE_TABLE[index2] * frac;
            out4 = out4 + amp4 * sine_val;
            break;
          }
        case 1: // sawtooth
          {
            float phase = (t4 - phi4) / pd4;
            phase = phase - floorf(phase);
            out4 = out4 + amp4 * (2.0f * phase - 1.0f);
            break;
          }
        case 2: // triangle
          {
            float phase = (t4 + phi4) / pd4;
            phase = phase - floorf(phase);
            out4 = out4 + amp4 * (4.0f * fabsf(phase - 0.5f) - 1.0f);
            break;
          }
        default:
          break;
      }
      
      t4 = t4 + (t_res * 1e-6f);
    }
  } else {
    t4 = 0;
  }
  
  out4 = -out4 + offset4;
  
  if (out4 > rail4_max) {
    out4 = rail4_max;
    setLED(0,1,1);
    railed4 = true;
  } else if (out4 < rail4_min) {
    setLED(1,0,1);
    out4 = rail4_min;
    railed4 = true;
  } else {
    railed4 = false;
  }
  
  //Keeping square wave separate from PID/Sweep
  if (sq[3].enabled && sq[3].mode == MODE_SQUARE) {
    dac4_value = sq[3].value;          // square-wave path
  } else {
    dac4_value = (float)out1;          // existing PID/sweep path
  }
  writeDAC(4, dac4_value);
  
  
  // ============================================================
  // TIMING END - MUST BE LAST
  // ============================================================
    // TIMING END - SIMPLIFIED
  uint32_t end_cycles = ARM_DWT_CYCCNT;
  uint32_t elapsed = end_cycles - start_cycles;
  
  if (elapsed < isr4_cycles_min) isr4_cycles_min = elapsed;
  if (elapsed > isr4_cycles_max) isr4_cycles_max = elapsed;
  isr4_cycles_sum += elapsed;
  isr4_count++;
}




void save_params(qCommand& qC, Stream& S) {
  setDebugWord(0xFEEDFACE);
  
  S.println("Saving all parameters to NVM...");
  
  // Save PID gains
  writeNVMpages(&p1, sizeof(p1), 0);
  writeNVMpages(&p2, sizeof(p2), 1);
  writeNVMpages(&p3, sizeof(p3), 2);
  writeNVMpages(&p4, sizeof(p4), 3);
  
  writeNVMpages(&i1, sizeof(i1), 4);
  writeNVMpages(&i2, sizeof(i2), 5);
  writeNVMpages(&i3, sizeof(i3), 6);
  writeNVMpages(&i4, sizeof(i4), 7);
  
  writeNVMpages(&d1, sizeof(d1), 8);
  writeNVMpages(&d2, sizeof(d2), 9);
  writeNVMpages(&d3, sizeof(d3), 10);
  writeNVMpages(&d4, sizeof(d4), 11);
  
  // Save setpoints
  writeNVMpages(&s1, sizeof(s1), 12);
  writeNVMpages(&s2, sizeof(s2), 13);
  writeNVMpages(&s3, sizeof(s3), 14);
  writeNVMpages(&s4, sizeof(s4), 15);
  
  // Save offsets
  writeNVMpages(&offset1, sizeof(offset1), 16);
  writeNVMpages(&offset2, sizeof(offset2), 17);
  writeNVMpages(&offset3, sizeof(offset3), 18);
  writeNVMpages(&offset4, sizeof(offset4), 19);
  
  // Save config
  writeNVMpages(&config, sizeof(config), 20);
  
  // Save enable states
  writeNVMpages(&enable1, sizeof(enable1), 25);
  writeNVMpages(&enable2, sizeof(enable2), 26);
  writeNVMpages(&enable3, sizeof(enable3), 27);
  writeNVMpages(&enable4, sizeof(enable4), 28);
  
  // Save tracking states
  //writeNVMpages(&track_error1, sizeof(track_error1), 29);
  //writeNVMpages(&track_error2, sizeof(track_error2), 30);
  //writeNVMpages(&track_error3, sizeof(track_error3), 31);
  //writeNVMpages(&track_error4, sizeof(track_error4), 32); -mg
  
  // Save feeding states
  //writeNVMpages(&feed_set1, sizeof(feed_set1), 33);
  //writeNVMpages(&feed_set2, sizeof(feed_set2), 34);
  //writeNVMpages(&feed_set3, sizeof(feed_set3), 35);
  //writeNVMpages(&feed_set4, sizeof(feed_set4), 36); -mg
  
  // Save sweep states
  writeNVMpages(&sweep1, sizeof(sweep1), 37);
  writeNVMpages(&sweep2, sizeof(sweep2), 38);
  writeNVMpages(&sweep3, sizeof(sweep3), 39);
  writeNVMpages(&sweep4, sizeof(sweep4), 40);
  
  // Save sweep parameters
  writeNVMpages(&amp1, sizeof(amp1), 41);
  writeNVMpages(&amp2, sizeof(amp2), 42);
  writeNVMpages(&amp3, sizeof(amp3), 43);
  writeNVMpages(&amp4, sizeof(amp4), 44);
  
  writeNVMpages(&pd1, sizeof(pd1), 45);
  writeNVMpages(&pd2, sizeof(pd2), 46);
  writeNVMpages(&pd3, sizeof(pd3), 47);
  writeNVMpages(&pd4, sizeof(pd4), 48);
  
  writeNVMpages(&phi1, sizeof(phi1), 49);
  writeNVMpages(&phi2, sizeof(phi2), 50);
  writeNVMpages(&phi3, sizeof(phi3), 51);
  writeNVMpages(&phi4, sizeof(phi4), 52);
  
  writeNVMpages(&swp_typ1, sizeof(swp_typ1), 53);
  writeNVMpages(&swp_typ2, sizeof(swp_typ2), 54);
  writeNVMpages(&swp_typ3, sizeof(swp_typ3), 55);
  writeNVMpages(&swp_typ4, sizeof(swp_typ4), 56);
  
  // Save ADC ranges
  writeNVMpages(&adc1_range, sizeof(adc1_range), 57);
  writeNVMpages(&adc2_range, sizeof(adc2_range), 58);
  writeNVMpages(&adc3_range, sizeof(adc3_range), 59);
  writeNVMpages(&adc4_range, sizeof(adc4_range), 60);
  
  // Save rails
  writeNVMpages(&rail1_min, sizeof(rail1_min), 61);
  writeNVMpages(&rail2_min, sizeof(rail2_min), 62);
  writeNVMpages(&rail3_min, sizeof(rail3_min), 63);
  writeNVMpages(&rail4_min, sizeof(rail4_min), 64);
  
  writeNVMpages(&rail1_max, sizeof(rail1_max), 65);
  writeNVMpages(&rail2_max, sizeof(rail2_max), 66);
  writeNVMpages(&rail3_max, sizeof(rail3_max), 67);
  writeNVMpages(&rail4_max, sizeof(rail4_max), 68);
  
  // Save lock precision
  writeNVMpages(&lock1_precision, sizeof(lock1_precision), 69);
  writeNVMpages(&lock2_precision, sizeof(lock2_precision), 70);
  writeNVMpages(&lock3_precision, sizeof(lock3_precision), 71);
  writeNVMpages(&lock4_precision, sizeof(lock4_precision), 72);
  
  // Save default hold
  writeNVMpages(&default_hold, sizeof(default_hold), 73);
  
  S.println("All parameters saved to NVM!");
  S.println("Press RESET button to reload, or power cycle.");
}

void dumpNVM(qCommand& qC, Stream& S) {
  setDebugWord(0xDEADBEEF);
  
  S.println("\n===== NVM MEMORY DUMP =====");
  
  // Read PID gains from NVM
  double p_nvm[4], i_nvm[4], d_nvm[4], s_nvm[4];
  
  for (int ch = 0; ch < 4; ch++) {
    readNVMblock(&p_nvm[ch], sizeof(double), ch * 128);           // Pages 0-3
    readNVMblock(&i_nvm[ch], sizeof(double), (ch + 4) * 128);     // Pages 4-7
    readNVMblock(&d_nvm[ch], sizeof(double), (ch + 8) * 128);     // Pages 8-11
    readNVMblock(&s_nvm[ch], sizeof(double), (ch + 12) * 128);    // Pages 12-15
  }
  
  S.println("\nStored in NVM:");
  S.println("Chan | Prop (P)    | Integral (I) | Derivative (D) | Setpoint (S)");
  S.println("-----|-------------|--------------|----------------|-------------");
  for (int ch = 0; ch < 4; ch++) {
    S.printf("  %d  | %11.6f | %12.6f | %14.6f | %11.6f\n", 
             ch + 1, p_nvm[ch], i_nvm[ch], d_nvm[ch], s_nvm[ch]);
  }
  
  S.println("\nCurrently Running:");
  S.println("Chan | Prop (P)    | Integral (I) | Derivative (D) | Setpoint (S)");
  S.println("-----|-------------|--------------|----------------|-------------");
  S.printf("  1  | %11.6f | %12.6f | %14.6f | %11.6f\n", p1, i1, d1, s1);
  S.printf("  2  | %11.6f | %12.6f | %14.6f | %11.6f\n", p2, i2, d2, s2);
  S.printf("  3  | %11.6f | %12.6f | %14.6f | %11.6f\n", p3, i3, d3, s3);
  S.printf("  4  | %11.6f | %12.6f | %14.6f | %11.6f\n", p4, i4, d4, s4);
  
  S.println("\n===========================\n");
}

