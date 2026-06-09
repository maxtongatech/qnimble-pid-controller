////////////////////////////////////////////////////////////////////////////////////
// config.h - Hardware Configuration & Constants
// qNimble v2.0 - Quantum Opus LLC
////////////////////////////////////////////////////////////////////////////////////

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

////////////////////////////////////////////////////////////////////////////////////
// HARDWARE DEFINITIONS
////////////////////////////////////////////////////////////////////////////////////

// CPU & Timing
#define CPU_MHZ 600.0f
#define DEFAULT_T_RES 100.0f  // Default time resolution in microseconds

// ADC Channels
#define NUM_ADC_CHANNELS 4
#define ADC_PIN_1 14  // A0
#define ADC_PIN_2 15  // A1
#define ADC_PIN_3 16  // A2
#define ADC_PIN_4 17  // A3

// DAC Channels - Using correct Teensy 4.1 DAC pins
#define NUM_DAC_CHANNELS 4
#define DAC_PIN_1 A12  // Teensy 4.1 DAC0 (pin 26)
#define DAC_PIN_2 A13  // Teensy 4.1 DAC1 (pin 27)

// Digital I/O
#define HOLD_PIN_1 1
#define HOLD_PIN_2 2
#define LOCK_PIN_1 5
#define LOCK_PIN_2 6

// LED Pins (assuming RGB LED)
#define LED_RED_PIN 3
#define LED_GREEN_PIN 4
#define LED_BLUE_PIN 7

////////////////////////////////////////////////////////////////////////////////////
// ADC CONFIGURATION - Use the board's native adc_scale enum
////////////////////////////////////////////////////////////////////////////////////

// The qNimble board package already defines these in adc.h:
// typedef enum {
//   BIPOLAR_1250mV = 3,
//   BIPOLAR_2500mV = 7,
//   BIPOLAR_5V = 11,
//   BIPOLAR_10V = 15
// } adc_scale;

// We'll use adc_scale as our ADC_Range type
typedef adc_scale ADC_Range;

// Helper function to get voltage from range
inline float getADCRangeVolts(ADC_Range range) {
    switch(range) {
        case BIPOLAR_1250mV: return 1.25f;
        case BIPOLAR_2500mV: return 2.5f;
        case BIPOLAR_5V: return 5.0f;
        case BIPOLAR_10V: return 10.0f;
        default: return 10.0f;
    }
}

// ADC Resolution
#define ADC_RESOLUTION 16
#define ADC_MAX_VALUE 65535
#define ADC_MID_VALUE 32768

// ADC Timing
#define ADC_MAX_SAMPLE_RATE 1000000  // 1 MSPS

////////////////////////////////////////////////////////////////////////////////////
// DAC CONFIGURATION
////////////////////////////////////////////////////////////////////////////////////

// DAC Resolution
#define DAC_RESOLUTION 12
#define DAC_MAX_VALUE 4095
#define DAC_MID_VALUE 2048

// DAC Voltage Range (assuming ±10V with external scaling)
#define DAC_VOLTAGE_RANGE 10.0f

////////////////////////////////////////////////////////////////////////////////////
// MEMORY CONFIGURATION
////////////////////////////////////////////////////////////////////////////////////

// EEPROM/NVM
#define NVM_SIZE 98304  // 96 kB
#define NVM_PAGE_SIZE 256
#define NVM_MAGIC_NUMBER 0xABCD1234  // For validating stored data

// Memory Map (page addresses)
#define NVM_CONFIG_PAGE 0
#define NVM_PID_GAINS_PAGE 1
#define NVM_SETPOINTS_PAGE 5
#define NVM_ADC_RANGES_PAGE 9

////////////////////////////////////////////////////////////////////////////////////
// PID DEFAULTS
////////////////////////////////////////////////////////////////////////////////////

#define DEFAULT_P_GAIN 1.0f
#define DEFAULT_I_GAIN 0.0f
#define DEFAULT_D_GAIN 0.0f
#define DEFAULT_SETPOINT 0.0f
#define DEFAULT_RAIL_MIN -10.0f
#define DEFAULT_RAIL_MAX 10.0f
#define DEFAULT_LOCK_PRECISION 0.01f

////////////////////////////////////////////////////////////////////////////////////
// SYSTEM LIMITS
////////////////////////////////////////////////////////////////////////////////////

#define MAX_INTEGRAL_WINDUP 10.0f
#define MIN_T_RES 10.0f    // Minimum 10 microseconds
#define MAX_T_RES 10000.0f // Maximum 10 milliseconds

////////////////////////////////////////////////////////////////////////////////////
// DEBUG & DIAGNOSTICS
////////////////////////////////////////////////////////////////////////////////////

#define ENABLE_DEBUG_WORDS true
#define ENABLE_PERFORMANCE_MONITORING true
#define STATUS_LED_BLINK_INTERVAL 500  // milliseconds

////////////////////////////////////////////////////////////////////////////////////
// VERSION INFO
////////////////////////////////////////////////////////////////////////////////////

#define FIRMWARE_VERSION "2.0.0"
#define FIRMWARE_DATE "2024-01"
#define HARDWARE_VERSION "qNimble v1"

#endif // CONFIG_H
