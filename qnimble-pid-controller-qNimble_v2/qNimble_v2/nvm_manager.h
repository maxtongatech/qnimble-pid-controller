////////////////////////////////////////////////////////////////////////////////////
// nvm_manager.h - Non-Volatile Memory Manager
// qNimble v2.0
////////////////////////////////////////////////////////////////////////////////////

#ifndef NVM_MANAGER_H
#define NVM_MANAGER_H

#include "config.h"
#include <Arduino.h>

// For Teensy 4.1 / qNimble, we'll use the built-in EEPROM emulation
// The board package provides EEPROM-like functionality through flash memory

////////////////////////////////////////////////////////////////////////////////////
// CONFIGURATION STRUCTURES
////////////////////////////////////////////////////////////////////////////////////

// PID gains for one channel
struct PID_Gains {
    float p_gain;
    float i_gain;
    float d_gain;
    float setpoint;
    float rail_min;
    float rail_max;
    float lock_precision;
};

// ADC configuration for one channel
struct ADC_Config {
    ADC_Range range;
    bool enabled;
    float sample_interval;
};

// DAC configuration for one channel
struct DAC_Config {
    bool enabled;
    float rail_min;
    float rail_max;
    float offset;
};

// Square wave configuration for one channel
struct SquareWave_Config {
    bool enabled;
    float period_ms;
    float duty_cycle;
    float high_setpoint;
    float low_setpoint;
};


// System configuration
struct System_Config {
    uint32_t magic_number;        // Validation marker
    uint16_t version;             // Config version
    float global_t_res;           // Global time resolution
    bool default_hold_logic;      // Hold pin logic (0 or 1)
    uint32_t checksum;            // Data integrity check
};

// Complete configuration package
struct Config_Package {
    System_Config system;
    PID_Gains pid[NUM_ADC_CHANNELS];
    ADC_Config adc[NUM_ADC_CHANNELS];
    DAC_Config dac[NUM_DAC_CHANNELS];
    SquareWave_Config sqwave[NUM_ADC_CHANNELS];
};

////////////////////////////////////////////////////////////////////////////////////
// NVM MANAGER CLASS
////////////////////////////////////////////////////////////////////////////////////

class NVM_Manager {
private:
    Config_Package config;
    bool config_valid;
    
    // Memory addresses (using qNimble's NVM system)
    static const uint16_t CONFIG_BASE_ADDR = 0;
    static const uint16_t CONFIG_SIZE = sizeof(Config_Package);
    
    // Helper functions
    uint32_t calculateChecksum(const Config_Package& pkg);
    bool validateConfig(const Config_Package& pkg);
    void setDefaults();
    
    // Low-level NVM access (will use qNimble's native functions)
    bool writeNVM(uint16_t addr, const void* data, size_t size);
    bool readNVM(uint16_t addr, void* data, size_t size);

public:
    // Constructor
    NVM_Manager();
    
    // Initialization
    void init();
    bool isConfigValid() const { return config_valid; }
    
    // Save/Load operations
    bool saveConfig();
    bool loadConfig();
    void factoryReset();
    
    // System configuration
    void setGlobalTimeRes(float t_res);
    float getGlobalTimeRes() const { return config.system.global_t_res; }
    
    void setDefaultHoldLogic(bool logic);
    bool getDefaultHoldLogic() const { return config.system.default_hold_logic; }
    
    // PID configuration (channel 1-4)
    void setPIDGains(uint8_t chan, float p, float i, float d);
    void getPIDGains(uint8_t chan, float& p, float& i, float& d);
    
    void setSetpoint(uint8_t chan, float sp);
    float getSetpoint(uint8_t chan);
    
    void setRails(uint8_t chan, float min_v, float max_v);
    void getRails(uint8_t chan, float& min_v, float& max_v);
    
    void setLockPrecision(uint8_t chan, float precision);
    float getLockPrecision(uint8_t chan);
    
    // ADC configuration (channel 1-4)
    void setADCRange(uint8_t chan, ADC_Range range);
    ADC_Range getADCRange(uint8_t chan);
    
    void setADCEnabled(uint8_t chan, bool enabled);
    bool getADCEnabled(uint8_t chan);
    
    void setADCSampleInterval(uint8_t chan, float interval);
    float getADCSampleInterval(uint8_t chan);
    
    // DAC configuration (channel 1-4)
    void setDACEnabled(uint8_t chan, bool enabled);
    bool getDACEnabled(uint8_t chan);
    
    void setDACRails(uint8_t chan, float min_v, float max_v);
    void getDACRails(uint8_t chan, float& min_v, float& max_v);
    
    void setDACOffset(uint8_t chan, float offset);
    float getDACOffset(uint8_t chan);
    
    // Diagnostics
    void printConfig(Stream& serial);
    void printMemoryInfo(Stream& serial);

        // Square wave configuration (channel 1-4)
    void setSquareWaveEnabled(uint8_t chan, bool enabled);
    bool getSquareWaveEnabled(uint8_t chan);
    
    void setSquareWavePeriod(uint8_t chan, float period);
    float getSquareWavePeriod(uint8_t chan);
    
    void setSquareWaveDuty(uint8_t chan, float duty);
    float getSquareWaveDuty(uint8_t chan);
    
    void setSquareWaveHigh(uint8_t chan, float high);
    float getSquareWaveHigh(uint8_t chan);
    
    void setSquareWaveLow(uint8_t chan, float low);
    float getSquareWaveLow(uint8_t chan);

};

#endif // NVM_MANAGER_H
