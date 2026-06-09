////////////////////////////////////////////////////////////////////////////////////
// hal_dac.h - DAC Hardware Abstraction Layer
// qNimble v2.0
////////////////////////////////////////////////////////////////////////////////////

#ifndef HAL_DAC_H
#define HAL_DAC_H

#include "config.h"

////////////////////////////////////////////////////////////////////////////////////
// DAC CHANNEL CLASS
////////////////////////////////////////////////////////////////////////////////////

class DAC_Channel {
private:
    uint8_t channel_num;      // 1-4
    uint8_t pin;              // Physical pin number (for DAC 1-2)
    bool enabled;             // Is this channel active?
    bool is_hardware_dac;     // True for DAC1/2, false for DAC3/4 (SPI)
    
    float voltage_value;      // Current output voltage
    uint16_t dac_value;       // Current DAC code
    float voltage_scale;      // DAC counts per volt
    
    // Hold state
    bool hold_active;
    float hold_voltage;
    
    // Limits
    float rail_min;
    float rail_max;
    
    // Statistics
    uint32_t write_count;

public:
    // Constructor
    DAC_Channel();
    
    // Initialization
    void init(uint8_t chan, uint8_t dac_pin, bool is_hw_dac);
    
    // Configuration
    void setRails(float min_v, float max_v);
    float getRailMin() const { return rail_min; }
    float getRailMax() const { return rail_max; }
    
    // Enable/Disable
    void enable();
    void disable();
    bool isEnabled() const { return enabled; }
    
    // Output
    void setVoltage(float volts);
    void setDACCode(uint16_t code);
    float getVoltage() const { return voltage_value; }
    uint16_t getDACCode() const { return dac_value; }
    
    // Hold functionality
    void setHold(bool hold_state);
    bool isHeld() const { return hold_active; }
    void setHoldVoltage(float volts);
    
    // Statistics
    uint32_t getWriteCount() const { return write_count; }
    void resetStatistics();
    
    // Info
    uint8_t getChannelNum() const { return channel_num; }
    uint8_t getPin() const { return pin; }
    bool isHardwareDAC() const { return is_hardware_dac; }
};

////////////////////////////////////////////////////////////////////////////////////
// DAC MANAGER CLASS
////////////////////////////////////////////////////////////////////////////////////

class DAC_Manager {
private:
    DAC_Channel channels[NUM_DAC_CHANNELS];
    uint8_t active_channel_mask;
    
    // SPI DAC support (for channels 3-4)
    void initSPI();
    void writeSPI_DAC(uint8_t dac_num, uint16_t value);

public:
    // Constructor
    DAC_Manager();
    
    // Initialization
    void init();
    
    // Channel access
    DAC_Channel* getChannel(uint8_t chan);  // 1-indexed (1-4)
    
    // Batch operations
    void enableChannel(uint8_t chan);
    void disableChannel(uint8_t chan);
    void enableAllChannels();
    void disableAllChannels();
    
    uint8_t getActiveChannelMask() const { return active_channel_mask; }
    uint8_t getActiveChannelCount() const;
    
    // Global operations
    void setAllVoltages(float volts);
    void setAllHold(bool hold_state);
    
    // Diagnostics
    void printStatus(Stream& serial);
    void printChannelInfo(uint8_t chan, Stream& serial);
};

#endif // HAL_DAC_H
