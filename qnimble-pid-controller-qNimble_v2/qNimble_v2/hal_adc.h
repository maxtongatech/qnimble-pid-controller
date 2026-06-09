////////////////////////////////////////////////////////////////////////////////////
// hal_adc.h - ADC Hardware Abstraction Layer
// qNimble v2.0
////////////////////////////////////////////////////////////////////////////////////

#ifndef HAL_ADC_H
#define HAL_ADC_H

#include "config.h"

////////////////////////////////////////////////////////////////////////////////////
// ADC CHANNEL CLASS
////////////////////////////////////////////////////////////////////////////////////

class ADC_Channel {
private:
    uint8_t channel_num;      // 1-4
    uint8_t pin;              // Physical pin number
    ADC_Range range;          // Voltage range
    bool enabled;             // Is this channel active?
    float voltage_scale;      // Volts per ADC count
    uint16_t raw_value;       // Last raw ADC reading
    float voltage_value;      // Last voltage reading
    
    // Timing
    float sample_interval;    // Microseconds between samples
    unsigned long last_sample_time;
    
    // Statistics
    uint32_t sample_count;
    float min_voltage;
    float max_voltage;

public:
    // Constructor
    ADC_Channel();
    
    // Initialization
    void init(uint8_t chan, uint8_t adc_pin);
    
    // Configuration
    void setRange(ADC_Range new_range);
    ADC_Range getRange() const { return range; }
    float getRangeVolts() const { return getADCRangeVolts(range); }
    
    void setSampleInterval(float interval_us);
    float getSampleInterval() const { return sample_interval; }
    
    // Enable/Disable
    void enable();
    void disable();
    bool isEnabled() const { return enabled; }
    
    // Reading
    bool readADC();  // Returns true if new sample taken
    uint16_t getRawValue() const { return raw_value; }
    float getVoltage() const { return voltage_value; }
    
    // Statistics
    uint32_t getSampleCount() const { return sample_count; }
    float getMinVoltage() const { return min_voltage; }
    float getMaxVoltage() const { return max_voltage; }
    void resetStatistics();
    
    // Info
    uint8_t getChannelNum() const { return channel_num; }
    uint8_t getPin() const { return pin; }
};

////////////////////////////////////////////////////////////////////////////////////
// ADC MANAGER CLASS
////////////////////////////////////////////////////////////////////////////////////

class ADC_Manager {
private:
    ADC_Channel channels[NUM_ADC_CHANNELS];
    uint8_t active_channel_mask;  // Bitmask of enabled channels
    float global_sample_interval;
    
public:
    // Constructor
    ADC_Manager();
    
    // Initialization
    void init();
    
    // Channel access
    ADC_Channel* getChannel(uint8_t chan);  // 1-indexed (1-4)
    
    // Global configuration
    void setGlobalSampleInterval(float interval_us);
    float getGlobalSampleInterval() const { return global_sample_interval; }
    
    // Batch operations
    void enableChannel(uint8_t chan);
    void disableChannel(uint8_t chan);
    void enableAllChannels();
    void disableAllChannels();
    
    uint8_t getActiveChannelMask() const { return active_channel_mask; }
    uint8_t getActiveChannelCount() const;
    
    // Reading
    void readAllChannels();  // Read all enabled channels
    
    // Diagnostics
    void printStatus(Stream& serial);
    void printChannelInfo(uint8_t chan, Stream& serial);
};

#endif // HAL_ADC_H
