////////////////////////////////////////////////////////////////////////////////////
// hal_adc.h - ADC Hardware Abstraction Layer
// qNimble v2.0
////////////////////////////////////////////////////////////////////////////////////

#ifndef HAL_ADC_H
#define HAL_ADC_H

#include "config.h"

// Note: ADC_Range is already defined in config.h as:
// typedef adc_scale ADC_Range;
// And adc_scale is defined in the board package's adc.h

////////////////////////////////////////////////////////////////////////////////////
// ADC_CHANNEL CLASS
////////////////////////////////////////////////////////////////////////////////////

class ADC_Channel {
private:
    uint8_t channel_num;
    uint8_t pin;
    ADC_Range range;
    bool enabled;

    float voltage_scale;
    uint16_t raw_value;
    float voltage_value;

    float sample_interval;
    unsigned long last_sample_time;

    // Statistics
    unsigned long sample_count;
    float min_voltage;
    float max_voltage;

public:
    ADC_Channel();

    void init(uint8_t chan, uint8_t adc_pin);
    void setRange(ADC_Range new_range);
    void setSampleInterval(float interval_us);

    void enable();
    void disable();
    bool isEnabled() const { return enabled; }

    bool readADC();

    // Getters
    uint8_t getChannel() const { return channel_num; }
    uint8_t getPin() const { return pin; }
    ADC_Range getRange() const { return range; }
    float getRangeVolts() const { return getADCRangeVolts(range); }

    float getVoltage() const { return voltage_value; }
    uint16_t getRawValue() const { return raw_value; }

    unsigned long getSampleCount() const { return sample_count; }
    float getMinVoltage() const { return min_voltage; }
    float getMaxVoltage() const { return max_voltage; }

    void resetStatistics();
};

////////////////////////////////////////////////////////////////////////////////////
// ADC_MANAGER CLASS
////////////////////////////////////////////////////////////////////////////////////

class ADC_Manager {
private:
    ADC_Channel channels[NUM_ADC_CHANNELS];
    uint8_t active_channel_mask;
    float global_sample_interval;

public:
    ADC_Manager();

    void init();

    ADC_Channel* getChannel(uint8_t chan);

    void setGlobalSampleInterval(float interval_us);

    void enableChannel(uint8_t chan);
    void disableChannel(uint8_t chan);
    void enableAllChannels();
    void disableAllChannels();

    uint8_t getActiveChannelCount() const;
    bool isChannelActive(uint8_t chan) const { 
        return (active_channel_mask & (1 << (chan - 1))) != 0; 
    }

    void readAllChannels();

    void printStatus(Stream& serial);
    void printChannelInfo(uint8_t chan, Stream& serial);
};

#endif // HAL_ADC_H