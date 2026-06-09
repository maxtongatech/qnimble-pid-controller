////////////////////////////////////////////////////////////////////////////////////
// hal_adc.cpp - ADC Hardware Abstraction Layer Implementation
// qNimble v2.0 - Polled mode (no ISRs)
////////////////////////////////////////////////////////////////////////////////////

#include "hal_adc.h"

extern "C" {
    #include "comm.h"
}

////////////////////////////////////////////////////////////////////////////////////
// ADC_CHANNEL IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////

ADC_Channel::ADC_Channel() {
    channel_num = 0;
    pin = 0;
    range = BIPOLAR_10V;
    enabled = false;
    voltage_scale = 0.0f;
    raw_value = 0;
    voltage_value = 0.0f;
    sample_interval = DEFAULT_T_RES;
    last_sample_time = 0;
    sample_count = 0;
    min_voltage = 999.9f;
    max_voltage = -999.9f;
}

void ADC_Channel::init(uint8_t chan, uint8_t adc_pin) {
    channel_num = chan;
    pin = adc_pin;
    
    setRange(BIPOLAR_10V);
    enabled = false;
}

void ADC_Channel::setRange(ADC_Range new_range) {
    range = new_range;
    float range_volts = getADCRangeVolts(range);
    voltage_scale = (2.0f * range_volts) / ADC_MAX_VALUE;
}

void ADC_Channel::setSampleInterval(float interval_us) {
    if (interval_us < MIN_T_RES) {
        sample_interval = MIN_T_RES;
    } else if (interval_us > MAX_T_RES) {
        sample_interval = MAX_T_RES;
    } else {
        sample_interval = interval_us;
    }
}

void ADC_Channel::enable() {
    enabled = true;
    resetStatistics();
}

void ADC_Channel::disable() {
    enabled = false;
}

bool ADC_Channel::readADC() {
    if (!enabled) return false;
    
    // Check if it's time to sample
    unsigned long current_time = micros();
    if (current_time - last_sample_time < sample_interval) {
        return false;
    }
    
    last_sample_time = current_time;
    
    // Read directly from data register (polled mode)
    // The ADC data is at base address + 2
    uint16_t address = 0x030 + 2 + ((channel_num - 1) * 4);  // ADC1 base = 0x030
    int16_t raw = (int16_t)readData(address);
    
    raw_value = (uint16_t)raw;
    
    // Convert to voltage based on range
    switch(range) {
        case BIPOLAR_1250mV:
            voltage_value = (raw * 1.25f) / 32768.0f;
            break;
        case BIPOLAR_2500mV:
            voltage_value = (raw * 2.5f) / 32768.0f;
            break;
        case BIPOLAR_5V:
            voltage_value = (raw * 5.0f) / 32768.0f;
            break;
        case BIPOLAR_10V:
        default:
            voltage_value = (raw * 10.24f) / 32768.0f;
            break;
    }
    
    // Update statistics
    sample_count++;
    if (voltage_value < min_voltage) min_voltage = voltage_value;
    if (voltage_value > max_voltage) max_voltage = voltage_value;
    
    return true;
}

void ADC_Channel::resetStatistics() {
    sample_count = 0;
    min_voltage = 999.9f;
    max_voltage = -999.9f;
}

////////////////////////////////////////////////////////////////////////////////////
// ADC_MANAGER IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////

ADC_Manager::ADC_Manager() {
    active_channel_mask = 0;
    global_sample_interval = DEFAULT_T_RES;
}

void ADC_Manager::init() {
    // Initialize all channels
    channels[0].init(1, ADC_PIN_1);
    channels[1].init(2, ADC_PIN_2);
    channels[2].init(3, ADC_PIN_3);
    channels[3].init(4, ADC_PIN_4);
    
    // Set global sample interval
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        channels[i].setSampleInterval(global_sample_interval);
    }
    
    active_channel_mask = 0;
}

ADC_Channel* ADC_Manager::getChannel(uint8_t chan) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return nullptr;
    return &channels[chan - 1];
}

void ADC_Manager::setGlobalSampleInterval(float interval_us) {
    global_sample_interval = interval_us;
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        channels[i].setSampleInterval(interval_us);
    }
}

void ADC_Manager::enableChannel(uint8_t chan) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return;
    
    channels[chan - 1].enable();
    active_channel_mask |= (1 << (chan - 1));
}

void ADC_Manager::disableChannel(uint8_t chan) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return;
    
    channels[chan - 1].disable();
    active_channel_mask &= ~(1 << (chan - 1));
}

void ADC_Manager::enableAllChannels() {
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        channels[i].enable();
    }
    active_channel_mask = 0x0F;
}

void ADC_Manager::disableAllChannels() {
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        channels[i].disable();
    }
    active_channel_mask = 0;
}

uint8_t ADC_Manager::getActiveChannelCount() const {
    uint8_t count = 0;
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        if (active_channel_mask & (1 << i)) count++;
    }
    return count;
}

void ADC_Manager::readAllChannels() {
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        if (channels[i].isEnabled()) {
            channels[i].readADC();
        }
    }
}

void ADC_Manager::printStatus(Stream& serial) {
    serial.println("=== ADC Manager Status ===");
    serial.printf("Active channels: %d (mask: 0x%02X)\n", 
                  getActiveChannelCount(), active_channel_mask);
    serial.printf("Sample interval: %.2f us\n", global_sample_interval);
    serial.println();
    
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        printChannelInfo(i + 1, serial);
    }
}

void ADC_Manager::printChannelInfo(uint8_t chan, Stream& serial) {
    ADC_Channel* ch = getChannel(chan);
    if (!ch) return;
    
    serial.printf("Channel %d: %s\n", chan, ch->isEnabled() ? "ENABLED" : "DISABLED");
    if (ch->isEnabled()) {
        serial.printf("  Pin: %d\n", ch->getPin());
        serial.printf("  Range: ±%.2f V\n", ch->getRangeVolts());
        serial.printf("  Current: %.4f V\n", ch->getVoltage());
        serial.printf("  Samples: %lu\n", ch->getSampleCount());
        serial.printf("  Min/Max: %.4f / %.4f V\n", 
                     ch->getMinVoltage(), ch->getMaxVoltage());
    }
    serial.println();
}