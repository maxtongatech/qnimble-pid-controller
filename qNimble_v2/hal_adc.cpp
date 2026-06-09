////////////////////////////////////////////////////////////////////////////////////
// hal_adc.cpp - ADC Hardware Abstraction Layer Implementation
// qNimble v2.0 - ISR mode with global storage
////////////////////////////////////////////////////////////////////////////////////

#include "hal_adc.h"

extern "C" {
    #include "adc.h"
    #include "comm.h"
}

////////////////////////////////////////////////////////////////////////////////////
// GLOBAL STORAGE FOR ISR DATA
////////////////////////////////////////////////////////////////////////////////////

static volatile double g_adc_voltages[4] = {0.0, 0.0, 0.0, 0.0};
static volatile bool g_adc_data_ready[4] = {false, false, false, false};

////////////////////////////////////////////////////////////////////////////////////
// ISR CALLBACKS - Store data in globals
////////////////////////////////////////////////////////////////////////////////////

extern "C" {
    void adc1_dummy_isr(void) {
        g_adc_voltages[0] = readADC1_from_ISR();
        g_adc_data_ready[0] = true;
    }
    
    void adc2_dummy_isr(void) {
        g_adc_voltages[1] = readADC2_from_ISR();
        g_adc_data_ready[1] = true;
    }
    
    void adc3_dummy_isr(void) {
        g_adc_voltages[2] = readADC3_from_ISR();
        g_adc_data_ready[2] = true;
    }
    
    void adc4_dummy_isr(void) {
        g_adc_voltages[3] = readADC4_from_ISR();
        g_adc_data_ready[3] = true;
    }
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
    
    // If already enabled, reconfigure FPGA
    if (enabled) {
        enable();  // Re-enable with new range
    }
}

void ADC_Channel::setSampleInterval(float interval_us) {
    if (interval_us < MIN_T_RES) {
        sample_interval = MIN_T_RES;
    } else if (interval_us > MAX_T_RES) {
        sample_interval = MAX_T_RES;
    } else {
        sample_interval = interval_us;
    }
    
    // If already enabled, reconfigure FPGA
    if (enabled) {
        enable();  // Re-enable with new interval
    }
}

void ADC_Channel::enable() {
    enabled = true;
    
    // Configure FPGA to start sampling this ADC
    uint16_t sample_us = (uint16_t)(sample_interval);
    adc_scale scale_enum = (adc_scale)range;
    
    // Get appropriate ISR callback
    void (*callback)(void) = nullptr;
    switch(channel_num) {
        case 1: callback = adc1_dummy_isr; break;
        case 2: callback = adc2_dummy_isr; break;
        case 3: callback = adc3_dummy_isr; break;
        case 4: callback = adc4_dummy_isr; break;
    }
    
    if (callback) {
        // This configures the FPGA to sample the ADC and call ISR
        configureADC(channel_num, sample_us, 0, scale_enum, callback);
    }
    
    resetStatistics();
}

void ADC_Channel::disable() {
    enabled = false;
    
    // Disable FPGA ADC sampling
    disableADC(channel_num);
}

bool ADC_Channel::readADC() {
    if (!enabled) return false;
    
    // Check if it's time to sample
    unsigned long current_time = micros();
    if (current_time - last_sample_time < sample_interval) {
        return false;
    }
    
    last_sample_time = current_time;
    
    // Read from ISR-updated global variable
    int idx = channel_num - 1;
    
    if (g_adc_data_ready[idx]) {
        // Get voltage from ISR
        voltage_value = g_adc_voltages[idx];
        g_adc_data_ready[idx] = false;  // Clear flag
        
        // Update statistics
        sample_count++;
        if (voltage_value < min_voltage) min_voltage = voltage_value;
        if (voltage_value > max_voltage) max_voltage = voltage_value;
        
        return true;
    }
    
    return false;
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