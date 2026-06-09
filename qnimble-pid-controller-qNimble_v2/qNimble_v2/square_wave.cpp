////////////////////////////////////////////////////////////////////////////////////
// square_wave.cpp - Square Wave Generator Implementation
// qNimble v2.0
////////////////////////////////////////////////////////////////////////////////////

#include "square_wave.h"

////////////////////////////////////////////////////////////////////////////////////
// SQUARE WAVE GENERATOR IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////

SquareWave_Generator::SquareWave_Generator() 
    : lastToggleTime(0), currentState(false), period_ms(1000.0), 
      dutyCycle(0.5), enabled(false), highSetpoint(0.0), lowSetpoint(0.0) {
}

void SquareWave_Generator::begin(float period_ms, float dutyCycle) {
    this->period_ms = period_ms;
    this->dutyCycle = constrain(dutyCycle, 0.0, 1.0);
    this->lastToggleTime = millis();
    this->currentState = false;
    this->enabled = true;
}

void SquareWave_Generator::update() {
    if (!enabled) {
        currentState = false;
        return;
    }
    
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - lastToggleTime;
    
    if (currentState) {
        // Currently HIGH, check if HIGH period is over
        if (elapsed >= (period_ms * dutyCycle)) {
            currentState = false;
            lastToggleTime = currentTime;
        }
    } else {
        // Currently LOW, check if LOW period is over
        if (elapsed >= (period_ms * (1.0 - dutyCycle))) {
            currentState = true;
            lastToggleTime = currentTime;
        }
    }
}

bool SquareWave_Generator::getState() const {
    return currentState;
}

// Before you ask "where's the setFrequency() function, Nikki? We sorta need that!": check in command_handler, you can call this command using Hz (since T = 1/f)
void SquareWave_Generator::setPeriod(float period_ms) {
    this->period_ms = period_ms;
}

void SquareWave_Generator::setDutyCycle(float dutyCycle) {
    this->dutyCycle = constrain(dutyCycle, 0.0, 1.0);
}

void SquareWave_Generator::setEnabled(bool enabled) {
    this->enabled = enabled;
    if (!enabled) {
        currentState = false;
    }
}

bool SquareWave_Generator::isEnabled() const {
    return enabled;
}

void SquareWave_Generator::setHighSetpoint(float setpoint) {
    this->highSetpoint = setpoint;
}

void SquareWave_Generator::setLowSetpoint(float setpoint) {
    this->lowSetpoint = setpoint;
}

// Convenience function - just sets both setpoints
void SquareWave_Generator::setRails(float low_v, float high_v) {
    this->lowSetpoint = low_v;
    this->highSetpoint = high_v;
}

// Convenience function - sets symmetric ±voltage
void SquareWave_Generator::setBipolarRails(float voltage) {
    this->lowSetpoint = -abs(voltage);
    this->highSetpoint = abs(voltage);
}

float SquareWave_Generator::getHighSetpoint() const {
    return highSetpoint;
}

float SquareWave_Generator::getLowSetpoint() const {
    return lowSetpoint;
}

float SquareWave_Generator::getCurrentSetpoint() const {
    return currentState ? highSetpoint : lowSetpoint;
}

void SquareWave_Generator::getConfig(float& period, float& duty, float& high_sp, float& low_sp) const {
    period = period_ms;
    duty = dutyCycle;
    high_sp = highSetpoint;
    low_sp = lowSetpoint;
}

void SquareWave_Generator::reset() {
    lastToggleTime = millis();
    currentState = false;
}

////////////////////////////////////////////////////////////////////////////////////
// SQUARE WAVE MANAGER IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////

SquareWave_Manager::SquareWave_Manager() {
    initialized = false;
}

void SquareWave_Manager::init() {
    Serial.println("[SQW] Initializing Square Wave Manager...");
    
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        generators[i].begin(0.5f, 0.5f);  // 2 kHz, 50% duty
        generators[i].setEnabled(false);
        generators[i].setHighSetpoint(0.0f);
        generators[i].setLowSetpoint(0.0f);
        
        Serial.printf("[SQW]   Channel %d initialized\n", i + 1);
    }
    
    initialized = true;
    Serial.println("[SQW] ✓ Square Wave Manager ready");
}

void SquareWave_Manager::update() {
    if (!initialized) return;
    
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        if (generators[i].isEnabled()) {
            generators[i].update();
        }
    }
}

SquareWave_Generator* SquareWave_Manager::getGenerator(uint8_t channel) {
    if (channel < 1 || channel > NUM_ADC_CHANNELS) {
        return nullptr;
    }
    return &generators[channel - 1];
}

void SquareWave_Manager::enableChannel(uint8_t channel) {
    if (channel < 1 || channel > NUM_ADC_CHANNELS) return;
    
    generators[channel - 1].setEnabled(true);
    generators[channel - 1].reset();
}

void SquareWave_Manager::disableChannel(uint8_t channel) {
    if (channel < 1 || channel > NUM_ADC_CHANNELS) return;
    
    generators[channel - 1].setEnabled(false);
}

void SquareWave_Manager::disableAllChannels() {
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        generators[i].setEnabled(false);
    }
}

void SquareWave_Manager::setChannelConfig(uint8_t channel, float period_ms, float duty_cycle) {
    if (channel < 1 || channel > NUM_ADC_CHANNELS) return;
    
    generators[channel - 1].begin(period_ms, duty_cycle);
}

void SquareWave_Manager::setChannelSetpoints(uint8_t channel, float low_v, float high_v) {
    if (channel < 1 || channel > NUM_ADC_CHANNELS) return;
    
    generators[channel - 1].setLowSetpoint(low_v);
    generators[channel - 1].setHighSetpoint(high_v);
}

void SquareWave_Manager::setChannelBipolarRails(uint8_t channel, float voltage) {
    if (channel < 1 || channel > NUM_ADC_CHANNELS) return;
    
    generators[channel - 1].setBipolarRails(voltage);
}

bool SquareWave_Manager::isChannelEnabled(uint8_t channel) {
    if (channel < 1 || channel > NUM_ADC_CHANNELS) return false;
    
    return generators[channel - 1].isEnabled();
}

float SquareWave_Manager::getCurrentSetpoint(uint8_t channel) {
    if (channel < 1 || channel > NUM_ADC_CHANNELS) return 0.0f;
    
    return generators[channel - 1].getCurrentSetpoint();
}

void SquareWave_Manager::printStatus(Stream& serial) {
    serial.println("\n=== Square Wave Manager Status ===");
    serial.printf("Initialized: %s\n\n", initialized ? "YES" : "NO");
    
    for (int i = 1; i <= NUM_ADC_CHANNELS; i++) {
        printChannelStatus(serial, i);
    }
    
    serial.println();
}

void SquareWave_Manager::printChannelStatus(Stream& serial, uint8_t channel) {
    if (channel < 1 || channel > NUM_ADC_CHANNELS) return;
    
    SquareWave_Generator* gen = &generators[channel - 1];
    
    float period, duty, high_sp, low_sp;
    gen->getConfig(period, duty, high_sp, low_sp);
    
    serial.printf("Channel %d:\n", channel);
    serial.printf("  Enabled:    %s\n", gen->isEnabled() ? "YES" : "NO");
    serial.printf("  State:      %s\n", gen->getState() ? "HIGH" : "LOW");
    serial.printf("  Period:     %.3f ms\n", period);
    serial.printf("  Frequency:  %.2f Hz\n", 1000.0f / period);
    serial.printf("  Duty Cycle: %.1f%%\n", duty * 100.0f);
    serial.printf("  High Level: %+.4f V\n", high_sp);
    serial.printf("  Low Level:  %+.4f V\n", low_sp);
    serial.printf("  Current SP: %+.4f V\n", gen->getCurrentSetpoint());
    serial.println();
}