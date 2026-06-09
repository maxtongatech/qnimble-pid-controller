////////////////////////////////////////////////////////////////////////////////////
// square_wave.h - Square Wave Generator
// qNimble v2.0
////////////////////////////////////////////////////////////////////////////////////

#ifndef SQUARE_WAVE_H
#define SQUARE_WAVE_H

#include <Arduino.h>

#include "config.h"  // needed for NUM_ADC_CHANNELS

////////////////////////////////////////////////////////////////////////////////////
// SQUARE WAVE GENERATOR CLASS
////////////////////////////////////////////////////////////////////////////////////

class SquareWave_Generator {
private:
    unsigned long lastToggleTime;
    bool currentState;
    float period_ms;
    float dutyCycle;
    bool enabled;
    float highSetpoint;
    float lowSetpoint;
public:
    SquareWave_Generator();
    
    // Initialize with period in milliseconds and duty cycle (0.0 to 1.0)
    void begin(float period_ms, float dutyCycle = 0.5);
    
    // Update function - call this in loop()
    void update();
    
    // Getters and setters
    bool getState() const;
    void setPeriod(float period_ms);
    void setDutyCycle(float dutyCycle);
    void setEnabled(bool enabled);
    bool isEnabled() const;

    // Setpoint/Rail management
    void setHighSetpoint(float setpoint);       // set high setpoint individually (V)
    void setLowSetpoint(float setpoint);        // set low setpoint individually (V)
    void setRails(float low_v, float high_v);   // low and high setpoints simultaneously, asymmetrical (V)
    void setBipolarRails(float voltage);        // low and high setpoints simultaneously, symmetrical  (-V and +V)
    
    float getHighSetpoint() const;  // getters
    float getLowSetpoint() const;
    float getCurrentSetpoint() const;
    
    // Configuration
    void getConfig(float& period, float& duty, float& high_sp, float& low_sp) const; // set everything @ once
    
    
    // Reset the wave generator
    void reset();
};

////////////////////////////////////////////////////////////////////////////////////
// SQUARE WAVE MANAGER IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////

//So we can creat 4 separate objects per ADC channel

class SquareWave_Manager {
private:
    SquareWave_Generator generators[NUM_ADC_CHANNELS];
    bool initialized;

public:
    SquareWave_Manager();
    
    void init();
    bool isInitialized() const { return initialized; }
    
    void update();
    
    SquareWave_Generator* getGenerator(uint8_t channel);
    
    void enableChannel(uint8_t channel);
    void disableChannel(uint8_t channel);
    void disableAllChannels();
    
    void setChannelConfig(uint8_t channel, float period_ms, float duty_cycle);
    void setChannelSetpoints(uint8_t channel, float low_v, float high_v);
    void setChannelBipolarRails(uint8_t channel, float voltage);
    
    bool isChannelEnabled(uint8_t channel);
    float getCurrentSetpoint(uint8_t channel);
    
    void printStatus(Stream& serial);
    void printChannelStatus(Stream& serial, uint8_t channel);
};


#endif // SQUARE_WAVE_H