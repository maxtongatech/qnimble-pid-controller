////////////////////////////////////////////////////////////////////////////////////
// pid_controller.h - PID Controller
// qNimble v2.0
////////////////////////////////////////////////////////////////////////////////////

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "config.h"
#include "hal_adc.h"
#include "hal_dac.h"

////////////////////////////////////////////////////////////////////////////////////
// PID CONTROLLER CLASS
////////////////////////////////////////////////////////////////////////////////////

class PID_Controller {
private:
    uint8_t channel_num;          // 1-4
    
    // PID gains
    float p_gain;
    float i_gain;
    float d_gain;
    
    // Setpoint and error
    float setpoint;
    float error;
    float prev_error;
    
    // PID terms
    float proportional;
    float integral;
    float derivative;
    
    // Output
    float output;
    float output_offset;          // DC offset added to output
    
    // Limits
    float rail_min;
    float rail_max;
    float integral_min;           // Anti-windup limits
    float integral_max;
    
    // Lock detection
    float lock_precision;         // Error threshold for "locked"
    bool is_locked;
    
    // State
    bool enabled;
    bool reset_integral;
    
    // Timing
    float dt;                     // Time step in seconds
    unsigned long last_update;
    
    // Statistics
    uint32_t update_count;
    float max_error;
    float min_error;
    float rms_error;
    float sum_squared_error;

public:
    // Constructor
    PID_Controller();
    
    // Initialization
    void init(uint8_t chan);
    
    // Configuration
    void setGains(float p, float i, float d);
    void getGains(float& p, float& i, float& d);
    
    void setSetpoint(float sp);
    float getSetpoint() const { return setpoint; }
    
    void setRails(float min_v, float max_v);
    void getRails(float& min_v, float& max_v);
    
    void setIntegralLimits(float min_v, float max_v);
    
    void setLockPrecision(float precision);
    float getLockPrecision() const { return lock_precision; }
    
    void setOutputOffset(float offset);
    float getOutputOffset() const { return output_offset; }
    
    void setTimeStep(float dt_seconds);
    float getTimeStep() const { return dt; }
    
    // Control
    void enable();
    void disable();
    bool isEnabled() const { return enabled; }
    
    void resetIntegral();
    void reset();  // Reset all state
    
    // Update (call this in servo loop)
    float update(float input);
    
    // Status
    float getError() const { return error; }
    float getOutput() const { return output; }
    
    float getProportional() const { return proportional; }
    float getIntegral() const { return integral; }
    float getDerivative() const { return derivative; }
    
    bool isLocked() const { return is_locked; }
    
    // Statistics
    uint32_t getUpdateCount() const { return update_count; }
    float getMaxError() const { return max_error; }
    float getMinError() const { return min_error; }
    float getRMSError();
    void resetStatistics();
    
    // Info
    uint8_t getChannelNum() const { return channel_num; }
    
    // Diagnostics
    void printStatus(Stream& serial);
};

////////////////////////////////////////////////////////////////////////////////////
// SERVO MANAGER CLASS
////////////////////////////////////////////////////////////////////////////////////

class Servo_Manager {
private:
    PID_Controller controllers[NUM_ADC_CHANNELS];
    
    ADC_Manager* adc_mgr;
    DAC_Manager* dac_mgr;
    
    bool servo_active[NUM_ADC_CHANNELS];
    unsigned long last_servo_time;
    float servo_rate;             // Hz
    float servo_dt;               // seconds
    
    // Performance monitoring
    uint32_t servo_cycle_count;
    float max_cycle_time;         // microseconds
    float avg_cycle_time;
    
public:
    // Constructor
    Servo_Manager();
    
    // Initialization
    void init(ADC_Manager* adc, DAC_Manager* dac);
    
    // Configuration
    void setServoRate(float rate_hz);
    float getServoRate() const { return servo_rate; }
    
    // Controller access
    PID_Controller* getController(uint8_t chan);  // 1-indexed
    
    // Servo control
    void enableServo(uint8_t chan);
    void disableServo(uint8_t chan);
    bool isServoActive(uint8_t chan);
    
    void enableAllServos();
    void disableAllServos();
    
    // Update (call this in main loop or ISR)
    void update();
    
    // Diagnostics
    void printStatus(Stream& serial);
    void printPerformance(Stream& serial);
};

#endif // PID_CONTROLLER_H
