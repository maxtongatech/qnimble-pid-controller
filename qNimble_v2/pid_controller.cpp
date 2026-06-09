////////////////////////////////////////////////////////////////////////////////////
// pid_controller.cpp - PID Controller Implementation
// qNimble v2.0
////////////////////////////////////////////////////////////////////////////////////

#include "pid_controller.h"
#include <math.h>

////////////////////////////////////////////////////////////////////////////////////
// PID_CONTROLLER IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////

PID_Controller::PID_Controller() {
    channel_num = 0;
    
    p_gain = DEFAULT_P_GAIN;
    i_gain = DEFAULT_I_GAIN;
    d_gain = DEFAULT_D_GAIN;
    
    setpoint = DEFAULT_SETPOINT;
    error = 0.0f;
    prev_error = 0.0f;
    
    proportional = 0.0f;
    integral = 0.0f;
    derivative = 0.0f;
    
    output = 0.0f;
    output_offset = 0.0f;
    
    rail_min = DEFAULT_RAIL_MIN;
    rail_max = DEFAULT_RAIL_MAX;
    integral_min = DEFAULT_RAIL_MIN;
    integral_max = DEFAULT_RAIL_MAX;
    
    lock_precision = DEFAULT_LOCK_PRECISION;
    is_locked = false;
    
    enabled = false;
    reset_integral = false;
    
    dt = DEFAULT_T_RES / 1000000.0f;  // Convert microseconds to seconds
    last_update = 0;
    
    update_count = 0;
    max_error = -999.9f;
    min_error = 999.9f;
    rms_error = 0.0f;
    sum_squared_error = 0.0f;
}

void PID_Controller::init(uint8_t chan) {
    channel_num = chan;
    reset();
}

////////////////////////////////////////////////////////////////////////////////////
// CONFIGURATION
////////////////////////////////////////////////////////////////////////////////////

void PID_Controller::setGains(float p, float i, float d) {
    p_gain = p;
    i_gain = i;
    d_gain = d;
}

void PID_Controller::getGains(float& p, float& i, float& d) {
    p = p_gain;
    i = i_gain;
    d = d_gain;
}

void PID_Controller::setSetpoint(float sp) {
    setpoint = sp;
}

void PID_Controller::setRails(float min_v, float max_v) {
    rail_min = min_v;
    rail_max = max_v;
}

void PID_Controller::getRails(float& min_v, float& max_v) {
    min_v = rail_min;
    max_v = rail_max;
}

void PID_Controller::setIntegralLimits(float min_v, float max_v) {
    integral_min = min_v;
    integral_max = max_v;
}

void PID_Controller::setLockPrecision(float precision) {
    lock_precision = fabs(precision);
}

void PID_Controller::setOutputOffset(float offset) {
    output_offset = offset;
}

void PID_Controller::setTimeStep(float dt_seconds) {
    dt = dt_seconds;
}

////////////////////////////////////////////////////////////////////////////////////
// CONTROL
////////////////////////////////////////////////////////////////////////////////////

void PID_Controller::enable() {
    enabled = true;
    reset();
}

void PID_Controller::disable() {
    enabled = false;
    output = 0.0f;
}

void PID_Controller::resetIntegral() {
    integral = 0.0f;
    reset_integral = false;
}

void PID_Controller::reset() {
    error = 0.0f;
    prev_error = 0.0f;
    proportional = 0.0f;
    integral = 0.0f;
    derivative = 0.0f;
    output = 0.0f;
    is_locked = false;
    resetStatistics();
}

////////////////////////////////////////////////////////////////////////////////////
// UPDATE
////////////////////////////////////////////////////////////////////////////////////

float PID_Controller::update(float input) {
    if (!enabled) {
        return 0.0f;
    }
    
    // Calculate error
    error = setpoint - input;
    
    // Proportional term
    proportional = p_gain * error;
    
    // Integral term with anti-windup
    if (reset_integral) {
        resetIntegral();
    }
    
    integral += i_gain * error * dt;
    
    // Clamp integral to prevent windup
    if (integral > integral_max) {
        integral = integral_max;
    } else if (integral < integral_min) {
        integral = integral_min;
    }
    
    // Derivative term
    derivative = d_gain * (error - prev_error) / dt;
    
    // Calculate output
    output = proportional + integral + derivative + output_offset;
    
    // Apply rail limits
    if (output > rail_max) {
        output = rail_max;
    } else if (output < rail_min) {
        output = rail_min;
    }
    
    // Check lock status
    if (fabs(error) <= lock_precision) {
        is_locked = true;
    } else {
        is_locked = false;
    }
    
    // Update statistics
    update_count++;
    if (error > max_error) max_error = error;
    if (error < min_error) min_error = error;
    sum_squared_error += error * error;
    
    // Save for next iteration
    prev_error = error;
    last_update = micros();
    
    return output;
}

////////////////////////////////////////////////////////////////////////////////////
// STATISTICS
////////////////////////////////////////////////////////////////////////////////////

float PID_Controller::getRMSError() {
    if (update_count == 0) return 0.0f;
    rms_error = sqrt(sum_squared_error / update_count);
    return rms_error;
}

void PID_Controller::resetStatistics() {
    update_count = 0;
    max_error = -999.9f;
    min_error = 999.9f;
    sum_squared_error = 0.0f;
    rms_error = 0.0f;
}

////////////////////////////////////////////////////////////////////////////////////
// DIAGNOSTICS
////////////////////////////////////////////////////////////////////////////////////

void PID_Controller::printStatus(Stream& serial) {
    serial.printf("=== PID Controller %d ===\n", channel_num);
    serial.printf("Status: %s\n", enabled ? "ENABLED" : "DISABLED");
    serial.printf("Locked: %s\n", is_locked ? "YES" : "NO");
    serial.println();
    
    serial.println("--- Gains ---");
    serial.printf("P: %.6f\n", p_gain);
    serial.printf("I: %.6f\n", i_gain);
    serial.printf("D: %.6f\n", d_gain);
    serial.println();
    
    serial.println("--- Setpoint & Error ---");
    serial.printf("Setpoint: %.4f V\n", setpoint);
    serial.printf("Error: %.4f V\n", error);
    serial.printf("Lock precision: %.4f V\n", lock_precision);
    serial.println();
    
    serial.println("--- PID Terms ---");
    serial.printf("Proportional: %.4f\n", proportional);
    serial.printf("Integral: %.4f\n", integral);
    serial.printf("Derivative: %.4f\n", derivative);
    serial.printf("Offset: %.4f\n", output_offset);
    serial.printf("Output: %.4f V\n", output);
    serial.println();
    
    serial.println("--- Limits ---");
    serial.printf("Rails: [%.2f, %.2f] V\n", rail_min, rail_max);
    serial.printf("Integral: [%.2f, %.2f]\n", integral_min, integral_max);
    serial.println();
    
    serial.println("--- Statistics ---");
    serial.printf("Updates: %lu\n", update_count);
    serial.printf("Error range: [%.4f, %.4f] V\n", min_error, max_error);
    serial.printf("RMS error: %.4f V\n", getRMSError());
    serial.printf("dt: %.6f s (%.2f Hz)\n", dt, 1.0f/dt);
    serial.println();
}

////////////////////////////////////////////////////////////////////////////////////
// SERVO_MANAGER IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////

Servo_Manager::Servo_Manager() {
    adc_mgr = nullptr;
    dac_mgr = nullptr;
    
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        servo_active[i] = false;
    }
    
    last_servo_time = 0;
    servo_rate = 10000.0f;  // 10 kHz default
    servo_dt = 1.0f / servo_rate;
    
    servo_cycle_count = 0;
    max_cycle_time = 0.0f;
    avg_cycle_time = 0.0f;
}

void Servo_Manager::init(ADC_Manager* adc, DAC_Manager* dac) {
    adc_mgr = adc;
    dac_mgr = dac;
    
    // Initialize all controllers
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        controllers[i].init(i + 1);
        controllers[i].setTimeStep(servo_dt);
    }
}

////////////////////////////////////////////////////////////////////////////////////
// CONFIGURATION
////////////////////////////////////////////////////////////////////////////////////

void Servo_Manager::setServoRate(float rate_hz) {
    servo_rate = rate_hz;
    servo_dt = 1.0f / servo_rate;
    
    // Update all controllers
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        controllers[i].setTimeStep(servo_dt);
    }
}

PID_Controller* Servo_Manager::getController(uint8_t chan) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return nullptr;
    return &controllers[chan - 1];
}

////////////////////////////////////////////////////////////////////////////////////
// SERVO CONTROL
////////////////////////////////////////////////////////////////////////////////////

void Servo_Manager::enableServo(uint8_t chan) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return;
    
    servo_active[chan - 1] = true;
    controllers[chan - 1].enable();
}

void Servo_Manager::disableServo(uint8_t chan) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return;
    
    servo_active[chan - 1] = false;
    controllers[chan - 1].disable();
    
    // Set DAC output to 0
    DAC_Channel* dac = dac_mgr->getChannel(chan);
    if (dac) {
        dac->setVoltage(0.0f);
    }
}

bool Servo_Manager::isServoActive(uint8_t chan) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return false;
    return servo_active[chan - 1];
}

void Servo_Manager::enableAllServos() {
    for (int i = 1; i <= NUM_ADC_CHANNELS; i++) {
        enableServo(i);
    }
}

void Servo_Manager::disableAllServos() {
    for (int i = 1; i <= NUM_ADC_CHANNELS; i++) {
        disableServo(i);
    }
}

////////////////////////////////////////////////////////////////////////////////////
// UPDATE
////////////////////////////////////////////////////////////////////////////////////

void Servo_Manager::update() {
    unsigned long start_time = micros();
    
    // Check if it's time to update
    float elapsed = (start_time - last_servo_time) / 1000000.0f;
    if (elapsed < servo_dt) {
        return;  // Not time yet
    }
    
    last_servo_time = start_time;
    
    // Update all active servos
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        if (servo_active[i]) {
            // Get ADC input
            ADC_Channel* adc = adc_mgr->getChannel(i + 1);
            if (!adc || !adc->isEnabled()) continue;
            
            float input = adc->getVoltage();
            
            // Update PID controller
            float output = controllers[i].update(input);
            
            // Set DAC output
            DAC_Channel* dac = dac_mgr->getChannel(i + 1);
            if (dac && dac->isEnabled()) {
                dac->setVoltage(output);
            }
        }
    }
    
    // Performance monitoring
    unsigned long end_time = micros();
    float cycle_time = end_time - start_time;
    
    servo_cycle_count++;
    if (cycle_time > max_cycle_time) {
        max_cycle_time = cycle_time;
    }
    avg_cycle_time = ((avg_cycle_time * (servo_cycle_count - 1)) + cycle_time) / servo_cycle_count;
}

////////////////////////////////////////////////////////////////////////////////////
// DIAGNOSTICS
////////////////////////////////////////////////////////////////////////////////////

void Servo_Manager::printStatus(Stream& serial) {
    serial.println("\n=== Servo Manager Status ===");
    serial.printf("Servo rate: %.2f Hz (dt = %.6f s)\n", servo_rate, servo_dt);
    serial.printf("Active servos: ");
    
    int active_count = 0;
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        if (servo_active[i]) {
            serial.printf("%d ", i + 1);
            active_count++;
        }
    }
    if (active_count == 0) {
        serial.print("None");
    }
    serial.println("\n");
    
    // Print each active controller
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        if (servo_active[i]) {
            controllers[i].printStatus(serial);
        }
    }
}

void Servo_Manager::printPerformance(Stream& serial) {
    serial.println("\n=== Servo Performance ===");
    serial.printf("Cycle count: %lu\n", servo_cycle_count);
    serial.printf("Max cycle time: %.2f us\n", max_cycle_time);
    serial.printf("Avg cycle time: %.2f us\n", avg_cycle_time);
    serial.printf("Target cycle time: %.2f us\n", servo_dt * 1000000.0f);
    
    float cpu_usage = (avg_cycle_time / (servo_dt * 1000000.0f)) * 100.0f;
    serial.printf("CPU usage: %.1f%%\n", cpu_usage);
    serial.println();
}