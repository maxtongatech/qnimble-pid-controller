////////////////////////////////////////////////////////////////////////////////////
// qNimble_v2.ino - Main Firmware
// qNimble v2.0 - Quantum Opus LLC
////////////////////////////////////////////////////////////////////////////////////

#include "config.h"
#include "hal_adc.h"
#include "hal_dac.h"
#include "nvm_manager.h"
#include "pid_controller.h"
#include "command_handler.h"

////////////////////////////////////////////////////////////////////////////////////
// GLOBAL OBJECTS
////////////////////////////////////////////////////////////////////////////////////

ADC_Manager adc_manager;
DAC_Manager dac_manager;
NVM_Manager nvm_manager;
Servo_Manager servo_manager;
Command_Handler command_handler;

// Status flags
bool quick_status_enabled = false;
unsigned long last_quick_status = 0;

////////////////////////////////////////////////////////////////////////////////////
// SETUP
////////////////////////////////////////////////////////////////////////////////////

void setup() {
    Serial.begin(115200);
    delay(2000);  // Give serial time to connect
    
    Serial.println("\n\n====================================");
    Serial.println("qNimble v2.0 - Servo Controller");
    Serial.println("Quantum Opus LLC");
    Serial.println("====================================\n");
    
    // Initialize NVM manager first
    Serial.println("Initializing NVM Manager...");
    nvm_manager.init();
    
    if (nvm_manager.isConfigValid()) {
        Serial.println("✓ Valid configuration loaded from memory");
    } else {
        Serial.println("⚠ No valid config found, using defaults");
    }
    
    // Initialize hardware
    Serial.println("\nInitializing ADC Manager...");
    adc_manager.init();
    
    Serial.println("Initializing DAC Manager...");
    dac_manager.init();
    
    Serial.println("Initializing Servo Manager...");
    servo_manager.init(&adc_manager, &dac_manager);
    
    Serial.println("Initializing Command Handler...");
    command_handler.init(&adc_manager, &dac_manager, &nvm_manager, &servo_manager, &Serial);
    Serial.println("✓ Command handler ready");
    
    // Apply saved configuration to hardware
    applyConfigToHardware();
    
    Serial.println("\n✓ Initialization complete!");
    Serial.println("⚠ Servos are DISABLED by default for safety");
    Serial.println("Type 'help' for command list\n");
}

////////////////////////////////////////////////////////////////////////////////////
// MAIN LOOP
////////////////////////////////////////////////////////////////////////////////////

void loop() {
    // Update servo loops (runs at configured rate)
    servo_manager.update();
    
    // Read ADC channels
    adc_manager.readAllChannels();
    
    // Handle serial commands
    command_handler.update();
    
    // Quick status output (if enabled)
    if (quick_status_enabled) {
        unsigned long now = millis();
        if (now - last_quick_status >= 100) {  // 10 Hz update
            last_quick_status = now;
            printQuickStatus();
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////

void applyConfigToHardware() {
    Serial.println("\nApplying saved configuration...");
    
    // Apply global time resolution
    float t_res = nvm_manager.getGlobalTimeRes();
    adc_manager.setGlobalSampleInterval(t_res);
    servo_manager.setServoRate(1000000.0f / t_res);  // Convert to Hz
    
    // Apply configuration to each channel
    for (int i = 1; i <= NUM_ADC_CHANNELS; i++) {
        // ADC configuration
        ADC_Channel* adc = adc_manager.getChannel(i);
        if (adc) {
            adc->setRange(nvm_manager.getADCRange(i));
            if (nvm_manager.getADCEnabled(i)) {
                adc_manager.enableChannel(i);
            }
        }
        
        // DAC configuration
        DAC_Channel* dac = dac_manager.getChannel(i);
        if (dac) {
            float min_v, max_v;
            nvm_manager.getDACRails(i, min_v, max_v);
            dac->setRails(min_v, max_v);
            if (nvm_manager.getDACEnabled(i)) {
                dac_manager.enableChannel(i);
            }
        }
        
        // PID configuration
        PID_Controller* pid = servo_manager.getController(i);
        if (pid) {
            float p, i_gain, d;
            nvm_manager.getPIDGains(i, p, i_gain, d);
            pid->setGains(p, i_gain, d);
            
            pid->setSetpoint(nvm_manager.getSetpoint(i));
            
            float rail_min, rail_max;
            nvm_manager.getRails(i, rail_min, rail_max);
            pid->setRails(rail_min, rail_max);
            
            pid->setLockPrecision(nvm_manager.getLockPrecision(i));
        }
    }
    
    Serial.println("✓ Configuration applied");
}

void printQuickStatus() {
    // Print compact status: channel,servo,adc,dac,error,locked
    for (int i = 1; i <= NUM_ADC_CHANNELS; i++) {
        ADC_Channel* adc = adc_manager.getChannel(i);
        DAC_Channel* dac = dac_manager.getChannel(i);
        PID_Controller* pid = servo_manager.getController(i);
        
        if (adc && dac && pid) {
            Serial.printf("%d,%d,%.6f,%.6f,%.6f,%d\n",
                i,
                servo_manager.isServoActive(i) ? 1 : 0,
                adc->getVoltage(),
                dac->getVoltage(),
                pid->getError(),
                pid->isLocked() ? 1 : 0);
        }
    }
}

void printSystemStatus() {
    Serial.println("\n=== System Status ===");
    Serial.printf("Uptime: %lu ms\n", millis());
    Serial.printf("Servo Rate: %.2f Hz\n", servo_manager.getServoRate());
    Serial.printf("Active ADCs: %d\n", adc_manager.getActiveChannelCount());
    Serial.printf("Active DACs: %d\n", dac_manager.getActiveChannelCount());
    Serial.println();
    
    // Print channel status
    for (int i = 1; i <= NUM_ADC_CHANNELS; i++) {
        ADC_Channel* adc = adc_manager.getChannel(i);
        DAC_Channel* dac = dac_manager.getChannel(i);
        PID_Controller* pid = servo_manager.getController(i);
        
        if (adc && dac && pid) {
            Serial.printf("Channel %d:\n", i);
            Serial.printf("  ADC: %s (%.4f V)\n", 
                adc->isEnabled() ? "ON " : "OFF", adc->getVoltage());
            Serial.printf("  DAC: %s (%.4f V)\n", 
                dac->isEnabled() ? "ON " : "OFF", dac->getVoltage());
            Serial.printf("  Servo: %s", 
                servo_manager.isServoActive(i) ? "ACTIVE" : "IDLE");
            if (servo_manager.isServoActive(i)) {
                Serial.printf(" (SP=%.4f, Err=%.4f, %s)\n",
                    pid->getSetpoint(),
                    pid->getError(),
                    pid->isLocked() ? "LOCKED" : "UNLOCKED");
            } else {
                Serial.println();
            }
        }
    }
    Serial.println();
}

////////////////////////////////////////////////////////////////////////////////////
// DIAGNOSTIC FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////

void printMemoryUsage() {
    Serial.println("\n=== Memory Usage ===");
    
    // Calculate approximate memory usage
    size_t adc_size = sizeof(ADC_Manager);
    size_t dac_size = sizeof(DAC_Manager);
    size_t nvm_size = sizeof(NVM_Manager);
    size_t servo_size = sizeof(Servo_Manager);
    size_t cmd_size = sizeof(Command_Handler);
    
    size_t total = adc_size + dac_size + nvm_size + servo_size + cmd_size;
    
    Serial.printf("ADC Manager:     %d bytes\n", adc_size);
    Serial.printf("DAC Manager:     %d bytes\n", dac_size);
    Serial.printf("NVM Manager:     %d bytes\n", nvm_size);
    Serial.printf("Servo Manager:   %d bytes\n", servo_size);
    Serial.printf("Command Handler: %d bytes\n", cmd_size);
    Serial.printf("Total:           %d bytes\n", total);
    Serial.println();
}

void printPerformanceStats() {
    Serial.println("\n=== Performance Statistics ===");
    servo_manager.printPerformance(Serial);
    Serial.println();
}

////////////////////////////////////////////////////////////////////////////////////
// ERROR HANDLERS
////////////////////////////////////////////////////////////////////////////////////

void handleError(const char* error_msg) {
    Serial.print("ERROR: ");
    Serial.println(error_msg);
    
    // Could add LED indication, logging, etc.
}

void handleWarning(const char* warning_msg) {
    Serial.print("WARNING: ");
    Serial.println(warning_msg);
}

////////////////////////////////////////////////////////////////////////////////////
// SAFETY FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////

void emergencyStop() {
    Serial.println("\n!!! EMERGENCY STOP !!!");
    
    // Disable all servos
    servo_manager.disableAllServos();
    
    // Set all DACs to 0V
    for (int i = 1; i <= NUM_DAC_CHANNELS; i++) {
        DAC_Channel* dac = dac_manager.getChannel(i);
        if (dac && dac->isEnabled()) {
            dac->setVoltage(0.0f);
        }
    }
    
    Serial.println("All servos disabled, DACs set to 0V");
}

bool checkSafetyLimits() {
    // Check for any out-of-range conditions
    for (int i = 1; i <= NUM_ADC_CHANNELS; i++) {
        ADC_Channel* adc = adc_manager.getChannel(i);
        if (adc && adc->isEnabled()) {
            float voltage = adc->getVoltage();
            float range = adc->getRangeVolts();
            
            // Check if voltage exceeds range (with 10% margin)
            if (abs(voltage) > range * 1.1f) {
                Serial.printf("WARNING: ADC%d voltage (%.2f V) exceeds range!\n", 
                    i, voltage);
                return false;
            }
        }
    }
    
    return true;
}

////////////////////////////////////////////////////////////////////////////////////
// CALIBRATION FUNCTIONS (for future use)
////////////////////////////////////////////////////////////////////////////////////

void calibrateADC(uint8_t channel) {
    Serial.printf("Calibrating ADC channel %d...\n", channel);
    // TODO: Implement ADC calibration routine
    Serial.println("Calibration not yet implemented");
}

void calibrateDAC(uint8_t channel) {
    Serial.printf("Calibrating DAC channel %d...\n", channel);
    // TODO: Implement DAC calibration routine
    Serial.println("Calibration not yet implemented");
}

////////////////////////////////////////////////////////////////////////////////////
// UTILITY FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float constrainFloat(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

////////////////////////////////////////////////////////////////////////////////////
// DEBUG FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////

#if ENABLE_DEBUG_WORDS
void debugPrint(const char* msg) {
    Serial.print("[DEBUG] ");
    Serial.println(msg);
}

void debugPrintf(const char* format, ...) {
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    Serial.print("[DEBUG] ");
    Serial.println(buffer);
}
#else
void debugPrint(const char* msg) { }
void debugPrintf(const char* format, ...) { }
#endif