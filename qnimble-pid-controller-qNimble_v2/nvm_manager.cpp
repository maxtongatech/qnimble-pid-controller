////////////////////////////////////////////////////////////////////////////////////
// nvm_manager.cpp - Non-Volatile Memory Manager Implementation
// qNimble v2.0
////////////////////////////////////////////////////////////////////////////////////

#include "nvm_manager.h"

// Forward declarations for qNimble's NVM functions
// These are provided by the qNimble board package
// NOTE: Using correct signatures from comm.h
extern "C" {
    void readNVMblock(void* data, uint16_t length, uint32_t start_addr);
    void writeNVMpages(void* data, uint16_t data_size, uint16_t first_page);
}

////////////////////////////////////////////////////////////////////////////////////
// CONSTRUCTOR
////////////////////////////////////////////////////////////////////////////////////

NVM_Manager::NVM_Manager() {
    config_valid = false;
}

////////////////////////////////////////////////////////////////////////////////////
// LOW-LEVEL NVM ACCESS
////////////////////////////////////////////////////////////////////////////////////

bool NVM_Manager::writeNVM(uint16_t addr, const void* data, size_t size) {
    Serial.printf("[NVM] writeNVM: addr=0x%04X, size=%d bytes\n", addr, size);
    
    // qNimble uses page-based writes
    // Each page is 128 bytes, starting at page 0
    uint16_t page = addr / 128;
    
    Serial.printf("[NVM] Starting at page %d\n", page);
    
    // For simplicity, we'll write the entire config as one block
    // This uses multiple pages
    uint16_t pages_needed = (size + 127) / 128;
    
    Serial.printf("[NVM] Pages needed: %d\n", pages_needed);
    
    for (uint16_t i = 0; i < pages_needed; i++) {
        uint8_t* src = (uint8_t*)data + (i * 128);
        uint16_t chunk_size = (i == pages_needed - 1) ? (size % 128) : 128;
        if (chunk_size == 0) chunk_size = 128;
        
        Serial.printf("[NVM] Writing page %d, size %d bytes\n", page + i, chunk_size);
        
        writeNVMpages(src, chunk_size, page + i);
        
        delay(10);  // Give NVM time to write
    }
    
    Serial.println("[NVM] Write complete");
    return true;
}

bool NVM_Manager::readNVM(uint16_t addr, void* data, size_t size) {
    // qNimble's readNVMblock reads from byte address
    readNVMblock(data, (uint16_t)size, addr);
    return true;
}

// ... rest of the file stays the same ...

////////////////////////////////////////////////////////////////////////////////////
// INITIALIZATION
////////////////////////////////////////////////////////////////////////////////////

void NVM_Manager::init() {
    // Try to load existing configuration
    if (loadConfig()) {
        config_valid = true;
    } else {
        // No valid config found, use defaults
        setDefaults();
        config_valid = false;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// CHECKSUM & VALIDATION
////////////////////////////////////////////////////////////////////////////////////

uint32_t NVM_Manager::calculateChecksum(const Config_Package& pkg) {
    // Calculate checksum over everything EXCEPT the checksum field
    uint32_t checksum = 0;
    
    // Create a temporary copy without the checksum
    Config_Package temp = pkg;
    temp.system.checksum = 0;  // Zero out checksum field
    
    const uint8_t* data = (const uint8_t*)&temp;
    size_t size = sizeof(Config_Package);
    
    // Simple XOR checksum with rotation
    for (size_t i = 0; i < size; i++) {
        checksum ^= data[i];
        checksum = (checksum << 1) | (checksum >> 31); // Rotate left
    }
    
    return checksum;
}

bool NVM_Manager::validateConfig(const Config_Package& pkg) {
    Serial.println("[NVM] Validating config...");
    
    // Check magic number
    Serial.printf("[NVM]   Magic number: 0x%08X (expected 0x%08X)\n", 
                  pkg.system.magic_number, NVM_MAGIC_NUMBER);
    if (pkg.system.magic_number != NVM_MAGIC_NUMBER) {
        Serial.println("[NVM]   ✗ Magic number mismatch!");
        return false;
    }
    Serial.println("[NVM]   ✓ Magic number OK");
    
    // Check version (for now, just check it exists)
    Serial.printf("[NVM]   Version: %d\n", pkg.system.version);
    if (pkg.system.version == 0 || pkg.system.version > 100) {
        Serial.println("[NVM]   ✗ Invalid version!");
        return false;
    }
    Serial.println("[NVM]   ✓ Version OK");
    
    // Verify checksum
    uint32_t calculated = calculateChecksum(pkg);
    Serial.printf("[NVM]   Checksum: 0x%08X (expected 0x%08X)\n", 
                  pkg.system.checksum, calculated);
    if (calculated != pkg.system.checksum) {
        Serial.println("[NVM]   ✗ Checksum mismatch!");
        return false;
    }
    Serial.println("[NVM]   ✓ Checksum OK");
    
    Serial.println("[NVM] ✓ Validation passed!");
    return true;
}

////////////////////////////////////////////////////////////////////////////////////
// DEFAULT CONFIGURATION
////////////////////////////////////////////////////////////////////////////////////

void NVM_Manager::setDefaults() {
    // System defaults
    config.system.magic_number = NVM_MAGIC_NUMBER;
    config.system.version = 1;
    config.system.global_t_res = DEFAULT_T_RES;
    config.system.default_hold_logic = false;
    
    // PID defaults for all channels
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        config.pid[i].p_gain = DEFAULT_P_GAIN;
        config.pid[i].i_gain = DEFAULT_I_GAIN;
        config.pid[i].d_gain = DEFAULT_D_GAIN;
        config.pid[i].setpoint = DEFAULT_SETPOINT;
        config.pid[i].rail_min = DEFAULT_RAIL_MIN;
        config.pid[i].rail_max = DEFAULT_RAIL_MAX;
        config.pid[i].lock_precision = DEFAULT_LOCK_PRECISION;
    }
    
    // ADC defaults for all channels
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        config.adc[i].range = BIPOLAR_10V;
        config.adc[i].enabled = false;
        config.adc[i].sample_interval = DEFAULT_T_RES;
    }
    
    // DAC defaults for all channels
    for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
        config.dac[i].enabled = false;
        config.dac[i].rail_min = DEFAULT_RAIL_MIN;
        config.dac[i].rail_max = DEFAULT_RAIL_MAX;
        config.dac[i].offset = 0.0f;
    }

    // Square wave defaults for all channels
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        config.sqwave[i].enabled = false;
        config.sqwave[i].period_ms = 0.5f;         // 0.5 ms = 2000 Hz (2 kHz)
        config.sqwave[i].duty_cycle = 0.5f;        // 50% duty
        config.sqwave[i].high_setpoint = 0.0f;
        config.sqwave[i].low_setpoint = 0.0f;
    }
    
    // Calculate checksum
    config.system.checksum = calculateChecksum(config);
}

////////////////////////////////////////////////////////////////////////////////////
// SAVE/LOAD OPERATIONS
////////////////////////////////////////////////////////////////////////////////////

bool NVM_Manager::saveConfig() {
    Serial.println("[NVM] Starting save...");
    
    // Recalculate checksum before saving
    config.system.checksum = calculateChecksum(config);
    Serial.printf("[NVM] Checksum calculated: 0x%08X\n", config.system.checksum);
    
    // Write to NVM
    Serial.printf("[NVM] Writing %d bytes to address 0x%04X\n", CONFIG_SIZE, CONFIG_BASE_ADDR);
    bool write_ok = writeNVM(CONFIG_BASE_ADDR, &config, CONFIG_SIZE);
    
    if (!write_ok) {
        Serial.println("[NVM] Write operation failed!");
        return false;
    }
    
    Serial.println("[NVM] Write complete, verifying...");
    
    // Verify write
    Config_Package verify;
    readNVM(CONFIG_BASE_ADDR, &verify, CONFIG_SIZE);
    
    Serial.printf("[NVM] Read back magic: 0x%08X (expected 0x%08X)\n", 
                  verify.system.magic_number, NVM_MAGIC_NUMBER);
    Serial.printf("[NVM] Read back checksum: 0x%08X (expected 0x%08X)\n",
                  verify.system.checksum, config.system.checksum);
    
    if (validateConfig(verify)) {
        config_valid = true;
        Serial.println("[NVM] Verification passed!");
        return true;
    }
    
    Serial.println("[NVM] Verification failed!");
    return false;
}

bool NVM_Manager::loadConfig() {
    // Read from NVM
    Config_Package loaded;
    readNVM(CONFIG_BASE_ADDR, &loaded, CONFIG_SIZE);
    
    // Validate
    if (validateConfig(loaded)) {
        config = loaded;
        config_valid = true;
        return true;
    }
    
    // Invalid config, use defaults
    setDefaults();
    config_valid = false;
    return false;
}

void NVM_Manager::factoryReset() {
    setDefaults();
    saveConfig();
    config_valid = true;
}

////////////////////////////////////////////////////////////////////////////////////
// SYSTEM CONFIGURATION
////////////////////////////////////////////////////////////////////////////////////

void NVM_Manager::setGlobalTimeRes(float t_res) {
    if (t_res >= MIN_T_RES && t_res <= MAX_T_RES) {
        config.system.global_t_res = t_res;
    }
}

void NVM_Manager::setDefaultHoldLogic(bool logic) {
    config.system.default_hold_logic = logic;
}

////////////////////////////////////////////////////////////////////////////////////
// PID CONFIGURATION
////////////////////////////////////////////////////////////////////////////////////

void NVM_Manager::setPIDGains(uint8_t chan, float p, float i, float d) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return;
    
    config.pid[chan - 1].p_gain = p;
    config.pid[chan - 1].i_gain = i;
    config.pid[chan - 1].d_gain = d;
}

void NVM_Manager::getPIDGains(uint8_t chan, float& p, float& i, float& d) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) {
        p = i = d = 0.0f;
        return;
    }
    
    p = config.pid[chan - 1].p_gain;
    i = config.pid[chan - 1].i_gain;
    d = config.pid[chan - 1].d_gain;
}

void NVM_Manager::setSetpoint(uint8_t chan, float sp) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return;
    config.pid[chan - 1].setpoint = sp;
}

float NVM_Manager::getSetpoint(uint8_t chan) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return 0.0f;
    return config.pid[chan - 1].setpoint;
}

void NVM_Manager::setRails(uint8_t chan, float min_v, float max_v) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return;
    
    config.pid[chan - 1].rail_min = min_v;
    config.pid[chan - 1].rail_max = max_v;
}

void NVM_Manager::getRails(uint8_t chan, float& min_v, float& max_v) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) {
        min_v = max_v = 0.0f;
        return;
    }
    
    min_v = config.pid[chan - 1].rail_min;
    max_v = config.pid[chan - 1].rail_max;
}

void NVM_Manager::setLockPrecision(uint8_t chan, float precision) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return;
    config.pid[chan - 1].lock_precision = precision;
}

float NVM_Manager::getLockPrecision(uint8_t chan) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return DEFAULT_LOCK_PRECISION;
    return config.pid[chan - 1].lock_precision;
}

////////////////////////////////////////////////////////////////////////////////////
// ADC CONFIGURATION
////////////////////////////////////////////////////////////////////////////////////

void NVM_Manager::setADCRange(uint8_t chan, ADC_Range range) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return;
    config.adc[chan - 1].range = range;
}

ADC_Range NVM_Manager::getADCRange(uint8_t chan) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return BIPOLAR_10V;
    return config.adc[chan - 1].range;
}

void NVM_Manager::setADCEnabled(uint8_t chan, bool enabled) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return;
    config.adc[chan - 1].enabled = enabled;
}

bool NVM_Manager::getADCEnabled(uint8_t chan) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return false;
    return config.adc[chan - 1].enabled;
}

void NVM_Manager::setADCSampleInterval(uint8_t chan, float interval) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return;
    config.adc[chan - 1].sample_interval = interval;
}

float NVM_Manager::getADCSampleInterval(uint8_t chan) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return DEFAULT_T_RES;
    return config.adc[chan - 1].sample_interval;
}

////////////////////////////////////////////////////////////////////////////////////
// DAC CONFIGURATION
////////////////////////////////////////////////////////////////////////////////////

void NVM_Manager::setDACEnabled(uint8_t chan, bool enabled) {
    if (chan < 1 || chan > NUM_DAC_CHANNELS) return;
    config.dac[chan - 1].enabled = enabled;
}

bool NVM_Manager::getDACEnabled(uint8_t chan) {
    if (chan < 1 || chan > NUM_DAC_CHANNELS) return false;
    return config.dac[chan - 1].enabled;
}

void NVM_Manager::setDACRails(uint8_t chan, float min_v, float max_v) {
    if (chan < 1 || chan > NUM_DAC_CHANNELS) return;
    
    config.dac[chan - 1].rail_min = min_v;
    config.dac[chan - 1].rail_max = max_v;
}

void NVM_Manager::getDACRails(uint8_t chan, float& min_v, float& max_v) {
    if (chan < 1 || chan > NUM_DAC_CHANNELS) {
        min_v = max_v = 0.0f;
        return;
    }
    
    min_v = config.dac[chan - 1].rail_min;
    max_v = config.dac[chan - 1].rail_max;
}

void NVM_Manager::setDACOffset(uint8_t chan, float offset) {
    if (chan < 1 || chan > NUM_DAC_CHANNELS) return;
    config.dac[chan - 1].offset = offset;
}

float NVM_Manager::getDACOffset(uint8_t chan) {
    if (chan < 1 || chan > NUM_DAC_CHANNELS) return 0.0f;
    return config.dac[chan - 1].offset;
}

////////////////////////////////////////////////////////////////////////////////////
// SQUARE WAVE CONFIGURATION
////////////////////////////////////////////////////////////////////////////////////

void NVM_Manager::setSquareWaveEnabled(uint8_t chan, bool enabled) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return; // do not enable if the channel isn't valid
    config.sqwave[chan - 1].enabled = enabled;
}

bool NVM_Manager::getSquareWaveEnabled(uint8_t chan) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return false; //square wave is assumed disabled if the channel isn't valid
    return config.sqwave[chan - 1].enabled;
}

void NVM_Manager::setSquareWavePeriod(uint8_t chan, float period) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return;
    if (period > 0) {
        config.sqwave[chan - 1].period_ms = period;
    }
}

float NVM_Manager::getSquareWavePeriod(uint8_t chan) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return 0.5f; // Assume default frequency is 2 kHz (0.5 ms)
    return config.sqwave[chan - 1].period_ms;
}

void NVM_Manager::setSquareWaveDuty(uint8_t chan, float duty) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return; 
    if (duty >= 0.0f && duty <= 1.0f) {
        config.sqwave[chan - 1].duty_cycle = duty;
    }
}

float NVM_Manager::getSquareWaveDuty(uint8_t chan) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return 0.5f; // Assume default duty cycle is 50%
    return config.sqwave[chan - 1].duty_cycle;
}

void NVM_Manager::setSquareWaveHigh(uint8_t chan, float high) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return;
    config.sqwave[chan - 1].high_setpoint = high;
}

float NVM_Manager::getSquareWaveHigh(uint8_t chan) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return 0.0f;
    return config.sqwave[chan - 1].high_setpoint;
}

void NVM_Manager::setSquareWaveLow(uint8_t chan, float low) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return;
    config.sqwave[chan - 1].low_setpoint = low;
}

float NVM_Manager::getSquareWaveLow(uint8_t chan) {
    if (chan < 1 || chan > NUM_ADC_CHANNELS) return 0.0f;
    return config.sqwave[chan - 1].low_setpoint;
}

////////////////////////////////////////////////////////////////////////////////////
// DIAGNOSTICS
////////////////////////////////////////////////////////////////////////////////////

void NVM_Manager::printConfig(Stream& serial) {
    serial.println("\n=== Configuration ===");
    serial.printf("Valid: %s\n", config_valid ? "YES" : "NO");
    serial.printf("Magic: 0x%08X\n", config.system.magic_number);
    serial.printf("Version: %d\n", config.system.version);
    serial.printf("Checksum: 0x%08X\n", config.system.checksum);
    serial.println();
    
    // System config
    serial.println("--- System ---");
    serial.printf("Global t_res: %.2f us\n", config.system.global_t_res);
    serial.printf("Hold logic: %d\n", config.system.default_hold_logic);
    serial.println();
    
    // PID config
    serial.println("--- PID Gains ---");
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        serial.printf("Channel %d: P=%.4f  I=%.4f  D=%.4f\n", 
                      i + 1,
                      config.pid[i].p_gain,
                      config.pid[i].i_gain,
                      config.pid[i].d_gain);
        serial.printf("           SP=%.4f  Rails=[%.2f, %.2f]  Lock=%.4f\n",
                      config.pid[i].setpoint,
                      config.pid[i].rail_min,
                      config.pid[i].rail_max,
                      config.pid[i].lock_precision);
    }
    serial.println();
    
    // ADC config
    serial.println("--- ADC Config ---");
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        serial.printf("Channel %d: Range=±%.2fV  Enabled=%s  Interval=%.2fus\n",
                      i + 1,
                      getADCRangeVolts(config.adc[i].range),
                      config.adc[i].enabled ? "YES" : "NO",
                      config.adc[i].sample_interval);
    }
    serial.println();
    
    // DAC config
    serial.println("--- DAC Config ---");
    for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
        serial.printf("Channel %d: Enabled=%s  Rails=[%.2f, %.2f]  Offset=%.4f\n",
                      i + 1,
                      config.dac[i].enabled ? "YES" : "NO",
                      config.dac[i].rail_min,
                      config.dac[i].rail_max,
                      config.dac[i].offset);
    }
    serial.println();

    // Square wave config
    serial.println("--- Square Wave Config ---");
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        serial.printf("Channel %d: Enabled=%s  Period=%.1fms  Duty=%.2f\n",
                      i + 1,
                      config.sqwave[i].enabled ? "YES" : "NO",
                      config.sqwave[i].period_ms,
                      config.sqwave[i].duty_cycle);
        serial.printf("           High=%.4fV  Low=%.4fV  Freq=%.4fHz\n",
                      config.sqwave[i].high_setpoint,
                      config.sqwave[i].low_setpoint,
                      1000.0f / config.sqwave[i].period_ms);
    }
    serial.println();

}

void NVM_Manager::printMemoryInfo(Stream& serial) {
    serial.println("\n=== Memory Info ===");
    serial.printf("Config size: %d bytes\n", CONFIG_SIZE);
    serial.printf("Config address: 0x%04X\n", CONFIG_BASE_ADDR);
    serial.printf("NVM size: %d bytes\n", NVM_SIZE);
    serial.printf("Pages used: %d\n", (CONFIG_SIZE + 127) / 128);
    serial.printf("Free space: %d bytes\n", NVM_SIZE - CONFIG_SIZE);
    serial.println();
}
