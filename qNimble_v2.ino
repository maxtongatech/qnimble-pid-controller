////////////////////////////////////////////////////////////////////////////////////
// qNimble_v2.ino - Main Application
// Quantum Opus LLC
// Version 2.0.0 - Stage 3: PID Control
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

// Control flags
bool quick_status_enabled = false;  // Start with it off for cleaner servo testing

////////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS
////////////////////////////////////////////////////////////////////////////////////

void applyConfigToHardware();
void runStage3Test();
void printQuickStatus();
void handleSerialCommand();
void handleSetDACCommand();
void handleSetPIDGainsCommand();
void handleSetSetpointCommand();
void handleFactoryReset();
void handleQuickPIDCommand(int channel);
void handleServoCommand();
void printHelp();

////////////////////////////////////////////////////////////////////////////////////
// SETUP
////////////////////////////////////////////////////////////////////////////////////

void setup() {
    // Initialize serial
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    Serial.println("\n\n====================================");
    Serial.println("qNimble v2.0 - Stage 3");
    Serial.println("PID Controller Implementation");
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
    
    Serial.println("\nHardware initialization complete!\n");
    
    // Run Stage 3 test
    runStage3Test();
    
    Serial.println("\n=== Entering main loop ===");
    printHelp();
}

////////////////////////////////////////////////////////////////////////////////////
// APPLY CONFIGURATION TO HARDWARE
////////////////////////////////////////////////////////////////////////////////////

void applyConfigToHardware() {
    Serial.println("Applying saved configuration...");
    
    // Apply ADC configuration
    for (int i = 1; i <= NUM_ADC_CHANNELS; i++) {
        ADC_Channel* ch = adc_manager.getChannel(i);
        if (ch) {
            ch->setRange(nvm_manager.getADCRange(i));
            ch->setSampleInterval(nvm_manager.getADCSampleInterval(i));
            
            if (nvm_manager.getADCEnabled(i)) {
                adc_manager.enableChannel(i);
            }
        }
        
        // Apply PID configuration
        PID_Controller* pid = servo_manager.getController(i);
        if (pid) {
            float p, i_gain, d;
            nvm_manager.getPIDGains(i, p, i_gain, d);
            pid->setGains(p, i_gain, d);
            
            pid->setSetpoint(nvm_manager.getSetpoint(i));
            
            float min_v, max_v;
            nvm_manager.getRails(i, min_v, max_v);
            pid->setRails(min_v, max_v);
            
            pid->setLockPrecision(nvm_manager.getLockPrecision(i));
        }
    }
    
    // Apply DAC configuration
    for (int i = 1; i <= NUM_DAC_CHANNELS; i++) {
        DAC_Channel* ch = dac_manager.getChannel(i);
        if (ch) {
            float min_v, max_v;
            nvm_manager.getDACRails(i, min_v, max_v);
            ch->setRails(min_v, max_v);
            
            if (nvm_manager.getDACEnabled(i)) {
                dac_manager.enableChannel(i);
            }
        }
    }
    
    Serial.println("✓ Configuration applied");
}

////////////////////////////////////////////////////////////////////////////////////
// MAIN LOOP
////////////////////////////////////////////////////////////////////////////////////

void loop() {
    static unsigned long last_print = 0;
    
    // Update servo (this runs at configured servo rate)
    servo_manager.update();
    
    // Read ADCs (for non-servo channels or monitoring)
    adc_manager.readAllChannels();
    
    // Print status every second (if enabled)
    if (quick_status_enabled && (millis() - last_print >= 1000)) {
        last_print = millis();
        printQuickStatus();
    }
    
    // Check for serial commands
    if (Serial.available()) {
        // Read the entire line first
        String cmd = "";
        unsigned long timeout = millis() + 100;  // 100ms timeout
        
        while (millis() < timeout) {
            if (Serial.available()) {
                char c = Serial.read();
                if (c == '\n' || c == '\r') {
                    break;  // End of command
                }
                cmd += c;
                timeout = millis() + 100;  // Reset timeout on new character
            }
            delay(1);
        }
        
        cmd.trim();
        
        if (cmd.length() == 0) {
            // Empty command, ignore
        } else if (cmd.length() == 1) {
            // Single character - use old handler
            // Put the character back for handleSerialCommand to read
            char single = cmd.charAt(0);
            
            // Manually handle single character commands
            switch (single) {
                case 'a':
                case 'A':
                    adc_manager.printStatus(Serial);
                    break;
                    
                case 'd':
                case 'D':
                    dac_manager.printStatus(Serial);
                    break;
                    
                case 'e':
                case 'E':
                    Serial.println("Enabling all ADC channels...");
                    adc_manager.enableAllChannels();
                    for (int i = 1; i <= NUM_ADC_CHANNELS; i++) {
                        nvm_manager.setADCEnabled(i, true);
                    }
                    Serial.println("✓ ADC channels enabled");
                    break;
                    
                case 'x':
                case 'X':
                    Serial.println("Disabling all ADC channels...");
                    adc_manager.disableAllChannels();
                    for (int i = 1; i <= NUM_ADC_CHANNELS; i++) {
                        nvm_manager.setADCEnabled(i, false);
                    }
                    Serial.println("✓ ADC channels disabled");
                    break;
                    
                case 'q':
                case 'Q':
                    quick_status_enabled = !quick_status_enabled;
                    Serial.printf("Quick status: %s\n", quick_status_enabled ? "ENABLED" : "DISABLED");
                    break;
                    
                case 'p':
                case 'P':
                    printQuickStatus();
                    break;
                    
                case 'c':
                case 'C':
                    nvm_manager.printConfig(Serial);
                    break;
                    
                case 'm':
                case 'M':
                    nvm_manager.printMemoryInfo(Serial);
                    break;
                    
                case 'w':
                case 'W':
                    Serial.println("Saving configuration to NVM...");
                    if (nvm_manager.saveConfig()) {
                        Serial.println("✓ Configuration saved");
                    } else {
                        Serial.println("✗ Save failed");
                    }
                    break;
                    
                case 'l':
                case 'L':
                    Serial.println("Loading configuration from NVM...");
                    if (nvm_manager.loadConfig()) {
                        Serial.println("✓ Configuration loaded");
                        applyConfigToHardware();
                    } else {
                        Serial.println("✗ Load failed");
                    }
                    break;
                    
                case 'r':
                case 'R':
                    handleFactoryReset();
                    break;
                    
                case '1':
                    handleQuickPIDCommand(1);
                    break;
                case '2':
                    handleQuickPIDCommand(2);
                    break;
                case '3':
                    handleQuickPIDCommand(3);
                    break;
                case '4':
                    handleQuickPIDCommand(4);
                    break;
                    
                case 'v':
                case 'V':
                    handleServoCommand();
                    break;
                    
                case 'h':
                case 'H':
                    printHelp();
                    break;
                    
                case 's':
                case 'S':
                    handleSetDACCommand();
                    break;
                    
                case 'g':
                case 'G':
                    handleSetPIDGainsCommand();
                    break;
                    
                case 't':
                case 'T':
                    handleSetSetpointCommand();
                    break;
                    
                default:
                    Serial.printf("Unknown command: '%c'\n", single);
                    Serial.println("Type 'h' for help");
                    break;
            }
        } else {
            // Multi-character - use new command handler
            command_handler.execute(cmd);
        }
    }
}


////////////////////////////////////////////////////////////////////////////////////
// STAGE 3 TEST
////////////////////////////////////////////////////////////////////////////////////

void runStage3Test() {
    Serial.println("=== Running Stage 3 Test ===\n");
    
    // Test 1: Show PID configuration
    Serial.println("Test 1: PID Configuration");
    for (int i = 1; i <= NUM_ADC_CHANNELS; i++) {
        PID_Controller* pid = servo_manager.getController(i);
        if (pid) {
            float p, i_gain, d;
            pid->getGains(p, i_gain, d);
            Serial.printf("Channel %d: P=%.4f  I=%.4f  D=%.4f  SP=%.4f\n",
                          i, p, i_gain, d, pid->getSetpoint());
        }
    }
    Serial.println();
    
    // Test 2: Enable ADCs and DACs for servo
    Serial.println("Test 2: Enabling channels for servo...");
    adc_manager.enableAllChannels();
    dac_manager.enableAllChannels();
    Serial.println("✓ Channels enabled");
    Serial.println();
    
    // Test 3: Show servo status
    Serial.println("Test 3: Servo Manager Status");
    servo_manager.printStatus(Serial);
    
    Serial.println("=== Stage 3 Test Complete ===\n");
    Serial.println("⚠ Servos are DISABLED by default for safety");
    Serial.println("  Type 'v' to enter servo control menu\n");
}

////////////////////////////////////////////////////////////////////////////////////
// STATUS PRINTING
////////////////////////////////////////////////////////////////////////////////////

void printQuickStatus() {
    Serial.println("--- Quick Status ---");
    
    // Print servo status for each channel
    for (int i = 1; i <= NUM_ADC_CHANNELS; i++) {
        if (servo_manager.isServoActive(i)) {
            PID_Controller* pid = servo_manager.getController(i);
            ADC_Channel* adc = adc_manager.getChannel(i);
            
            if (pid && adc) {
                Serial.printf("CH%d: SP=%.3f  IN=%.3f  ERR=%+.4f  OUT=%.3f  %s\n",
                              i,
                              pid->getSetpoint(),
                              adc->getVoltage(),
                              pid->getError(),
                              pid->getOutput(),
                              pid->isLocked() ? "LOCK" : "    ");
            }
        }
    }
    
    Serial.println();
}

////////////////////////////////////////////////////////////////////////////////////
// SERIAL COMMAND HANDLER
////////////////////////////////////////////////////////////////////////////////////

void handleSerialCommand() {
    char cmd = Serial.read();
    
    switch (cmd) {
        case 'a':
        case 'A':
            adc_manager.printStatus(Serial);
            break;
            
        case 'd':
        case 'D':
            dac_manager.printStatus(Serial);
            break;
            
        case 'e':
        case 'E':
            Serial.println("Enabling all ADC channels...");
            adc_manager.enableAllChannels();
            for (int i = 1; i <= NUM_ADC_CHANNELS; i++) {
                nvm_manager.setADCEnabled(i, true);
            }
            Serial.println("✓ ADC channels enabled");
            break;
            
        case 'x':
        case 'X':
            Serial.println("Disabling all ADC channels...");
            adc_manager.disableAllChannels();
            for (int i = 1; i <= NUM_ADC_CHANNELS; i++) {
                nvm_manager.setADCEnabled(i, false);
            }
            Serial.println("✓ ADC channels disabled");
            break;
            
        case 's':
        case 'S':
            handleSetDACCommand();
            break;
            
        case 'q':
        case 'Q':
            quick_status_enabled = !quick_status_enabled;
            Serial.printf("Quick status: %s\n", quick_status_enabled ? "ENABLED" : "DISABLED");
            break;
            
        case 'p':
        case 'P':
            printQuickStatus();
            break;
            
        // MEMORY COMMANDS
        case 'c':
        case 'C':
            nvm_manager.printConfig(Serial);
            break;
            
        case 'm':
        case 'M':
            nvm_manager.printMemoryInfo(Serial);
            break;
            
        case 'w':
        case 'W':
            Serial.println("Saving configuration to NVM...");
            if (nvm_manager.saveConfig()) {
                Serial.println("✓ Configuration saved");
            } else {
                Serial.println("✗ Save failed");
            }
            break;
            
        case 'l':
        case 'L':
            Serial.println("Loading configuration from NVM...");
            if (nvm_manager.loadConfig()) {
                Serial.println("✓ Configuration loaded");
                applyConfigToHardware();
            } else {
                Serial.println("✗ Load failed");
            }
            break;
            
        case 'r':
        case 'R':
            handleFactoryReset();
            break;
            
        // QUICK CHANNEL CONFIG
        case '1':
            handleQuickPIDCommand(1);
            break;
        case '2':
            handleQuickPIDCommand(2);
            break;
        case '3':
            handleQuickPIDCommand(3);
            break;
        case '4':
            handleQuickPIDCommand(4);
            break;
            
        // SERVO CONTROL
        case 'v':
        case 'V':
            handleServoCommand();
            break;
            
        case 'h':
        case 'H':
            printHelp();
            break;
            
        case '\n':
        case '\r':
            break;
            
        default:
            Serial.printf("Unknown command: '%c'\n", cmd);
            Serial.println("Type 'h' for help");
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// COMMAND HANDLERS
////////////////////////////////////////////////////////////////////////////////////

void handleSetDACCommand() {
    while (!Serial.available()) delay(1);
    int channel = Serial.parseInt();
    
    while (!Serial.available()) delay(1);
    float voltage = Serial.parseFloat();
    
    while (Serial.available()) Serial.read();
    
    if (channel < 1 || channel > NUM_DAC_CHANNELS) {
        Serial.printf("Invalid channel: %d (must be 1-4)\n", channel);
        return;
    }
    
    DAC_Channel* ch = dac_manager.getChannel(channel);
    if (!ch) {
        Serial.println("Error: Could not access DAC channel");
        return;
    }
    
    if (!ch->isEnabled()) {
        Serial.printf("Enabling DAC channel %d...\n", channel);
        dac_manager.enableChannel(channel);
        nvm_manager.setDACEnabled(channel, true);
    }
    
    ch->setVoltage(voltage);
    Serial.printf("DAC%d set to %.4f V\n", channel, ch->getVoltage());
    Serial.println("Type 'w' to save to memory");
}

void handleSetPIDGainsCommand() {
    // Format: g [channel] [P] [I] [D]
    Serial.println("Enter: channel P I D");
    
    while (!Serial.available()) delay(1);
    int channel = Serial.parseInt();
    
    while (!Serial.available()) delay(1);
    float p = Serial.parseFloat();
    
    while (!Serial.available()) delay(1);
    float i = Serial.parseFloat();
    
    while (!Serial.available()) delay(1);
    float d = Serial.parseFloat();
    
    while (Serial.available()) Serial.read();
    
    if (channel < 1 || channel > NUM_ADC_CHANNELS) {
        Serial.printf("Invalid channel: %d (must be 1-4)\n", channel);
        return;
    }
    
    nvm_manager.setPIDGains(channel, p, i, d);
    
    PID_Controller* pid = servo_manager.getController(channel);
    if (pid) {
        pid->setGains(p, i, d);
    }
    
    Serial.printf("Channel %d PID gains set: P=%.4f  I=%.4f  D=%.4f\n", channel, p, i, d);
    Serial.println("Type 'w' to save to memory");
}

void handleSetSetpointCommand() {
    // Format: t [channel] [setpoint]
    Serial.println("Enter: channel setpoint");
    
    while (!Serial.available()) delay(1);
    int channel = Serial.parseInt();
    
    while (!Serial.available()) delay(1);
    float setpoint = Serial.parseFloat();
    
    while (Serial.available()) Serial.read();
    
    if (channel < 1 || channel > NUM_ADC_CHANNELS) {
        Serial.printf("Invalid channel: %d (must be 1-4)\n", channel);
        return;
    }
    
    nvm_manager.setSetpoint(channel, setpoint);
    
    PID_Controller* pid = servo_manager.getController(channel);
    if (pid) {
        pid->setSetpoint(setpoint);
    }
    
    Serial.printf("Channel %d setpoint set to %.4f V\n", channel, setpoint);
    Serial.println("Type 'w' to save to memory");
}

void handleFactoryReset() {
    Serial.println("⚠ Factory reset - are you sure? Type 'Y' to confirm...");
    
    // Wait for confirmation
    unsigned long timeout = millis() + 5000;
    while (millis() < timeout) {
        if (Serial.available()) {
            char confirm = Serial.read();
            while (Serial.available()) Serial.read(); // Clear buffer
            
            if (confirm == 'Y' || confirm == 'y') {
                Serial.println("Performing factory reset...");
                nvm_manager.factoryReset();
                applyConfigToHardware();
                Serial.println("✓ Factory reset complete");
                return;
            } else {
                Serial.println("✗ Factory reset cancelled");
                return;
            }
        }
        delay(10);
    }
    
    Serial.println("✗ Factory reset cancelled (timeout)");
}

void handleQuickPIDCommand(int channel) {
    Serial.printf("\n=== Quick Config for Channel %d ===\n", channel);
    
    // Show current values
    PID_Controller* pid = servo_manager.getController(channel);
    if (!pid) {
        Serial.println("Error: Invalid channel");
        return;
    }
    
    float p, i, d;
    pid->getGains(p, i, d);
    float sp = pid->getSetpoint();
    
    Serial.printf("Current: P=%.4f  I=%.4f  D=%.4f  SP=%.4f\n\n", p, i, d, sp);
    
    // Simple menu
    Serial.println("Commands:");
    Serial.println("  p [value] - Set P gain");
    Serial.println("  i [value] - Set I gain");
    Serial.println("  d [value] - Set D gain");
    Serial.println("  s [value] - Set setpoint");
    Serial.println("  w - Save to memory");
    Serial.println("  x - Exit");
    Serial.println();
    
    // Wait for sub-command
    while (true) {
        if (Serial.available()) {
            char subcmd = Serial.read();
            
            switch (subcmd) {
                case 'p':
                case 'P': {
                    while (!Serial.available()) delay(1);
                    float val = Serial.parseFloat();
                    while (Serial.available()) Serial.read();
                    pid->getGains(p, i, d);
                    pid->setGains(val, i, d);
                    nvm_manager.setPIDGains(channel, val, i, d);
                    Serial.printf("P gain set to %.4f\n", val);
                    break;
                }
                
                case 'i':
                case 'I': {
                    while (!Serial.available()) delay(1);
                    float val = Serial.parseFloat();
                    while (Serial.available()) Serial.read();
                    pid->getGains(p, i, d);
                    pid->setGains(p, val, d);
                    nvm_manager.setPIDGains(channel, p, val, d);
                    Serial.printf("I gain set to %.4f\n", val);
                    break;
                }
                
                case 'd':
                case 'D': {
                    while (!Serial.available()) delay(1);
                    float val = Serial.parseFloat();
                    while (Serial.available()) Serial.read();
                    pid->getGains(p, i, d);
                    pid->setGains(p, i, val);
                    nvm_manager.setPIDGains(channel, p, i, val);
                    Serial.printf("D gain set to %.4f\n", val);
                    break;
                }
                
                case 's':
                case 'S': {
                    while (!Serial.available()) delay(1);
                    float val = Serial.parseFloat();
                    while (Serial.available()) Serial.read();
                    pid->setSetpoint(val);
                    nvm_manager.setSetpoint(channel, val);
                    Serial.printf("Setpoint set to %.4f V\n", val);
                    break;
                }
                
                case 'w':
                case 'W':
                    if (nvm_manager.saveConfig()) {
                        Serial.println("✓ Configuration saved");
                    } else {
                        Serial.println("✗ Save failed");
                    }
                    break;
                
                case 'x':
                case 'X':
                    Serial.println("Exiting channel config\n");
                    return;
                
                case '\n':
                case '\r':
                    break;
                    
                default:
                    Serial.printf("Unknown: '%c'\n", subcmd);
                    break;
            }
        }
        delay(10);
    }
}

void handleServoCommand() {
    Serial.println("\n=== Servo Control Menu ===");
    Serial.println("Commands:");
    Serial.println("  1-4 - Enable servo on channel");
    Serial.println("  x   - Disable all servos");
    Serial.println("  s   - Show servo status");
    Serial.println("  p   - Show performance stats");
    Serial.println("  q   - Toggle quick status");
    Serial.println("  r [Hz] - Set servo rate");
    Serial.println("  e   - Exit");
    Serial.println();
    
    while (true) {
        if (Serial.available()) {
            char subcmd = Serial.read();
            
            switch (subcmd) {
                case '1':
                case '2':
                case '3':
                case '4': {
                    int chan = subcmd - '0';
                    servo_manager.enableServo(chan);
                    Serial.printf("✓ Servo %d ENABLED\n", chan);
                    break;
                }
                
                case 'x':
                case 'X':
                    servo_manager.disableAllServos();
                    Serial.println("✓ All servos DISABLED");
                    break;
                
                case 's':
                case 'S':
                    servo_manager.printStatus(Serial);
                    break;
                
                case 'p':
                case 'P':
                    servo_manager.printPerformance(Serial);
                    break;
                
                case 'q':
                case 'Q':
                    quick_status_enabled = !quick_status_enabled;
                    Serial.printf("Quick status: %s\n", quick_status_enabled ? "ON" : "OFF");
                    break;
                
                case 'r':
                case 'R': {
                    Serial.println("Enter servo rate (Hz):");
                    while (!Serial.available()) delay(1);
                    float rate = Serial.parseFloat();
                    while (Serial.available()) Serial.read();
                    
                    if (rate >= 1.0f && rate <= 100000.0f) {
                        servo_manager.setServoRate(rate);
                        Serial.printf("Servo rate set to %.2f Hz\n", rate);
                    } else {
                        Serial.println("Invalid rate (must be 1-100000 Hz)");
                    }
                    break;
                }
                
                case 'e':
                case 'E':
                    Serial.println("Exiting servo menu\n");
                    return;
                
                case '\n':
                case '\r':
                    break;
                    
                default:
                    Serial.printf("Unknown: '%c'\n", subcmd);
                    break;
            }
        }
        
        // Keep servo running while in menu
        servo_manager.update();
        delay(1);
    }
}

void printHelp() {
    Serial.println("\n=== qNimble v2.0 Commands ===");
    Serial.println("\n--- Hardware Control ---");
    Serial.println("  'a' - Print ADC status");
    Serial.println("  'd' - Print DAC status");
    Serial.println("  'e' - Enable all ADCs");
    Serial.println("  'x' - Disable all ADCs");
    Serial.println("  's [chan] [volts]' - Set DAC voltage");
    Serial.println("  'q' - Toggle quick status on/off");
    Serial.println("  'p' - Print quick status once");
    
    Serial.println("\n--- Servo Control ---");
    Serial.println("  'v' - Servo control menu ⭐ NEW");
    Serial.println("  '1-4' - Quick config for channel");
    
    Serial.println("\n--- Configuration ---");
    Serial.println("  'c' - Print current configuration");
    Serial.println("  'm' - Print memory info");
    Serial.println("  'w' - Write (save) config to NVM");
    Serial.println("  'l' - Load config from NVM");
    Serial.println("  'r' - Factory reset");
    
    Serial.println("\n--- Help ---");
    Serial.println("  'h' - This help message\n");
}