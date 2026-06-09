////////////////////////////////////////////////////////////////////////////////////
// command_handler.h - Serial Command Handler
// qNimble v2.0
////////////////////////////////////////////////////////////////////////////////////

#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "config.h"
#include "hal_adc.h"
#include "hal_dac.h"
#include "nvm_manager.h"
#include "pid_controller.h"

////////////////////////////////////////////////////////////////////////////////////
// COMMAND HANDLER CLASS
////////////////////////////////////////////////////////////////////////////////////

class Command_Handler {
private:
    ADC_Manager* adc_mgr;
    DAC_Manager* dac_mgr;
    NVM_Manager* nvm_mgr;
    Servo_Manager* servo_mgr;
    
    Stream* serial;
    
    // Command parsing
    String readCommand();
    void parseAndExecute(String cmd);
    
    // Command categories
    void handlePIDCommand(String cmd);
    void handleServoCommand(String cmd);
    void handleADCCommand(String cmd);
    void handleDACCommand(String cmd);
    void handleConfigCommand(String cmd);
    void handleQueryCommand(String cmd);
    void handleSystemCommand(String cmd);

public:
    // Constructor
    Command_Handler();
    
    // Initialization
    void init(ADC_Manager* adc, DAC_Manager* dac, 
              NVM_Manager* nvm, Servo_Manager* servo, Stream* s);
    
    // Main update (call in loop)
    void update();
    
    // Direct command execution (for testing)
    void execute(String cmd);
};

#endif // COMMAND_HANDLER_H