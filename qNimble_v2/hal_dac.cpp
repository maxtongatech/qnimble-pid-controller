////////////////////////////////////////////////////////////////////////////////////
// hal_dac.cpp - DAC Hardware Abstraction Layer Implementation
// qNimble v2.0 - Uses qNimble Quarto board package functions
////////////////////////////////////////////////////////////////////////////////////

#include "hal_dac.h"

extern "C" {
    #include "dac.h"
}

////////////////////////////////////////////////////////////////////////////////////
// DAC_CHANNEL IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////

DAC_Channel::DAC_Channel() {
    channel_num = 0;
    pin = 0;
    enabled = false;
    is_hardware_dac = true;
    voltage_value = 0.0f;
    dac_value = DAC_MID_VALUE;
    hold_active = false;
    hold_voltage = 0.0f;
    rail_min = -DAC_VOLTAGE_RANGE;
    rail_max = DAC_VOLTAGE_RANGE;
    write_count = 0;
    voltage_scale = DAC_MAX_VALUE / (2.0f * DAC_VOLTAGE_RANGE);
}

void DAC_Channel::init(uint8_t chan, uint8_t dac_pin, bool is_hw_dac) {
    channel_num = chan;
    pin = dac_pin;
    is_hardware_dac = true;

    // Set to mid-scale (0V)
    setVoltage(0.0f);
    enabled = false;
}

void DAC_Channel::setRails(float min_v, float max_v) {
    rail_min = constrain(min_v, -DAC_VOLTAGE_RANGE, DAC_VOLTAGE_RANGE);
    rail_max = constrain(max_v, -DAC_VOLTAGE_RANGE, DAC_VOLTAGE_RANGE);

    if (rail_min > rail_max) {
        float temp = rail_min;
        rail_min = rail_max;
        rail_max = temp;
    }
}

void DAC_Channel::enable() {
    enabled = true;
}

void DAC_Channel::disable() {
    enabled = false;
    setVoltage(0.0f);
}

void DAC_Channel::setVoltage(float volts) {
    if (!enabled && !hold_active) return;

    // Apply rail limits
    volts = constrain(volts, rail_min, rail_max);

    // Store voltage
    voltage_value = volts;

    // Write to qNimble hardware using board package function
    writeDAC(channel_num, volts);

    write_count++;
}

void DAC_Channel::setDACCode(uint16_t code) {
    code = constrain(code, 0, DAC_MAX_VALUE);
    dac_value = code;

    // Convert DAC code to voltage
    int16_t signed_code = (int16_t)(code - DAC_MID_VALUE);
    voltage_value = (signed_code * 10.24f) / 32768.0f;

    // Write using board package function
    writeDACRAW(channel_num, signed_code);

    write_count++;
}

void DAC_Channel::setHold(bool hold_state) {
    if (hold_state && !hold_active) {
        hold_voltage = voltage_value;
        hold_active = true;
    } else if (!hold_state && hold_active) {
        hold_active = false;
    }
}

void DAC_Channel::setHoldVoltage(float volts) {
    hold_voltage = volts;
    if (hold_active) {
        setVoltage(hold_voltage);
    }
}

void DAC_Channel::resetStatistics() {
    write_count = 0;
}

////////////////////////////////////////////////////////////////////////////////////
// DAC_MANAGER IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////

DAC_Manager::DAC_Manager() {
    active_channel_mask = 0;
}

void DAC_Manager::init() {
    // Initialize all DAC channels
    channels[0].init(1, DAC_PIN_1, true);
    channels[1].init(2, DAC_PIN_2, true);
    channels[2].init(3, 0, true);
    channels[3].init(4, 0, true);

    // Zero all DACs using board package function
    zeroDACs();

    active_channel_mask = 0;
}

void DAC_Manager::initSPI() {
    // Not needed for qNimble Quarto
}

void DAC_Manager::writeSPI_DAC(uint8_t dac_num, uint16_t value) {
    // Not needed for qNimble Quarto
}

DAC_Channel* DAC_Manager::getChannel(uint8_t chan) {
    if (chan < 1 || chan > NUM_DAC_CHANNELS) return nullptr;
    return &channels[chan - 1];
}

void DAC_Manager::enableChannel(uint8_t chan) {
    if (chan < 1 || chan > NUM_DAC_CHANNELS) return;

    channels[chan - 1].enable();
    active_channel_mask |= (1 << (chan - 1));
}

void DAC_Manager::disableChannel(uint8_t chan) {
    if (chan < 1 || chan > NUM_DAC_CHANNELS) return;

    channels[chan - 1].disable();
    active_channel_mask &= ~(1 << (chan - 1));
}

void DAC_Manager::enableAllChannels() {
    for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
        channels[i].enable();
    }
    active_channel_mask = 0x0F;
}

void DAC_Manager::disableAllChannels() {
    for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
        channels[i].disable();
    }
    active_channel_mask = 0;
}

uint8_t DAC_Manager::getActiveChannelCount() const {
    uint8_t count = 0;
    for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
        if (active_channel_mask & (1 << i)) count++;
    }
    return count;
}

void DAC_Manager::setAllVoltages(float volts) {
    for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
        if (channels[i].isEnabled()) {
            channels[i].setVoltage(volts);
        }
    }
}

void DAC_Manager::setAllHold(bool hold_state) {
    for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
        channels[i].setHold(hold_state);
    }
}

void DAC_Manager::printStatus(Stream& serial) {
    serial.println("=== DAC Manager Status ===");
    serial.printf("Active channels: %d (mask: 0x%02X)\n", 
                  getActiveChannelCount(), active_channel_mask);
    serial.println();

    for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
        printChannelInfo(i + 1, serial);
    }
}

void DAC_Manager::printChannelInfo(uint8_t chan, Stream& serial) {
    DAC_Channel* ch = getChannel(chan);
    if (!ch) return;

    serial.printf("Channel %d: %s\n", chan, ch->isEnabled() ? "ENABLED" : "DISABLED");
    serial.printf("  Type: FPGA\n");
    serial.printf("  Voltage: %.4f V\n", ch->getVoltage());
    serial.printf("  Rails: %.2f to %.2f V\n", 
                 ch->getRailMin(), ch->getRailMax());
    serial.printf("  Hold: %s", ch->isHeld() ? "ACTIVE" : "INACTIVE");
    if (ch->isHeld()) {
        serial.printf(" (%.4f V)", ch->getVoltage());
    }
    serial.println();
    serial.printf("  Writes: %lu\n", ch->getWriteCount());
    serial.println();
}