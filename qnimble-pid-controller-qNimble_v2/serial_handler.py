"""
serial_handler.py - qNimble Serial Communication Handler
Handles all serial communication with the qNimble device
"""

import serial
import serial.tools.list_ports
from PyQt5.QtCore import QObject, pyqtSignal, QTimer
import time

class SerialHandler(QObject):
    # Signals
    connected = pyqtSignal()
    disconnected = pyqtSignal()
    data_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.is_connected = False
        self.port_name = None
        self.baud_rate = 115200
        
    def get_available_ports(self):
        """Get list of available serial ports"""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def connect(self, port_name, baud_rate=115200):
        """Connect to qNimble"""
        try:
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=baud_rate,
                timeout=1,
                write_timeout=1
            )
            self.is_connected = True
            self.port_name = port_name
            self.baud_rate = baud_rate
            
            # Give device time to reset
            time.sleep(2)
            
            # Clear any startup messages
            self.serial_port.reset_input_buffer()
            
            self.connected.emit()
            return True
            
        except Exception as e:
            self.error_occurred.emit(f"Connection failed: {str(e)}")
            return False
    
    def disconnect(self):
        """Disconnect from qNimble"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.is_connected = False
        self.disconnected.emit()
    
    def send_command(self, command):
        """Send command to qNimble"""
        if not self.is_connected or not self.serial_port:
            self.error_occurred.emit("Not connected")
            return None
        
        try:
            # Send command
            self.serial_port.write(f"{command}\n".encode())
            self.serial_port.flush()
            
            # Wait a bit for response
            time.sleep(0.05)
            
            # Read response
            response = ""
            while self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    response += line + "\n"
            
            return response.strip()
            
        except Exception as e:
            self.error_occurred.emit(f"Command failed: {str(e)}")
            return None
    
    def set_p_gain(self, channel, value):
        """Set P gain for channel"""
        return self.send_command(f"prop{channel} {value}")
    
    def set_i_gain(self, channel, value):
        """Set I gain for channel"""
        return self.send_command(f"int{channel} {value}")
    
    def set_d_gain(self, channel, value):
        """Set D gain for channel"""
        return self.send_command(f"deriv{channel} {value}")
    
    def set_setpoint(self, channel, value):
        """Set setpoint for channel"""
        return self.send_command(f"setpoint{channel} {value}")
    
    def enable_servo(self, channel, enable=True):
        """Enable/disable servo"""
        state = 1 if enable else 0
        return self.send_command(f"enable{channel} {state}")
    
    def get_adc_input(self, channel):
        """Get ADC input voltage"""
        response = self.send_command(f"input{channel}")
        try:
            return float(response)
        except:
            return None
    
    def get_dac_output(self, channel):
        """Get DAC output voltage"""
        response = self.send_command(f"output{channel}")
        try:
            return float(response)
        except:
            return None
    
    def get_error(self, channel):
        """Get servo error"""
        response = self.send_command(f"error{channel}")
        try:
            return float(response)
        except:
            return None
    
    def get_status(self):
        """Get status of all channels"""
        response = self.send_command("status")
        if not response:
            return None
        
        # Parse status response
        # Format: channel,servo_active,adc_input,dac_output,error,locked
        channels = []
        for line in response.split('\n'):
            if line and ',' in line:
                try:
                    parts = line.split(',')
                    channel_data = {
                        'channel': int(parts[0]),
                        'servo_active': bool(int(parts[1])),
                        'adc_input': float(parts[2]),
                        'dac_output': float(parts[3]),
                        'error': float(parts[4]),
                        'locked': bool(int(parts[5]))
                    }
                    channels.append(channel_data)
                except:
                    pass
        
        return channels
    
    def save_config(self):
        """Save configuration to NVM"""
        return self.send_command("save")
    
    def load_config(self):
        """Load configuration from NVM"""
        return self.send_command("load")
    
    def factory_reset(self):
        """Factory reset"""
        return self.send_command("reset")
    
    def set_adc_range(self, channel, voltage):
        """Set ADC range"""
        return self.send_command(f"range{channel} {voltage}")
    
    def set_dac_output(self, channel, voltage):
        """Set DAC output directly"""
        return self.send_command(f"dac{channel} {voltage}")