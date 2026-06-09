
"""
square_wave_tab.py - Square Wave Generator Tab
Configure and control square wave modulation
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, 
                             QGroupBox, QLabel, QPushButton, QDoubleSpinBox, 
                             QCheckBox, QGridLayout, QComboBox, QScrollArea)

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

class SquareWaveTab(QWidget):
    "Square wave generator control tab"

    def __init__(self, serial_handler):
        super().__init__()
        self.serial = serial_handler
        self.setup_ui()

    def setup_ui(self):
        """Setup the UI"""
        main_layout = QVBoxLayout()

        # Scroll Area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        # Create content widget
        content = QWidget()
        layout = QVBoxLayout()

        # Title
        title = QLabel("Square Wave Generator")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        # Control group for each channel
        for channel in range(1,5):
            channel_group = self.create_channel_group(channel)
            layout.addWidget(channel_group)

        layout.addStretch()
        content.setLayout(layout)

         # Add content to scroll area
        scroll.setWidget(content)
        
        # Add scroll area to main layout
        main_layout.addWidget(scroll)
        self.setLayout(main_layout)

    def create_channel_group(self, channel):
        """Control group for a single channel"""
        group = QGroupBox(f"Channel {channel}")
        layout = QGridLayout()

         # Enable checkbox
        enable_cb = QCheckBox("Enable Square Wave")
        enable_cb.stateChanged.connect(lambda state, ch=channel: self.toggle_square_wave(ch, state))
        layout.addWidget(enable_cb, 0, 0, 1, 2)
        
        # Frequency control
        layout.addWidget(QLabel("Frequency (Hz):"), 1, 0)
        freq_spin = QDoubleSpinBox()
        freq_spin.setRange(0.001, 10000)
        freq_spin.setDecimals(3)
        freq_spin.setValue(2000)  # Default 2 kHz
        freq_spin.setSingleStep(0.1)
        freq_spin.valueChanged.connect(lambda v, ch=channel: self.set_frequency(ch, v))
        layout.addWidget(freq_spin, 1, 1)
        
        # Period display (read-only)
        layout.addWidget(QLabel("Period (ms):"), 2, 0)
        period_label = QLabel("0.500")
        period_label.setStyleSheet("background-color: #f0f0f0; padding: 5px;")
        layout.addWidget(period_label, 2, 1)
        
        # Duty cycle control
        layout.addWidget(QLabel("Duty Cycle (%):"), 3, 0)
        duty_spin = QDoubleSpinBox()
        duty_spin.setRange(0, 100)
        duty_spin.setDecimals(1)
        duty_spin.setValue(50)
        duty_spin.setSingleStep(1)
        duty_spin.valueChanged.connect(lambda v, ch=channel: self.set_duty_cycle(ch, v))
        layout.addWidget(duty_spin, 3, 1)
        
        # High setpoint
        layout.addWidget(QLabel("High Level (V):"), 4, 0)
        high_spin = QDoubleSpinBox()
        high_spin.setRange(-10, 10)
        high_spin.setDecimals(4)
        high_spin.setValue(0.0)
        high_spin.setSingleStep(0.1)
        high_spin.valueChanged.connect(lambda v, ch=channel: self.set_high_level(ch, v))
        layout.addWidget(high_spin, 4, 1)
        
        # Low setpoint
        layout.addWidget(QLabel("Low Level (V):"), 5, 0)
        low_spin = QDoubleSpinBox()
        low_spin.setRange(-10, 10)
        low_spin.setDecimals(4)
        low_spin.setValue(0.0)
        low_spin.setSingleStep(0.1)
        low_spin.valueChanged.connect(lambda v, ch=channel: self.set_low_level(ch, v))
        layout.addWidget(low_spin, 5, 1)
        
        # Quick bipolar rails button
        bipolar_layout = QHBoxLayout()
        bipolar_layout.addWidget(QLabel("Quick ±V:"))
        bipolar_spin = QDoubleSpinBox()
        bipolar_spin.setRange(0, 10)
        bipolar_spin.setDecimals(2)
        bipolar_spin.setValue(2.5)
        bipolar_spin.setSingleStep(0.5)
        bipolar_layout.addWidget(bipolar_spin)
        
        bipolar_btn = QPushButton("Set ±Rails")
        bipolar_btn.clicked.connect(lambda ch=channel, spin=bipolar_spin: 
                                    self.set_bipolar_rails(ch, spin.value()))
        bipolar_layout.addWidget(bipolar_btn)
        layout.addLayout(bipolar_layout, 6, 0, 1, 2)
        
        # Status indicator
        status_label = QLabel("Status: Disabled")
        status_label.setStyleSheet("background-color: #FFB6C1; padding: 5px;")
        layout.addWidget(status_label, 7, 0, 1, 2)
        
        # Store widgets for later access
        setattr(self, f'enable_cb_{channel}', enable_cb)
        setattr(self, f'freq_spin_{channel}', freq_spin)
        setattr(self, f'period_label_{channel}', period_label)
        setattr(self, f'duty_spin_{channel}', duty_spin)
        setattr(self, f'high_spin_{channel}', high_spin)
        setattr(self, f'low_spin_{channel}', low_spin)
        setattr(self, f'status_label_{channel}', status_label)
        
        group.setLayout(layout)
        return group
    
    def toggle_square_wave(self, channel, state):
        """Enable/disable square wave"""
        if not self.serial.is_connected:
            return
        
        if state == Qt.Checked:
            cmd = f"sqwave{channel} enable"
            status_label = getattr(self, f'status_label_{channel}')
            status_label.setText("Status: Enabled")
            status_label.setStyleSheet("background-color: #90EE90; padding: 5px;")
        else:
            cmd = f"sqwave{channel} disable"
            status_label = getattr(self, f'status_label_{channel}')
            status_label.setText("Status: Disabled")
            status_label.setStyleSheet("background-color: #FFB6C1; padding: 5px;")
        
        self.serial.send_command(cmd)
    
    def set_frequency(self, channel, freq_hz):
        """Set square wave frequency"""
        if not self.serial.is_connected:
            return
        
        # Update period display
        period_ms = 1000.0 / freq_hz if freq_hz > 0 else 0
        period_label = getattr(self, f'period_label_{channel}')
        period_label.setText(f"{period_ms:.3f}")
        
        # Send command
        cmd = f"sqwave{channel} freq {freq_hz}"
        self.serial.send_command(cmd)
    
    def set_duty_cycle(self, channel, duty_percent):
        """Set duty cycle"""
        if not self.serial.is_connected:
            return
        
        duty_fraction = duty_percent / 100.0
        cmd = f"sqwave{channel} duty {duty_fraction}"
        self.serial.send_command(cmd)
    
    def set_high_level(self, channel, voltage):
        """Set high voltage level"""
        if not self.serial.is_connected:
            return
        
        cmd = f"sqwave{channel} high {voltage}"
        self.serial.send_command(cmd)
    
    def set_low_level(self, channel, voltage):
        """Set low voltage level"""
        if not self.serial.is_connected:
            return
        
        cmd = f"sqwave{channel} low {voltage}"
        self.serial.send_command(cmd)
    
    def set_bipolar_rails(self, channel, voltage):
        """Set symmetric ±voltage rails"""
        if not self.serial.is_connected:
            return
        
        # Update spinboxes
        high_spin = getattr(self, f'high_spin_{channel}')
        low_spin = getattr(self, f'low_spin_{channel}')
        high_spin.setValue(voltage)
        low_spin.setValue(-voltage)
        
        # Send command
        cmd = f"sqwave{channel} brails {voltage}"
        self.serial.send_command(cmd)

    def update_status(self, channel, sqwave_data):
        """Update display with current square wave status"""
        if not sqwave_data:
            return

        # Get widgets for this channel
        enable_cb = getattr(self, f'enable_cb_{channel}', None)
        freq_spin = getattr(self, f'freq_spin_{channel}', None)
        period_label = getattr(self, f'period_label_{channel}', None)
        duty_spin = getattr(self, f'duty_spin_{channel}', None)
        high_spin = getattr(self, f'high_spin_{channel}', None)
        low_spin = getattr(self, f'low_spin_{channel}', None)
        status_label = getattr(self, f'status_label_{channel}', None)

        if not all([enable_cb, freq_spin, period_label, duty_spin, high_spin, low_spin, status_label]):
            return

        # Update enable checkbox (block signals to prevent triggering commands)
        enable_cb.blockSignals(True)
        enable_cb.setChecked(sqwave_data['enabled'])
        enable_cb.blockSignals(False)

        # Update frequency and period
        if sqwave_data['period_ms'] > 0:
            frequency = 1000.0 / sqwave_data['period_ms']
            freq_spin.blockSignals(True)
            freq_spin.setValue(frequency)
            freq_spin.blockSignals(False)
            period_label.setText(f"{sqwave_data['period_ms']:.3f}")

        # Update duty cycle
        duty_percent = sqwave_data['duty_cycle'] * 100.0
        duty_spin.blockSignals(True)
        duty_spin.setValue(duty_percent)
        duty_spin.blockSignals(False)
    
        # Update voltage levels
        high_spin.blockSignals(True)
        high_spin.setValue(sqwave_data['high_setpoint'])
        high_spin.blockSignals(False)

        low_spin.blockSignals(True)
        low_spin.setValue(sqwave_data['low_setpoint'])
        low_spin.blockSignals(False)

        # Update status label with state and color coding
        if sqwave_data['enabled']:
            current_state = sqwave_data.get('state', 'UNKNOWN')
            current_setpoint = sqwave_data.get('current_setpoint', 0.0)

            if current_state == 'HIGH':
                status_label.setText(f"Status: Enabled - HIGH ({current_setpoint:+.4f} V)")
                status_label.setStyleSheet("background-color: #90EE90; padding: 5px; font-weight: bold;")
            elif current_state == 'LOW':
                status_label.setText(f"Status: Enabled - LOW ({current_setpoint:+.4f} V)")
                status_label.setStyleSheet("background-color: #ADD8E6; padding: 5px; font-weight: bold;")
            else:
                status_label.setText("Status: Enabled")
                status_label.setStyleSheet("background-color: #90EE90; padding: 5px;")
        else:
            status_label.setText("Status: Disabled")
            status_label.setStyleSheet("background-color: #FFB6C1; padding: 5px;")


    def update_data(self):
        """Update all channels (called periodically from main loop)"""
        if not self.serial.is_connected:
            return

        # Request status for each channel
        for channel in range(1, 5):
            sqwave_data = self.get_channel_status(channel)
            if sqwave_data:
                self.update_status(channel, sqwave_data)


    def get_channel_status(self, channel):
        """Get square wave status for a channel"""
        if not self.serial.is_connected:
            return None

        try:
            # Send status command
            cmd = f"sqwave{channel} status"
            response = self.serial.send_command(cmd)

            if not response:
                return None

            # Parse the response
            # Expected format from firmware:
            # Square Wave X Status:
            #   Enabled: Yes/No
            #   State: HIGH/LOW
            #   Period: X.X ms
            #   Frequency: X.XXXX Hz
            #   Duty Cycle: X.XX
            #   High Setpoint: X.XXXX V
            #   Low Setpoint: X.XXXX V
            #   Current SP: X.XXXX V

            sqwave_data = {
                'enabled': False,
                'state': 'LOW',
                'period_ms': 0.5,
                'duty_cycle': 0.5,
                'high_setpoint': 0.0,
                'low_setpoint': 0.0,
                'current_setpoint': 0.0
            }

            # Parse multi-line response
            lines = response.split('\n')
            for line in lines:
                line = line.strip()

                if 'Enabled:' in line:
                    sqwave_data['enabled'] = 'Yes' in line or 'YES' in line
                elif 'State:' in line:
                    sqwave_data['state'] = 'HIGH' if 'HIGH' in line else 'LOW'
                elif 'Period:' in line:
                    # Extract number from "Period: 0.500 ms"
                    parts = line.split(':')
                    if len(parts) > 1:
                        try:
                            sqwave_data['period_ms'] = float(parts[1].split()[0])
                        except:
                            pass
                elif 'Duty Cycle:' in line:
                    parts = line.split(':')
                    if len(parts) > 1:
                        try:
                            sqwave_data['duty_cycle'] = float(parts[1].strip())
                        except:
                            pass
                elif 'High Setpoint:' in line:
                    parts = line.split(':')
                    if len(parts) > 1:
                        try:
                            sqwave_data['high_setpoint'] = float(parts[1].split()[0])
                        except:
                            pass
                elif 'Low Setpoint:' in line:
                    parts = line.split(':')
                    if len(parts) > 1:
                        try:
                            sqwave_data['low_setpoint'] = float(parts[1].split()[0])
                        except:
                            pass
                elif 'Current SP:' in line:
                    parts = line.split(':')
                    if len(parts) > 1:
                        try:
                            sqwave_data['current_setpoint'] = float(parts[1].split()[0])
                        except:
                            pass
                        
            return sqwave_data

        except Exception as e:
            print(f"Error getting square wave status: {e}")
            return None