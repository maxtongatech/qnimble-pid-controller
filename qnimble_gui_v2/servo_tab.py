"""
servo_tab.py - Servo Control Tab
Main PID control interface for all 4 channels
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
                              QLabel, QPushButton, QSlider, QCheckBox, 
                              QGridLayout, QDoubleSpinBox, QMessageBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

class ChannelControl(QGroupBox):
    """Control widget for a single servo channel"""
    
    def __init__(self, channel_num, serial_handler):
        super().__init__(f"Channel {channel_num}")
        self.channel = channel_num
        self.serial = serial_handler
        self.setup_ui()
        
    def setup_ui(self):
        """Setup the UI for this channel"""
        layout = QGridLayout()
        
        # Row 0: Enable checkbox and lock indicator
        self.enable_checkbox = QCheckBox("Enable Servo")
        self.enable_checkbox.stateChanged.connect(self.on_enable_changed)
        layout.addWidget(self.enable_checkbox, 0, 0, 1, 2)
        
        self.lock_label = QLabel("●")
        self.lock_label.setStyleSheet("color: gray; font-size: 20px;")
        self.lock_label.setToolTip("Lock indicator")
        layout.addWidget(self.lock_label, 0, 2)
        
        # Row 1: P Gain
        layout.addWidget(QLabel("P Gain:"), 1, 0)
        self.p_spin = QDoubleSpinBox()
        self.p_spin.setRange(0, 100)
        self.p_spin.setDecimals(4)
        self.p_spin.setSingleStep(0.1)
        self.p_spin.setValue(1.0)
        self.p_spin.valueChanged.connect(self.on_p_changed)
        layout.addWidget(self.p_spin, 1, 1)
        
        self.p_slider = QSlider(Qt.Horizontal)
        self.p_slider.setRange(0, 10000)
        self.p_slider.setValue(1000)
        self.p_slider.valueChanged.connect(self.on_p_slider_changed)
        layout.addWidget(self.p_slider, 1, 2)
        
        # Row 2: I Gain
        layout.addWidget(QLabel("I Gain:"), 2, 0)
        self.i_spin = QDoubleSpinBox()
        self.i_spin.setRange(0, 100)
        self.i_spin.setDecimals(4)
        self.i_spin.setSingleStep(0.01)
        self.i_spin.setValue(0.0)
        self.i_spin.valueChanged.connect(self.on_i_changed)
        layout.addWidget(self.i_spin, 2, 1)
        
        self.i_slider = QSlider(Qt.Horizontal)
        self.i_slider.setRange(0, 10000)
        self.i_slider.setValue(0)
        self.i_slider.valueChanged.connect(self.on_i_slider_changed)
        layout.addWidget(self.i_slider, 2, 2)
        
        # Row 3: D Gain
        layout.addWidget(QLabel("D Gain:"), 3, 0)
        self.d_spin = QDoubleSpinBox()
        self.d_spin.setRange(0, 100)
        self.d_spin.setDecimals(4)
        self.d_spin.setSingleStep(0.01)
        self.d_spin.setValue(0.0)
        self.d_spin.valueChanged.connect(self.on_d_changed)
        layout.addWidget(self.d_spin, 3, 1)
        
        self.d_slider = QSlider(Qt.Horizontal)
        self.d_slider.setRange(0, 10000)
        self.d_slider.setValue(0)
        self.d_slider.valueChanged.connect(self.on_d_slider_changed)
        layout.addWidget(self.d_slider, 3, 2)
        
        # Row 4: Setpoint
        layout.addWidget(QLabel("Setpoint (V):"), 4, 0)
        self.setpoint_spin = QDoubleSpinBox()
        self.setpoint_spin.setRange(-10, 10)
        self.setpoint_spin.setDecimals(4)
        self.setpoint_spin.setSingleStep(0.1)
        self.setpoint_spin.setValue(0.0)
        self.setpoint_spin.valueChanged.connect(self.on_setpoint_changed)
        layout.addWidget(self.setpoint_spin, 4, 1, 1, 2)
        
        # Row 5-7: Current Values (Read-only displays)
        layout.addWidget(QLabel("Input:"), 5, 0)
        self.input_label = QLabel("0.0000 V")
        self.input_label.setStyleSheet("background-color: #f0f0f0; padding: 5px;")
        layout.addWidget(self.input_label, 5, 1, 1, 2)
        
        layout.addWidget(QLabel("Output:"), 6, 0)
        self.output_label = QLabel("0.0000 V")
        self.output_label.setStyleSheet("background-color: #f0f0f0; padding: 5px;")
        layout.addWidget(self.output_label, 6, 1, 1, 2)
        
        layout.addWidget(QLabel("Error:"), 7, 0)
        self.error_label = QLabel("0.0000 V")
        self.error_label.setStyleSheet("background-color: #f0f0f0; padding: 5px;")
        layout.addWidget(self.error_label, 7, 1, 1, 2)
        
        self.setLayout(layout)
        
    def on_enable_changed(self, state):
        """Handle enable checkbox change"""
        enable = (state == Qt.Checked)
        if self.serial.is_connected:
            self.serial.enable_servo(self.channel, enable)
            
    def on_p_changed(self, value):
        """Handle P gain spinbox change"""
        self.p_slider.blockSignals(True)
        self.p_slider.setValue(int(value * 1000))
        self.p_slider.blockSignals(False)
        if self.serial.is_connected:
            self.serial.set_p_gain(self.channel, value)
            
    def on_p_slider_changed(self, value):
        """Handle P gain slider change"""
        self.p_spin.blockSignals(True)
        self.p_spin.setValue(value / 1000.0)
        self.p_spin.blockSignals(False)
        
    def on_i_changed(self, value):
        """Handle I gain spinbox change"""
        self.i_slider.blockSignals(True)
        self.i_slider.setValue(int(value * 1000))
        self.i_slider.blockSignals(False)
        if self.serial.is_connected:
            self.serial.set_i_gain(self.channel, value)
            
    def on_i_slider_changed(self, value):
        """Handle I gain slider change"""
        self.i_spin.blockSignals(True)
        self.i_spin.setValue(value / 1000.0)
        self.i_spin.blockSignals(False)
        
    def on_d_changed(self, value):
        """Handle D gain spinbox change"""
        self.d_slider.blockSignals(True)
        self.d_slider.setValue(int(value * 1000))
        self.d_slider.blockSignals(False)
        if self.serial.is_connected:
            self.serial.set_d_gain(self.channel, value)
            
    def on_d_slider_changed(self, value):
        """Handle D gain slider change"""
        self.d_spin.blockSignals(True)
        self.d_spin.setValue(value / 1000.0)
        self.d_spin.blockSignals(False)
        
    def on_setpoint_changed(self, value):
        """Handle setpoint change"""
        if self.serial.is_connected:
            self.serial.set_setpoint(self.channel, value)
            
    def update_status(self, channel_data):
        """Update display with current status"""
        if channel_data:
            self.input_label.setText(f"{channel_data['adc_input']:.4f} V")
            self.output_label.setText(f"{channel_data['dac_output']:.4f} V")
            
            error = channel_data['error']
            self.error_label.setText(f"{error:+.4f} V")
            
            if abs(error) < 0.01:
                self.error_label.setStyleSheet("background-color: #90EE90; padding: 5px;")
            elif abs(error) < 0.1:
                self.error_label.setStyleSheet("background-color: #FFFFE0; padding: 5px;")
            else:
                self.error_label.setStyleSheet("background-color: #FFB6C1; padding: 5px;")
            
            if channel_data['locked']:
                self.lock_label.setStyleSheet("color: green; font-size: 20px;")
                self.lock_label.setToolTip("Locked!")
            else:
                self.lock_label.setStyleSheet("color: gray; font-size: 20px;")
                self.lock_label.setToolTip("Not locked")
            
            self.enable_checkbox.blockSignals(True)
            self.enable_checkbox.setChecked(channel_data['servo_active'])
            self.enable_checkbox.blockSignals(False)


class ServoTab(QWidget):
    """Main servo control tab with all 4 channels"""
    
    def __init__(self, serial_handler):
        super().__init__()
        self.serial = serial_handler
        self.setup_ui()
        
    def setup_ui(self):
        """Setup the UI"""
        layout = QVBoxLayout()
        
        title = QLabel("PID Servo Control")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)
        
        grid_layout = QGridLayout()
        
        self.channel1 = ChannelControl(1, self.serial)
        grid_layout.addWidget(self.channel1, 0, 0)
        
        self.channel2 = ChannelControl(2, self.serial)
        grid_layout.addWidget(self.channel2, 0, 1)
        
        self.channel3 = ChannelControl(3, self.serial)
        grid_layout.addWidget(self.channel3, 1, 0)
        
        self.channel4 = ChannelControl(4, self.serial)
        grid_layout.addWidget(self.channel4, 1, 1)
        
        layout.addLayout(grid_layout)
        
        button_layout = QHBoxLayout()
        
        self.enable_all_btn = QPushButton("Enable All Servos")
        self.enable_all_btn.clicked.connect(self.enable_all)
        button_layout.addWidget(self.enable_all_btn)
        
        self.disable_all_btn = QPushButton("Disable All Servos")
        self.disable_all_btn.clicked.connect(self.disable_all)
        button_layout.addWidget(self.disable_all_btn)
        
        self.save_btn = QPushButton("Save Configuration")
        self.save_btn.clicked.connect(self.save_config)
        button_layout.addWidget(self.save_btn)
        
        layout.addLayout(button_layout)
        self.setLayout(layout)
        
    def enable_all(self):
        """Enable all servos"""
        for i in range(1, 5):
            self.serial.enable_servo(i, True)
        self.update_data()
        
    def disable_all(self):
        """Disable all servos"""
        for i in range(1, 5):
            self.serial.enable_servo(i, False)
        self.update_data()
        
    def save_config(self):
        """Save configuration to qNimble"""
        if self.serial.is_connected:
            response = self.serial.save_config()
            if response and "saved" in response.lower():
                QMessageBox.information(self, "Success", "Configuration saved to qNimble memory!")
            
    def update_data(self):
        """Update all channel displays"""
        if self.serial.is_connected:
            status = self.serial.get_status()
            if status:
                for channel_data in status:
                    channel_num = channel_data['channel']
                    if channel_num == 1:
                        self.channel1.update_status(channel_data)
                    elif channel_num == 2:
                        self.channel2.update_status(channel_data)
                    elif channel_num == 3:
                        self.channel3.update_status(channel_data)
                    elif channel_num == 4:
                        self.channel4.update_status(channel_data)