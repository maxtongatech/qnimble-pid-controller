"""
hardware_tab.py - Hardware Monitoring Tab
Real-time ADC/DAC monitoring and control
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
                              QLabel, QPushButton, QGridLayout, QDoubleSpinBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

class HardwareTab(QWidget):
    """Hardware monitoring and control tab"""
    
    def __init__(self, serial_handler):
        super().__init__()
        self.serial = serial_handler
        self.setup_ui()
        
    def setup_ui(self):
        """Setup the UI"""
        layout = QVBoxLayout()
        
        title = QLabel("Hardware Monitor")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)
        
        adc_group = QGroupBox("ADC Inputs")
        adc_layout = QGridLayout()
        
        adc_layout.addWidget(QLabel("Channel"), 0, 0)
        adc_layout.addWidget(QLabel("Voltage"), 0, 1)
        adc_layout.addWidget(QLabel("Range"), 0, 2)
        
        self.adc_labels = []
        for i in range(1, 5):
            adc_layout.addWidget(QLabel(f"ADC {i}:"), i, 0)
            
            voltage_label = QLabel("0.0000 V")
            voltage_label.setStyleSheet("background-color: #f0f0f0; padding: 5px; min-width: 100px;")
            adc_layout.addWidget(voltage_label, i, 1)
            self.adc_labels.append(voltage_label)
            
            range_label = QLabel("±10.0 V")
            adc_layout.addWidget(range_label, i, 2)
        
        adc_group.setLayout(adc_layout)
        layout.addWidget(adc_group)
        
        dac_group = QGroupBox("DAC Outputs")
        dac_layout = QGridLayout()
        
        dac_layout.addWidget(QLabel("Channel"), 0, 0)
        dac_layout.addWidget(QLabel("Voltage"), 0, 1)
        dac_layout.addWidget(QLabel("Set Output"), 0, 2)
        
        self.dac_labels = []
        self.dac_spins = []
        
        for i in range(1, 5):
            dac_layout.addWidget(QLabel(f"DAC {i}:"), i, 0)
            
            voltage_label = QLabel("0.0000 V")
            voltage_label.setStyleSheet("background-color: #f0f0f0; padding: 5px; min-width: 100px;")
            dac_layout.addWidget(voltage_label, i, 1)
            self.dac_labels.append(voltage_label)
            
            spin = QDoubleSpinBox()
            spin.setRange(-10, 10)
            spin.setDecimals(4)
            spin.setSingleStep(0.1)
            spin.setValue(0.0)
            spin.valueChanged.connect(lambda v, ch=i: self.set_dac_output(ch, v))
            dac_layout.addWidget(spin, i, 2)
            self.dac_spins.append(spin)
        
        dac_group.setLayout(dac_layout)
        layout.addWidget(dac_group)
        
        button_layout = QHBoxLayout()
        
        self.zero_all_btn = QPushButton("Zero All DACs")
        self.zero_all_btn.clicked.connect(self.zero_all_dacs)
        button_layout.addWidget(self.zero_all_btn)
        
        layout.addLayout(button_layout)
        layout.addStretch()
        
        self.setLayout(layout)
        
    def set_dac_output(self, channel, voltage):
        """Set DAC output voltage"""
        if self.serial.is_connected:
            self.serial.set_dac_output(channel, voltage)
            
    def zero_all_dacs(self):
        """Set all DACs to 0V"""
        for spin in self.dac_spins:
            spin.setValue(0.0)
            
    def update_data(self):
        """Update display with current data"""
        if self.serial.is_connected:
            status = self.serial.get_status()
            if status:
                for channel_data in status:
                    idx = channel_data['channel'] - 1
                    
                    if idx < len(self.adc_labels):
                        voltage = channel_data['adc_input']
                        self.adc_labels[idx].setText(f"{voltage:.4f} V")
                        
                        if abs(voltage) > 9.5:
                            self.adc_labels[idx].setStyleSheet(
                                "background-color: #FFB6C1; padding: 5px; min-width: 100px;")
                        else:
                            self.adc_labels[idx].setStyleSheet(
                                "background-color: #90EE90; padding: 5px; min-width: 100px;")
                    
                    if idx < len(self.dac_labels):
                        voltage = channel_data['dac_output']
                        self.dac_labels[idx].setText(f"{voltage:.4f} V")