"""
config_tab.py - Configuration Tab
Save/load configuration and system settings
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
                              QLabel, QPushButton, QTextEdit, QMessageBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

class ConfigTab(QWidget):
    """Configuration management tab"""
    
    def __init__(self, serial_handler):
        super().__init__()
        self.serial = serial_handler
        self.setup_ui()
        
    def setup_ui(self):
        """Setup the UI"""
        layout = QVBoxLayout()
        
        # Title
        title = QLabel("Configuration Management")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)
        
        # Memory operations
        memory_group = QGroupBox("Memory Operations")
        memory_layout = QVBoxLayout()
        
        button_layout = QHBoxLayout()
        
        self.save_btn = QPushButton("Save to qNimble Memory")
        self.save_btn.clicked.connect(self.save_config)
        button_layout.addWidget(self.save_btn)
        
        self.load_btn = QPushButton("Load from qNimble Memory")
        self.load_btn.clicked.connect(self.load_config)
        button_layout.addWidget(self.load_btn)
        
        self.reset_btn = QPushButton("Factory Reset")
        self.reset_btn.clicked.connect(self.factory_reset)
        self.reset_btn.setStyleSheet("background-color: #FFB6C1;")
        button_layout.addWidget(self.reset_btn)
        
        memory_layout.addLayout(button_layout)
        
        # Status display
        self.status_text = QTextEdit()
        self.status_text.setReadOnly(True)
        self.status_text.setMaximumHeight(200)
        memory_layout.addWidget(QLabel("Status:"))
        memory_layout.addWidget(self.status_text)
        
        memory_group.setLayout(memory_layout)
        layout.addWidget(memory_group)
        
        # Info section
        info_group = QGroupBox("System Information")
        info_layout = QVBoxLayout()
        
        self.info_label = QLabel("Connect to qNimble to view system information")
        info_layout.addWidget(self.info_label)
        
        self.refresh_info_btn = QPushButton("Refresh Info")
        self.refresh_info_btn.clicked.connect(self.refresh_info)
        info_layout.addWidget(self.refresh_info_btn)
        
        info_group.setLayout(info_layout)
        layout.addWidget(info_group)
        
        layout.addStretch()
        self.setLayout(layout)
        
    def save_config(self):
        """Save configuration to qNimble memory"""
        if not self.serial.is_connected:
            QMessageBox.warning(self, "Not Connected", "Please connect to qNimble first")
            return
            
        response = self.serial.save_config()
        self.status_text.append(f"Save: {response}")
        
        # Check if save actually worked (look for success indicators OR lack of error)
        if response:
            if "saved" in response.lower() or "complete" in response.lower():
                QMessageBox.information(self, "Success", "Configuration saved successfully!")
            elif "fail" in response.lower() or "error" in response.lower():
                QMessageBox.warning(self, "Failed", "Save operation failed")
            else:
                # Ambiguous response - let user decide
                QMessageBox.information(self, "Save Attempted", 
                                       "Save command sent. Check status output for details.")
        else:
            QMessageBox.warning(self, "No Response", "No response from qNimble")
            
    def load_config(self):
        """Load configuration from qNimble memory"""
        if not self.serial.is_connected:
            QMessageBox.warning(self, "Not Connected", "Please connect to qNimble first")
            return
            
        response = self.serial.load_config()
        self.status_text.append(f"Load: {response}")
        
        if response and "loaded" in response.lower():
            QMessageBox.information(self, "Success", "Configuration loaded successfully!")
        else:
            QMessageBox.warning(self, "Failed", "Load operation failed")
            
    def factory_reset(self):
        """Factory reset qNimble"""
        reply = QMessageBox.question(self, "Factory Reset",
                                     "Are you sure you want to reset to factory defaults?\n"
                                     "This will erase all saved settings!",
                                     QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            if not self.serial.is_connected:
                QMessageBox.warning(self, "Not Connected", "Please connect to qNimble first")
                return
                
            response = self.serial.factory_reset()
            self.status_text.append(f"Reset: {response}")
            QMessageBox.information(self, "Reset Complete", "Factory reset completed")
            
    def refresh_info(self):
        """Refresh system information"""
        if not self.serial.is_connected:
            self.info_label.setText("Not connected")
            return
            
        # Get version info
        response = self.serial.send_command("version")
        if response:
            self.info_label.setText(response)
        
    def update_data(self):
        """Update tab data"""
        pass  # Nothing to update continuously