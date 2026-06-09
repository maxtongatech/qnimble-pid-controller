"""
main.py - qNimble Control GUI
Main application window with tabs for different functions
"""

import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QTabWidget, 
                              QVBoxLayout, QWidget, QMenuBar, QAction,
                              QStatusBar, QMessageBox)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QIcon
from serial_handler import SerialHandler
from hardware_tab import HardwareTab
from servo_tab import ServoTab
from config_tab import ConfigTab
from terminal_tab import TerminalTab
from square_wave_tab import SquareWaveTab

class qNimbleGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("qNimble Control v2.0")
        self.setGeometry(100, 100, 1200, 800)
        
        # Create serial handler
        self.serial = SerialHandler()
        self.serial.connected.connect(self.on_connected)
        self.serial.disconnected.connect(self.on_disconnected)
        self.serial.error_occurred.connect(self.on_error)
        
        # Setup UI
        self.setup_ui()
        
        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_data)
        
    def setup_ui(self):
        """Setup the user interface"""
        # Central widget with tabs
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        
        # Create tabs
        self.hardware_tab = HardwareTab(self.serial)
        self.servo_tab = ServoTab(self.serial)
        self.config_tab = ConfigTab(self.serial)
        self.square_wave_tab = SquareWaveTab(self.serial)
        self.terminal_tab = TerminalTab(self.serial)
        
        # Add tabs
        self.tabs.addTab(self.hardware_tab, "Hardware")
        self.tabs.addTab(self.servo_tab, "Servo Control")
        self.tabs.addTab(self.config_tab, "Configuration")
        self.tabs.addTab(self.terminal_tab, "Terminal")
        self.tabs.addTab(self.square_wave_tab, "Square Wave Generator")
        
        # Menu bar
        self.create_menu_bar()
        
        # Status bar
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage("Disconnected")
        
    def create_menu_bar(self):
        """Create menu bar"""
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu("File")
        
        connect_action = QAction("Connect", self)
        connect_action.triggered.connect(self.connect_dialog)
        file_menu.addAction(connect_action)
        
        disconnect_action = QAction("Disconnect", self)
        disconnect_action.triggered.connect(self.disconnect)
        file_menu.addAction(disconnect_action)
        
        file_menu.addSeparator()
        
        exit_action = QAction("Exit", self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # Help menu
        help_menu = menubar.addMenu("Help")
        
        about_action = QAction("About", self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)
        
    def connect_dialog(self):
        """Show connection dialog"""
        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QComboBox, QPushButton, QLabel
        
        dialog = QDialog(self)
        dialog.setWindowTitle("Connect to qNimble")
        layout = QVBoxLayout()
        
        # Port selection
        layout.addWidget(QLabel("Select Port:"))
        port_combo = QComboBox()
        ports = self.serial.get_available_ports()
        port_combo.addItems(ports)
        layout.addWidget(port_combo)
        
        # Connect button
        connect_btn = QPushButton("Connect")
        connect_btn.clicked.connect(lambda: self.connect(port_combo.currentText(), dialog))
        layout.addWidget(connect_btn)
        
        dialog.setLayout(layout)
        dialog.exec_()
        
    def connect(self, port, dialog=None):
        """Connect to qNimble"""
        if self.serial.connect(port):
            self.statusBar.showMessage(f"Connected to {port}")
            self.update_timer.start(1000)  # Update every second
            if dialog:
                dialog.accept()
        else:
            QMessageBox.warning(self, "Connection Failed", 
                              f"Could not connect to {port}")
    
    def disconnect(self):
        """Disconnect from qNimble"""
        self.update_timer.stop()
        self.serial.disconnect()
        self.statusBar.showMessage("Disconnected")
        
    def update_data(self):
        """Update data from qNimble"""
        if self.serial.is_connected:
            # Update active tab
            current_tab = self.tabs.currentWidget()
            if hasattr(current_tab, 'update_data'):
                current_tab.update_data()
    
    def on_connected(self):
        """Handle connection event"""
        self.statusBar.showMessage("Connected", 3000)
        
    def on_disconnected(self):
        """Handle disconnection event"""
        self.statusBar.showMessage("Disconnected")
        self.update_timer.stop()
        
    def on_error(self, error_msg):
        """Handle error event"""
        self.statusBar.showMessage(f"Error: {error_msg}", 5000)
        
    def show_about(self):
        """Show about dialog"""
        QMessageBox.about(self, "About qNimble Control",
                         "qNimble Control v2.0\n\n"
                         "Advanced PID servo control interface\n"
                         "for the qNimble device.\n\n"
                         "© 2024 Quantum Opus LLC")
    
    def closeEvent(self, event):
        """Handle window close"""
        if self.serial.is_connected:
            reply = QMessageBox.question(self, "Disconnect?",
                                        "Disconnect from qNimble before closing?",
                                        QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.Yes:
                self.disconnect()
        event.accept()

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Modern look
    
    window = qNimbleGUI()
    window.show()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()