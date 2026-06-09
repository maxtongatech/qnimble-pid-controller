"""
terminal_tab.py - Terminal Tab
Direct serial command interface
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout,
                            QTextEdit, QLineEdit, QPushButton, QLabel)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QTextCursor

class TerminalTab(QWidget):
    """Terminal interface for direct commands"""
    
    def __init__(self, serial_handler):
        super().__init__()
        self.serial = serial_handler
        self.command_history = []
        self.history_index = 0
        self.setup_ui()
        
    def setup_ui(self):
        """Setup the UI"""
        layout = QVBoxLayout()
        
        # Title
        title = QLabel("Serial Terminal")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)
        
        # Terminal display
        self.terminal_display = QTextEdit()
        self.terminal_display.setReadOnly(True)
        self.terminal_display.setFont(QFont("Courier", 10))
        layout.addWidget(self.terminal_display)
        
        # Command input
        input_layout = QHBoxLayout()
        
        input_layout.addWidget(QLabel("Command:"))
        
        self.command_input = QLineEdit()
        self.command_input.returnPressed.connect(self.send_command)
        input_layout.addWidget(self.command_input)
        
        self.send_btn = QPushButton("Send")
        self.send_btn.clicked.connect(self.send_command)
        input_layout.addWidget(self.send_btn)
        
        self.clear_btn = QPushButton("Clear")
        self.clear_btn.clicked.connect(self.clear_terminal)
        input_layout.addWidget(self.clear_btn)
        
        layout.addLayout(input_layout)
        
        # Help text
        help_text = QLabel("Quick commands: help, status, version, prop1 0.5, enable1 1, save")
        help_text.setStyleSheet("color: gray; font-style: italic;")
        layout.addWidget(help_text)
        
        self.setLayout(layout)
        
        # Welcome message
        self.terminal_display.append("=== qNimble Terminal ===")
        self.terminal_display.append("Type 'help' for command list\n")
        
    def send_command(self):
        """Send command to qNimble"""
        command = self.command_input.text().strip()
        
        if not command:
            return
            
        if not self.serial.is_connected:
            self.terminal_display.append("ERROR: Not connected\n")
            return
            
        # Add to history
        self.command_history.append(command)
        self.history_index = len(self.command_history)
        
        # Display command
        self.terminal_display.append(f"> {command}")
        
        # Send and display response
        response = self.serial.send_command(command)
        if response:
            self.terminal_display.append(response)
        else:
            self.terminal_display.append("(no response)")
        self.terminal_display.append("")  # Blank line
        
        # Scroll to bottom
        self.terminal_display.moveCursor(QTextCursor.End)
        
        # Clear input
        self.command_input.clear()
        
    def clear_terminal(self):
        """Clear terminal display"""
        self.terminal_display.clear()
        self.terminal_display.append("=== qNimble Terminal ===")
        self.terminal_display.append("Type 'help' for command list\n")
        
    def update_data(self):
        """Update tab data"""
        pass  # Nothing to update continuously