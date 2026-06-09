"""
test_serial.py - Test the serial handler
"""

from serial_handler import SerialHandler
import time

# Create handler
handler = SerialHandler()

# List available ports
print("Available ports:")
ports = handler.get_available_ports()
for port in ports:
    print(f"  {port}")

# Connect (change COM3 to your port!)
print("\nConnecting to qNimble...")
if handler.connect("COM3"):  # ← CHANGE THIS TO YOUR PORT!
    print("✓ Connected!")
    
    # Test commands
    print("\nTesting commands...")
    
    # Set P gain
    print("Setting P gain to 0.5...")
    response = handler.set_p_gain(1, 0.5)
    print(f"Response: {response}")
    
    # Get status
    print("\nGetting status...")
    status = handler.get_status()
    if status:
        for ch in status:
            print(f"Channel {ch['channel']}: Servo={ch['servo_active']}, Input={ch['adc_input']:.4f}V")
    
    # Disconnect
    handler.disconnect()
    print("\n✓ Disconnected")
else:
    print("✗ Connection failed")