# main.py -- Continuous I2C Scanner for Debugging
from machine import I2C, Pin
import time

# Initialize I2C
# Adjust the pins according to your board configuration
# Common configurations:
# ESP32: scl=Pin(22), sda=Pin(21)
# ESP8266: scl=Pin(5), sda=Pin(4)
# Pico: scl=Pin(5), sda=Pin(4)
i2c = I2C(scl=Pin(7), sda=Pin(6))

print("Continuous I2C Scanner")
print("Connect/disconnect devices to see changes")
print("Press Ctrl+C to stop")
print("=" * 50)
print()

previous_devices = []

try:
    while True:
        # Scan for devices
        devices = i2c.scan()
        
        # Check if the device list has changed
        if devices != previous_devices:
            print(f"[{time.ticks_ms()}] Scan Update:")
            
            if len(devices) == 0:
                print("  No I2C devices found")
            else:
                print(f"  Found {len(devices)} device(s):")
                for device in devices:
                    # Check if this is a newly connected device
                    if device not in previous_devices:
                        print(f"    [NEW] Decimal: {device:3d} | Hex: 0x{device:02X}")
                    else:
                        print(f"          Decimal: {device:3d} | Hex: 0x{device:02X}")
            
            # Check for disconnected devices
            disconnected = [d for d in previous_devices if d not in devices]
            if disconnected:
                print("  Disconnected:")
                for device in disconnected:
                    print(f"    [REMOVED] Decimal: {device:3d} | Hex: 0x{device:02X}")
            
            print("-" * 50)
            previous_devices = devices.copy()
        
        # Scan every 0.5 seconds
        time.sleep(0.5)
        
except KeyboardInterrupt:
    print("\n" + "=" * 50)
    print("Scanner stopped by user")
    print(f"Final device count: {len(previous_devices)}")