"""
BLE Keyboard/Gamepad Control - WASD or gamepad for throttle/steering, X for LED
"""

import asyncio
from bleak import BleakScanner, BleakClient
import combined_input as inp
from pathlib import Path

TARGET_DEVICE_NAME = "BLE_Sensor_Hub"
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
CHAR_UUID_LED = "12345678-1234-5678-1234-56789abcdef1"
CHAR_UUID_THROTTLE = "12345678-1234-5678-1234-56789abcdef2"
CHAR_UUID_STEERING = "12345678-1234-5678-1234-56789abcdef3"
CHAR_UUID_OMEGA = "12345678-1234-5678-1234-56789abcdef4"

def load_scales():
    try:
        config_path = Path(__file__).parent / "scales.txt"
        with open(config_path) as f:
            lines = f.read().strip().split('\n')
            return float(lines[0]), float(lines[1])
    except:
        return 1.0, 0.65

def to_byte(val):
    # Convert -1.0 to +1.0 range to 0-255 byte
    # Matches C toBipolar: (value - 128) / 128.0
    result = int(val * 128.0 + 128.0)
    return max(0, min(255, result))

async def control_loop():
    print(f"Scanning for {TARGET_DEVICE_NAME}...")
    devices = await BleakScanner.discover(timeout=10.0)
    target_device = next((d for d in devices if d.name == TARGET_DEVICE_NAME), None)
    
    if not target_device:
        print(f"✗ Device not found")
        return
    
    print(f"✓ Found at {target_device.address}")
    print(f"Connecting...")
    
    async with BleakClient(target_device.address) as client:
        if not client.is_connected:
            print("✗ Connection failed")
            return
        
        print(f"✓ Connected\n")
        
        last_throttle = last_steering = last_omega = None
        
        try:
            while True:
                THROTTLE_SCALE, STEERING_SCALE = load_scales()
                
                throttle = to_byte(inp.get_bipolar_ctrl('w', 's', 'LY') * THROTTLE_SCALE)
                steering = to_byte(inp.get_bipolar_ctrl('d', 'a', 'RX') * STEERING_SCALE)
                omega = to_byte(inp.get_bipolar_ctrl('e', 'q', 'RY'))  # Q/E keys or right stick Y
                
                if throttle != last_throttle:
                    await client.write_gatt_char(CHAR_UUID_THROTTLE, bytearray([throttle]))
                    last_throttle = throttle
                
                if steering != last_steering:
                    await client.write_gatt_char(CHAR_UUID_STEERING, bytearray([steering]))
                    last_steering = steering
                
                if omega != last_omega:
                    await client.write_gatt_char(CHAR_UUID_OMEGA, bytearray([omega]))
                    last_omega = omega
                
                # LED: OFF when pressed, ON when released
                led = 0 if (inp.is_pressed('x') or inp.is_pressed('A')) else 1
                await client.write_gatt_char(CHAR_UUID_LED, bytearray([led]))
                
                if inp.is_pressed('Key.esc'):
                    break
                
                await asyncio.sleep(0.05)
                
        except KeyboardInterrupt:
            pass

if __name__ == "__main__":
    try:
        asyncio.run(control_loop())
    except KeyboardInterrupt:
        print("Interrupted")
    except Exception as e:
        print(f"Error: {e}")
