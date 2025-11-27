"""
BLE Keyboard/Gamepad Control - WASD or gamepad for throttle/steering, X for LED
"""

import asyncio
from bleak import BleakScanner, BleakClient
import combined_input as inp
from pathlib import Path
import time

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

class CarBLEClient:
    def __init__(self, device_name="BLE_Sensor_Hub", base_uuid="12345678-1234-5678-1234-56789abcdef"):
        self.device_name = device_name
        self.base_uuid = base_uuid
        self.device = None
        self.client = None
        
        # Cache last sent values to avoid redundant BLE writes
        self._cached_char_values = {}
        self._cached_char_timestamps = {}
    
    async def find_device(self):
        print(f"Scanning for {self.device_name}...")
        devices = await BleakScanner.discover(timeout=10.0)
        device = next((d for d in devices if d.name == self.device_name), None)
        if not device:
            print(f"✗ Device not found")
            return None
        print(f"✓ Found at {device.address}")
        return device
    
    async def connect(self):
        self.device = await self.find_device()
        if not self.device:
            raise Exception("Device not found")
        
        print(f"Connecting...")
        self.client = BleakClient(self.device.address)
        await self.client.connect()
        if not self.client.is_connected:
            raise Exception("Connection failed")
        print(f"✓ Connected\n")
    
    async def disconnect(self):
        if self.client:
            await self.client.disconnect()
            print("Disconnected.")
    
    async def set_char(self, char_uuid, value, force=False, min_interval=0.1):
        # Normalize value to bytes
        if isinstance(value, int):
            value_bytes = bytes([value])
        elif isinstance(value, (bytes, bytearray, list)):
            value_bytes = bytes(value)
        else:
            raise TypeError("value must be int, bytes, bytearray, or list")
        
        # Check value cache (compare bytes)
        if not force and self._cached_char_values.get(char_uuid) == value_bytes:
            return  # No change, skip write
        
        # Check timestamp cache
        now = time.time()
        last_time = self._cached_char_timestamps.get(char_uuid, 0)
        if not force and (now - last_time) < min_interval:
            return  # Too soon, skip write
        
        await self.client.write_gatt_char(char_uuid, value_bytes)
        self._cached_char_values[char_uuid] = value_bytes
        self._cached_char_timestamps[char_uuid] = now
    
    async def set_waypoint(self, x, y, omega):
        """Set waypoint (x, y in cm, omega in degrees)"""
        # Encode as int16_t with ×10 scaling
        x_mm = int(x * 10)
        y_mm = int(y * 10)
        omega_dec = int(omega * 10)
        
        # Pack as little-endian int16_t (6 bytes total)
        data = [
            x_mm & 0xFF, (x_mm >> 8) & 0xFF,
            y_mm & 0xFF, (y_mm >> 8) & 0xFF,
            omega_dec & 0xFF, (omega_dec >> 8) & 0xFF
        ]
    
        await self.set_char(self.base_uuid + "5", data, force=True)

    async def set_led(self, state):
        await self.set_char(self.base_uuid + "1", int(state))
    
    async def set_throttle(self, throttle):
        """Set throttle (-1.0 to 1.0)"""
        mapped = to_byte(throttle)
        await self.set_char(self.base_uuid + "2", mapped)
    
    async def set_steering(self, steering):
        """Set steering (-1.0 to 1.0)"""
        mapped = to_byte(steering)
        await self.set_char(self.base_uuid + "3", mapped)
    
    async def set_omega(self, omega):
        """Set omega (-1.0 to 1.0)"""
        mapped = to_byte(omega)
        await self.set_char(self.base_uuid + "4", mapped)

async def control_loop():
    car = CarBLEClient()
    await car.connect()
    
    try:
        while True:
            THROTTLE_SCALE, STEERING_SCALE = load_scales()
            
            throttle = inp.get_bipolar_ctrl('w', 's', 'LY') * THROTTLE_SCALE
            steering = inp.get_bipolar_ctrl('d', 'a', 'RX') * STEERING_SCALE
            omega = inp.get_bipolar_ctrl('e', 'q', 'RY')  # Q/E keys or right stick Y
            
            await car.set_throttle(throttle)
            await car.set_steering(steering)
            await car.set_omega(omega)
            
            # LED: OFF when pressed, ON when released
            led = 0 if (inp.is_pressed('x') or inp.is_pressed('A')) else 1
            await car.set_led(led)
            
            if inp.is_pressed('Key.esc'):
                break
            
            await asyncio.sleep(0.05)
            
    except KeyboardInterrupt:
        pass
    finally:
        await car.disconnect()

async def debug_loop():
    car = CarBLEClient()
    await car.connect()
    
    try:
        # Set a test waypoint
        # await car.set_waypoint(50.0, 100.0, 90.0)
        # print("Waypoint set to (50.0, 100.0, 90.0)")
        pass
        
    except KeyboardInterrupt:
        pass
    finally:
        await car.disconnect()

if __name__ == "__main__":
    try:
        # asyncio.run(control_loop())
        asyncio.run(debug_loop())
    except KeyboardInterrupt:
        print("Interrupted")
    except Exception as e:
        print(f"Error: {e}")
