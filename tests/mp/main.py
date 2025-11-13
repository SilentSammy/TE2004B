import ble_server as bs
from machine import Pin

# Setup LED
led = Pin(8, Pin.OUT)

def set_led(value):
    """Handle LED control (0-255, but treat as binary)"""
    led.value(1 if value else 0)

# Set up BLE server
DEVICE_NAME = "BLE_Test"
bs.control_callbacks = {
    1: set_led,
}

if __name__ == "__main__":
    print("Starting BLE server...")
    bs.start()
