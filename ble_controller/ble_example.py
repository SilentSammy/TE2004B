"""
Example: Import and use CarBLEClient for custom control
"""

import asyncio
from ble_client import CarBLEClient
import time

async def example_loop():
    # Create client instance
    car = CarBLEClient()
    
    # Connect to the device
    await car.connect()
    
    try:
        while True:
            # await car.set_waypoint(45, 45)
            await car.set_led(0)
            time.sleep(0.5)
            await car.set_led(1)
            time.sleep(0.5)
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        await car.disconnect()

if __name__ == "__main__":
    try:
        asyncio.run(example_loop())
    except KeyboardInterrupt:
        print("Exited")
    except Exception as e:
        print(f"Error: {e}")
