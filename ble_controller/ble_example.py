"""
Example: Import and use CarBLEClient for custom control
"""

import asyncio
from ble_client import CarBLEClient

async def example_loop():
    # Create client instance
    car = CarBLEClient()
    
    # Connect to the device
    await car.connect()
    
    try:
        while True:
            # Example 1: Set constant values
            await car.set_throttle(0.5)    # 50% forward
            await car.set_steering(0.3)    # 30% right
            await car.set_omega(0.0)       # no rotation
            await car.set_waypoint(10.0, 20.0, 45.0)  # Set waypoint to (10, 20) with 45 degrees orientation
        
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
