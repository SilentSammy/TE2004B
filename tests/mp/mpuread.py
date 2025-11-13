# Minimalistic MPU-9250 Reader (raw values)
from machine import I2C, Pin
import time
import struct

# Setup
i2c = I2C(scl=Pin(7), sda=Pin(6))
MPU = 0x68

# Wake up MPU-9250
try:
    i2c.writeto_mem(MPU, 0x6B, b'\x00')
    time.sleep(0.1)
    print("MPU-9250 Ready")
except:
    print("MPU-9250 not found, will retry...")

# Continuous reading
while True:
    try:
        # Read accel (6 bytes from register 0x3B)
        accel = struct.unpack('>hhh', i2c.readfrom_mem(MPU, 0x3B, 6))
        ax, ay, az = [x/16384.0 for x in accel]
        
        # Read gyro (6 bytes from register 0x43)
        gyro = struct.unpack('>hhh', i2c.readfrom_mem(MPU, 0x43, 6))
        gx, gy, gz = [x/131.0 for x in gyro]
        
        print(f"A: {ax:6.2f} {ay:6.2f} {az:6.2f} | G: {gx:7.1f} {gy:7.1f} {gz:7.1f}")
    except:
        # Silently handle errors, try to re-init
        try:
            i2c.writeto_mem(MPU, 0x6B, b'\x00')
        except:
            pass
    
    time.sleep(0.2)
