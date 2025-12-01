#!/usr/bin/env python3
import time
import json
import os
import smbus

# ==== EDIT THESE IF NEEDED ====
I2C_BUS   = 7
I2C_ADDR  = 0x28
CALIB_FILE = os.path.expanduser("~/.ros/bno055_offsets.json")
# ===============================

# BNO055 registers / constants
BNO055_OPR_MODE        = 0x3D
BNO055_SYS_TRIGGER     = 0x3F
BNO055_POWER_MODE      = 0x3E

BNO055_UNIT_SEL        = 0x3B
BNO055_PAGE_ID         = 0x07

BNO055_CALIB_STAT      = 0x35

# Operation modes
OPR_MODE_CONFIG        = 0x00
OPR_MODE_NDOF          = 0x0C  # gyro+acc+mag fusion

# Offset registers start (acc, mag, gyro, radii)
BNO055_OFFSET_START    = 0x55
BNO055_OFFSET_END      = 0x6A  # inclusive


def write8(bus, addr, reg, val):
    bus.write_byte_data(addr, reg, val & 0xFF)


def read8(bus, addr, reg):
    return bus.read_byte_data(addr, reg) & 0xFF


def set_mode(bus, addr, mode):
    write8(bus, addr, BNO055_OPR_MODE, mode)
    time.sleep(0.03)


def set_units_and_power(bus, addr):
    # Use degrees, deg/s, m/s^2, Celsius, Android orientation
    # UNIT_SEL: 0b0000_0000 => default, but we set explicitly
    write8(bus, addr, BNO055_UNIT_SEL, 0x00)
    # Normal power mode
    write8(bus, addr, BNO055_POWER_MODE, 0x00)
    # Ensure page 0
    write8(bus, addr, BNO055_PAGE_ID, 0x00)
    time.sleep(0.05)


def get_calib_status(bus, addr):
    stat = read8(bus, addr, BNO055_CALIB_STAT)
    sys  = (stat >> 6) & 0x03
    gyro = (stat >> 4) & 0x03
    accel= (stat >> 2) & 0x03
    mag  =  stat       & 0x03
    return sys, gyro, accel, mag


def read_offsets(bus, addr):
    # BNO055 offset block is 22 bytes (0x55–0x6A)
    length = BNO055_OFFSET_END - BNO055_OFFSET_START + 1
    raw = bus.read_i2c_block_data(addr, BNO055_OFFSET_START, length)

    def s16(lo, hi):
        v = (hi << 8) | lo
        if v & 0x8000:
            v -= 0x10000
        return v

    offs = {}
    # Order from datasheet:
    # ACC_OFFSET_X_LSB (0x55), ACC_OFFSET_X_MSB (0x56), ...
    offs["acc_offset_x"] = s16(raw[0],  raw[1])
    offs["acc_offset_y"] = s16(raw[2],  raw[3])
    offs["acc_offset_z"] = s16(raw[4],  raw[5])

    offs["mag_offset_x"] = s16(raw[6],  raw[7])
    offs["mag_offset_y"] = s16(raw[8],  raw[9])
    offs["mag_offset_z"] = s16(raw[10], raw[11])

    offs["gyro_offset_x"] = s16(raw[12], raw[13])
    offs["gyro_offset_y"] = s16(raw[14], raw[15])
    offs["gyro_offset_z"] = s16(raw[16], raw[17])

    offs["acc_radius"] = s16(raw[18], raw[19])
    offs["mag_radius"] = s16(raw[20], raw[21])

    return offs


def main():
    bus = smbus.SMBus(I2C_BUS)

    print("Putting BNO055 into CONFIG mode...")
    set_mode(bus, I2C_ADDR, OPR_MODE_CONFIG)
    set_units_and_power(bus, I2C_ADDR)

    print("Switching to NDOF mode (gyro+acc+mag fusion)...")
    set_mode(bus, I2C_ADDR, OPR_MODE_NDOF)

    print("Move the IMU slowly to calibrate:")
    print("  - Leave flat & still for gyro.")
    print("  - Tilt around X/Y for accel.")
    print("  - Do slow figure-8 for mag.")
    print("Waiting for calib: sys>=3, gyro=3, accel=3, mag=3 ...")

    while True:
        sys, gyro, accel, mag = get_calib_status(bus, I2C_ADDR)
        print(f"Calib status -> sys:{sys} gyro:{gyro} accel:{accel} mag:{mag}")
        if sys >= 3 and gyro == 3 and accel == 3 and mag == 3:
            print("All calibrated (3/3/3/3).")
            break
        time.sleep(1.0)

    print("Switch back to CONFIG mode to read offsets...")
    set_mode(bus, I2C_ADDR, OPR_MODE_CONFIG)

    offs = read_offsets(bus, I2C_ADDR)
    print("Read offsets:")
    for k, v in offs.items():
        print(f"  {k}: {v}")

    os.makedirs(os.path.dirname(CALIB_FILE), exist_ok=True)
    with open(CALIB_FILE, "w") as f:
        json.dump(offs, f, indent=2)

    print(f"\nSaved offsets to: {CALIB_FILE}")
    print("You only need to run this script occasionally (after big changes).")


if __name__ == "__main__":
    main()
