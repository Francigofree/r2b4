#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest

from driver.bno055 import BNO055IMU


class _FakeBus:
    def __init__(self):
        self.regs = {BNO055IMU.REG_CHIP_ID: BNO055IMU.CHIP_ID}
        self.block_reads = 0
        self.writes = []
        self.closed = False

    def write_byte_data(self, addr, reg, value):
        self.writes.append((addr, reg, value))
        self.regs[reg] = value

    def read_byte_data(self, addr, reg):
        if reg == BNO055IMU.REG_CALIB_STAT:
            return 0b11111111
        if reg == BNO055IMU.REG_SYS_STATUS:
            return 5
        if reg == BNO055IMU.REG_SYS_ERR:
            return 0
        return self.regs.get(reg, 0)

    def read_i2c_block_data(self, addr, reg, length):
        self.block_reads += 1
        data = [0] * length

        def put_s16(offset, value):
            if value < 0:
                value = (1 << 16) + value
            data[offset] = value & 0xFF
            data[offset + 1] = (value >> 8) & 0xFF

        # gyro raw: 16 LSB / dps
        put_s16(0, 160)
        put_s16(2, -80)
        put_s16(4, 32)
        # euler raw: 16 LSB / degree
        put_s16(6, 1440)
        put_s16(8, -160)
        put_s16(10, 80)
        # quaternion raw: 16384 LSB / unit
        put_s16(12, 16384)
        put_s16(14, 0)
        put_s16(16, 0)
        put_s16(18, 0)
        # linear acceleration raw: 100 LSB / m/s2
        put_s16(20, 981)
        put_s16(22, 0)
        put_s16(24, -100)
        # gravity raw
        put_s16(26, 0)
        put_s16(28, 0)
        put_s16(30, 981)
        return data

    def close(self):
        self.closed = True


class BNO055DriverTests(unittest.TestCase):
    def test_initialize_and_read_fused_sample(self):
        bus = _FakeBus()
        imu = BNO055IMU(bus=bus, operation_mode="NDOF")

        self.assertTrue(imu.initialize())
        sample = imu.read_sample(force=True)

        self.assertEqual(sample["source"], "bno055")
        self.assertEqual(sample["gyro_dps"], (10.0, -5.0, 2.0))
        self.assertAlmostEqual(sample["accel_g"][0], 1.0003, places=3)
        self.assertEqual(sample["euler"]["heading_deg"], 90.0)
        self.assertEqual(sample["quaternion"], (1.0, 0.0, 0.0, 0.0))
        self.assertEqual(sample["calibration"], {"sys": 3, "gyro": 3, "accel": 3, "mag": 3})
        self.assertTrue(any(reg == BNO055IMU.REG_OPR_MODE and value == BNO055IMU.MODE_NDOF for _, reg, value in bus.writes))

    def test_atomic_sample_api_uses_cache_within_update_period(self):
        bus = _FakeBus()
        imu = BNO055IMU(bus=bus)
        imu.initialize()
        bus.block_reads = 0
        imu._last_sample_ts = 0.0

        first = imu.read_sample()
        second = imu.read_sample()

        self.assertEqual(first["gyro_dps"], (10.0, -5.0, 2.0))
        self.assertAlmostEqual(first["accel_g"][0], 1.0003, places=3)
        self.assertEqual(first["heading_deg"], 90.0)
        self.assertEqual(second, first)
        self.assertEqual(bus.block_reads, 1)


if __name__ == "__main__":
    unittest.main()
