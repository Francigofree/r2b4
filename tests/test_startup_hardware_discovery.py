import subprocess
import unittest
from unittest.mock import patch

from startup.phases import _i2c_scan, _i2c_scan_for_imu, _i2c_scan_until_imu_ready, _parse_i2cdetect_output


class StartupHardwareDiscoveryTests(unittest.TestCase):
    def test_parse_i2cdetect_output_extracts_hex_addresses(self):
        output = """
             0 1 2 3 4 5 6 7 8 9 a b c d e f
        00:                         -- -- -- -- -- -- --
        10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        50: -- -- -- 53 -- -- -- -- -- -- -- -- -- -- -- --
        60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
        70: -- -- -- -- -- -- -- --
        """
        self.assertEqual(_parse_i2cdetect_output(output), ["0x53", "0x68"])

    def test_parse_i2cdetect_output_extracts_sen0253_addresses(self):
        output = """
             0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
        20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- --
        70: -- -- -- -- -- -- 76 77
        """
        self.assertEqual(_parse_i2cdetect_output(output), ["0x28", "0x76", "0x77"])

    @patch("startup.phases._probe_i2c_addr_fallback")
    @patch("startup.phases.subprocess.run")
    def test_i2c_scan_uses_bno055_i2cdetect_when_available(self, run_mock, fallback_mock):
        run_mock.return_value = subprocess.CompletedProcess(
            args=["i2cdetect", "-y", "1"],
            returncode=0,
            stdout="""
                 0 1 2 3 4 5 6 7 8 9 a b c d e f
            20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- --
            """,
            stderr="",
        )

        found = _i2c_scan(1)

        self.assertEqual(found, ["0x28"])
        fallback_mock.assert_not_called()

    @patch("startup.phases._probe_i2c_addr_fallback")
    @patch("startup.phases.subprocess.run")
    def test_i2c_scan_supplements_partial_i2cdetect_result_with_bno055_probe(self, run_mock, fallback_mock):
        run_mock.return_value = subprocess.CompletedProcess(
            args=["i2cdetect", "-y", "1"],
            returncode=0,
            stdout="""
                 0 1 2 3 4 5 6 7 8 9 a b c d e f
            70: -- -- -- -- -- -- 76 --
            """,
            stderr="",
        )

        def _fallback(*, bus_num, addr, register=0x00, timeout_s=0.8):
            self.assertEqual(bus_num, 1)
            return addr == 0x28

        fallback_mock.side_effect = _fallback

        found = _i2c_scan(1)

        self.assertEqual(found, ["0x28", "0x76"])
        probed_addrs = [call.kwargs["addr"] for call in fallback_mock.call_args_list]
        self.assertEqual(probed_addrs, [0x28])

    @patch("startup.phases._probe_i2c_addr_fallback")
    @patch("startup.phases.subprocess.run")
    def test_i2c_scan_falls_back_to_bounded_bno055_probe(self, run_mock, fallback_mock):
        run_mock.side_effect = subprocess.TimeoutExpired(cmd=["i2cdetect", "-y", "1"], timeout=4.0)

        def _fallback(*, bus_num, addr, register=0x00, timeout_s=0.8):
            self.assertEqual(bus_num, 1)
            return addr == 0x28

        fallback_mock.side_effect = _fallback

        found = _i2c_scan(1)

        self.assertEqual(found, ["0x28"])
        self.assertEqual(fallback_mock.call_count, 1)

    @patch("startup.phases._probe_i2c_addr_fallback")
    @patch("startup.phases.subprocess.run")
    def test_i2c_scan_accepts_bno055_provider(self, run_mock, fallback_mock):
        run_mock.return_value = subprocess.CompletedProcess(
            args=["i2cdetect", "-y", "1"],
            returncode=0,
            stdout="""
                 0 1 2 3 4 5 6 7 8 9 a b c d e f
            20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- --
            70: -- -- -- -- -- -- 76 --
            """,
            stderr="",
        )

        found = _i2c_scan_for_imu(1, imu_provider="bno055", bno055_addr=0x28)

        self.assertEqual(found, ["0x28", "0x76"])
        fallback_mock.assert_not_called()

    @patch("startup.phases.time.sleep")
    @patch("startup.phases._i2c_scan_for_imu")
    def test_i2c_scan_until_imu_ready_retries_transient_miss(self, scan_mock, sleep_mock):
        scan_mock.side_effect = [
            ["0x76"],
            ["0x28", "0x76"],
        ]

        found, attempts = _i2c_scan_until_imu_ready(1, attempts=3, delay_s=0.01)

        self.assertEqual(found, ["0x28", "0x76"])
        self.assertEqual(attempts, [["0x76"], ["0x28", "0x76"]])
        self.assertEqual(scan_mock.call_count, 2)
        sleep_mock.assert_called_once()

    @patch("startup.phases.time.sleep")
    @patch("startup.phases._i2c_scan_for_imu")
    def test_i2c_scan_until_imu_ready_retries_bno055(self, scan_mock, sleep_mock):
        scan_mock.side_effect = [
            ["0x76"],
            ["0x28", "0x76"],
        ]

        found, attempts = _i2c_scan_until_imu_ready(
            1,
            attempts=3,
            delay_s=0.01,
            imu_provider="bno055",
            bno055_addr=0x28,
        )

        self.assertEqual(found, ["0x28", "0x76"])
        self.assertEqual(attempts, [["0x76"], ["0x28", "0x76"]])
        self.assertEqual(scan_mock.call_count, 2)
        sleep_mock.assert_called_once()


if __name__ == "__main__":
    unittest.main()
