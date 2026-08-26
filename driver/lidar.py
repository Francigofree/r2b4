#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import struct
import time
import math
import threading
import os
import glob


def _decode_scan_packet(raw):
    """
    Decode one 5-byte standard scan packet.
    Returns None when packet framing/check bits are invalid.
    """
    if not raw or len(raw) != 5:
        return None
    b0, b1, b2, b3, b4 = struct.unpack("BBBBB", raw)

    # RPLIDAR standard packet framing:
    # b0.bit0 = start flag, b0.bit1 = inverse start flag (must differ)
    # b1.bit0 = check bit (must be 1)
    start_flag = b0 & 0x01
    not_start_flag = (b0 >> 1) & 0x01
    check_bit = b1 & 0x01
    if (start_flag ^ not_start_flag) != 1 or check_bit != 1:
        return None

    quality = (b0 >> 2) & 0x3F
    angle_q6 = ((b1 >> 1) | (b2 << 7))
    angle = float(angle_q6) / 64.0
    if not (0.0 <= angle < 360.0):
        return None

    dist_q2 = (b3 | (b4 << 8))
    dist_mm = float(dist_q2) / 4.0
    if not math.isfinite(dist_mm):
        return None

    return {
        "new_scan_start": bool(start_flag),
        "quality": int(quality),
        "angle": float(angle),
        "angle_rad": float(math.radians(angle)),
        "dist_mm": float(dist_mm),
    }


def _resolve_default_port() -> str:
    """
    Prefer stable serial-by-id path. Avoid scanning /dev/ttyUSB* automatically.
    """
    try:
        by_id = sorted(glob.glob("/dev/serial/by-id/*"))
        if by_id:
            return str(by_id[0])
    except Exception:
        pass
    return "/dev/ttyUSB0"


RAW_SECTOR_SUMMARY_SOURCE = "DRIVER_CURRENT_RAW_SCAN_ACCUMULATOR"


def _new_raw_sector_accumulator():
    return {
        "min_front_mm": float("inf"),
        "min_front_point": None,
        "min_front_narrow_mm": float("inf"),
        "min_front_narrow_point": None,
        "min_back_mm": float("inf"),
        "left_sum_mm": 0.0,
        "left_count": 0,
        "right_sum_mm": 0.0,
        "right_count": 0,
        "valid_point_count": 0,
    }


def _accumulate_raw_sector_point(
    accumulator,
    *,
    angle_deg,
    dist_mm,
    min_distance_mm,
    max_distance_mm,
    quality=None,
):
    angle = float(angle_deg)
    distance = float(dist_mm)
    if not math.isfinite(angle) or not math.isfinite(distance):
        return
    if distance <= 0.0 or distance < float(min_distance_mm):
        return
    if float(max_distance_mm) > 0.0 and distance > float(max_distance_mm):
        return
    angle %= 360.0
    try:
        packet_quality = int(quality) if quality is not None else None
    except (TypeError, ValueError):
        packet_quality = None
    point_provenance = {
        "angle_deg": float(angle),
        "distance_mm": float(distance),
        "distance_m": float(distance / 1000.0),
        "quality": packet_quality,
    }
    accumulator["valid_point_count"] += 1
    if angle < 45.0 or angle > 315.0:
        if distance < accumulator["min_front_mm"]:
            accumulator["min_front_mm"] = distance
            accumulator["min_front_point"] = dict(point_provenance)
        if angle < 25.0 or angle > 335.0:
            if distance < accumulator["min_front_narrow_mm"]:
                accumulator["min_front_narrow_mm"] = distance
                accumulator["min_front_narrow_point"] = dict(
                    point_provenance
                )
    elif 135.0 < angle < 225.0:
        if distance < accumulator["min_back_mm"]:
            accumulator["min_back_mm"] = distance
    elif 225.0 <= angle <= 315.0:
        accumulator["left_sum_mm"] += distance
        accumulator["left_count"] += 1
    elif 45.0 <= angle <= 135.0:
        accumulator["right_sum_mm"] += distance
        accumulator["right_count"] += 1


def _finalize_raw_sector_summary(
    accumulator,
    *,
    scan_seq,
    min_distance_mm,
    max_distance_mm,
):
    def _minimum_m(key):
        value = float(accumulator[key])
        return value / 1000.0 if math.isfinite(value) else 10.0

    def _minimum_point(key):
        point = accumulator.get(key)
        if not isinstance(point, dict):
            return {}
        out = dict(point)
        out["raw_scan_id"] = int(scan_seq)
        return out

    left_count = int(accumulator["left_count"])
    right_count = int(accumulator["right_count"])
    avg_left = (
        float(accumulator["left_sum_mm"]) / left_count / 1000.0
        if left_count
        else 10.0
    )
    avg_right = (
        float(accumulator["right_sum_mm"]) / right_count / 1000.0
        if right_count
        else 10.0
    )
    return {
        "source": RAW_SECTOR_SUMMARY_SOURCE,
        "scan_seq": int(scan_seq),
        "min_distance_m": float(min_distance_mm) / 1000.0,
        "max_distance_m": float(max_distance_mm) / 1000.0,
        "min_dist": _minimum_m("min_front_mm"),
        "min_dist_point": _minimum_point("min_front_point"),
        "min_dist_narrow": _minimum_m("min_front_narrow_mm"),
        "min_dist_narrow_point": _minimum_point(
            "min_front_narrow_point"
        ),
        "min_back": _minimum_m("min_back_mm"),
        "avg_left": float(avg_left),
        "avg_right": float(avg_right),
        "raw_safety_valid_point_count": int(
            accumulator["valid_point_count"]
        ),
    }


class LidarC1Driver:
    """
    RPLiDAR C1 driver Raspberry Pi 5-re, USB2 csatlakozással.
    - EKF kompatibilis nyers pontokat szolgáltat.
    - Folyamatos háttérben futó frissítés.
    """
    def __init__(
        self,
        port=None,
        baudrate=460800,
        min_distance_mm=50.0,
        max_distance_mm=12000.0,
        read_chunk_size=512,
        stale_timeout_s=0.5,
        read_timeout_s=0.1,
    ):
        resolved_port = str(port).strip() if port is not None else ""
        if not resolved_port:
            resolved_port = _resolve_default_port()
        self.port = resolved_port
        self.baudrate = int(baudrate)
        self.ser = None
        self.running = False
        self.last_scan = []
        self.lock = threading.Lock()
        self.thread = None
        self.min_distance_mm = max(0.0, float(min_distance_mm))
        self.max_distance_mm = float(max_distance_mm)
        self.read_chunk_size = max(32, int(read_chunk_size))
        self.stale_timeout_s = max(0.1, float(stale_timeout_s))
        self.read_timeout_s = max(0.01, float(read_timeout_s))
        self._stale_grace_s = max(1.5, self.stale_timeout_s * 2.0)
        self._stale_grace_until = time.time() + self._stale_grace_s
        self._reconnect_cooldown_s = 0.4
        self._last_reconnect_attempt = 0.0
        self._stream_seen = False
        self._rx_buffer = bytearray()
        self._invalid_packet_count = 0
        self._reconnect_count = 0
        self.last_data_time = time.time()
        self._last_scan_mono_ts = 0.0
        self._scan_seq = 0
        self._last_raw_sector_summary = {}

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.read_timeout_s)
            self.ser.dtr = False
            self.ser.rts = False
            # Stop parancs
            self.ser.write(b'\xA5\x25')
            time.sleep(0.5)
            self.ser.reset_input_buffer()
            # Scan indítása
            self.ser.write(b'\xA5\x20')
            self.ser.read(7)  # Fejléc eldobása
            self._rx_buffer.clear()
            self._invalid_packet_count = 0
            self.last_data_time = time.time()
            self._stale_grace_until = self.last_data_time + self._stale_grace_s
            self._stream_seen = False
            self.running = True
            return True
        except Exception as e:
            print(f"[Lidar] Hiba a csatlakozásnál: {e}")
            return False

    def _reconnect(self):
        now = time.time()
        if (now - self._last_reconnect_attempt) < self._reconnect_cooldown_s:
            time.sleep(0.01)
            return False
        self._last_reconnect_attempt = now
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self._reconnect_count += 1
        time.sleep(0.2)
        if not self.running:
            return False
        return bool(self.connect())

    def _distance_valid(self, dist_mm):
        if dist_mm <= 0.0 or dist_mm < self.min_distance_mm:
            return False
        if self.max_distance_mm > 0.0 and dist_mm > self.max_distance_mm:
            return False
        return True

    def _update_loop(self):
        """Háttér frissítő szál."""
        temp_points = []
        temp_sector_accumulator = _new_raw_sector_accumulator()
        while self.running:
            try:
                if self.ser is None or not getattr(self.ser, "is_open", False):
                    self._reconnect()
                    time.sleep(0.01)
                    continue

                now_wall = time.time()
                stale_reconnect_enabled = bool(
                    self._stream_seen or (now_wall - self._stale_grace_until) > 8.0
                )
                if (
                    stale_reconnect_enabled
                    and
                    now_wall >= self._stale_grace_until
                    and (now_wall - self.last_data_time) > self.stale_timeout_s
                ):
                    print("[Lidar] STALE → reconnect")
                    self._reconnect()
                    continue

                waiting = int(getattr(self.ser, "in_waiting", 0) or 0)
                if waiting <= 0:
                    # Egyes USB-serial drivereknél az in_waiting átmenetileg 0 maradhat,
                    # miközben már érkezik adat. Ilyenkor is próbálunk kis read-et.
                    chunk = self.ser.read(min(64, self.read_chunk_size))
                    if not chunk:
                        time.sleep(0.002)
                        continue
                else:
                    read_len = min(max(waiting, 5), self.read_chunk_size)
                    chunk = self.ser.read(read_len)
                if waiting > 0 and not chunk:
                    # A driver jelez olvashatóságot, de nem érkezik byte.
                    print("[Lidar] ghost readiness → reset buffer")
                    self._rx_buffer.clear()
                    time.sleep(0.01)
                    continue
                if not chunk:
                    time.sleep(0.001)
                    continue
                self.last_data_time = time.time()
                self._stream_seen = True

                self._rx_buffer.extend(chunk)
                if len(self._rx_buffer) > 4096:
                    self._rx_buffer.clear()
                    time.sleep(0.001)
                    continue

                # Byte-aligned recovery: invalid packet header => drop one byte and retry.
                parsed_packet = False
                while len(self._rx_buffer) >= 5:
                    packet = _decode_scan_packet(self._rx_buffer[:5])
                    if packet is None:
                        del self._rx_buffer[0]
                        self._invalid_packet_count += 1
                        continue

                    del self._rx_buffer[:5]
                    parsed_packet = True

                    if packet["new_scan_start"] and temp_points:
                        with self.lock:
                            self.last_scan = temp_points.copy()
                            self._scan_seq += 1
                            self._last_scan_mono_ts = time.monotonic()
                            self._last_raw_sector_summary = (
                                _finalize_raw_sector_summary(
                                    temp_sector_accumulator,
                                    scan_seq=self._scan_seq,
                                    min_distance_mm=self.min_distance_mm,
                                    max_distance_mm=self.max_distance_mm,
                                )
                            )
                        temp_points = []
                        temp_sector_accumulator = _new_raw_sector_accumulator()

                    dist_mm = float(packet["dist_mm"])
                    if self._distance_valid(dist_mm):
                        angle = float(packet["angle"])
                        quality = int(packet["quality"])
                        temp_points.append(
                            {
                                "angle": angle,
                                "angle_rad": float(packet["angle_rad"]),
                                "dist": dist_mm,
                                "quality": quality,
                            }
                        )
                        _accumulate_raw_sector_point(
                            temp_sector_accumulator,
                            angle_deg=angle,
                            dist_mm=dist_mm,
                            min_distance_mm=self.min_distance_mm,
                            max_distance_mm=self.max_distance_mm,
                            quality=quality,
                        )
                if not parsed_packet:
                    # CPU védelem: ha még nincs teljes/érvényes csomag, ne pörögjön a ciklus.
                    time.sleep(0.0005)
            except Exception as e:
                print(f"[Lidar] Olvasási hiba: {e}")
                self._reconnect()
                time.sleep(0.01)

    def get_runtime_status(self):
        now = time.time()
        age_s = max(0.0, float(now - self.last_data_time))
        connected = bool(self.ser is not None and getattr(self.ser, "is_open", False))
        with self.lock:
            scan_seq = int(self._scan_seq)
            scan_age_s = (
                max(0.0, float(time.monotonic() - self._last_scan_mono_ts))
                if self._last_scan_mono_ts > 0.0
                else float("inf")
            )
        return {
            "running": bool(self.running),
            "connected": connected,
            "last_data_age_s": float(age_s),
            "scan_seq": int(scan_seq),
            "scan_age_s": float(scan_age_s),
            "rx_buffer_len": int(len(self._rx_buffer)),
            "invalid_packet_count": int(self._invalid_packet_count),
            "reconnect_count": int(self._reconnect_count),
            "stream_seen": bool(self._stream_seen),
            "port": str(self.port),
            "baudrate": int(self.baudrate),
            "read_chunk_size": int(self.read_chunk_size),
            "stale_timeout_s": float(self.stale_timeout_s),
        }

    def start(self):
        if self.running:
            return True # Már fut, nem kell újraindítani

        if self.connect():
            self.thread = threading.Thread(target=self._update_loop, daemon=True)
            self.thread.start()
            return True
        return False

    def get_latest_scan(self):
        """EKF-nek: legfrissebb nyers pontok."""
        with self.lock:
            return self.last_scan.copy()

    def get_latest_scan_meta(self):
        """Latest raw scan + sequence for non-blocking producer pipeline."""
        with self.lock:
            return {
                "scan": self.last_scan.copy(),
                "scan_seq": int(self._scan_seq),
                "scan_ts_mono": float(self._last_scan_mono_ts),
                "raw_sector_summary": dict(self._last_raw_sector_summary),
            }

    def stop(self):
        if not self.running:
            return

        self.running = False
        if self.thread and self.thread.is_alive():
            # A szál leáll a self.running False miatt, de itt nem joinoljuk blokkolóan
            # hogy a vezérlés gyors maradjon.
            pass

        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b'\xA5\x25')  # Stop
                time.sleep(0.1)
                self.ser.dtr = True
                self.ser.close()
            except Exception as e:
                print(f"[Lidar] Leállítási hiba: {e}")


# --- Teszt és karakteres vizualizáció ---
if __name__ == "__main__":
    lidar = LidarC1Driver()
    labels = {
        0: "ELŐRE (É)", 45: "JOBB-EL (ÉK)", 90: "JOBBRA (K)", 135: "JOBB-HÁ (DK)",
        180: "HÁTRA (D)", 225: "BAL-HÁ (DNY)", 270: "BALRA (NY)", 315: "BAL-EL (ÉNY)"
    }

    if lidar.start():
        try:
            while True:
                scan_data = lidar.get_latest_scan()
                display_distances = {i: 0.0 for i in range(0, 360, 45)}
                if scan_data:
                    for p in scan_data:
                        idx = int(((p['angle'] + 22.5) % 360) / 45) * 45
                        display_distances[idx] = p['dist']

                print("\033[H", end="")
                print("--- LIDAR C1 STABIL 8 IRÁNY ---")
                print(f" Idő: {time.strftime('%H:%M:%S')} | RPi 5 OK")
                print("-" * 40)

                for ang in sorted(display_distances.keys()):
                    d = display_distances[ang]
                    bar_len = int(min(d, 2000) / 100)
                    bar = "#" * bar_len
                    val_str = f"{d:>7.1f} mm" if d > 0 else "  ---    "
                    print(f"{labels[ang]:<15}: {val_str} {bar:<20}")

                print("-" * 40)
                print("Kilépés: Ctrl+C                          ")
                time.sleep(0.1)

        except KeyboardInterrupt:
            lidar.stop()
            print("\n[+] Leállítva. Szia!")
    else:
        print("Hiba: Nem sikerült a Lidarhoz csatlakozni!")
