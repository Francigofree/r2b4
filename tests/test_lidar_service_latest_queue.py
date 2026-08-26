import queue
import threading
import unittest

from sensors.lidar_service import LidarService


class LidarServiceLatestQueueTests(unittest.TestCase):
    @staticmethod
    def _service(queue_size=2):
        service = LidarService.__new__(LidarService)
        service._queue = queue.Queue(maxsize=queue_size)
        service._lock = threading.Lock()
        service._runtime_status = {"queue_drops": 0}
        return service

    def test_new_packet_replaces_single_waiting_packet_before_queue_is_full(self):
        service = self._service(queue_size=2)
        first = {"scan_seq": 1}
        latest = {"scan_seq": 2}

        service._queue_latest(first)
        service._queue_latest(latest)

        self.assertIs(service._queue.get_nowait(), latest)
        self.assertTrue(service._queue.empty())
        self.assertEqual(service._runtime_status["queue_drops"], 1)

    def test_new_packet_discards_every_waiting_packet_and_counts_each_drop(self):
        service = self._service(queue_size=3)
        service._queue.put_nowait({"scan_seq": 10})
        service._queue.put_nowait({"scan_seq": 11})
        service._queue.put_nowait({"scan_seq": 12})
        latest = {"scan_seq": 13}

        service._queue_latest(latest)

        self.assertIs(service._queue.get_nowait(), latest)
        self.assertTrue(service._queue.empty())
        self.assertEqual(service._runtime_status["queue_drops"], 3)

    def test_first_packet_is_not_reported_as_a_drop(self):
        service = self._service(queue_size=2)
        packet = {"scan_seq": 21}

        service._queue_latest(packet)

        self.assertIs(service._queue.get_nowait(), packet)
        self.assertEqual(service._runtime_status["queue_drops"], 0)


if __name__ == "__main__":
    unittest.main()
