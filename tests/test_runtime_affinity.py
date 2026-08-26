import unittest

from controller.runtime_affinity import (
    RuntimeAffinityError,
    apply_runtime_affinity,
    config_from_root,
)


class RuntimeAffinityTests(unittest.TestCase):
    @staticmethod
    def _root(service=(0, 1, 2), control=(3,), *, enabled=True, required=True):
        return {
            "vezerles": {
                "idozites": {
                    "runtime_cpu_affinity": {
                        "enabled": enabled,
                        "required": required,
                        "service_cpus": list(service),
                        "control_cpus": list(control),
                    }
                }
            }
        }

    def test_service_and_control_masks_are_disjoint_and_verified(self):
        config = config_from_root(self._root())
        effective = {0, 1, 2, 3}

        def setter(_task, mask):
            nonlocal effective
            effective = set(mask)

        def getter(_task):
            return set(effective)

        service = apply_runtime_affinity(
            config, role="service", setter=setter, getter=getter, cpu_count=4
        )
        control = apply_runtime_affinity(
            config, role="control", setter=setter, getter=getter, cpu_count=4
        )

        self.assertEqual(service["roles"]["service"]["effective_cpus"], [0, 1, 2])
        self.assertTrue(service["roles"]["service"]["verified"])
        self.assertEqual(control["roles"]["control"]["effective_cpus"], [3])
        self.assertTrue(control["roles"]["control"]["verified"])
        self.assertFalse(control["roles"]["control"]["scheduler_policy_changed"])

        writer = apply_runtime_affinity(
            config, role="status_writer", setter=setter, getter=getter, cpu_count=4
        )
        self.assertEqual(writer["roles"]["status_writer"]["effective_cpus"], [0, 1, 2])

    def test_overlap_is_rejected(self):
        with self.assertRaisesRegex(RuntimeAffinityError, "cpu_sets_overlap"):
            config_from_root(self._root(service=(0, 1), control=(1,)))

    def test_required_unavailable_cpu_fails_closed(self):
        config = config_from_root(self._root(control=(3,), required=True))
        with self.assertRaisesRegex(RuntimeAffinityError, "cpu_unavailable"):
            apply_runtime_affinity(
                config,
                role="control",
                setter=lambda _task, _mask: None,
                getter=lambda _task: {0},
                cpu_count=2,
            )


if __name__ == "__main__":
    unittest.main()
