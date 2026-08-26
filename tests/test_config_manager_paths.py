import importlib.util
import json
import shutil
import tempfile
import unittest
from pathlib import Path


class ConfigManagerPathTests(unittest.TestCase):
    def test_copied_candidate_loads_its_own_configuration(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            candidate = Path(tmp_dir)
            shutil.copy2(Path(__file__).resolve().parents[1] / "config_manager.py", candidate / "config_manager.py")
            conf = candidate / "conf"
            conf.mkdir()
            names = (
                "fizika",
                "vezerles",
                "hardver",
                "intelligencia",
                "speed_map",
                "control_mode",
                "security",
                "cam",
            )
            for name in names:
                (conf / f"{name}.json").write_text(
                    json.dumps({"source": "candidate", "name": name}),
                    encoding="utf-8",
                )
            spec = importlib.util.spec_from_file_location("candidate_config_manager", candidate / "config_manager.py")
            self.assertIsNotNone(spec)
            self.assertIsNotNone(spec.loader)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)

            self.assertEqual(Path(module.config.conf_dir), conf)
            self.assertTrue(all(not Path(path).is_absolute() for path in module.config._files.values()))
            self.assertEqual(module.config.get("control_mode", "source"), "candidate")


if __name__ == "__main__":
    unittest.main()
