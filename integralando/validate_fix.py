"""Self-contained smoke validation for the encoder robustness replacement files."""
from __future__ import annotations

import importlib.util
import sys
import types
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

ROOT = Path(__file__).resolve().parent

# Minimal package skeleton so the replacement modules can be executed without
# needing a second copy of the whole R2B4 repository in this artifact folder.
v3 = types.ModuleType("v3"); v3.__path__ = []
adapters = types.ModuleType("v3.adapters"); adapters.__path__ = []
layers = types.ModuleType("v3.layers"); layers.__path__ = []
sys.modules.update({"v3": v3, "v3.adapters": adapters, "v3.layers": layers})

contracts = types.ModuleType("v3.contracts")

@dataclass(frozen=True)
class TickContext:
    tick_id: int
    monotonic_ns: int

@dataclass(frozen=True)
class DataField:
    key: str
    value: object

@dataclass(frozen=True)
class Observation:
    kind: str
    source_device_id: str
    source_sequence: int
    captured_monotonic_ns: int
    values: tuple[DataField, ...]

@dataclass(frozen=True)
class AdmittedFrame:
    context: TickContext
    accepted: tuple[Observation, ...]
    rejected: tuple = ()
    degraded_sources: tuple[str, ...] = ()

@dataclass(frozen=True)
class WheelVelocitySetpoint:
    context: TickContext
    left_mps: float
    right_mps: float

@dataclass(frozen=True)
class ActuatorRequest:
    context: TickContext
    left_normalized: float
    right_normalized: float
    saturated: bool = False

for item in (TickContext, DataField, Observation, AdmittedFrame, WheelVelocitySetpoint, ActuatorRequest):
    setattr(contracts, item.__name__, item)
sys.modules["v3.contracts"] = contracts

live = types.ModuleType("v3.adapters.live_encoder")
class EncoderRejectionCode(str, Enum):
    NONE = "NONE"
    BASELINE = "BASELINE"
    NONINCREASING_TICK_TIME = "NONINCREASING_TICK_TIME"
    INVALID_EDGE_TIMING = "INVALID_EDGE_TIMING"
    COUNTER_NOT_RUNNING = "COUNTER_NOT_RUNNING"
    SAMPLE_INTERVAL_EXCEEDED = "SAMPLE_INTERVAL_EXCEEDED"
    COUNTER_READ_ERROR_CHANGED = "COUNTER_READ_ERROR_CHANGED"
    COUNTER_INVALID_ALERT_CHANGED = "COUNTER_INVALID_ALERT_CHANGED"
    COUNTER_READ_ERROR_AND_INVALID_ALERT_CHANGED = "COUNTER_READ_ERROR_AND_INVALID_ALERT_CHANGED"
    LEFT_VELOCITY_LIMIT_EXCEEDED = "LEFT_VELOCITY_LIMIT_EXCEEDED"
    RIGHT_VELOCITY_LIMIT_EXCEEDED = "RIGHT_VELOCITY_LIMIT_EXCEEDED"
    BOTH_VELOCITY_LIMIT_EXCEEDED = "BOTH_VELOCITY_LIMIT_EXCEEDED"
class EncoderEdgeDiagnostics:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)
@dataclass(frozen=True)
class EncoderVelocityReading:
    sequence: int
    captured_monotonic_ns: int
    left_mps: float
    right_mps: float
    trust: float
    stale: bool
    timing_valid: bool
    diagnostics: object | None = None
live.EncoderRejectionCode = EncoderRejectionCode
live.EncoderEdgeDiagnostics = EncoderEdgeDiagnostics
live.EncoderVelocityReading = EncoderVelocityReading
sys.modules["v3.adapters.live_encoder"] = live

def load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod

counter = load("v3.adapters.counter_encoder", ROOT / "v3/adapters/counter_encoder.py")
gpio_mod = load("v3.adapters.gpio_counter", ROOT / "v3/adapters/gpio_counter.py")
l11 = load("v3.layers.l11_actuator_control", ROOT / "v3/layers/l11_actuator_control.py")

class Counter:
    running = True
    def __init__(self, snapshots):
        self.snapshots = iter(snapshots)
    def snapshot(self):
        return next(self.snapshots)

def snap(count, edges=(), invalid=0):
    return counter.SignedPulseCounterSnapshot(count, invalid_alerts=invalid, edge_history=tuple(edges))

def edge(t, c):
    return counter.SignedPulseEdge(t, c)

CFG = counter.CounterEncoderBackendConfig(
    0.000644429262323014,
    0.000644429262323014,
    maximum_sample_interval_ns=100_000_000,
    maximum_abs_velocity_mps=1.5,
    minimum_estimation_pulses=4,
    minimum_estimation_window_ns=40_000_000,
    maximum_estimation_window_ns=160_000_000,
)

# 1) A very short reversal candidate is retained in diagnostics, but cannot be
# full-trust or become the velocity-limit fault that stopped the live PROBA.
old = (edge(970_000_000, 7), edge(980_000_000, 8), edge(990_000_000, 9), edge(1_000_000_000, 10))
rev = old + (edge(1_000_100_000, 9), edge(1_000_250_000, 8))
backend = counter.NativeCounterEncoderBackend(Counter((snap(10, old), snap(8, rev))), Counter((snap(10, old), snap(8, rev))), CFG)
backend.read(TickContext(0, 1_000_000_000))
r = backend.read(TickContext(1, 1_001_000_000))
assert r.diagnostics.rejection_code is EncoderRejectionCode.BASELINE
assert r.left_mps == r.right_mps == 0.0
assert 0.0 < r.diagnostics.left_measurement_trust < 1.0
assert abs(r.diagnostics.computed_left_mps) > CFG.maximum_abs_velocity_mps

# 2) One stationary wheel no longer makes the moving wheel stale.
left_edges = tuple(edge(1_150_000_000 + i * 10_000_000, i + 1) for i in range(5))
backend = counter.NativeCounterEncoderBackend(Counter((snap(0), snap(5, left_edges))), Counter((snap(0), snap(0))), CFG)
backend.read(TickContext(0, 1_000_000_000))
r = backend.read(TickContext(1, 1_200_000_000))
assert not r.stale and r.timing_valid
assert r.left_mps > 0.0 and r.right_mps == 0.0
assert r.diagnostics.right_estimation_timebase is None

class FakeCallback:
    def __init__(self, fn): self.function = fn
    def cancel(self): return 0
class FakeGpio:
    RISING_EDGE=1; BOTH_EDGES=3; SET_PULL_UP=32
    def __init__(self): self.levels={18:1,23:0}; self.callbacks={}
    def gpiochip_open(self, chip): return 7
    def gpio_claim_alert(self, *args): return 0
    def gpio_set_debounce_micros(self, *args): return 0
    def gpio_read(self, handle, pin): return self.levels.get(pin,0)
    def callback(self, handle,pin,edge,function): self.callbacks[pin]=FakeCallback(function); return self.callbacks[pin]
    def gpio_free(self,*args): return 0
    def gpiochip_close(self,*args): return 0
    def emit(self,pin,level,tick): self.callbacks[pin].function(2,pin,level,tick)

gcfg = gpio_mod.GpioCounterPairConfig(
    gpio_mod.GpioCounterChannelConfig(17,18,forward_b_level=1,a_debounce_micros=150),
    gpio_mod.GpioCounterChannelConfig(22,23,forward_b_level=0),
    gpio_chip=2,
)
gpio = FakeGpio(); owner = gpio_mod.NativeGpioSignedCounterPair(gpio,gcfg)
# B physically changes after A (900 us), but its callback is delivered first.
gpio.emit(18,0,900_000)
gpio.emit(17,1,1_000_000)  # debounced callback => physical A at 850 us
s = owner.left_counter.snapshot()
assert s.pulse_count == 1
assert s.edge_history[-1].timestamp_ns == 850_000
# A late B event belonging before the committed A is diagnostic, not sign state.
gpio.emit(18,0,800_000)
assert owner.left_counter.snapshot().invalid_alerts == 1
owner.close()

RAW_MAP = {
    "schema":"R2B4_WHEEL_SPEED_MAP_V2", "map_state":"ACTIVE",
    "curves": {
        name: {"maintenance_pwm":0.10,"startup_pwm":0.15,"points":[{"speed_mps":0.10,"pwm":0.16},{"speed_mps":0.30,"pwm":0.30}]}
        for name in ("left_forward","left_reverse","right_forward","right_reverse")
    }
}
speed_map = l11.WheelSpeedMap.from_mapping(RAW_MAP)
pi_cfg = l11.WheelPiConfig(0.25,0.08,0.18,0.95,max_feedback_uncertainty_ns=100_000_000)

def frame(ctx, *, lt, rt, lm=0.0, rm=0.0, ltb="GPIO_EDGE_HISTORY", rtb="GPIO_EDGE_HISTORY", degraded=False, stale=False, code="NONE"):
    vals=(
        DataField("left_mps",lm), DataField("right_mps",rm), DataField("trust",min(lt,rt)),
        DataField("left_measurement_trust",lt), DataField("right_measurement_trust",rt),
        DataField("left_estimation_timebase",ltb), DataField("right_estimation_timebase",rtb),
        DataField("measurement_stale",stale), DataField("measurement_timing_valid",True),
        DataField("rejection_code",code), DataField("left_counter_running",True), DataField("right_counter_running",True),
        DataField("left_read_error_delta",0), DataField("right_read_error_delta",0),
        DataField("left_invalid_alert_delta",0), DataField("right_invalid_alert_delta",0),
    )
    obs=Observation("wheel_velocity","enc",ctx.tick_id,ctx.monotonic_ns,vals)
    return AdmittedFrame(ctx,(obs,),(),("enc",) if degraded else ())

# 3) BASELINE/low-trust zero is FF-only, never PI zero feedback, and the bound
# is monotonic time (100 ms here), not a magic tick count.
ctl=l11.WheelActuatorController(speed_map,pi_cfg)
ff=speed_map.lookup("left",0.15)[0]
for i in range(5):
    ctx=TickContext(i,1_000_000_000+i*20_000_000)
    out=ctl(WheelVelocitySetpoint(ctx,0.15,0.15),frame(ctx,lt=0.0,rt=0.0,code="BASELINE"))
    assert out.left_normalized == ff and out.right_normalized == ff
ctx=TickContext(5,1_100_000_000)
try:
    ctl(WheelVelocitySetpoint(ctx,0.15,0.15),frame(ctx,lt=0.0,rt=0.0,code="BASELINE"))
except ValueError as exc:
    assert "uncertain too long" in str(exc)
else:
    raise AssertionError("L11 did not fail closed after the uncertainty budget")

# 4) A deliberately stationary right wheel is ignored by the right feedback
# watchdog; the left wheel can remain in closed-loop PI beyond 100 ms.
ctl=l11.WheelActuatorController(speed_map,pi_cfg)
for i in range(10):
    ctx=TickContext(i,2_000_000_000+i*20_000_000)
    out=ctl(
        WheelVelocitySetpoint(ctx,0.15,0.0),
        frame(ctx,lt=1.0,rt=0.0,lm=0.14,rm=0.0,rtb="TICK_SNAPSHOT",code="BASELINE"),
    )
    assert out.left_normalized > 0.0 and out.right_normalized == 0.0

print("PASS: encoder robustness smoke validation")
