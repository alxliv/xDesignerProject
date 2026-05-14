"""Smoke tests for the core Simulator loop."""
import pytest

from xDesigner import Block, Simulator
from xDesigner.blocks import SignalSource


class Gain(Block):
    """y = k * u — minimal one-port-in, one-port-out block for tests."""

    def __init__(self, name, k=1.0):
        super().__init__(name)
        self.k = k
        self.add_input("u")
        self.add_output("y")

    def step(self, t, dt):
        self.outputs["y"].value = self.k * self.inputs["u"].value


class Counter(Block):
    """Counts steps it has been stepped; resettable."""

    def __init__(self, name):
        super().__init__(name)
        self.add_output("count")
        self.n = 0

    def reset(self):
        self.n = 0

    def step(self, t, dt):
        self.n += 1
        self.outputs["count"].value = self.n


class Sum(Block):
    """y = a + b — two-input summing junction."""

    def __init__(self, name):
        super().__init__(name)
        self.add_input("a")
        self.add_input("b")
        self.add_output("y")

    def step(self, t, dt):
        self.outputs["y"].value = self.inputs["a"].value + self.inputs["b"].value


def test_dt_must_be_positive():
    with pytest.raises(ValueError):
        Simulator(dt=0.0)
    with pytest.raises(ValueError):
        Simulator(dt=-1e-3)


def test_connect_rejects_wrong_port_kinds():
    sim = Simulator(dt=1e-3)
    a = sim.add(Gain("a", k=1.0))
    b = sim.add(Gain("b", k=1.0))
    # output → input: OK
    sim.connect(a["y"], b["u"])
    # input → input: must fail
    with pytest.raises(ValueError):
        sim.connect(a["u"], b["u"])
    # output → output: must fail
    with pytest.raises(ValueError):
        sim.connect(a["y"], b["y"])


def test_forward_chain_propagates_in_one_step():
    sim = Simulator(dt=1e-3)
    src = sim.add(SignalSource("src", waveform="const", amplitude=2.5))
    g1 = sim.add(Gain("g1", k=3.0))
    g2 = sim.add(Gain("g2", k=-1.0))
    sim.connect(src["value"], g1["u"])
    sim.connect(g1["y"], g2["u"])
    t, log = sim.run(duration=3e-3, probes=["g2.y"])
    # 2.5 * 3 * -1 = -7.5 every step
    assert all(v == pytest.approx(-7.5) for v in log["g2.y"])


def test_feedback_loop_carries_one_sample_delay():
    """An output of a later block feeding an earlier block lags by one
    sample — this is how real digital control loops behave."""
    sim = Simulator(dt=1e-3)
    src = sim.add(SignalSource("src", waveform="const", amplitude=1.0))
    summer = sim.add(Sum("sum"))
    fb = sim.add(Gain("fb", k=1.0))  # passthrough used as feedback path
    sim.connect(src["value"], summer["a"])
    sim.connect(fb["y"], summer["b"])      # feedback: later block → earlier
    sim.connect(summer["y"], fb["u"])
    t, log = sim.run(duration=4e-3, probes=["sum.y"])
    # Trace (each step): sum.a = 1 (current src), sum.b = fb.y from prev step.
    #   step 1: sum.b = 0     → sum.y = 1; fb.y = 1
    #   step 2: sum.b = 1     → sum.y = 2; fb.y = 2
    #   step 3: sum.b = 2     → sum.y = 3; fb.y = 3
    #   step 4: sum.b = 3     → sum.y = 4; fb.y = 4
    assert log["sum.y"] == [pytest.approx(v) for v in (1.0, 2.0, 3.0, 4.0)]


def test_reset_clears_state_between_runs():
    sim = Simulator(dt=1e-3)
    c = sim.add(Counter("c"))
    sim.run(duration=5e-3, probes=["c.count"])
    assert c.n == 5
    sim.run(duration=3e-3, probes=["c.count"])
    # Counter must have been reset before the second run.
    assert c.n == 3


def test_unknown_probe_name_raises():
    sim = Simulator(dt=1e-3)
    sim.add(Gain("a", k=1.0))
    with pytest.raises(KeyError):
        sim.run(duration=1e-3, probes=["nonexistent.port"])


def test_probe_returns_logs_aligned_to_time():
    sim = Simulator(dt=1e-3)
    sim.add(SignalSource("s", waveform="ramp", slope=10.0, t_start=0.0))
    t, log = sim.run(duration=5e-3, probes=["s.value"])
    assert len(t) == len(log["s.value"]) == 5
    # ramp value at the END of step i is slope * t_i  (where t_i is the time at
    # the start of the step, since SignalSource.step uses the t argument).
    assert log["s.value"][0] == pytest.approx(0.0)
    assert log["s.value"][4] == pytest.approx(10.0 * 4e-3)
