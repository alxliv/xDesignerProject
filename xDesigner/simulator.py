"""Time-stepped simulator for xDesigner.

Usage:
    sim = Simulator(dt=5e-4)
    a = sim.add(SomeBlock("a"))
    b = sim.add(OtherBlock("b"))
    sim.connect(a["out_port"], b["in_port"])
    time, log = sim.run(duration=2.0, probes=["a.out_port", "b.state"])

Execution order is the order in which blocks are `add`ed. For feedback
loops, signals connecting a *later* block back to an *earlier* block
naturally incur a one-sample delay, which is the same thing real digital
control loops do, so it's fine.
"""
from __future__ import annotations
from collections import defaultdict
from typing import Dict, List, Tuple, Iterable, Optional

from .block import Block, Port


class Simulator:
    def __init__(self, dt: float = 5e-4):
        if dt <= 0:
            raise ValueError("dt must be > 0")
        self.dt = dt
        self.blocks: List[Block] = []
        self.connections: List[Tuple[Port, Port]] = []
        # cache: outgoing connections per block, for fast propagation
        self._outgoing: Dict[int, List[Tuple[Port, Port]]] = defaultdict(list)
        self.time_log: List[float] = []
        self.log: Dict[str, List[float]] = {}

    # ---- construction -----------------------------------------------------
    def add(self, block: Block) -> Block:
        self.blocks.append(block)
        return block

    def connect(self, src: Port, dst: Port) -> None:
        if src.kind != "out":
            raise ValueError(f"source must be an output port, got {src!r}")
        if dst.kind != "in":
            raise ValueError(f"destination must be an input port, got {dst!r}")
        self.connections.append((src, dst))
        self._outgoing[id(src.block)].append((src, dst))

    # ---- internal helpers -------------------------------------------------
    def _propagate_all(self) -> None:
        for src, dst in self.connections:
            dst.value = src.value

    def _propagate_from(self, block: Block) -> None:
        for src, dst in self._outgoing.get(id(block), ()):
            dst.value = src.value

    def _lookup(self, dotted: str) -> float:
        if "." not in dotted:
            raise KeyError(f"probe must be 'block.port', got {dotted!r}")
        block_name, port_name = dotted.split(".", 1)
        for b in self.blocks:
            if b.name == block_name:
                return b[port_name].value
        raise KeyError(f"no block named {block_name!r}")

    # ---- main loop --------------------------------------------------------
    def reset(self) -> None:
        for b in self.blocks:
            b.reset()
        self.time_log.clear()
        self.log.clear()

    def run(
        self,
        duration: float,
        probes: Optional[Iterable[str]] = None,
    ) -> Tuple[List[float], Dict[str, List[float]]]:
        """Run for `duration` seconds, logging the probe signals.

        `probes` is an iterable of "block.port" strings.
        Returns (time_array, {probe_name: values}).
        """
        probes = list(probes or [])
        for p in probes:
            self.log[p] = []

        self.reset()
        for p in probes:
            self.log[p] = []

        self._propagate_all()

        n_steps = int(round(duration / self.dt))
        t = 0.0
        for _ in range(n_steps):
            for b in self.blocks:
                b.step(t, self.dt)
                self._propagate_from(b)

            # log
            self.time_log.append(t)
            for p in probes:
                self.log[p].append(self._lookup(p))

            t += self.dt

        return self.time_log, dict(self.log)
