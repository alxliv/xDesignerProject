"""Base class for xDesigner blocks.

Every block has named input and output ports plus an internal state.
At each simulation step the block reads from its inputs and writes
to its outputs in `step(t, dt)`.
"""
from __future__ import annotations
from typing import Dict


class Port:
    """A named scalar signal port belonging to a block."""

    __slots__ = ("name", "block", "kind", "value")

    def __init__(self, name: str, block: "Block", kind: str):
        assert kind in ("in", "out")
        self.name = name
        self.block = block
        self.kind = kind
        self.value = 0.0

    @property
    def full_name(self) -> str:
        return f"{self.block.name}.{self.name}"

    def __repr__(self) -> str:
        return f"<Port {self.full_name} ({self.kind}) = {self.value:g}>"


class Block:
    """Base block class.

    Subclasses should declare ports in `__init__` via `add_input` /
    `add_output`, optionally override `reset()` to clear state, and
    must implement `step(t, dt)`.
    """

    def __init__(self, name: str):
        self.name = name
        self.inputs: Dict[str, Port] = {}
        self.outputs: Dict[str, Port] = {}

    # ---- port construction ------------------------------------------------
    def add_input(self, name: str, default: float = 0.0) -> Port:
        p = Port(name, self, "in")
        p.value = default
        self.inputs[name] = p
        return p

    def add_output(self, name: str, default: float = 0.0) -> Port:
        p = Port(name, self, "out")
        p.value = default
        self.outputs[name] = p
        return p

    # ---- convenience ------------------------------------------------------
    def __getitem__(self, port_name: str) -> Port:
        if port_name in self.inputs:
            return self.inputs[port_name]
        if port_name in self.outputs:
            return self.outputs[port_name]
        raise KeyError(f"{self.name!r} has no port {port_name!r}")

    def get(self, port_name: str) -> float:
        return self[port_name].value

    # ---- lifecycle --------------------------------------------------------
    def reset(self) -> None:
        """Clear internal state. Called at the start of each `run`."""
        pass

    def step(self, t: float, dt: float) -> None:
        raise NotImplementedError

    def __repr__(self) -> str:
        return f"<{type(self).__name__} {self.name!r}>"
