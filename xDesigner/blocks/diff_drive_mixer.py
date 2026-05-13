"""DiffDriveMixer: translate (throttle, steering) into per-wheel commands.

A skid-steer rover has no steering linkage; turning is produced by
differential wheel speeds. This block is the arcade-drive ``balancer``
the Pico would run::

    cmd_left  = throttle - mixing_gain * steering
    cmd_right = throttle + mixing_gain * steering

Both outputs are clamped to [-1, +1] after mixing — saturating the
*command*, not the inputs, which means a hard turn at full throttle
clips one side and gives up some forward speed in favour of yaw (the
right intuitive behaviour for a rover).
"""
from ..block import Block


class DiffDriveMixer(Block):
    """Arcade-drive mixer.

    Parameters
    ----------
    mixing_gain : how much steering authority the differential gets,
                  relative to throttle. 1.0 = equal weighting; smaller
                  values reduce yaw aggressiveness.

    Ports
    -----
    in:  throttle (-1..+1),  steering (-1..+1)
    out: command_left, command_right  (each clamped to [-1, +1])
    """

    def __init__(self, name: str, mixing_gain: float = 0.7):
        super().__init__(name)
        if mixing_gain <= 0:
            raise ValueError("mixing_gain must be > 0")
        self.mixing_gain = mixing_gain

        self.add_input("throttle")
        self.add_input("steering")
        self.add_output("command_left")
        self.add_output("command_right")

    def step(self, t: float, dt: float) -> None:
        thr = self.inputs["throttle"].value
        stg = self.inputs["steering"].value
        # clamp inputs first so a runaway upstream command can't push the
        # mixer into nonsense magnitudes
        if thr >  1.0: thr =  1.0
        elif thr < -1.0: thr = -1.0
        if stg >  1.0: stg =  1.0
        elif stg < -1.0: stg = -1.0

        cmd_L = thr - self.mixing_gain * stg
        cmd_R = thr + self.mixing_gain * stg
        if cmd_L >  1.0: cmd_L =  1.0
        elif cmd_L < -1.0: cmd_L = -1.0
        if cmd_R >  1.0: cmd_R =  1.0
        elif cmd_R < -1.0: cmd_R = -1.0

        self.outputs["command_left"].value = cmd_L
        self.outputs["command_right"].value = cmd_R
