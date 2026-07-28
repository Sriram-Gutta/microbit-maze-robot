"""Desktop stand-in for the micro:bit `machine` module.

`time_pulse_us` is the ultrasonic echo timer. The simulator replaces the
higher-level distance function directly, so this only needs to return a
timeout so nothing blocks if it is ever reached.
"""


def time_pulse_us(pin, level, timeout_us):
    return -1    # timeout / no echo
