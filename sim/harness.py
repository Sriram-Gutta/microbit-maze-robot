"""Import the robot firmware on a desktop Python interpreter.

`src/main.py` targets MicroPython, so importing it under CPython needs two
things arranged first:

  1. The `sim/stubs` directory ahead of everything on `sys.path`, so
     `microbit`, `machine`, `radio` and `robotbit_library` resolve to the
     desktop stand-ins.
  2. A `time` module that has MicroPython's `sleep_us`. CPython's does not,
     so a shim carrying the real module's contents plus the missing functions
     is registered under that name.

Both the simulator and the test suite go through `load_firmware()`, so the
code under test is always the real firmware rather than a copy of it.
"""

import os
import sys
import time as _real_time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
STUBS = os.path.join(HERE, "stubs")
SRC = os.path.join(ROOT, "src")


def _install_time_shim():
    """Give `time` the MicroPython timing helpers main.py imports."""
    if hasattr(_real_time, "sleep_us"):
        return
    shim = types.ModuleType("time")
    shim.__dict__.update(_real_time.__dict__)
    shim.sleep_us = lambda us: None
    shim.sleep_ms = lambda ms: None
    shim.ticks_ms = lambda: int(_real_time.time() * 1000)
    shim.ticks_us = lambda: int(_real_time.time() * 1000000)
    shim.ticks_diff = lambda a, b: a - b
    sys.modules["time"] = shim


def _install_paths():
    for path in (SRC, STUBS):
        if path in sys.path:
            sys.path.remove(path)
    sys.path.insert(0, SRC)
    sys.path.insert(0, STUBS)    # stubs win over anything else


def load_firmware():
    """Return the imported `src/main.py` module, ready to drive."""
    _install_time_shim()
    _install_paths()
    import main
    return main
