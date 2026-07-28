"""Desktop stand-in for the micro:bit `microbit` module.

Implements just enough of the API for src/main.py to be imported and run on a
normal Python interpreter. The LED display is captured into a buffer so the
simulator can render what the real device would be showing.
"""

import time as _time

_START = _time.time()


class _Display:
    def __init__(self):
        self.pixels = [[0] * 5 for _ in range(5)]
        self.text = ""

    def set_pixel(self, x, y, value):
        self.pixels[y][x] = value

    def clear(self):
        self.pixels = [[0] * 5 for _ in range(5)]

    def show(self, value):
        self.text = str(value)

    def scroll(self, value):
        self.text = str(value)

    def render(self):
        """Return the LED matrix as text, brightest pixels as the boldest glyph."""
        ramp = " .:-=+*#%@"
        return "\n".join(
            " ".join(ramp[min(v, 9)] for v in row) for row in self.pixels
        )


display = _Display()


class Image:
    HAPPY = "HAPPY"
    SAD = "SAD"
    NO = "NO"


class _Pin:
    def __init__(self, name):
        self.name = name
        self._value = 0

    def read_digital(self):
        return self._value

    def write_digital(self, value):
        self._value = value

    def read_analog(self):
        return 0


pin0 = _Pin("pin0")
pin1 = _Pin("pin1")
pin2 = _Pin("pin2")
pin14 = _Pin("pin14")
pin15 = _Pin("pin15")
pin16 = _Pin("pin16")


class _I2C:
    """No-op I2C. scan() returns the Robotbit address so setup() succeeds."""

    def init(self, *args, **kwargs):
        pass

    def scan(self):
        return [0x40]

    def write(self, addr, buf):
        pass

    def read(self, addr, n):
        return bytes(n)


i2c = _I2C()

# Simulated time runs instantly; real sleeps would make a run take minutes.
SPEED = 0.0


def sleep(ms):
    if SPEED:
        _time.sleep((ms / 1000.0) * SPEED)


def running_time():
    return int((_time.time() - _START) * 1000)


def panic(code=0):
    raise SystemExit("panic({})".format(code))


def reset():
    raise SystemExit("reset()")
