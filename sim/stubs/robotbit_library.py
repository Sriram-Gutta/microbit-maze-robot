"""Desktop stand-in for the Robotbit motor driver.

Records motor commands instead of driving hardware. The simulator derives the
robot's pose from the movement primitives in main.py rather than from raw PWM,
so this only needs to accept the calls and keep a log.
"""

M1A = 0x1
M1B = 0x2
M2A = 0x3
M2B = 0x4

S1, S2, S3, S4 = 0x1, 0x2, 0x3, 0x4

commands = []


def setup():
    commands.append(("setup",))


def motor(index, pct_speed):
    commands.append(("motor", index, pct_speed))


def motor_stop(index):
    commands.append(("motor_stop", index))


def all_stop():
    commands.append(("all_stop",))


def servo(index, degree):
    commands.append(("servo", index, degree))


def servoc(index, pct_speed):
    commands.append(("servoc", index, pct_speed))


def servoc_stop(index):
    commands.append(("servoc_stop", index))
