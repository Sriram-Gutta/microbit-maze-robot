"""Bench test for the ultrasonic sensor.

Flash this on its own to stream live distance readings to the serial REPL
while holding an obstacle at known distances. Used to pick the BLOCK_MM
threshold in src/main.py. Reads two pins so a second sensor can be checked
without editing the file.
"""

from microbit import *
from time import sleep_us
from machine import time_pulse_us
import robotbit_library as r

r.setup()

TIME_OUT = 100000    # echo timeout in us; larger = longer range, slower sampling

TRIGGER = pin15
ECHO = pin15
TRIGGER2 = pin14
ECHO2 = pin14


def distance(tp, ep):
    """Fire the trigger pin and time the echo. Returns mm, or <= 0 on timeout."""
    ep.read_digital()        # clear echo
    tp.write_digital(1)      # 10 us trigger pulse
    sleep_us(10)
    tp.write_digital(0)
    ep.read_digital()        # clear echo again (needed for a ping sensor)
    ts = time_pulse_us(ep, 1, TIME_OUT)
    if ts > 0:
        ts = ts * 17 // 100  # us -> mm
    return ts


def main():
    display.show("S")
    while True:
        print(distance(TRIGGER, ECHO), 'mm on pin15')
        print(distance(TRIGGER2, ECHO2), 'mm on pin14')
        sleep(200)


if __name__ == "__main__":
    main()
