# micro:bit Maze Robot

A small autonomous robot car, built on a BBC micro:bit, that solves a 5×5 grid
maze and drives itself through it. It receives the map over radio, computes the
shortest route with a wavefront (breadth-first) search, converts that route into
physical motion, and watches for unmapped obstacles with an ultrasonic sensor as
it goes.

This was a group project — I worked on it as part of a small team, and everything
described here is the team's shared work.

## What it does

The robot runs in two stages.

**Stage 1 — navigate a known map.** On start-up the robot broadcasts a request
over radio and receives a 25-character string describing the course. That string
becomes a 5×5 grid of open cells, walls, a start cell (`S`) and a goal cell
(`X`). A wavefront search floods outward from the start, labelling each reachable
cell with its distance, then walks backward from the goal to recover the shortest
path. The path is animated on the micro:bit LED display before the robot drives
it, one cell at a time.

**Stage 2 — react to an obstacle that isn't on the map.** After every move the
robot scans the cell ahead with the ultrasonic sensor. If something is close
enough, that cell is marked as a wall, the grid is re-encoded, and the route is
recomputed from the robot's current position. Driving resumes on the new path.

A map like `S01000010001110000101000X` decodes to:

```
S 0 1 0 0        S  start cell
0 0 1 0 0        X  goal cell
0 1 1 1 0        1  wall
0 0 0 1 0        0  open
1 0 0 0 X
```

## Hardware

- BBC micro:bit (running MicroPython)
- Robotbit expansion board — motor driver and power distribution
- Two DC gear motors driving the rear wheels
- HC-SR04-style ultrasonic sensor, fixed forward-facing on the front of the chassis
- Rechargeable battery pack

The sensor is mounted rigidly rather than on a servo. Because the robot always
turns to face a cell before entering it, a fixed forward-facing sensor sees
everything the navigation logic actually needs, which removed a moving part and
a whole class of calibration work.

## Repository layout

```
src/main.py                 Flight code — the full stage 1 + stage 2 program
src/robotbit_library.py     PCA9685 motor/servo driver for the Robotbit board
tools/ultrasonic_test.py    Bench test that streams live distance readings
docs/design-notes.md        Design decisions, calibration, and results
```

`src/main.py` is the program that ran in the demonstration. Only the files above
were part of the working system; earlier prototypes and unrelated class exercises
are deliberately not included.

## Running it

1. Copy `src/robotbit_library.py` onto the micro:bit filesystem (the Mu editor's
   *Files* pane, or `ufs put`). `main.py` imports it and will not run without it.
2. Flash `src/main.py` to the micro:bit.
3. Set `RADIO_CHANNEL` in `src/main.py` to match the channel the map transmitter
   is broadcasting on.
4. Place the robot on the start cell, **facing "up"** relative to the grid — the
   heading tracker assumes this starting orientation.
5. Power on. The display shows `R` while waiting for a map, sweeps a pixel across
   the middle row as a progress indicator, then animates the solved path before
   driving. `NOREPLY` means no map arrived; `NOPATH` means the goal is
   unreachable.

## Calibration

Two timing constants in `src/main.py` control all motion, and both are specific
to a given chassis and battery state:

| Constant | Value | Meaning |
|---|---|---|
| `CELL_TIME_MS` | 856 | ms at 50% power to travel one cell (10 in) |
| `TURN_TIME_MS` | 368 | ms at 50% power for a 90° rotation |

These were measured experimentally with the robot on a charging cable, so the
motors ran at full torque. Expect to re-measure them for any other build — and
note that a partly charged battery changes them, which caused real problems for
us. See [docs/design-notes.md](docs/design-notes.md).

## Known limitations

- **The obstacle threshold is too aggressive.** `BLOCK_MM = 50` was the value
  used in the demonstration, and it fires too late: the robot has often already
  begun entering the blocked cell before the reading trips. Somewhere in the
  70–100 mm range is a better starting point, re-calibrated against the drive
  timings.
- **Replanning happens between moves, not during one.** A move command runs to
  completion before the next scan, so there is no way to abort a step already in
  progress. Checking the cell ahead *before* committing to a move would be the
  more robust design.
- **Dead reckoning only.** Position is tracked by counting commanded moves, with
  no encoders or closed-loop feedback, so heading error accumulates over a run.
