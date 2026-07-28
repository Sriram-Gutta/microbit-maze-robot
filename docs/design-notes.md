# Design Notes

Written up from the team's project report. Covers why the system is built the
way it is, how it was calibrated, how it actually performed, and what we would
do differently.

## Goal

Build a micro:bit-controlled vehicle that autonomously navigates a 5×5 obstacle
course in two stages:

- **Stage 1** — receive a map of the course, compute a route, and drive it.
- **Stage 2** — do the same, but detect an obstacle that is *not* on the map and
  find a new route around it.

## Hardware considerations

The chassis was assembled to the provided reference instructions, which fixed the
motor, board, and wheel layout. The one part left to the team was the ultrasonic
sensor: teams were free to choose where it went and whether it moved.

**Fixed sensor over a servo-mounted one.** We initially planned to mount the
ultrasonic sensor on a servo so it could sweep for peripheral vision, and dropped
that idea partway through. The reasoning was that the robot only ever needs to
know about the cell it is about to enter — and since the navigation logic already
turns the robot to face that cell before moving, a rigid forward-facing sensor
sees exactly the cell in question. The servo would have added a moving part, more
calibration, and more code for information the algorithm never used.

**Power was the dominant hardware problem.** Motor torque was inconsistent
throughout the project. We first assumed a wiring fault between the battery and
the motors, but it turned out to be insufficient charge reaching the motors. The
battery was charged repeatedly and still frequently failed to deliver enough
power; a replacement battery had the same issue. Full, reliable torque only
happened with the robot connected by USB cable. Because every movement constant
is a *duration* at a given power level, inconsistent torque directly corrupts
calibration — this is the root cause of most of the problems further down.

A way to read the actual charge level of the battery would have saved a large
amount of time. Without it there was no way to tell whether a bad run meant bad
code or a flat battery.

## Software design

### Receiving the map

The run begins with a radio transmission carrying a 25-character map string. The
transmission code came from provided course material and needed almost no
modification — only the radio channel was changed to match the assigned channel.

### Pathfinding

The wavefront implementation started from a provided reference. It takes the 5×5
grid of walls, open cells, start and goal, floods distances outward from the
start, and returns the shortest path as a list of `(x, y)` cells.

That gets you a path, not motion. Two things still had to be built on top:

**`direction_from_cells(a, b)`** — compares the current and next cell
coordinates and returns `UP`, `DOWN`, `LEFT`, or `RIGHT` based on which axis
changed by ±1. This is the *target* heading.

**`turn_to(target)`** — compares the target heading against the heading the
robot is currently facing, using `right_of()` and `left_of()` helpers. If the
target is one turn to the right, turn right; one to the left, turn left; if it is
neither and not already correct, it must be a 180°, so turn right twice. The
function updates the tracked `facing` variable so the next step starts from the
correct heading.

The tracked heading is what makes stage 2 possible. Because the robot always
knows both where it is and which way it points, it can stop mid-route and hand a
fresh start position to the pathfinder.

### Replanning around a new obstacle

This was the hardest part of the software. The wavefront function needs a 5×5
grid, so detecting an obstacle means getting the robot's *current* state back
into that grid form. Three functions do it:

1. **`scan_and_update_map()`** — runs after every move. Reads the distance to the
   cell ahead; if something is within the threshold, marks that cell as a wall.
   Border cells are skipped, since a reading there is just the wall around the
   course. Returns `True` when the map changed, which breaks the drive loop.
2. **`update_sector_from_grid()`** — re-encodes the modified grid back into a
   25-character string, writing the robot's current cell as the new `S` and the
   detected obstacle as `1`.
3. **`load_sector_map()`** — parses that string back into a 5×5 array.

Round-tripping through the string is the key choice here. It is not the most
direct way to mutate a grid, but it means the replanning path and the
radio-received path converge on *exactly the same* parsing code — so if the
wavefront search works on a received map, it works on a regenerated one too. One
code path, tested twice.

### Obstacle detection

The sensor code has two parts: measuring and triggering. `scan_front_mm()` sends
a 10 µs trigger pulse and times the echo, scaling microseconds to millimetres.
`scan_and_update_map()` compares that against a 50 mm threshold.

50 mm was chosen to be conservative — far enough not to trigger on the course
walls, close enough to catch a box in the adjacent cell. In practice it was the
wrong call; see below.

### Movement calibration

Calibration was done by experiment rather than from the reference table, because
the battery problems made any borrowed numbers meaningless for this particular
robot. Measurements were taken with the robot on a charging cable so the wheels
ran at proper torque:

- **856 ms** at 50% power on both wheels to travel 10 in (one cell)
- **368 ms** at 50% power to complete a 90° turn

The right motor runs at 47% rather than 50% during forward motion to correct a
consistent drift.

Taking these measurements on USB power is itself a source of error: the numbers
are correct for full torque and progressively wrong as the battery drains. That
is very likely why calibration looked good in testing and drifted during the
stage 2 run.

### Display

The LED display shows the grid — start, goal, walls, open cells and the solved
path at different brightness levels — using the display routines from the
provided wavefront material. We added a happy face on completion and a sad face
on failure.

## Results

### Stage 1

Roughly 30 minutes before the demonstration, calibration was close to ideal:
turns within about ±2° of 90°, consistent across 8 consecutive turns, and forward
motion reliably carrying the robot from one cell centre to the next.

Two things went wrong on the day.

**An orientation error on the first run.** Testing had been done from a different
reference frame than the actual course, so the programmed left and right were
effectively mirrored. This was corrected on the spot.

**Accumulating drift.** Once the mirroring was fixed, the underlying calibration
problem resurfaced. The robot generally moved toward the correct cells, but each
successive move was less precise than the last, and the errors compounded — the
robot ended up at angles not parallel to any grid line. This traces back to the
motor and power inconsistency.

Stage 1 went well overall. The maze was solved computationally and a correct path
was generated; with reliable calibration the robot would have completed the route
without leaving the map or hitting a fixed obstacle. The software logic was more
successful than the hardware execution.

### Stage 2

Stage 2 was substantially less successful.

The detection itself worked — the obstacle was picked up and displayed correctly
on the updated map. What failed was acting on it. After detection, the robot did
not meaningfully re-run pathfinding from its current position; it continued along
the original route as though the obstacle were not there, drove into the blocked
path, and only reached the destination because of imperfect movement rather than
because replanning worked.

The likely explanation is timing. Movement commands run to completion, so by the
time an obstacle was recognised the robot was already committed to the next move
— leaving no effective interruption point at which to stop, recompute and
restart. Poor calibration made this worse: the robot was frequently not squarely
facing the obstacle, and was sometimes already inside the obstacle's cell when
the reading came in.

The 50 mm threshold compounded it. A conservative threshold reduces false
positives, but it also shrinks the margin available for true positives — and with
inconsistent motion, that margin was what the system needed most.

## Conclusions

The limiting factor was not pathfinding logic; it was system integration. The
software could reliably produce a valid route. The robot could not execute that
route consistently enough for the route to matter.

Obstacle detection turned out to be far less precise in practice than on paper.
Detecting an object in front of the robot did not cleanly correspond to the grid
cell the robot *believed* it was in, particularly when it was not squarely
aligned to the grid. And because the obstacle was small relative to a cell, the
robot could partially enter a blocked region before detection reflected its true
position. Movement, sensing and replanning could not be treated as independent
subsystems — each one depended on the others being reliable at the same moment.

## Recommendations

- **Raise the detection threshold.** 50 mm is too late. Start around 70–100 mm
  and re-calibrate against the drive timings.
- **Check before committing to a move,** not after completing one. Scanning the
  next cell before starting the forward command gives the replanner an actual
  interruption point.
- **Prioritise reliability over ambition.** Reusing provided code, minimising
  unnecessary modifications, and spending the saved time on repeated full-system
  testing would have produced a better result than added features. Creative
  additions are only worth it once the core objective is already dependable.
- **Test the whole system repeatedly, even when nothing changed.** Small
  interactions between hardware, movement and sensing produced new failures
  without any code change at all. Iterative full-system testing was the single
  most valuable activity in the project.

## Follow-up: reproducing and fixing the stage 2 failure

After the project, the failure was reproduced away from the hardware. `sim/`
puts stub `microbit`, `machine`, `radio` and `robotbit_library` modules on the
import path and drives the real `src/main.py` against a ground-truth world,
replacing only the four functions that touch the physical world: `front()`,
`left()`, `right()` and `scan_front_mm()`. Everything else — search, path
reconstruction, heading tracking, obstacle marking, re-planning — is the code
that ran on the robot.

Given a course containing an obstacle absent from the transmitted map, the
simulator reproduced the demonstration failure exactly, and made the mechanism
visible in a way the live run could not:

1. The robot turns toward the next cell and drives into it.
2. *Only then* does it scan — in its new heading, at the cell beyond.
3. So the obstacle in the cell it just entered is never checked before entry.
4. Once its believed position has diverged from its true position, the reading
   is attributed to the wrong cell. It marks a wall it never saw, and the real
   obstacle stays unmapped.

Step 4 explains the most confusing part of the live run: the obstacle *did*
appear on the LED map, yet the robot behaved as though it were somewhere else.
It was marking the wrong cell.

Moving the scan to after the turn but before the drive gives the planner a real
interruption point — `start` is still the current cell and `facing` already
equals the target heading, so the scan lands on exactly the cell about to be
entered. Measured over the same course:

| | scan after move (as demonstrated) | scan before move |
|---|---|---|
| reached goal | no | yes |
| collisions | 3 | 0 |
| moves | 8 | 10 |
| sensor reads | 3 | 5 |

The two extra moves are the detour around the obstacle. Both behaviours are
kept in the firmware, selected by `SCAN_BEFORE_MOVE`, and both are covered by
tests — including a regression test asserting that the old behaviour still
collides, so the original defect stays reproducible.

This also reframes the threshold question. 50 mm was blamed for the failure at
the time, and it is genuinely too tight, but it was not the root cause: with
the scan in the wrong place, no threshold value works, because there is no
point at which the robot can act on the reading. Ordering was the defect;
threshold was a contributing factor.

## Remaining limitations

- **The obstacle threshold is still tight.** `BLOCK_MM = 50` is the demonstrated
  value. 70–100 mm is a better starting point on real hardware, re-calibrated
  against the drive timings, since a real sensor has noise and a beam width that
  the simulator does not model.
- **Dead reckoning only.** Position is tracked by counting commanded moves, with
  no encoders or closed-loop feedback, so heading error accumulates over a run.
  This was the dominant physical failure mode and the simulator deliberately
  does not model it — it assumes perfect motion, which is exactly what isolates
  logic bugs from mechanical ones.
- **Border cells are never sensed.** `interior_cell()` skips readings at the
  edge of the grid, since a reading there is just the wall around the course.
  An obstacle placed on a border cell would not be detected.
- **The sensor cannot see sideways.** With a fixed forward mount, the robot only
  learns about a cell by turning to face it, so obstacles are discovered one at
  a time along the route rather than surveyed in advance.
