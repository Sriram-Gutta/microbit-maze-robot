#!/usr/bin/env python3
"""Run the robot's firmware on a desktop, with no micro:bit attached.

This does not re-implement the robot. It puts stub `microbit`, `machine`,
`radio` and `robotbit_library` modules on the import path, imports the real
`src/main.py`, and swaps out only the functions that touch the physical world:

    front(), left(), right()   -> update a simulated pose instead of PWM
    scan_front_mm()            -> measure against a ground-truth world map

Everything else -- the wavefront search, path reconstruction, heading
tracking, obstacle marking and re-planning -- is the exact code that runs on
the robot.

The simulated world contains an obstacle that is *not* in the transmitted map,
so the firmware has to discover it with the sensor and route around it. That
makes the stage 2 re-planning path reproducible on a laptop, which is how the
demonstration failure was diagnosed.

Usage:
    python sim/simulate.py                 # current firmware
    python sim/simulate.py --legacy        # scan-after-move, as demonstrated
    python sim/simulate.py --compare       # both, side by side
    python sim/simulate.py --map S0000... --obstacle 3,2 --delay 0.3
"""

import argparse
import contextlib
import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from harness import load_firmware                # noqa: E402

main = load_firmware()

import microbit                                  # noqa: E402  (the stub)
import radio                                     # noqa: E402  (the stub)

# A 5x5 course with two gaps through the middle wall. Both routes to the goal
# are 8 moves, so blocking whichever one the robot picks forces a genuine
# re-plan through the other rather than a trivial sidestep.
DEFAULT_MAP = "S00000000010101000000000X"

ARROW = {"UP": "^", "RIGHT": ">", "DOWN": "v", "LEFT": "<"}
STEP = {"UP": (0, -1), "RIGHT": (1, 0), "DOWN": (0, 1), "LEFT": (-1, 0)}

CLEAR_MM = 300      # reading when nothing is ahead
BLOCKED_MM = 40     # reading when the next cell is occupied

_ORIGINAL = {
    "front": main.front,
    "left": main.left,
    "right": main.right,
    "scan_front_mm": main.scan_front_mm,
    "show_grid": main.show_grid,
}


class World:
    """Ground truth: what is really out there, as opposed to what the robot
    has been told or has worked out so far."""

    def __init__(self, map_string, hidden_obstacles):
        self.walls = set()
        self.start = None
        self.goal = None
        for i, ch in enumerate(map_string):
            cell = (i % 5, i // 5)
            if ch == "1":
                self.walls.add(cell)
            elif ch == "S":
                self.start = cell
            elif ch == "X":
                self.goal = cell
        self.hidden = set(hidden_obstacles)
        self.pose = self.start
        self.facing = "UP"
        self.moves = 0
        self.turns = 0
        self.collisions = 0
        self.scans = 0
        self.pending_render = False

    def blocked(self, cell):
        x, y = cell
        if not (0 <= x < 5 and 0 <= y < 5):
            return True
        return cell in self.walls or cell in self.hidden

    def ahead(self):
        dx, dy = STEP[self.facing]
        return (self.pose[0] + dx, self.pose[1] + dy)

    def advance(self):
        target = self.ahead()
        self.moves += 1
        if self.blocked(target):
            self.collisions += 1
            return False        # robot drove into something; pose unchanged
        self.pose = target
        return True


def render(world, note=""):
    """Print ground truth, the robot's internal map, and the LED display."""
    truth, belief = [], []
    for y in range(5):
        trow, brow = [], []
        for x in range(5):
            if (x, y) == world.pose:
                trow.append(ARROW[world.facing])
            elif (x, y) in world.hidden:
                trow.append("O")
            elif (x, y) in world.walls:
                trow.append("#")
            elif (x, y) == world.goal:
                trow.append("X")
            else:
                trow.append(".")

            if (x, y) == main.start:
                brow.append("R")
            elif (x, y) == main.goal:
                brow.append("X")
            elif main.grid[y][x] == 1:
                brow.append("#")
            else:
                brow.append(".")
        truth.append(" ".join(trow))
        belief.append(" ".join(brow))

    led = microbit.display.render().split("\n")

    print()
    if note:
        print("  " + note)
    print("  {:<14}{:<14}{}".format("WORLD", "ROBOT'S MAP", "LED"))
    for i in range(5):
        print("  {:<14}{:<14}{}".format(truth[i], belief[i], led[i]))
    if world.pose != main.start:
        print("  !! robot believes it is at {} but is really at {}".format(
            main.start, world.pose))


def install(world, delay=0.0, quiet=False):
    """Replace the firmware's physical-world functions with simulated ones."""
    original_show_grid = _ORIGINAL["show_grid"]

    def front():
        ok = world.advance()
        if not ok and not quiet:
            print("\n  *** COLLISION: drove into {} ***".format(world.ahead()))
        world.pending_render = True

    def right():
        world.facing = main.right_of(world.facing)
        world.turns += 1

    def left():
        world.facing = main.left_of(world.facing)
        world.turns += 1

    def scan_front_mm():
        world.scans += 1
        d = BLOCKED_MM if world.blocked(world.ahead()) else CLEAR_MM
        if not quiet:
            print("  sensor: {} mm toward {} (cell {})".format(
                d, world.facing, world.ahead()))
        return d

    def show_grid():
        # Called by the firmware right after it updates its believed
        # position, which is the correct moment to draw a completed move.
        original_show_grid()
        if world.pending_render:
            world.pending_render = False
            if not quiet:
                render(world, "move {} -> {}".format(world.moves, world.facing))

    main.front = front
    main.right = right
    main.left = left
    main.scan_front_mm = scan_front_mm
    main.show_grid = show_grid
    microbit.SPEED = delay


def restore():
    for name, fn in _ORIGINAL.items():
        setattr(main, name, fn)


def initial_path(map_string):
    """Solve the transmitted map once, to see where the robot intends to go."""
    main.load_sector_map(map_string)
    _, path = main.wavefront(main.grid, main.start, main.goal)
    return path


def pick_auto_obstacle(map_string):
    """Drop an obstacle on the first interior cell of the intended route.

    The firmware only trusts sensor readings for non-border cells, since a
    reading at the edge is just the wall around the course. Putting the
    obstacle on an interior cell of the planned path guarantees the
    re-planning logic is actually exercised.
    """
    for cell in initial_path(map_string)[1:]:
        if main.interior_cell(cell) and cell != main.goal:
            return [cell]
    return []


def parse_obstacles(text, map_string):
    if text == "auto":
        return pick_auto_obstacle(map_string)
    if text in ("none", ""):
        return []
    cells = []
    for part in text.split(";"):
        x, y = part.split(",")
        cells.append((int(x), int(y)))
    return cells


def run(map_string, obstacles, scan_before_move, delay=0.0, quiet=False,
        block_mm=None):
    """Run one full mission and return the resulting World."""
    main.SCAN_BEFORE_MOVE = scan_before_move
    if block_mm is not None:
        main.BLOCK_MM = block_mm

    world = World(map_string, obstacles)
    install(world, delay, quiet)
    radio.load_map(map_string)
    try:
        if quiet:
            # The firmware echoes the received map to serial; swallow it.
            with contextlib.redirect_stdout(io.StringIO()):
                main.main()
        else:
            main.main()
    finally:
        restore()
    return world


def summarise(world, label):
    reached = world.pose == world.goal
    print()
    print("=" * 64)
    print("  {}".format(label))
    print("  reached goal   {}".format("yes" if reached else "NO"))
    print("  final cell     {} (robot believed {})".format(world.pose, main.start))
    print("  moves / turns  {} / {}".format(world.moves, world.turns))
    print("  sensor reads   {}".format(world.scans))
    print("  collisions     {}".format(world.collisions))
    print("  display        {}".format(microbit.display.text))
    print("=" * 64)
    return reached


def banner(map_string, obstacles, planned, block_mm):
    print("=" * 64)
    print("  micro:bit maze robot - desktop simulation")
    print("=" * 64)
    print("  map            {}".format(map_string))
    print("  planned route  {} moves".format(max(len(planned) - 1, 0)))
    print("  hidden objects {}".format(sorted(obstacles) if obstacles else "none"))
    print("  threshold      {} mm".format(block_mm))


def compare(map_string, obstacles, block_mm):
    """Run the demonstrated behaviour and the fixed behaviour back to back."""
    legacy = run(map_string, obstacles, False, quiet=True, block_mm=block_mm)
    fixed = run(map_string, obstacles, True, quiet=True, block_mm=block_mm)

    rows = [
        ("reached goal", "no" if legacy.pose != legacy.goal else "yes",
         "no" if fixed.pose != fixed.goal else "yes"),
        ("collisions", legacy.collisions, fixed.collisions),
        ("moves", legacy.moves, fixed.moves),
        ("turns", legacy.turns, fixed.turns),
        ("sensor reads", legacy.scans, fixed.scans),
    ]
    print()
    print("  {:<16}{:>22}{:>22}".format("", "scan AFTER move", "scan BEFORE move"))
    print("  {:<16}{:>22}{:>22}".format("", "(as demonstrated)", "(current default)"))
    print("  " + "-" * 60)
    for name, a, b in rows:
        print("  {:<16}{:>22}{:>22}".format(name, a, b))
    print()
    return 0 if fixed.collisions == 0 and fixed.pose == fixed.goal else 1


def main_cli():
    ap = argparse.ArgumentParser(
        description="Run the maze robot firmware without hardware.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", default=DEFAULT_MAP,
                    help="25-character sector string (default: a two-route course)")
    ap.add_argument("--obstacle", default="auto",
                    help="'auto', 'none', or 'x,y' cells separated by ';'")
    ap.add_argument("--block-mm", type=int, default=None,
                    help="override the firmware's obstacle threshold")
    ap.add_argument("--legacy", action="store_true",
                    help="scan after moving, as in the demonstration")
    ap.add_argument("--compare", action="store_true",
                    help="run both behaviours and print a comparison")
    ap.add_argument("--delay", type=float, default=0.0,
                    help="slow playback; 1.0 runs at real robot speed")
    ap.add_argument("--quiet", action="store_true",
                    help="only print the summary")
    args = ap.parse_args()

    if len(args.map) != 25:
        ap.error("map must be 25 characters, got {}".format(len(args.map)))

    obstacles = parse_obstacles(args.obstacle, args.map)
    planned = initial_path(args.map)
    block_mm = args.block_mm if args.block_mm is not None else main.BLOCK_MM

    banner(args.map, obstacles, planned, block_mm)

    if args.compare:
        return compare(args.map, obstacles, args.block_mm)

    world = run(args.map, obstacles, not args.legacy,
                delay=args.delay, quiet=args.quiet, block_mm=args.block_mm)
    label = "scan AFTER move (as demonstrated)" if args.legacy \
        else "scan BEFORE move (current default)"
    ok = summarise(world, label)
    return 0 if ok and world.collisions == 0 else 1


if __name__ == "__main__":
    sys.exit(main_cli())
