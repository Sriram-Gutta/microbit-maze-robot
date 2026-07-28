"""Autonomous maze-solving robot for the BBC micro:bit.

Flow:
  1. Request the 25-character sector map over radio.
  2. Parse it into a 5x5 grid (walls, open cells, start, goal).
  3. Run a wavefront (BFS) search to get the shortest path.
  4. Drive the path cell by cell, scanning ahead with the ultrasonic
     sensor after each move. A newly detected obstacle is written into
     the grid and the path is recomputed from the current cell.

Requires robotbit_library.py to be present on the micro:bit filesystem.
"""

from microbit import *
from time import sleep_us
from machine import time_pulse_us
import radio
import robotbit_library as r

# --- Radio ---------------------------------------------------------------
RADIO_CHANNEL = 19       # must match the map transmitter's channel
TIMEOUT_MS = 50000       # give up waiting for a map after this long

# --- Drive train ---------------------------------------------------------
LEFT_MOTOR = r.M1A
RIGHT_MOTOR = r.M2B
LEFT_FWD = -50
RIGHT_FWD = 50
CELL_TIME_MS = 856       # ms at 50% power to travel one cell (10 in)
TURN_TIME_MS = 368       # ms at 50% power for a 90 degree rotation

# --- Ultrasonic ----------------------------------------------------------
TIME_OUT = 100000        # echo timeout in us; larger = longer range, slower
ULTRA_PIN = pin15
# Distance at which the cell ahead is treated as blocked. 50 mm proved too
# short in testing: the robot was already entering the blocked cell before
# the reading tripped. 70-100 mm is the recommended starting point, but it
# needs re-calibration against the drive timing above.
BLOCK_MM = 50

# When True, the sensor checks the next cell after turning but *before*
# driving into it, so a newly found obstacle can be avoided rather than hit.
# When False, the scan happens after the move completes -- the behaviour used
# in the demonstration, kept so the original failure can be reproduced. See
# docs/design-notes.md; sim/simulate.py --compare runs both.
SCAN_BEFORE_MOVE = True

# --- Grid / display ------------------------------------------------------
W = 5
H = 5
BRIGHT_OPEN = 1
BRIGHT_WALL = 5
BRIGHT_START = 6
BRIGHT_END = 8
BRIGHT_PATH = 9

sector = ""
grid_text = []
grid = [[0] * W for _ in range(H)]
start = None
goal = None
facing = "UP"

r.setup()


# ---------------------------------------------------------------- radio ---
def update_progress(cnt):
    """Sweep a pixel across the middle row while waiting for the map."""
    if cnt < 50:
        if cnt % 10:
            x = cnt // 10
            display.set_pixel(x, 2, 9)
    else:
        cnt = 0
        display.clear()
    return cnt + 1


def request_map():
    """Ask for the sector map and return it, or '' if nothing arrives."""
    radio.config(channel=RADIO_CHANNEL)
    radio.on()

    msg = ''
    cnt = 0
    map_string = ''

    radio.send('Request')
    timer = running_time()

    while True:
        msg = radio.receive()
        if msg:
            if len(msg) == 25:
                map_string = msg
                print(msg)
                break
        else:
            cnt = update_progress(cnt)

        if running_time() - timer > TIMEOUT_MS:
            break

        sleep(250)

    radio.off()
    return map_string


# ----------------------------------------------------------- ultrasonic ---
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


def distance_mm():
    return distance(ULTRA_PIN, ULTRA_PIN)


def scan_front_mm():
    d = distance_mm()
    print('front mm =', d)
    return d


# ------------------------------------------------------------- movement ---
def stop_motors():
    r.motor(LEFT_MOTOR, 0)
    r.motor(RIGHT_MOTOR, 0)


def front():
    # Right motor runs slightly slower to compensate for drift.
    r.motor(LEFT_MOTOR, -50)
    r.motor(RIGHT_MOTOR, 47)
    sleep(CELL_TIME_MS)
    stop_motors()
    sleep(250)


def right():
    r.motor(LEFT_MOTOR, LEFT_FWD)
    r.motor(RIGHT_MOTOR, -RIGHT_FWD)
    sleep(TURN_TIME_MS)
    stop_motors()
    sleep(250)


def left():
    r.motor(LEFT_MOTOR, -LEFT_FWD)
    r.motor(RIGHT_MOTOR, RIGHT_FWD)
    sleep(TURN_TIME_MS)
    stop_motors()
    sleep(250)


# ------------------------------------------------------- map conversion ---
def load_sector_map(map_string):
    """Turn a 25-char sector string into the grid, start and goal globals."""
    global sector, grid_text, grid, start, goal

    sector = map_string
    grid_text = []

    for i in range(0, 25, 5):
        grid_text.append(sector[i:i + 5])

    grid = [[0] * W for _ in range(H)]
    start = None
    goal = None

    for y, row in enumerate(grid_text):
        if len(row) != W:
            raise ValueError('wrong map size')
        for x, ch in enumerate(row):
            if ch == '1':
                grid[y][x] = 1
            elif ch == '0':
                grid[y][x] = 0
            elif ch == 'S':
                start = (x, y)
                grid[y][x] = 0
            elif ch == 'X':
                goal = (x, y)
                grid[y][x] = 0
            else:
                raise ValueError("unexpected character in map")

    if start is None or goal is None:
        raise ValueError("map is missing a start or goal")


def show_grid():
    for y in range(H):
        for x in range(W):
            if (x, y) == start:
                b = BRIGHT_START
            elif (x, y) == goal:
                b = BRIGHT_END
            elif grid[y][x] == 1:
                b = BRIGHT_WALL
            else:
                b = BRIGHT_OPEN
            display.set_pixel(x, y, b)


# ------------------------------------------------------------ wavefront ---
def wavefront(g, start, goal):
    """Breadth-first flood fill from start; walk the distances back from goal.

    Returns (distance_grid, path). path is [] when the goal is unreachable.
    """
    sx, sy = start
    gx, gy = goal
    if g[sy][sx] == 1 or g[gy][gx] == 1:
        return None, []

    dist = [[-1] * W for _ in range(H)]
    dist[sy][sx] = 0

    qx, qy = [sx], [sy]
    head = 0

    while head < len(qx):
        x = qx[head]
        y = qy[head]
        head += 1

        if x == gx and y == gy:
            break

        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= nx < W and 0 <= ny < H:
                if g[ny][nx] == 0 and dist[ny][nx] == -1:
                    dist[ny][nx] = dist[y][x] + 1
                    qx.append(nx)
                    qy.append(ny)

    if dist[gy][gx] == -1:
        return dist, []

    path = [(gx, gy)]
    x, y = gx, gy
    d = dist[y][x]
    while d > 0:
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= nx < W and 0 <= ny < H and dist[ny][nx] == d - 1:
                path.append((nx, ny))
                x, y = nx, ny
                d -= 1
                break
    path.reverse()
    return dist, path


def animate_path_step(path):
    if not path:
        return
    for x, y in path:
        if (x, y) != start and (x, y) != goal:
            display.set_pixel(x, y, BRIGHT_PATH)
            sleep(130)


def show_path(path):
    for x, y in path:
        if (x, y) != start and (x, y) != goal:
            display.set_pixel(x, y, BRIGHT_PATH)


def blink_path(path, times=2):
    if not path:
        return
    for _ in range(times):
        show_path(path)
        sleep(350)
        show_grid()
        sleep(250)


# ------------------------------------------------------------- heading ---
def right_of(direction):
    if direction == 'UP':
        return 'RIGHT'
    elif direction == 'RIGHT':
        return 'DOWN'
    elif direction == 'DOWN':
        return 'LEFT'
    else:
        return 'UP'


def left_of(direction):
    if direction == 'UP':
        return 'LEFT'
    elif direction == 'LEFT':
        return 'DOWN'
    elif direction == 'DOWN':
        return 'RIGHT'
    else:
        return 'UP'


def direction_from_cells(a, b):
    """Heading needed to step from cell a to adjacent cell b."""
    x1, y1 = a
    x2, y2 = b

    if x2 == x1 and y2 == y1 - 1:
        return 'UP'
    elif x2 == x1 + 1 and y2 == y1:
        return 'RIGHT'
    elif x2 == x1 and y2 == y1 + 1:
        return 'DOWN'
    else:
        return 'LEFT'


def turn_to(target_dir):
    """Rotate in place until facing target_dir, tracking the heading."""
    global facing

    if facing == target_dir:
        return

    if right_of(facing) == target_dir:
        right()
        facing = right_of(facing)
    elif left_of(facing) == target_dir:
        left()
        facing = left_of(facing)
    else:
        right()
        facing = right_of(facing)
        right()
        facing = right_of(facing)


def neighbor_in_direction(cell, direction):
    x, y = cell
    if direction == 'UP':
        return (x, y - 1)
    elif direction == 'RIGHT':
        return (x + 1, y)
    elif direction == 'DOWN':
        return (x, y + 1)
    else:
        return (x - 1, y)


def interior_cell(cell):
    """True for cells not on the border, where a reading means an obstacle
    rather than the wall around the course."""
    x, y = cell
    return 0 < x < W - 1 and 0 < y < H - 1


def mark_wall(cell):
    x, y = cell
    if (x, y) == goal:
        return False
    if grid[y][x] == 0:
        grid[y][x] = 1
        return True
    return False


def update_sector_from_grid():
    """Re-encode the (possibly updated) grid back into a sector string."""
    global sector
    chars = []
    for y in range(H):
        for x in range(W):
            if (x, y) == start:
                chars.append('S')
            elif (x, y) == goal:
                chars.append('X')
            elif grid[y][x] == 1:
                chars.append('1')
            else:
                chars.append('0')
    sector = ''.join(chars)


def scan_and_update_map():
    """Look at the cell ahead; record a wall if something is close.

    Returns True when the map changed and the path needs recomputing.
    """
    global start

    found_new_wall = False
    current = start

    front_cell = neighbor_in_direction(current, facing)
    if 0 <= front_cell[0] < W and 0 <= front_cell[1] < H:
        if interior_cell(front_cell):
            d_front = scan_front_mm()
            if d_front > 0 and d_front <= BLOCK_MM:
                if mark_wall(front_cell):
                    found_new_wall = True
    if found_new_wall:
        update_sector_from_grid()
        show_grid()
        sleep(300)
    return found_new_wall


# ----------------------------------------------------------- navigation ---
def solve_and_show():
    show_grid()
    sleep(700)
    dist, path = wavefront(grid, start, goal)

    if not path:
        display.scroll('NOPATH')
        show_grid()
        return []

    animate_path_step(path)
    blink_path(path, 2)
    show_path(path)
    return path


def navigate_to_goal():
    """Drive to the goal, replanning whenever a new obstacle turns up."""
    global start

    while start != goal:
        path = solve_and_show()
        if not path:
            return False

        moved = False
        replanned = False

        for i in range(len(path) - 1):
            current_cell = path[i]
            next_cell = path[i + 1]

            target_dir = direction_from_cells(current_cell, next_cell)
            turn_to(target_dir)

            if SCAN_BEFORE_MOVE:
                # Facing now equals target_dir and start is still
                # current_cell, so this scans exactly the cell we are about
                # to enter -- while there is still time to abort.
                if scan_and_update_map():
                    replanned = True
                    break

            front()
            moved = True

            start = next_cell
            show_grid()
            sleep(100)

            if start == goal:
                return True

            if not SCAN_BEFORE_MOVE:
                if scan_and_update_map():
                    replanned = True
                    break    # abandon this path; the outer loop replans

        # Re-planning without moving is progress: the grid gained a wall, so
        # the next pass produces a different route. Only a pass that neither
        # moved nor learned anything means we are genuinely stuck.
        if not moved and not replanned:
            return False

    return True


def main():
    global facing

    display.show('R')
    map_string = request_map()

    if map_string == '':
        display.scroll('NOREPLY')
        return

    load_sector_map(map_string)
    facing = 'UP'    # robot must be placed facing "up" relative to the grid

    success = navigate_to_goal()

    if success:
        display.show(Image.HAPPY)
        stop_motors()
    else:
        display.show(Image.SAD)
        stop_motors()


if __name__ == '__main__':
    main()
