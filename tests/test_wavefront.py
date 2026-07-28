"""Tests for the robot firmware, run on a desktop against the real src/main.py.

The stub modules in sim/stubs let the MicroPython firmware be imported by
CPython, so these exercise the shipped code rather than a reimplementation.

    python -m unittest discover -s tests -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, "sim"))

from harness import load_firmware      # noqa: E402

main = load_firmware()

# Two gaps through a middle wall; both routes to the goal are 8 moves.
TWO_ROUTE_MAP = "S00000000010101000000000X"
# One passage through the middle wall.
ONE_ROUTE_MAP = "S00000000011101000000000X"
# Goal walled off completely.
SEALED_MAP = "S00000000011111000100010X"


def cells_adjacent(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1


class MapParsingTests(unittest.TestCase):
    def test_parses_grid_start_and_goal(self):
        main.load_sector_map(TWO_ROUTE_MAP)
        self.assertEqual(main.start, (0, 0))
        self.assertEqual(main.goal, (4, 4))
        self.assertEqual(main.grid[2], [1, 0, 1, 0, 1])
        # Start and goal cells are traversable, not walls.
        self.assertEqual(main.grid[0][0], 0)
        self.assertEqual(main.grid[4][4], 0)

    def test_rejects_wrong_length(self):
        with self.assertRaises(ValueError):
            main.load_sector_map("S0X")

    def test_rejects_unknown_character(self):
        with self.assertRaises(ValueError):
            main.load_sector_map("S000000000101010000000Z0X")

    def test_rejects_missing_goal(self):
        with self.assertRaises(ValueError):
            main.load_sector_map("S000000000101010000000000")

    def test_sector_round_trips(self):
        main.load_sector_map(TWO_ROUTE_MAP)
        main.update_sector_from_grid()
        self.assertEqual(main.sector, TWO_ROUTE_MAP)

    def test_discovered_wall_appears_in_sector(self):
        main.load_sector_map(TWO_ROUTE_MAP)
        main.mark_wall((1, 1))
        main.update_sector_from_grid()
        self.assertEqual(main.sector[1 * 5 + 1], "1")

    def test_goal_is_never_marked_as_wall(self):
        main.load_sector_map(TWO_ROUTE_MAP)
        self.assertFalse(main.mark_wall(main.goal))
        self.assertEqual(main.grid[4][4], 0)


class WavefrontTests(unittest.TestCase):
    def setUp(self):
        main.load_sector_map(TWO_ROUTE_MAP)

    def test_finds_a_shortest_path(self):
        _, path = main.wavefront(main.grid, main.start, main.goal)
        # Manhattan distance from (0,0) to (4,4) is 8, so 9 cells.
        self.assertEqual(len(path), 9)

    def test_path_starts_at_start_and_ends_at_goal(self):
        _, path = main.wavefront(main.grid, main.start, main.goal)
        self.assertEqual(path[0], main.start)
        self.assertEqual(path[-1], main.goal)

    def test_path_steps_are_adjacent_and_avoid_walls(self):
        _, path = main.wavefront(main.grid, main.start, main.goal)
        for a, b in zip(path, path[1:]):
            self.assertTrue(cells_adjacent(a, b), "{} -> {} is not a step".format(a, b))
        for x, y in path:
            self.assertEqual(main.grid[y][x], 0, "path crosses a wall at {}".format((x, y)))

    def test_distances_increase_by_one_along_the_path(self):
        dist, path = main.wavefront(main.grid, main.start, main.goal)
        for i, (x, y) in enumerate(path):
            self.assertEqual(dist[y][x], i)

    def test_returns_empty_path_when_goal_is_sealed_off(self):
        main.load_sector_map(SEALED_MAP)
        _, path = main.wavefront(main.grid, main.start, main.goal)
        self.assertEqual(path, [])

    def test_reroutes_when_the_chosen_gap_is_blocked(self):
        _, path = main.wavefront(main.grid, main.start, main.goal)
        gap = next(c for c in path if main.interior_cell(c))
        main.mark_wall(gap)
        _, rerouted = main.wavefront(main.grid, main.start, main.goal)
        self.assertTrue(rerouted, "no alternative route found")
        self.assertNotIn(gap, rerouted)

    def test_single_passage_map_has_no_alternative(self):
        """Blocking the only gap in the middle wall must report no path,
        rather than silently returning a route that crosses it."""
        main.load_sector_map(ONE_ROUTE_MAP)
        passage = (3, 2)
        _, path = main.wavefront(main.grid, main.start, main.goal)
        self.assertIn(passage, path, "expected the route through the only gap")

        main.mark_wall(passage)
        _, rerouted = main.wavefront(main.grid, main.start, main.goal)
        self.assertEqual(rerouted, [])


class HeadingTests(unittest.TestCase):
    def test_right_of_cycles_through_all_four_headings(self):
        d = "UP"
        seen = []
        for _ in range(4):
            seen.append(d)
            d = main.right_of(d)
        self.assertEqual(seen, ["UP", "RIGHT", "DOWN", "LEFT"])
        self.assertEqual(d, "UP")

    def test_left_of_is_the_inverse_of_right_of(self):
        for d in ("UP", "RIGHT", "DOWN", "LEFT"):
            self.assertEqual(main.left_of(main.right_of(d)), d)

    def test_direction_from_cells(self):
        origin = (2, 2)
        self.assertEqual(main.direction_from_cells(origin, (2, 1)), "UP")
        self.assertEqual(main.direction_from_cells(origin, (3, 2)), "RIGHT")
        self.assertEqual(main.direction_from_cells(origin, (2, 3)), "DOWN")
        self.assertEqual(main.direction_from_cells(origin, (1, 2)), "LEFT")

    def test_neighbor_in_direction_matches_direction_from_cells(self):
        origin = (2, 2)
        for d in ("UP", "RIGHT", "DOWN", "LEFT"):
            neighbour = main.neighbor_in_direction(origin, d)
            self.assertEqual(main.direction_from_cells(origin, neighbour), d)

    def test_turn_to_reaches_every_heading(self):
        for target in ("UP", "RIGHT", "DOWN", "LEFT"):
            main.facing = "UP"
            main.turn_to(target)
            self.assertEqual(main.facing, target)

    def test_turn_to_handles_a_180(self):
        main.facing = "UP"
        main.turn_to("DOWN")
        self.assertEqual(main.facing, "DOWN")


class InteriorCellTests(unittest.TestCase):
    def test_border_cells_are_not_interior(self):
        for cell in ((0, 0), (4, 4), (0, 2), (2, 0), (4, 2), (2, 4)):
            self.assertFalse(main.interior_cell(cell), cell)

    def test_middle_cells_are_interior(self):
        for cell in ((1, 1), (2, 2), (3, 3), (1, 3), (3, 1)):
            self.assertTrue(main.interior_cell(cell), cell)


class MissionTests(unittest.TestCase):
    """End-to-end runs through the simulator, driving the real firmware."""

    def setUp(self):
        import simulate
        self.sim = simulate
        self.obstacles = simulate.pick_auto_obstacle(TWO_ROUTE_MAP)
        self._flag = main.SCAN_BEFORE_MOVE

    def tearDown(self):
        main.SCAN_BEFORE_MOVE = self._flag

    def test_reaches_goal_with_no_obstacles(self):
        world = self.sim.run(TWO_ROUTE_MAP, [], True, quiet=True)
        self.assertEqual(world.pose, world.goal)
        self.assertEqual(world.collisions, 0)
        self.assertEqual(world.moves, 8)      # optimal route, no detour

    def test_routes_around_an_unmapped_obstacle(self):
        world = self.sim.run(TWO_ROUTE_MAP, self.obstacles, True, quiet=True)
        self.assertEqual(world.pose, world.goal)
        self.assertEqual(world.collisions, 0)
        self.assertGreater(world.moves, 8)    # detour cost the extra moves

    def test_scanning_after_moving_reproduces_the_demo_failure(self):
        """Regression guard for the defect found in the demonstration.

        Scanning only after a move completes leaves no point at which to
        abort, so the robot drives into the obstacle it just detected.
        """
        world = self.sim.run(TWO_ROUTE_MAP, self.obstacles, False, quiet=True)
        self.assertGreater(world.collisions, 0)
        self.assertNotEqual(world.pose, world.goal)


if __name__ == "__main__":
    unittest.main(verbosity=2)
