import copy

import simpy
import random
import numpy as np
import arcade
import time

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

"""" CONFIGURATION PARAMETERS  """
# Parameters
NUM_ROBOTS = 2
NUM_LUGGAGE = 3
AVG_LOADING_TIME = 2

# Stacks with the available luggage storage (row, col)
LEFT_STORAGE = [(19, 23), (19, 24), (19, 25), (18, 23), (18, 24), (18, 25), (17, 23),
                (17, 24), (17, 25),
                (16, 23), (16, 24), (16, 25), (15, 23), (15, 24), (15, 25), (14, 23), (14, 24), (14, 25)]
RIGHT_STORAGE = [(0, 23), (0, 24), (0, 25), (1, 23), (1, 24), (1, 25), (2, 23), (2, 24), (2, 25), (3, 23), (3, 24),
                 (3, 25), (4, 23), (4, 24), (4, 25), (5, 23), (5, 24), (5, 25),
                 (6, 23), (6, 24), (6, 25), (7, 23), (7, 24), (7, 25), (8, 23), (8, 24), (8, 25), (9, 23), (9, 24),
                 (9, 25), (10, 23), (10, 24), (10, 25),
                 (11, 23), (11, 24), (11, 25), (12, 23), (12, 24), (12, 25)]

WALL = [(0, 22), (0, 26), (1, 22), (1, 26), (2, 22), (2, 26), (3, 22), (3, 26), (4, 22), (4, 26), (5, 22), (5, 26),
        (6, 22), (6, 26), (7, 22), (7, 26),
        (8, 22), (8, 26), (9, 22), (9, 26), (10, 22), (10, 26), (11, 22), (11, 26), (12, 18), (12, 19), (12, 20),
        (12, 21), (12, 22), (12, 26), (13, 26),
        (14, 22), (14, 26), (14, 18), (14, 19), (14, 20), (14, 21), (15, 22), (15, 26), (16, 22), (16, 26), (17, 22),
        (17, 26), (18, 22), (18, 26),
        (19, 22), (19, 26), (20, 22), (20, 26)]

RAMP = [(13, 18), (13, 19), (13, 20), (13, 21), (13, 22)]

STORED_SLOTS = []

"""" LOGIC/GRID PARAMETERS """
# counter variable
loaded_luggage = 0

# Global parameters for robots
RAMP_IS_AVAILABLE = True
RAMP_WALKING_TIME = 5 / 60  # 5 seconds

LUGGAGE_ROW = 10
LUGGAGE_COL = 0

LUGGAGE_UNLOAD_ROW = 10
LUGGAGE_UNLOAD_COL = 15

RAMP_ENTRANCE_ROW = 13
RAMP_ENTRANCE_COL = 18

AIRPLANE_ENTRANCE_ROW = 13
AIRPLANE_ENTRANCE_COL = 23

# Grid setup - Rows / Columns / Width / Height / Margin
ROW_COUNT = 20
COLUMN_COUNT = 30
WIDTH = 20
HEIGHT = 20
MARGIN = 2

# Screen setup - Screen_width / Screen_Height / Screen_Title
SCREEN_WIDTH = (WIDTH + MARGIN) * COLUMN_COUNT + MARGIN
SCREEN_HEIGHT = (HEIGHT + MARGIN) * ROW_COUNT + MARGIN
SCREEN_TITLE = "Automated loading system"

SPRITE_SCALING_ROBOT = 1

ROBOT_COLOR = arcade.color.GREEN
ROBOT_COLOR_LUGGAGE = arcade.color.YELLOW
LUGGAGE_COLOR = arcade.color.ORANGE
OBSTACLE_COLOR = arcade.color.BLACK
BACKGROUND_COLOR = arcade.color.GRAY
RAMP_COLOR = arcade.color.RED
LUGGAGE_DROP_SPACE_COLOR = arcade.color.BABY_PINK

ACTIVE_ROBOTS = []
logic_grid = []

simulation_renders = []


def update_active_robot(id):
    for i in range(len(ACTIVE_ROBOTS)):
        if ACTIVE_ROBOTS[i].luggageID == id:

            global RAMP_IS_AVAILABLE
            global STORED_SLOTS
            #(ACTIVE_ROBOTS[i])

            # If the robot has unloaded the luggage then find the way back to luggage spawn
            if not ACTIVE_ROBOTS[i].isCarrying:

                grid = Grid(matrix=logic_grid)

                # create start and end point
                # node(col,rows)

                start = grid.node(ACTIVE_ROBOTS[i].col, ACTIVE_ROBOTS[i].row)
                end = grid.node(LUGGAGE_COL, LUGGAGE_ROW)
                # create instance of finder. Disable diagonal movement
                # finds path from start -> end, and amount of runs required to find a path
                finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
                path, runs = finder.find_path(start, end, grid)
                if len(path) == 0:
                    break
                #    yield self.env.timeout(1 / 60)

                # set new robot position
                ACTIVE_ROBOTS[i].moveRobot(path[1][1], path[1][0])
                logic_grid[path[1][1]][path[1][0]] = 0
                # print(ACTIVE_ROBOTS[i], path)
                # set old position to unlocked
                logic_grid[path[0][1]][path[0][0]] = 1
                temp_grid = copy.deepcopy(logic_grid)
                simulation_renders.append(temp_grid)

                if ACTIVE_ROBOTS[i].col == LUGGAGE_COL and ACTIVE_ROBOTS[i].row == LUGGAGE_ROW:
                    logic_grid[LUGGAGE_ROW][LUGGAGE_COL] = 1

            # If outside and carrying a luggage, find the way to the ramp entrance
            elif ACTIVE_ROBOTS[i].isCarrying and ACTIVE_ROBOTS[i].isOutside and not ACTIVE_ROBOTS[i].isOnRamp:
                # create grid from the matrix

                grid = Grid(matrix=logic_grid)

                # create start and end point
                # node(col,rows)

                start = grid.node(ACTIVE_ROBOTS[i].col, ACTIVE_ROBOTS[i].row)
                end = grid.node(RAMP_ENTRANCE_COL, RAMP_ENTRANCE_ROW)
                # create instance of finder. Disable diagonal movement
                # finds path from start -> end, and amount of runs required to find a path
                finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
                path, runs = finder.find_path(start, end, grid)
                if len(path) == 0:
                    break
                #    yield self.env.timeout(1 / 60)

                # set new robot position
                ACTIVE_ROBOTS[i].moveRobot(path[1][1], path[1][0])
                logic_grid[path[1][1]][path[1][0]] = 0
                #print(ACTIVE_ROBOTS[i], path)
                # set old position to unlocked
                logic_grid[path[0][1]][path[0][0]] = 1
                temp_grid = copy.deepcopy(logic_grid)
                simulation_renders.append(temp_grid)
                if ACTIVE_ROBOTS[i].row == RAMP_ENTRANCE_ROW and ACTIVE_ROBOTS[i].col == RAMP_ENTRANCE_COL:
                    ACTIVE_ROBOTS[i].setIsOnRamp(True)
                    RAMP_IS_AVAILABLE = False

            # When on the ramp and carrying luggage, go inside the airplane
            elif ACTIVE_ROBOTS[i].isOnRamp and ACTIVE_ROBOTS[i].isCarrying:
                logic_grid[ACTIVE_ROBOTS[i].row][ACTIVE_ROBOTS[i].col] = 1
                ACTIVE_ROBOTS[i].moveRobot(ACTIVE_ROBOTS[i].row, ACTIVE_ROBOTS[i].col+1)
                logic_grid[ACTIVE_ROBOTS[i].row][ACTIVE_ROBOTS[i].col] = 0

                temp_grid = copy.deepcopy(logic_grid)
                simulation_renders.append(temp_grid)

                if ACTIVE_ROBOTS[i].row == AIRPLANE_ENTRANCE_ROW and ACTIVE_ROBOTS[i].col == AIRPLANE_ENTRANCE_COL:
                    ACTIVE_ROBOTS[i].setIsOutside(False)
                    ACTIVE_ROBOTS[i].setIsOnRamp(False)
                    RAMP_IS_AVAILABLE = True

            # When on the ramp and not carrying luggage, go left
            elif ACTIVE_ROBOTS[i].isOnRamp and not ACTIVE_ROBOTS[i].isCarrying:
                logic_grid[ACTIVE_ROBOTS[i].row][ACTIVE_ROBOTS[i].col] = 1
                ACTIVE_ROBOTS[i].moveRobot(ACTIVE_ROBOTS[i].row, ACTIVE_ROBOTS[i].col + 1)
                logic_grid[ACTIVE_ROBOTS[i].row][ACTIVE_ROBOTS[i].col] = 0
                temp_grid = copy.deepcopy(logic_grid)
                simulation_renders.append(temp_grid)

                if ACTIVE_ROBOTS[i].row == AIRPLANE_ENTRANCE_ROW and ACTIVE_ROBOTS[i].col == AIRPLANE_ENTRANCE_COL:
                    ACTIVE_ROBOTS[i].setIsOutside(False)
                    ACTIVE_ROBOTS[i].setIsOnRamp(False)
                    RAMP_IS_AVAILABLE = True

            # if inside and carrying luggage, find a place to unload it
            elif not ACTIVE_ROBOTS[i].isOutside and not ACTIVE_ROBOTS[i].isUnloading:
                if len(LEFT_STORAGE) > 0:
                    ACTIVE_ROBOTS[i].setUnloadCoordinates(LEFT_STORAGE[0][0],LEFT_STORAGE[0][1])
                    ACTIVE_ROBOTS[i].setIsUnloading(True)
                    #STORED_SLOTS.append(LEFT_STORAGE[0])
                    LEFT_STORAGE.pop(0)
                elif len(RIGHT_STORAGE) > 0:
                    ACTIVE_ROBOTS[i].setUnloadCoordinates(RIGHT_STORAGE[0][0], RIGHT_STORAGE[0][1])
                    ACTIVE_ROBOTS[i].setIsUnloading(True)
                    #STORED_SLOTS.append(RIGHT_STORAGE[0])
                    RIGHT_STORAGE.pop(0)
                else:
                    print("AIRPLANE FULL")

            # If the robot is inside the airplane and in the process of unloading the luggage
            elif not ACTIVE_ROBOTS[i].isOutside and ACTIVE_ROBOTS[i].isUnloading and ACTIVE_ROBOTS[i].isUnloading:
                grid = Grid(matrix=logic_grid)

                # create start and end point
                # node(col,rows)
                start = grid.node(ACTIVE_ROBOTS[i].col, ACTIVE_ROBOTS[i].row)
                end = grid.node(ACTIVE_ROBOTS[i].unloadCol, ACTIVE_ROBOTS[i].unloadRow)
                # create instance of finder. Disable diagonal movement
                # finds path from start -> end, and amount of runs required to find a path
                finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
                path, runs = finder.find_path(start, end, grid)
                if len(path) == 0:
                    break
                #    yield self.env.timeout(1 / 60)

                # set new robot position
                ACTIVE_ROBOTS[i].moveRobot(path[1][1], path[1][0])
                logic_grid[path[1][1]][path[1][0]] = 0
                # print(ACTIVE_ROBOTS[i], path)
                # set old position to unlocked
                logic_grid[path[0][1]][path[0][0]] = 1
                temp_grid = copy.deepcopy(logic_grid)
                simulation_renders.append(temp_grid)

                # If robot is on the unloading point
                if ACTIVE_ROBOTS[i].row == ACTIVE_ROBOTS[i].unloadRow \
                        and ACTIVE_ROBOTS[i].col == ACTIVE_ROBOTS[i].unloadCol:
                    ACTIVE_ROBOTS[i].setIsCarrying(False)
                    ACTIVE_ROBOTS[i].setIsUnloading(False)

            break
        """
        # If the robot is at the unloading point, then set isCarrying to false
        if ACTIVE_ROBOTS[i].x == LUGGAGE_UNLOAD_COL and ACTIVE_ROBOTS[i].y == LUGGAGE_UNLOAD_ROW:
            ACTIVE_ROBOTS[i].unload()
            logic_grid[ACTIVE_ROBOTS[i].y][ACTIVE_ROBOTS[i].x + 1] = "luggage_drop"

        # move to unload point
        if ACTIVE_ROBOTS[i].isCarrying:
            logic_grid[ACTIVE_ROBOTS[i].y][ACTIVE_ROBOTS[i].x] = 0
            ACTIVE_ROBOTS[i].moveRobot(ACTIVE_ROBOTS[i].x + 1, ACTIVE_ROBOTS[i].y)
            logic_grid[ACTIVE_ROBOTS[i].y][ACTIVE_ROBOTS[i].x] = 1
            temp_grid = copy.deepcopy(logic_grid)
            simulation_renders.append(temp_grid)

        # move to luggage point
        elif ACTIVE_ROBOTS[i].x > 0:
            logic_grid[ACTIVE_ROBOTS[i].y][ACTIVE_ROBOTS[i].x] = 0
            ACTIVE_ROBOTS[i].moveRobot(ACTIVE_ROBOTS[i].x - 1, ACTIVE_ROBOTS[i].y)
            if ACTIVE_ROBOTS[i].x > 0:
                logic_grid[ACTIVE_ROBOTS[i].y][ACTIVE_ROBOTS[i].x] = 2
            else:
                logic_grid[ACTIVE_ROBOTS[i].y][ACTIVE_ROBOTS[i].x] = 0
            temp_grid = copy.deepcopy(logic_grid)
            simulation_renders.append(temp_grid)
        break
        """


class Visualization(arcade.Window):

    def __init__(self, width, height, title):

        super().__init__(width, height, title)

        for row in range(ROW_COUNT):
            logic_grid.append([])
            for column in range(COLUMN_COUNT):
                if row == LUGGAGE_ROW and column == LUGGAGE_COL:
                    logic_grid[row].append(1)
                elif (column == 22 and (row < 13 or row > 13)) or column == 26 or \
                        (row == 12 and (17 < column < 23)) or (row == 14 and (17 < column < 23)):
                    logic_grid[row].append(0)
                elif row == 13 and 17 < column < 23:
                    logic_grid[row].append(1)
                elif (22 < column < 26) and (row < 13 or row > 13):
                    logic_grid[row].append(1)
                else:
                    logic_grid[row].append(1)

        arcade.set_background_color(arcade.color.ANTIQUE_WHITE)

        run_simulation()

        """
        for row in range(ROW_COUNT):
            logic_grid.append([])
            for column in range(COLUMN_COUNT):
                if row == LUGGAGE_ROW and column == LUGGAGE_COL:
                    logic_grid[row].append("luggage_start")
                elif (column == 22 and (row < 13 or row > 13)) or column == 26 or \
                        (row == 12 and (17 < column < 23)) or (row == 14 and (17 < column < 23)):
                    logic_grid[row].append("wall")
                elif row == 13 and 17 < column < 23:
                    logic_grid[row].append("ramp")
                elif (22 < column < 26) and (row < 13 or row > 13):
                    logic_grid[row].append("luggage_drop_space")
                else:
                    logic_grid[row].append(1)

        arcade.set_background_color(arcade.color.ANTIQUE_WHITE)

        run_simulation()
        """

    def on_draw(self):
        self.clear()

        if len(simulation_renders) == 0:
            quit()

        time.sleep(0.2)

        for row in range(ROW_COUNT):
            for column in range(COLUMN_COUNT):
                # If luggage starting point
                if row == LUGGAGE_ROW and column == LUGGAGE_COL:
                    # simulation_renders[0][row][column] = 0
                    color = LUGGAGE_COLOR
                # if wall
                elif (column == 22 and (row < 13 or row > 13)) or column == 26 or \
                        (row == 12 and (17 < column < 23)) or (row == 14 and (17 < column < 23)):
                    # simulation_renders[0][row][column] = 0
                    color = OBSTACLE_COLOR
                elif simulation_renders[0][row][column] == 0:
                    color = ROBOT_COLOR
                # if ramp
                elif row == 13 and 17 < column < 23:
                    # simulation_renders[0][row][column] = 1
                    color = RAMP_COLOR
                # if luggage drop space
                elif (22 < column < 26) and (row < 13 or row > 13):
                    # simulation_renders[0][row][column] = 1
                    color = LUGGAGE_DROP_SPACE_COLOR
                # if luggage has been loaded here
                elif (22 < column < 26) and (row < 13 or row > 13) and simulation_renders[0][row][column] == 1:
                    color = LUGGAGE_COLOR
                else:
                    # simulation_renders[0][row][column] = 1
                    color = BACKGROUND_COLOR
                print(STORED_SLOTS)
                #for i in range(len(STORED_SLOTS)):
                #    if STORED_SLOTS[i][0] == row and STORED_SLOTS[i][1] == column:
                #        color = LUGGAGE_COLOR
                #        print(color)
                #        break
                # Do the math to figure out where the box is
                x = (MARGIN + WIDTH) * column + MARGIN + WIDTH // 2
                y = (MARGIN + HEIGHT) * row + MARGIN + HEIGHT // 2

                # Draw the box
                arcade.draw_rectangle_filled(x, y, WIDTH, HEIGHT, color)

        # Remove the current display from the simulation_renders list
        simulation_renders.pop(0)

        """
        for row in range(ROW_COUNT):
            for column in range(COLUMN_COUNT):
                # Figure out what color to draw the box
                if simulation_renders[0][row][column] == 0:
                    color = BACKGROUND_COLOR
                elif simulation_renders[0][row][column] == 1:
                    color = ROBOT_COLOR_LUGGAGE
                elif simulation_renders[0][row][column] == 2:
                    color = ROBOT_COLOR
                elif simulation_renders[0][row][column] == "luggage_start":
                    color = LUGGAGE_COLOR
                elif simulation_renders[0][row][column] == "luggage_drop":
                    color = LUGGAGE_COLOR
                elif simulation_renders[0][row][column] == "luggage_drop_space":
                    color = LUGGAGE_DROP_SPACE_COLOR
                elif simulation_renders[0][row][column] == "wall":
                    color = OBSTACLE_COLOR
                elif simulation_renders[0][row][column] == "ramp":
                    color = RAMP_COLOR
                # Do the math to figure out where the box is
                x = (MARGIN + WIDTH) * column + MARGIN + WIDTH // 2
                y = (MARGIN + HEIGHT) * row + MARGIN + HEIGHT // 2
            """


class Robot(object):
    def __init__(self, luggageID, row, col, isCarrying=True, isOutside=True, isUnloading=False, isOnRamp=False,
                 unloadRow=None,unloadCol=None):
        self.luggageID = luggageID
        self.col = col
        self.row = row
        self.isCarrying = isCarrying
        self.isOutside = isOutside
        self.isUnloading = isUnloading
        self.isOnRamp = isOnRamp
        self.unloadRow = unloadRow
        self.unloadCol = unloadCol

    def moveRobot(self, row, col):
        self.row = row
        self.col = col

    def setIsCarrying(self, isCarrying):
        self.isCarrying = isCarrying

    def setIsOutside(self, isOutside):
        self.isOutside = isOutside

    def setIsUnloading(self, isUnloading):
        self.isUnloading = isUnloading

    def setIsOnRamp(self, isOnRamp):
        self.isOnRamp = isOnRamp

    def setUnloadCoordinates(self,row,col):
        self.unloadRow = row
        self.unloadCol = col

    def __repr__(self):
        return f"<row:{self.row} col:{self.col} luggageID:{self.luggageID} isCarrying:{self.isCarrying} " \
               f"isOutside:{self.isOutside} isUnloading:{self.isUnloading} isOnRamp:{self.isOnRamp}>"


class Gate:
    def __init__(self, env, num_robots, num_luggage, loading_time, ramp_walking_time, ramp_is_available):
        self.env = env
        self.robot = simpy.Resource(env, num_robots)
        self.luggage = simpy.Resource(env, num_luggage)
        self.loading_time = loading_time
        self.ramp_walking_time = ramp_walking_time
        self.ramp_is_available = ramp_is_available

    def load(self, robot, id):
        ACTIVE_ROBOTS.append(robot)

        while robot.row != LUGGAGE_ROW or robot.col != LUGGAGE_COL:
            update_active_robot(id)
            time_for_step = max(1, np.random.normal(self.loading_time, 1))
            yield self.env.timeout(time_for_step)

        # remove robot from ACTIVE_ROBOTS
        for i in range(len(ACTIVE_ROBOTS)):
            if ACTIVE_ROBOTS[i].luggageID == id:
                ACTIVE_ROBOTS.pop(i)
                break

        print(f"Finished loading luggage {robot.luggageID} at {self.env.now:.2f}")


def luggage(env, id, gate):
    print(f"Luggage (id: {id}) ready for loading. {env.now:.2f}")
    with gate.robot.request() as request:
        yield request
        while logic_grid[LUGGAGE_ROW][LUGGAGE_COL + 1] == 0:
            if logic_grid[LUGGAGE_ROW][LUGGAGE_COL + 1] == 0:
                logic_grid[LUGGAGE_ROW][LUGGAGE_COL + 1] = 1
            else:
                yield env.timeout(1)
        robot = Robot(id, LUGGAGE_ROW, LUGGAGE_COL + 1)
        print(f"Luggage {id} is being loaded. {env.now:.2f}")
        yield env.process(gate.load(robot, id))
        print(f"Luggage (id: {id}) has been loaded. {env.now:.2f}")


def load_simulation(env, num_robots, num_luggage, loading_time, ramp_walking_time, ramp_is_available):
    gate = Gate(env, num_robots, num_luggage, loading_time, ramp_walking_time, ramp_is_available)

    # This for-loop populates the gate with luggage before running
    # Make the starting luggage count be equal to the amount of robots so each robot is active when program starts
    luggage_id = 1
    # for robot_id in range(1, num_robots + 1):
    #    env.process(luggage(env, luggage_id, gate))

    # this line creates more luggages after the simulation has started
    while luggage_id <= num_luggage:
        yield env.timeout(0)
        env.process(luggage(env, luggage_id, gate))
        luggage_id += 1


def run_simulation():
    print("Starting Luggage loading simulation")
    env = simpy.Environment()
    env.process(load_simulation(env, NUM_ROBOTS, NUM_LUGGAGE, AVG_LOADING_TIME, RAMP_WALKING_TIME, RAMP_IS_AVAILABLE))
    env.run()


def grid():
    print("Starting grid")
    Visualization(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_TITLE)
    arcade.run()


if __name__ == "__main__":
    grid()

"""
    def outside_distance(self, luggage):
        print(f"walking to/from luggage_starting_pos and ramp (id: {luggage})")
        random_time = max(1, np.random.normal(self.loading_time, 1))
        yield self.env.timeout(random_time)

    def walk_on_ramp(self, luggage):
        print(f"walking up/down the ramp (id: {luggage})")
        random_time = max(1, np.random.normal(self.ramp_walking_time, 1))
        yield self.env.timeout(random_time)

    def unload_luggage(self, luggage):
        print(f"unloading the luggage (id: {luggage}) inside the airplane if there are less than # robots inside")
        random_time = max(1, np.random.normal(self.loading_time, 1))
        yield self.env.timeout(random_time)


def luggage(env, id, gate):
    print(f"Luggage (id: {id}) ready for loading. {env.now:.2f}")
    with gate.robot.request() as request:
        yield request
        yield env.process(gate.outside_distance(id))
        yield env.process(gate.walk_on_ramp(id))
        yield env.process(gate.unload_luggage(id))
        yield env.process(gate.walk_on_ramp(id))
        yield env.process(gate.outside_distance(id))
        print(f"Walking to luggage starting point. Luggage (id: {id}) has been loaded. {env.now:.2f}")
    """
