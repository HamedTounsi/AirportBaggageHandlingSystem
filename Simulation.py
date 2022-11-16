import copy

import simpy
import random
import numpy as np
import arcade
import time

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from threading import Thread

# Parameters
NUM_ROBOTS = 1
NUM_LUGGAGE = 2
AVG_LOADING_TIME = 2

# counter variable
loaded_luggage = 0

# Global parameters for robots
RAMP_IS_AVAILABLE = True
RAMP_WALKING_TIME = 5 / 60  # 5 seconds
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
LUGGAGE_COLOR = arcade.color.BLACK
BACKGROUND_COLOR = arcade.color.GRAY

LUGGAGE_ROW = 10
LUGGAGE_COL = 0

# Stacks with the available luggage storage
LEFT_STORAGE = [(20,23),(20,24),(20,25),(19,23),(19,24),(19,25),(18,23),(18,24),(18,25),(17,23),(17,24),(17,25),
                 (16,23),(16,24),(16,25),(15,23),(15,24),(15,25),(14,23),(14,24),(14,25)]
RIGHT_STORAGE = [(0,23),(0,24),(0,25),(1,23),(1,24),(1,25),(2,23),(2,24),(2,25),(3,23),(3,24),(3,25),(4,23),(4,24),(4,25),(5,23),(5,24),(5,25),
                 (6,23),(6,24),(6,25),(7,23),(7,24),(7,25),(8,23),(8,24),(8,25),(9,23),(9,24),(9,25),(10,23),(10,24),(10,25),
                 (11,23),(11,24),(11,25),(12,23),(12,24),(12,25)]

WALL = [(0,22),(0,26),(1,22),(1,26),(2,22),(2,26),(3,22),(3,26),(4,22),(4,26),(5,22),(5,26),(6,22),(6,26),(7,22),(7,26),
        (8,22),(8,26),(9,22),(9,26),(10,22),(10,26),(11,22),(11,26),(12,18),(12,19),(12,20),(12,21),(12,22),(12,26),(13,26),
        (14,22),(14,26),(14,18),(14,19),(14,20),(14,21),(15,22),(15,26),(16,22),(16,26),(17,22),(17,26),(18,22),(18,26),
        (19,22),(19,26),(20,22),(20,26)]

RAMP = [(13,18),(13,19),(13,20),(13,21),(13,22)]

LUGGAGE_UNLOAD_ROW = 10
LUGGAGE_UNLOAD_COL = 15

ACTIVE_ROBOTS = []
logic_grid = []

simulation_renders = []


def update_active_robots():
    for i in range(len(ACTIVE_ROBOTS)):
        # If the robot is at the unloading point, then set isCarrying to false
        if ACTIVE_ROBOTS[i].x == LUGGAGE_UNLOAD_COL and ACTIVE_ROBOTS[i].y == LUGGAGE_UNLOAD_ROW:
            ACTIVE_ROBOTS[i].unload()
            logic_grid[ACTIVE_ROBOTS[i].y][ACTIVE_ROBOTS[i].x+1] = "luggage_drop"

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


class Grid(arcade.Window):

    def __init__(self, width, height, title):

        super().__init__(width, height, title)

        for row in range(ROW_COUNT):
            logic_grid.append([])
            for column in range(COLUMN_COUNT):
                if row == LUGGAGE_ROW and column == LUGGAGE_COL:
                    logic_grid[row].append("luggage_start")
                else:
                    logic_grid[row].append(0)

        arcade.set_background_color(arcade.color.ANTIQUE_WHITE)

        run_simulation()

    # def setup(self):
    #    self.on_draw()

    """"
        self.robot_list = arcade.SpriteList()
        #robot_img = "robot_.png"
        self.robot_sprite = arcade.SpriteSolidColor(WIDTH, HEIGHT, ROBOT_COLOR)

        self.robot_sprite.center_x = 1
        self.robot_sprite.center_y = 1
        self.robot_list.append(self.robot_sprite)
    """

    def on_draw(self):
        self.clear()

        if len(simulation_renders) == 0:
            quit()

        time.sleep(0.2)

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
                # Giv color to luggage storage, wall, and ramp
                for (x, y) in LEFT_STORAGE:
                    if row == x and column == y:
                        color = arcade.color.BABY_PINK
                for (x, y) in RIGHT_STORAGE:
                    if row == x and column == y:
                        color = arcade.color.BABY_PINK
                for (x, y) in WALL:
                    if row == x and column == y:
                        color = arcade.color.BLACK
                for (x, y) in RAMP:
                    if row == x and column == y:
                        color = arcade.color.RED

                
                # Do the math to figure out where the box is
                x = (MARGIN + WIDTH) * column + MARGIN + WIDTH // 2
                y = (MARGIN + HEIGHT) * row + MARGIN + HEIGHT // 2

                # Draw the box
                arcade.draw_rectangle_filled(x, y, WIDTH, HEIGHT, color)

        # Remove the current display from the simulation_renders list
        simulation_renders.pop(0)


class Robot(object):
    def __init__(self, luggageID, x, y, isCarrying):
        self.luggageID = luggageID
        self.x = x
        self.y = y
        self.isCarrying = isCarrying

    def moveRobot(self, x, y):
        self.x = x
        self.y = y

    def unload(self):
        self.isCarrying = False

    def __repr__(self):
        return f"<Test x:{self.x} y:{self.y}>"

"""
class UpdateGrid(Thread):
    def __init__(self, event):
        Thread.__init__(self)
        self.stopped = event

    def run(self):
        while not self.stopped.wait(0.5):
            print("updating grid")
            update_active_robots()
            # call a function
"""


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
        # print(robot.x != LUGGAGE_UNLOAD_COL or robot.y != LUGGAGE_UNLOAD_ROW)
        # print(robot.x, LUGGAGE_UNLOAD_COL, robot.y, LUGGAGE_UNLOAD_ROW)

        start = (robot.y, robot.x)
        end = LEFT_STORAGE.pop(0)
        while robot.x != LUGGAGE_COL or robot.y != LUGGAGE_ROW:
            update_active_robots()
            #time.sleep(0.1)

        # remove robot from ACTIVE_ROBOTS
        for i in range(len(ACTIVE_ROBOTS)):
            if ACTIVE_ROBOTS[i].luggageID == id:
                ACTIVE_ROBOTS.pop(i)

        random_time = max(1, np.random.normal(self.loading_time, 1))
        yield self.env.timeout(random_time)
        print(f"Finished loading luggage {robot.luggageID} at {self.env.now:.2f}")


def luggage(env, id, gate):
    print(f"Luggage (id: {id}) ready for loading. {env.now:.2f}")
    with gate.robot.request() as request:
        robot = Robot(id, LUGGAGE_COL+1, LUGGAGE_ROW, True)
        yield request
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
    Grid(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_TITLE)
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
