import simpy
import random
import numpy as np
import arcade
from PIL import Image

# Parameters
NUM_ROBOTS = 5
NUM_LUGGAGE = 30
AVG_LOADING_TIME = 2
SIMULATION_TIME = 90

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

SPRITE_SCALING_ROBOT = 0.5

ROBOT_COLOR = arcade.color.GREEN
ROBOT_COLOR_LUGGAGE = arcade.color.YELLOW
LUGGAGE_COLOR = arcade.color.ORANGE


class Grid(arcade.Window):

    def __init__(self, width, height, title):

        super().__init__(width, height, title)

        self.robot_list = None
        self.robot_sprite = None
        #self.luggage_list = None
        self.grid = []
        for row in range(ROW_COUNT):
            self.grid.append([])
            for column in range(COLUMN_COUNT):
                self.grid[row].append(0)

        arcade.set_background_color(arcade.color.ANTIQUE_WHITE)

    def setup(self):
        self.robot_list = arcade.SpriteList()
        robot_img = "robot_.png"
        self.robot_sprite = arcade.Sprite(robot_img, SPRITE_SCALING_ROBOT)
        self.robot_sprite.center_x = SCREEN_WIDTH/2
        self.robot_sprite.center_y = SCREEN_HEIGHT/2
        self.robot_list.append(self.robot_sprite)

    def on_draw(self):
        self.clear()

        for row in range(ROW_COUNT):
            for column in range(COLUMN_COUNT):
                x = (MARGIN + WIDTH) * column + MARGIN + WIDTH // 2
                y = (MARGIN + HEIGHT) * row + MARGIN + HEIGHT // 2

                arcade.draw_rectangle_filled(x, y, WIDTH, HEIGHT, arcade.color.GRAY)
                if row == 5 and column == 5:
                    arcade.draw_rectangle_filled(x, y, WIDTH, HEIGHT, arcade.color.YELLOW)
        self.robot_list.draw()


class Gate:
    def __init__(self, env, num_robots, num_luggage, loading_time, ramp_walking_time, ramp_is_available):

        self.env = env
        self.robot = simpy.Resource(env, num_robots)
        self.luggage = simpy.Resource(env, num_luggage)
        self.loading_time = loading_time
        self.ramp_walking_time = ramp_walking_time
        self.ramp_is_available = ramp_is_available

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
    window = Grid(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_TITLE)
    window.setup()
    arcade.run()


if __name__ == "__main__":
    grid()