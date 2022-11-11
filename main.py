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

        self.robot_list.draw()

class Gate:
    def __init__(self, env, num_robots, num_luggage, loading_time):
        self.env = env
        self.robot = simpy.Resource(env, num_robots)
        self.luggage = simpy.Resource(env, num_luggage)
        self.loading_time = loading_time


    def load(self, luggage):
        random_time = max(1, np.random.normal(self.loading_time, 1))
        yield self.env.timeout(random_time)
        print(f"Finished loading luggage {luggage} at {self.env.now:.2f}")


def luggage(env, id, gate):
    print(f"Luggage (id: {id}) ready for loading. {env.now:.2f}")
    with gate.robot.request() as request:
        yield request
        print(f"Luggage {id} is being loaded. {env.now:.2f}")
        yield env.process(gate.load(id))
        print(f"Luggage (id: {id}) has been loaded. {env.now:.2f}")


def load_simulation(env, num_robots, num_luggage, loading_time):
    gate = Gate(env, num_robots, num_luggage, loading_time)

    # This for-loop populates the gate with luggage before running
    # Make the starting luggage count be equal to the amount of robots so each robot is active when program starts
    robot_id = 1
    for robot_id in range(1, num_robots+1):
        env.process(luggage(env, robot_id, gate))

    # this line creates more luggages after the simulation has started
    while robot_id <= num_luggage:
        yield env.timeout(random.randint(loading_time - 1, loaded_luggage + 1))
        env.process(luggage(env, robot_id, gate))
        robot_id += 1

def run_simulation():
    print("Starting Luggage loading simulation")
    env = simpy.Environment()
    env.process(load_simulation(env, NUM_ROBOTS, NUM_LUGGAGE, AVG_LOADING_TIME))
    env.run()
    print("Luggage loaded: ", loaded_luggage)

def grid():
    window = Grid(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_TITLE)
    window.setup()
    arcade.run()

if __name__ == "__main__":
    grid()
