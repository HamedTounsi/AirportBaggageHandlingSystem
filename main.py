import simpy
import random
import numpy as np

# Parameters
NUM_ROBOTS = 5
NUM_LUGGAGE = 30
AVG_LOADING_TIME = 2
SIMULATION_TIME = 90

# counter variable
loaded_luggage = 0


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


print("Starting Luggage loading simulation")
env = simpy.Environment()
env.process(load_simulation(env, NUM_ROBOTS, NUM_LUGGAGE, AVG_LOADING_TIME))
env.run()

print("Luggage loaded: ", loaded_luggage)