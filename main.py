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

# Global parameters for robots
RAMP_IS_AVAILABLE = True
RAMP_WALKING_TIME = 5 / 60  # 5 seconds


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


print("Starting Luggage loading simulation")
env = simpy.Environment()
env.process(load_simulation(env, NUM_ROBOTS, NUM_LUGGAGE, AVG_LOADING_TIME, RAMP_WALKING_TIME, RAMP_IS_AVAILABLE))
env.run()

