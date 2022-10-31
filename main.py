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
    global loaded_luggage
    print(f"Luggage (id: {id}) ready for loading. {env.now:.2f}")
    with gate.robot.request() as request:
        yield request
        print(f"Luggage {id} is being loaded. {env.now:.2f}")
        yield env.process(gate.load(id))
        print(f"Luggage (id: {id}) has been loaded. {env.now:.2f}")
        loaded_luggage += 1


def setup(env, num_robots, num_luggage, loading_time):
    gate = Gate(env, num_robots, num_luggage, loading_time)

    for i in range(1, num_luggage+1):
        env.process(luggage(env, i, gate))

    while True:
        yield env.timeout(random.randint(loading_time - 1, loaded_luggage + 1))
        i += 1
        env.process(luggage(env, i, gate))


print("Starting Luggage loading simulation")
env = simpy.Environment()
env.process(setup(env, NUM_ROBOTS, NUM_LUGGAGE, AVG_LOADING_TIME))
env.run(until=SIMULATION_TIME)

print("Luggage loaded: ", loaded_luggage)