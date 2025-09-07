import numpy as np
from simulator import Simulator

def controller(x):
    ... # YOUR CODE HERE
    return np.array([0.0, 0.0])

sim = Simulator(controller)
sim.run()
sim.animate()
sim.plot()