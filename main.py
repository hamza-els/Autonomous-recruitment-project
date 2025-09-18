import numpy as np
import math
from simulator import Simulator, centerline

sim = Simulator()

def controller(x):
    """controller for a car

    Args:
        x (ndarray): numpy array of shape (5,) containing [x, y, heading, velocity, steering angle]

    Returns:
        ndarray: numpy array of shape (2,) containing [fwd acceleration, steering rate]
    """
    ... # YOUR CODE HERE
    # heading = 0 is pointing directly right/ positive x axis
    # we can scale d on this function depending on the velocity of the car, 
    # that way being more careful at slower speeds
    # might need to account for steering rate
    def pointAhead(d):
        """returns a point d distance in front of the car"""
        return np.array([x[0] + d*math.cos(x[2]), x[1] + d*math.sin(x[2])])
    point = pointAhead(1)
    print("Current heading is: ", x[2], " Position is: ",  x[0], x[1], " point ahead by 1 meter: ", point[0], point[1])
    turn = 0
    currCenter = centerline(x[0])
    # if(currCenter[0] > x[0]):
    #     turn += 0.5
    # else:
    #     turn -= 0.5
    if(currCenter[1] > x[1]):
        if(x[4] < 0.45):
            turn += 0.15
    else:
        if(x[4] > -0.45):
            turn -= 0.15
    a = 0
    if(x[3] < 2):
        a = 12
    return np.array([a, turn])

def closestCones(p):
    """returns the 4 closest cones to a point"""
    result = np.zeros(4)
    allCones = Simulator.cones()
    # for cone in allCones:




# find a point slightly infront of the car, find the cones near there, then find a an average of those cones, drive there
# find heading direction, find closest cones in that direction, then aim for exactly between them
sim.set_controller(controller)
sim.run()
sim.animate()
sim.plot()
