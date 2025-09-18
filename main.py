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
    def pointAhead(d, direction):
        """returns a point d distance at direction from point p"""
        return np.array([x[0] + d*math.cos(direction), x[1] + d*math.sin(direction)])
    
    ... # YOUR CODE HERE
    # heading = 0 is pointing directly right/ positive x axis
    # we can scale d on this function depending on the velocity of the car, 
    # that way being more careful at slower speeds
    # might need to account for steering rate
    print("Current heading is: ", x[2], " Position is: ",  x[0], x[1])
    
    turn = 0
    currCenter = centerline(x[0])
    if(currCenter[1] > x[1]):
        if(x[4] < 0.45):
            turn += 0.15
    else:
        if(x[4] > -0.45):
            turn -= 0.15

    rightPoint = pointAhead(3, x[2] - (math.pi)/4)
    leftPoint = pointAhead(3, x[2] + (math.pi)/4)
    forPoint = pointAhead(3, x[2])
    # pointsAhead = np.array([leftPoint, forPoint, rightPoint])

    rightCones = closestCones(rightPoint)
    leftCones = closestCones(leftPoint)
    forCones = closestCones(forPoint)
    conesAhead = np.array([leftCones, forCones, rightCones])

    avePoint = np.array([0.0,0.0])

    # pointRows = pointsAhead.shape[0]
    # conesRows = conesAhead.shape[0]
    for i in range(3):
        if(i == 0):
            print("left Cones: ")
        elif(i == 1):
            print("mid Cones: ")
        elif(i == 2):
            print("right Cones: ")

        for j in range(4):
            currCone = conesAhead[i][j]
            print("current cone is: " , currCone)
            avePoint[0] += currCone[0]
            avePoint[1] += currCone[1]

    avePoint[0] = avePoint[0] / 12.0
    avePoint[1] = avePoint[1] / 12.0
    print("Average point was: " , avePoint)

    a = 12
    if(x[3] > 2):
        a = -2
    return np.array([a, turn])


def closestCones(p):
    """returns the 4 closest cones to a point"""
    closestCones = np.zeros((4,2)) # variable to return
    sDists = np.array([math.inf] * 4)  #corresponding distances
    allCones = sim.cones
    for currCone in allCones:
        d = ((currCone[0] - p[0])**2 + (currCone[1] - p[1])**2) ** 0.5
        if d < max(sDists):
            coneIn = np.argmax(sDists)
            sDists[coneIn] = d
            closestCones[coneIn][0], closestCones[coneIn][1]  = currCone[0], currCone[1]
    return closestCones
                




# find a point slightly infront of the car, find the cones near there, then find a an average of those cones, drive there
# find heading direction, find closest cones in that direction, then aim for exactly between them
sim.set_controller(controller)
sim.run()
sim.animate()
sim.plot()
