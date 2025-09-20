import numpy as np
import math
from simulator import Simulator, centerline

sim = Simulator()



def findAngle(p1, p2):
    return math.atan2((p2[1] - p1[1]), p2[0] - p1[0])

# def closestCones(p, k = 4):
#     """returns the k closest cones to a point"""
#     closestCones = np.zeros((k,2)) # variable to return
#     sDists = np.array([math.inf] * k)  #corresponding distances
#     allCones = sim.cones
#     for currCone in allCones:
#         d = ((currCone[0] - p[0])**2 + (currCone[1] - p[1])**2) ** 0.5
#         if d < max(sDists):
#             coneIn = np.argmax(sDists)
#             sDists[coneIn] = d
#             closestCones[coneIn][0], closestCones[coneIn][1]  = currCone[0], currCone[1]
#     return closestCones

# def projection(u, v):
#     return (np.dot(u, v) / np.dot(v, v)) * v



centerTravel   = oldTraveled = 0
currCenter = centerline(0.0)
prevSpeed = 1

def controller(x):
    """controller for a car
    Args:
        x (ndarray): numpy array of shape (5,) containing [x, y, heading, velocity, steering angle]
    """
    ... # YOUR CODE HERE
    # def pointAhead(d, direction):
    #     """returns a point d distance at direction from point p"""
    #     return np.array([x[0] + d*math.cos(direction), x[1] + d*math.sin(direction)])
    

    global oldTraveled, centerTravel, currCenter, prevSpeed

    x[2] = (x[2] + math.pi) % (2 * math.pi) - math.pi

    def closestCenter():
        closestD = math.inf
        closestP = np.array([0.0, 0.0])
        bestTravel = oldTraveled

        start = max(oldTraveled - max(prevSpeed, x[3]) / 200, 0)
        end = oldTraveled + max(prevSpeed, x[3]) / 50

        possible = np.arange(start, end, 0.005)
        
        for d in possible:
            c = centerline(d)
            distanceAway = np.hypot(c[1] - x[1], c[0] - x[0])
            if  distanceAway < closestD:
                bestTravel = d
                closestD = distanceAway
                closestP = c
        
        return closestP, bestTravel
    
    oldCenter = currCenter
    oldTraveled = centerTravel

    currCenter, centerTravel = closestCenter()
    nextCenter = centerline(centerTravel + max(x[3] / 3.25, 4))
    turnAngle = findAngle(np.array([x[0], x[1]]), nextCenter) 
    headingDiff = turnAngle - x[2]
    steerDiff = headingDiff - x[4]
    # figure out the steering sign, when car is looking left it turn opposite direction
    turn = steerDiff * (2)
    turn = min(turn,  sim.steering_limits[1])


    print("Position: " , x[0], x[1], ", Closest point: ", currCenter, "Center Travel: ", centerTravel)

    prevSpeed = x[3]
    a = 8
    if(x[3] > 2):
        a = 0
    
    
    return np.array([a, turn])


sim.set_controller(controller)
sim.run()
sim.animate()
sim.plot()
