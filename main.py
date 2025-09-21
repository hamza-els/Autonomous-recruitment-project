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



centerTravel   = oldTraveled = time = 0
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
    

    global oldTraveled, centerTravel, currCenter, prevSpeed, time

    time += 0.01

    x[2] = (x[2] + math.pi) % (2 * math.pi) - math.pi

    if(centerTravel >= 104):
        print("Lap completed in ", time, " seconds!")

    centerTravel = centerTravel % 104

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
    
    # oldCenter = currCenter
    oldTraveled = centerTravel

    currCenter, centerTravel = closestCenter()
    nextCenter = centerline(centerTravel + max(x[3] / (2.44), 4))
    turnAngle = findAngle(np.array([x[0], x[1]]), nextCenter) 
    headingDiff = turnAngle - x[2]
    headingDiff = (headingDiff + math.pi) % (2 * math.pi) - math.pi
    steerDiff = headingDiff - x[4]
    
    turn = steerDiff * (3.5)
    turn = min(turn,  sim.steering_limits[1])
    turn = max(turn, sim.steering_limits[0])


    # print("Position: " , x[0], x[1], ", Closest point: ", currCenter, "Center Travel: ", centerTravel)

    prevSpeed = x[3]
    
    # can make a different heading diff for accel and steering
    goalVel = abs(1.5 * math.pi / (headingDiff))
    a = (goalVel - x[3]) * 24
    a = min(a, sim.ubu[0])
    a = max(a, sim.lbu[0])
    # if (x[3] > 14):
    #     a = 0
    # a = min(abs(math.pi / x[4]), abs(math.pi / headingDiff))
    # a = min(8, a)
    # if(x[3] > 12):
    #     if(a > 0):
    #         a = a * -0.2
    #     else:
    #         a = a * 1.3
    return np.array([a, turn])


sim.set_controller(controller)
sim.run(15)
sim.animate()
sim.plot()
print(sim.get_results())
