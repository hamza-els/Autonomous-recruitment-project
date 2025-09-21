import numpy as np
import math
from simulator import Simulator, centerline

sim = Simulator()


def findAngle(p1, p2):
    return math.atan2((p2[1] - p1[1]), p2[0] - p1[0])


centerTravel   = oldTraveled = time = lap = 0
currCenter = centerline(0.0)
prevSpeed = 1

# STEER_PM, STEER_IM, STEER_DM = 1.0, 0.1, 0.02
# ACCEL_PM, ACCEL_IM, ACCEL_DM = 1.0, 0.5, 0.05

steerIntegral = steerPrevErr = steerDeriv = 0.0  # filtered derivative

accelIntegral = accelPrevErr = accelDeriv = 0.0

def controller(x):
    """controller for a car
    Args:
        x (ndarray): numpy array of shape (5,) containing [x, y, heading, velocity, steering angle]
    """
    ... # YOUR CODE HERE
    global oldTraveled, centerTravel, currCenter, prevSpeed, time, lap

    time += 0.01

    x[2] = (x[2] + math.pi) % (2 * math.pi) - math.pi
    
    if(centerTravel > 104.6):
        lap += 1
        print("Lap ", lap, " completed in ", time, " seconds!")
        time = 0

    centerTravel = centerTravel % 104.6

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
    
    def steer():
        global steerIntegral, steerPrevErr, steerDeriv

        nextCenter = centerline((centerTravel + max(x[3] / 3.65, 6.5)) % 104.6)
        turnAngle = findAngle(np.array([x[0], x[1]]), nextCenter) 
        headingDiff = turnAngle - x[2]
        headingDiff = (headingDiff + math.pi) % (2 * math.pi) - math.pi
        steerDiff = headingDiff - x[4]

        steerDeriv = (steerDiff - steerPrevErr) / 0.01

        steerPrevErr = steerDiff

        turn = steerDiff * 5.5 + steerDeriv * 0.9
        turn = min(turn,  sim.ubu[1])
        turn = max(turn, sim.lbu[1])

        return turn

    steering = steer()
    # print("Position: " , x[0], x[1], ", Closest point: ", currCenter, "Center Travel: ", centerTravel)
    
    # can make a different heading diff for accel and steering
    def accel():
        global accelIntegral, accelPrevErr, accelDeriv

        # nextCenter = centerline((centerTravel + max(x[3]/1.2, 6.5)) % 104.6)
        # turnAngle = findAngle(np.array([x[0], x[1]]), nextCenter) 
        # headingDiff = turnAngle - x[2]
        # headingDiff = (headingDiff + math.pi) % (2 * math.pi) - math.pi

        
        # goalVel = abs(20 * math.pi / (headingDiff))
        # velDiff = (goalVel - x[3])

        # accelDeriv = (velDiff - accelPrevErr) / 0.01
        
        # accelPrevErr = velDiff
        
        # a = velDiff * math.inf + accelDeriv * 0.4
        # # if(a > 0):
        # #     a = a * 24
        # # else:
        # #     a = a * 10
        # a = min(a, sim.ubu[0])
        # a = max(a, sim.lbu[0])

        a = 12
        return a
    
    a = accel()

    prevSpeed = x[3]
    return np.array([a, steering])


sim.set_controller(controller)
sim.run(30)
# print(sim.get_results())
sim.animate()
sim.plot()

