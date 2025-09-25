import numpy as np
import math
from simulator import Simulator, centerline

sim = Simulator()


def findAngle(p1, p2):
    return math.atan2((p2[1] - p1[1]), p2[0] - p1[0])


centerTravel = oldTraveled = time = lap = 0
currCenter = centerline(0.0)
prevSpeed = 1

# STEER_PM, STEER_IM, STEER_DM = 1.0, 0.1, 0.02
# ACCEL_PM, ACCEL_IM, ACCEL_DM = 1.0, 0.5, 0.05

steerPrevErr = steerDeriv = 0.0  # filtered derivative

accelPrevErr = accelDeriv = 0.0

inTurn = turnFound = False
nextTurn = 0.0

MAX_ACCEL = 12
WB = 1.58
MAX_STEER = 0.35

goalVelOnTurn = np.sqrt((WB * (MAX_ACCEL)) / (np.tan(MAX_STEER)))

def controller(x):
    """controller for a car
    Args:
        x (ndarray): numpy array of shape (5,) containing [x, y, heading, velocity, steering angle]
    """
    ... # YOUR CODE HERE

    # Set up
    global oldTraveled, centerTravel, currCenter, prevSpeed, time, lap
    global MAX_ACCEL, WB, MAX_STEER, inTurn, nextTurn, goalVelOnTurn, turnFound

    time += 0.01
    currHeading = (x[2] + math.pi) % (2 * math.pi) - math.pi
    currVel = x[3]
    currSteer = x[4]

    if(centerTravel > 104.6):
        lap += 1
        print("Lap ", lap, " completed in ", round(time, 2), " seconds!")
        time = 0
    centerTravel = centerTravel % 104.6

    # Functions:
    def closestCenter():
        """Returns distance and point of the closest point on the centerline"""
        closestD = math.inf
        closestP = np.array([0.0, 0.0])
        bestTravel = oldTraveled

        start = max(oldTraveled - max(prevSpeed, currVel) / 200, 0)
        end = oldTraveled + max(prevSpeed, currVel) / 50

        possible = np.arange(start, end, 0.005)
        
        for d in possible:
            c = centerline(d % 104.6)
            distanceAway = np.hypot(c[1] - x[1], c[0] - x[0])
            if  distanceAway < closestD:
                bestTravel = d
                closestD = distanceAway
                closestP = c
        
        return closestP, bestTravel
    
    def steer():
        """Determines steer"""
        global steerPrevErr, steerDeriv

        nextCenter = centerline((centerTravel + max(currVel / 3.5, 6)) % 104.6)
        turnAngle = findAngle(np.array([x[0], x[1]]), nextCenter) 
        headingDiff = turnAngle - currHeading
        headingDiff = (headingDiff + math.pi) % (2 * math.pi) - math.pi
        steerDiff = headingDiff - currSteer

        rawDeriv = (steerDiff - steerPrevErr) / 0.01
        rate = 0.85
        steerDeriv = rate * steerDeriv + (1 - rate) * rawDeriv
        steerPrevErr = steerDiff

        steerIntegral = steerDiff

        turn = steerDiff * 4.5 + steerDeriv 
        turn = min(turn,  sim.ubu[1])
        turn = max(turn, sim.lbu[1])
        
        if(currSteer > MAX_STEER):
            if turn > 0:
                turn = 0
        elif(currSteer < -MAX_STEER):
            if(turn < 0):
                turn = 0

        return turn

    def findNextTurn():
        """Determines where the next turn will be"""
        global inTurn, turnFound, nextTurn
        if(not inTurn and not turnFound):
            nextTurn = centerTravel
            headingDiff = 0
            while(True):
                nextTurn += 0.1
                for d in range(4, 7):
                    nextCenter = centerline((nextTurn + d) % 104.6)
                
                    headingDiff = abs(findAngle(centerline(nextTurn % 104.6), nextCenter)\
                                    - findAngle(centerline(nextTurn % 104.6),centerline(nextTurn + 0.001) % 104.6))
                    headingDiff = (headingDiff + np.pi) % (2 * np.pi) - np.pi
                    if(headingDiff > 0.32 * np.pi):
                        turnFound = True
                        return nextTurn % 104.6
        return nextTurn
    
    def brake():
        # Need to account for modulo somehow
        """Determines if it is time to brake or not"""
        dNeeded = (goalVelOnTurn ** 2 - currVel ** 2) / (-8)
        if(dNeeded >= ((nextTurn - centerTravel) + 104.6) % 104.6):
            return True
        else:
            return False

    def accel():
        """Determines acceleration"""
        global accelPrevErr, accelDeriv
        if(inTurn):
            # if(centerTravel > nextTurn + 2 and currSteer < 0.15):
            #     a = MAX_ACCEL - normalAccel
            #     # can make a for loop so its not forever
            #     # while(np.sqrt(a ** 2 + normalAccel ** 2) > MAX_ACCEL):
            #     #     a -= 0.1

            #     return a
            # else:
            return 0
        elif(brake()):
            return -4
        elif(not brake()):
            return MAX_ACCEL - normalAccel

    
    # Body:
    oldTraveled = centerTravel
    currCenter, centerTravel = closestCenter() 
    nextTurn = findNextTurn()

    steering = steer()
    
    if(centerTravel > nextTurn + 104.6 * lap):
        inTurn = True

    if(centerTravel > nextTurn + 2 and abs(currSteer) < 0.17):
        inTurn = False
        turnFound = False
        nextTurn = findNextTurn()
        
    normalAccel = abs((currVel ** 2) * np.tan(currSteer) / 1.58)
    
    a = accel()
    
    prevSpeed = currVel
    return np.array([a, steering])


sim.set_controller(controller)
sim.run(25)
# print(sim.get_results())
sim.animate()
sim.plot()

