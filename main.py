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

inTurn = turnFound = inBrake = False
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
    global oldTraveled, centerTravel, currCenter, prevSpeed, time, lap, inBrake
    global MAX_ACCEL, WB, MAX_STEER, inTurn, nextTurn, goalVelOnTurn, turnFound

    time += 0.01
    currHeading = (x[2] + math.pi) % (2 * math.pi) - math.pi
    currVel = x[3]
    currSteer = x[4]
    currPos = [x[0], x[1]]

    if(centerTravel > 104.6):
        lap += 1
        print("Lap ", lap, " completed in ", round(time, 2), " seconds!")
        time = 0
        inTurn = False
        inBrake = False
    centerTravel = centerTravel % 104.6

    # Functions:
    def closestCenter():
        """Returns distance and point of the closest point on the centerline"""
        closestD = math.inf
        closestP = np.array([0.0, 0.0])
        bestTravel = oldTraveled

        
        start = max(oldTraveled - max(prevSpeed, currVel) / 200, 0)
        end = oldTraveled + max(prevSpeed, currVel) / 50

        if not (np.isfinite(start) and np.isfinite(end)):
            return centerline(oldTraveled), oldTraveled
        
        possible = np.arange(start, end, 0.005)
        
        for d in possible:
            c = centerline(d % 104.6)
            distanceAway = np.hypot(c[1] - currPos[1], c[0] - currPos[0])
            if  distanceAway < closestD:
                bestTravel = d
                closestD = distanceAway
                closestP = c
        
        return closestP, bestTravel
    
    def steer():
        """Determines steer"""
        global steerPrevErr, steerDeriv

        nextCenter = centerline((centerTravel + max(currVel / 3.5, 6)) % 104.6)
        turnAngle = findAngle(np.array([currPos[0], currPos[1]]), nextCenter) 
        headingDiff = turnAngle - currHeading
        headingDiff = (headingDiff + math.pi) % (2 * math.pi) - math.pi
        steerDiff = headingDiff - currSteer

        rawDeriv = (steerDiff - steerPrevErr) / 0.01
        rate = 0.85
        steerDeriv = rate * steerDeriv + (1 - rate) * rawDeriv
        steerPrevErr = steerDiff

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
                nextTurn += 0.01
                nextTurn = nextTurn % 104.6
                for d in range(3, 8):
                    nextCenter = centerline((nextTurn + d) % 104.6)

                    headingDiff = abs(findAngle(centerline(nextTurn % 104.6), nextCenter)\
                        - findAngle(centerline(nextTurn % 104.6),centerline(nextTurn + 0.005) % 104.6))
                    headingDiff = (headingDiff + np.pi) % (2 * np.pi) - np.pi
                    if(headingDiff > 0.3 * np.pi):
                        turnFound = True
                        return nextTurn
        return nextTurn
    
    def brake():
        global inBrake
        # Need to account for modulo somehow
        """Determines if it is time to brake or not"""
        nextTurnPoint = centerline(nextTurn)
        if(not inBrake):
            inBrake = dNeeded >= abs(np.hypot(nextTurnPoint[0] - currPos[0],\
             nextTurnPoint[1] - currPos[1]))
            return inBrake
        else:
            return inBrake
        # break permanently, no bouncing, can be implemented by finding if we finished the
        # turn we braked for

    def accel():
        """Determines acceleration"""
        global accelPrevErr, accelDeriv
        if(inTurn):
            # return -np.sqrt(abs(MAX_ACCEL ** 2 - normalAccel ** 2))
            goalVel = np.sqrt(WB * (MAX_ACCEL)) / (np.tan(min(abs(currSteer) * 1.16, MAX_STEER)))
            # accelDeriv = ((goalVelOnTurn - currVel) - accelPrevErr) / 0.01
            # accelPrevErr = goalVelOnTurn - currVel
            return (goalVel- currVel) * 0.2 + accelDeriv * 0.5
            # return 0
        elif(brake()):
            return -10
        else:
            # nextTurnPoint = centerline(nextTurn)

            # A = 1.0 / (2.0 * 4)
            # B = 1.0 / (2.0 * 10)

            # v_peak_sq = abs(abs(np.hypot(nextTurnPoint[0] - currPos[0],\
            #  nextTurnPoint[1] - currPos[1]) - dNeeded) \
            #     + A * currVel**2 + B * goalVelOnTurn**2) / (A + B)
            # goalVel = math.sqrt(v_peak_sq) + 1
            
            # accelDeriv = ((goalVel - currVel) - accelPrevErr) / 0.01

            # accelPrevErr = goalVel - currVel

            return  np.sqrt(MAX_ACCEL ** 2 - normalAccel ** 2)

    
    # Body:
    dNeeded = (goalVelOnTurn ** 2 - currVel ** 2) / (-20)
    oldTraveled = centerTravel
    currCenter, centerTravel = closestCenter() 
    nextTurn = findNextTurn()

    steering = steer()
    
    if(centerTravel > nextTurn ):
        inTurn = True

    if(centerTravel > nextTurn + 2 and abs(currSteer) < 0.15):
        nextTurn = np.inf
        inTurn = False
        turnFound = False
        inBrake = False
        
    normalAccel = abs((currVel ** 2) * np.tan(currSteer) / 1.58)
    
    a = accel()
    
    prevSpeed = currVel
    return np.array([a, steering])


sim.set_controller(controller)
sim.run(15)
# print(sim.get_results())
sim.animate()
sim.plot()

