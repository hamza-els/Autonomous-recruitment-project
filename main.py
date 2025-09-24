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

inTurn = False
nextTurn = 0.0

MAX_ACCEL = 12

def controller(x):
    """controller for a car
    Args:
        x (ndarray): numpy array of shape (5,) containing [x, y, heading, velocity, steering angle]
    """
    ... # YOUR CODE HERE
    global oldTraveled, centerTravel, currCenter, prevSpeed, time, lap
    global MAX_ACCEL, inTurn, nextTurn

    time += 0.01

    x[2] = (x[2] + math.pi) % (2 * math.pi) - math.pi
    
    if(centerTravel > 104.6):
        lap += 1
        print("Lap ", lap, " completed in ", round(time, 2), " seconds!")
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

        nextCenter = centerline((centerTravel + max(x[3] / 3.5, 6)) % 104.6)
        turnAngle = findAngle(np.array([x[0], x[1]]), nextCenter) 
        headingDiff = turnAngle - x[2]
        headingDiff = (headingDiff + math.pi) % (2 * math.pi) - math.pi
        steerDiff = headingDiff - x[4]

        rawDeriv = (steerDiff - steerPrevErr) / 0.01
        rate = 0.85
        steerDeriv = rate * steerDeriv + (1 - rate) * rawDeriv
        steerPrevErr = steerDiff

        steerIntegral = steerDiff

        turn = steerDiff * 4.5 + steerDeriv 
        turn = min(turn,  0.7)
        turn = max(turn, -0.7)
        
        if(x[4] > 0.32):
            if turn > 0:
                turn = 0
        elif(x[4] < -0.32):
            if(turn < 0):
                turn = 0

        return turn

    steering = steer()
    # print("Position: " , x[0], x[1], ", Closest point: ", currCenter, "Center Travel: ", centerTravel)
    
    # can make a different heading diff for accel and steering

    normalAccel = ((x[3] ** 2) * np.tan(x[4])) / 1.58
    totalAccel = np.sqrt(a ** 2 + normalAccel ** 2)

    a = np.sqrt(MAX_ACCEL ** 2 - normalAccel ** 2)
    
    def accel():
        global accelIntegral, accelPrevErr, accelDeriv
        # Detect there is a turn, make it so that the goal velocity is to survive that turn with
        # normal accel < 14, can calculate 
        
        if(centerTravel > nextTurn):
            inTurn = True
        
        
        
        nextTurn = centerTravel
        if(not inTurn):
            while(headingDiff < 0.15 * np.pi):
                nextTurn += 0.1
                nextCenter = centerline((nextTurn + 5) % 104.6)
                turnAngle = findAngle(centerline(nextTurn), nextCenter)
                nextTurnHeading = findAngle(centerline(nextTurn), centerline(nextTurn + 0.01))
                headingDiff = abs(turnAngle - nextTurnHeading)
                headingDiff = (headingDiff + math.pi) % (2 * math.pi) - math.pi
        

        if(abs(headingDiff) > 0.1 * math.pi):
            goalVel = 0.1 * ((abs((MAX_ACCEL * 1.58) / np.tan(0.32))) ** 0.5)
        else:
            goalVel = min(abs(4 * math.pi / (headingDiff)), 14)
        

        velDiff = (goalVel - x[3])

        rawDeriv = (velDiff - accelPrevErr) / 0.01
        rate = 0.5
        accelDeriv = rate * accelDeriv + (1 - rate) * rawDeriv
        accelPrevErr = velDiff
        
        a = velDiff * 10 + accelDeriv 

        # if a > 0:
        #     a = min(a,  (MAX_ACCEL - normalAccel) ** 0.5)
        # elif(a < 0):
        #     a = max(a, - (MAX_ACCEL - normalAccel) ** 0.5)
        # a = max(a, sim.lbu[0])
        if(abs(steering) > 0.2):
            return 
        

        
        return a
         
    
    a = accel()


    prevSpeed = x[3]
    return np.array([a, steering])


sim.set_controller(controller)
sim.run(30)
# print(sim.get_results())
sim.animate()
sim.plot()

