    # # def pointAhead(d, direction):
    #     """returns a point d distance at direction from point p"""
    #     return np.array([x[0] + d*math.cos(direction), x[1] + d*math.sin(direction)])
    # 
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
    # 
    # 
    # 
    # currCenter = centerline(x[0])
    # if(currCenter[1] > x[1]):
    #     if(x[4] < 0.45):
    #         turn += 0.15
    # else:
    #     if(x[4] > -0.45):
    #         turn -= 0.15

    # rightPoint = pointAhead(7, x[2] - (math.pi)/3.7)
    # leftPoint = pointAhead(7, x[2] + (math.pi)/3.7)
    # # forPoint = pointAhead(3, x[2])
    # # pointsAhead = np.array([leftPoint, forPoint, rightPoint])

    # rightCones = closestCones(rightPoint, 7)
    # leftCones = closestCones(leftPoint, 7)
    # # forCones = closestCones(forPoint)
    # conesAhead = np.array([leftCones, rightCones])

    # avePoint = np.array([0.0,0.0])

    # # pointRows = pointsAhead.shape[0]
    # conesRows = conesAhead.shape[0]
    # for i in range(2):
    #     # if(i == 0):
    #     #     print("left Cones: ")
    #     # elif(i == 1):
    #     #     print("mid Cones: ")
    #     # elif(i == 2):
    #     #     print("right Cones: ")

    #     for j in range(conesRows):
    #         currCone = conesAhead[i][j]
    #         # print("current cone is: " , currCone)
    #         avePoint[0] += currCone[0]
    #         avePoint[1] += currCone[1]

    # avePoint[0] = avePoint[0] / 2 * conesRows
    # avePoint[1] = avePoint[1] / 2 * conesRows
    # # print("Average point was: " , avePoint)
    # angleToAve = findAngle(np.array([x[0], x[1]]), avePoint)

    # if (x[2] > 2 * math.pi):
    #     x[2] = x[2] % (2 * math.pi)
    # elif(x[2] < -(2 * math.pi)):
    #     x[2] = -(abs(x[2]) % (2 * math.pi))

    # if((x[2] + x[4]) != (angleToAve)):
    #     turn =  angleToAve - (x[2] + x[4]) * 0.3
    # # if(turn > 0.4):
    # #     turn = 0.4
    # # elif(turn < -0.4):
    # #     turn = -0.4

    # # print("Heading: " , x[2], ", Steering: " , x[4], ", Angle to Ave: ", angleToAve, ", Turn: ", turn)

