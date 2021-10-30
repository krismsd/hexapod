from math import cos, radians, sin
import serial
import time

hexapod = serial.Serial(port="COM3", baudrate=9600)

# print("writing...")
# hexapod.readline()
# hexapod.write("hello\n".encode())
# print("hello written...")
# time.sleep(0.5)
# print("about to read...")
# while True:
#     reply = hexapod.readline()
#     if reply is None:
#         continue
#     reply = reply.rstrip()
#     print("got reply...")
#     print(reply.decode("ascii"))


def sendCommand(text):
    # print(text)
    # return "ack"
    hexapod.write((text + "\n").encode('ascii'))
    return hexapod.readline().decode("utf-8").rstrip()


degreesBetweenLegs = 60
legYOffsetToOrigin = 67

# cos(theta) and sin(theta) can be cached for perf
# cos(theta) is the same for both legToBody and bodyToLeg
# use -sin(theta) for legToBody, +sin(theta) for bodyToLeg

#

sinCache = [sin(radians(i * degreesBetweenLegs)) for i in range(6)]
cosCache = [cos(radians(i * degreesBetweenLegs)) for i in range(6)]

def legToBodySpace(legIdx, point):
    x, y, z = point

    costheta = cosCache[legIdx]
    sintheta = -sinCache[legIdx]
    return (
        round(x * costheta - y * sintheta, 2),
        round(y * costheta + x * sintheta, 2),
        z # will only change once we consider body roll/pitch
    )

def bodyToLegSpace(legIdx, point):
    x, y, z = point

    costheta = cosCache[legIdx]
    sintheta = sinCache[legIdx]
    return (
        round(x * costheta - y * sintheta, 2),
        round(y * costheta + x * sintheta, 2),
        z # will only change once we consider body roll/pitch
    )


import matplotlib.pyplot as plt




y = 100
z = 80
x = 0




# r = legToBodySpace(1, (x,y,z))

# ogpoints = [legToBodySpace(i, (x,y,z))[:2] for i in range(6)]

# newpoints = [(x,y+20) for (x,y) in ogpoints]

# legpoints = [bodyToLegSpace(i, (*newpoints[i], z))[:2] for i in range(6)]
# newbodypoints = [legToBodySpace(i, (*legpoints[i], z)) for i in range(6)]

# points = ogpoints + newpoints + newbodypoints


# fig = plt.figure()
# ax = fig.add_subplot(111)
# ax.set_aspect('equal')
# plt.scatter([i[0] for i in points], [i[1] for i in points], color=['red']*6+['green']*6+['purple']*6)
# plt.show()


fig = plt.figure()
ax = fig.add_subplot()
ax.set_aspect('equal')
ax.set_xbound(0, 200)
ax.set_ybound(0, 200)

plt.show(block=False)

# ax.

delta = 0
incr = 2



# 3 vars to the animation: angle of travel, speed of animation (tick speed?), individual leg travel distance (offset limit?) 
# can we abstract out the legset idea to 3-3, 4-2, 5-1 gaits; 2/3/6 legsets?







input("Press enter to prime legs...")

while True:
    delta += incr
    if delta < -20 or delta > 20:
        incr *= -1
        time.sleep(1)

    legpoints = []
    for i in range(6):
        bx, by, bz = legToBodySpace(i, (x,y,z))
        by += delta
        legspace = bodyToLegSpace(i, (bx, by, bz))
        legspace = [int(i) for i in legspace]
        legpoints += [legspace]
    

    # plotpoints = [legToBodySpace(j, legpoints[j]) for j in range(6)]
    # plotpoints += [(0,0)]

    # ax.clear()
    # ax.scatter([i[0] for i in plotpoints], [i[1] for i in plotpoints], color=['red']*6+['black'])
    # plt.draw()
    # # plt.show(block=False)
    # plt.pause(0.01)


    for i in range(len(legpoints)):
        # if i % 2 == 0:
        #     legpoints[i][2] = 30

        r = sendCommand("legik {} {} {} {}".format(i, *legpoints[i]))
        if r != "ack":
            print("Unable to move leg to position")
            break
        print(r)





    # time.sleep(0.05)
# # sendCommand("jointmove 0 1500")
# sendCommand("jointmove 1 600")
# time.sleep(1)
# # sendCommand("jointmove 1 1460")
# print(hexapod.readline().rstrip())


 


 