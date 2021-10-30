import statistics
import time

import sample_plotter
from joint import COXA, FEMUR, TIBIA


servoAngles = {
    COXA: (150, 210),
    FEMUR: (90, 270),
    TIBIA: (30, 210),
}

servoIndexes = {
    COXA: 0,
    FEMUR: 1,
    TIBIA: 2,
}

servoAngleDutyRelation = {
    COXA: -1,
    FEMUR: 1,
    TIBIA: -1,
}

class AngleStatisticCollector:
    def __init__(self) -> None:
        self.stack = []
        self.stackLimit = 5

    def push(self, value):
        if value is None:
            return

        self.stack.append(value)
        while len(self.stack) > self.stackLimit:
            self.stack.pop(0)

    def clear(self):
        self.stack.clear()

    def getAngleMode(self):
        if len(self.stack) < self.stackLimit:
            return None

        return statistics.mode(self.stack)

class JointCalibrationWorkflow:
    STATE_INIT = "INIT"
    STATE_WAIT_TO_START = "WAIT_TO_START"
    STATE_MOVING = "MOVING"
    STATE_COLLECTING = "COLLECTING"
    STATE_FINISHED = "FINISHED"

    DIR_FORWARDS = 0
    DIR_REVERSE = 1

    def __init__(self, joint, hexapodSerial):
        if joint not in servoAngles:
            raise Exception("Unknown joint type")

        self.hexapod = hexapodSerial

        self.joint = joint
        self.startAngle, self.endAngle = servoAngles[joint]
        self.servoIndex = servoIndexes[joint]
        self.jointMsAngleMulti = servoAngleDutyRelation[joint]
        self.stepAngle = 2

        self.startTime = time.time()

        self.state = self.STATE_INIT
        self.currentAngle = self.startAngle
        self.pauseUntil = time.time()
        self.statCollector = AngleStatisticCollector()
        self.lastMs = 1500
        self.angleMapping = {}
        self.previousDirection = None
        self.samplesOfCurrentAngle = []

        self.plotter = sample_plotter.Plotter((self.startAngle, 0, self.endAngle - self.startAngle, 2700))

        self.actionFns = {
            self.STATE_INIT: self.actionInit,
            self.STATE_MOVING: self.actionMoving,
            self.STATE_COLLECTING: self.actionCollecting,
        }

    def sendCommand(self, text):
        print("Sending command:" + text)
        self.hexapod.write((text + "\n").encode('ascii'))
        commandResponse = str(self.hexapod.readline()).rstrip()
        print(commandResponse)

    def pushSample(self, angle):
        self.statCollector.push(angle)

    def move(self, deltaMs):
        self.lastMs += deltaMs
        self.lastMs = min(2700, max(400, self.lastMs))
        self.sendCommand("jointmove {} {}".format(self.servoIndex, self.lastMs))

    def tick(self):
        if self.pauseUntil > time.time() or self.state == self.STATE_FINISHED:
            return

        actionFn = self.actionFns.get(self.state)
        pauseTime = 0
        if actionFn:
            pauseTime = actionFn()
        self.pauseUntil = time.time() + pauseTime

    def start(self):
        if self.state == self.STATE_WAIT_TO_START:
            self.state = self.STATE_MOVING

    def printMapping(self):
        print("==RAW ANGLE MAPPING==")
        print(self.angleMapping)
        print("==SAMPLE MEANS==")
        print(self.plotter.meanSamples)
        print("==END==")

        self.plotter.plot()

    def actionInit(self):
        print("Init calibration")

        if self.joint == COXA:
            # When calibrating coxa, straighten out tibia and femur
            self.sendCommand("jointmove {} {}".format(1, 1444))
            self.sendCommand("jointmove {} {}".format(2, 800))
        elif self.joint == FEMUR:
            # When calibrating femur, move tibia into safe space
            self.sendCommand("jointmove {} {}".format(2, 1800))
        elif self.joint == TIBIA:
            # When calibrating tibia, straighten out femur
            self.sendCommand("jointmove {} {}".format(1, 1644))

        # Reset servo to 1500
        self.lastMs = 1500
        self.sendCommand("jointmove {} {}".format(self.servoIndex, self.lastMs))
        self.state = self.STATE_WAIT_TO_START
        return 1

    def actionMoving(self):
        self.statCollector.clear()

        self.state = self.STATE_COLLECTING
        return 0

    def actionCollecting(self):
        angle = self.statCollector.getAngleMode()
        if angle is None:
            return 0

        angleDelta = self.currentAngle - angle
        direction = self.DIR_FORWARDS if angleDelta > 0 else self.DIR_REVERSE
        if self.previousDirection is None:
            self.previousDirection = direction

        boundaryReached = self.lastMs <= 400 or self.lastMs >= 2700
        if len(self.samplesOfCurrentAngle) >= 5 or boundaryReached:
            # Record found angle and progress
            if not boundaryReached:
                self.angleMapping[self.currentAngle] = self.samplesOfCurrentAngle[:]
                print("Recording sample mean {}deg = {}".format(self.currentAngle, round(statistics.mean((i[0] for i in self.samplesOfCurrentAngle)))))
            else:
                self.angleMapping[self.currentAngle] = None
                print("Unable to home to {}deg - skipping".format(self.currentAngle))

            self.plotter.updateData(self.angleMapping)

            self.currentAngle += self.stepAngle

            self.previousDirection = None
            self.samplesOfCurrentAngle.clear()

            if self.currentAngle > self.endAngle:
                print("Calibration finished")
                self.printMapping()

                self.state = self.STATE_FINISHED
                return 0
       
        if direction != self.previousDirection:
            self.samplesOfCurrentAngle.append((self.lastMs, round(angleDelta, 2)))
    
        # Home in on angle
        print("Homing on {}deg, current: {}, samples: {}".format(self.currentAngle, angle, len(self.samplesOfCurrentAngle)))
        msToMove = self.getMsToMove(angleDelta)
        self.move(msToMove)
        self.previousDirection = direction

        self.state = self.STATE_MOVING
        return max(0.5, min(1, 0.5 * abs(angleDelta)))

    def getMsToMove(self, angleDelta):
        ms = angleDelta * self.jointMsAngleMulti * 10
        multi = 1 if ms > 0 else -1 # Get final sign - also takes care of exact angle case

        # Bound ms between 2-200 and give it correct sign        
        return multi * int(min(200, max(2, abs(ms))))

    def getTimeElapsed(self):
        return int(time.time() - self.startTime)

    def getTimeRemaining(self):
        percentDone = (self.currentAngle - self.startAngle) / (self.endAngle - self.startAngle)
        if percentDone == 0:
            return 100000
        elapsed = self.getTimeElapsed()
        rateOfChange = elapsed / percentDone

        return int((1 - percentDone) * rateOfChange)
