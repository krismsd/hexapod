import matplotlib.pyplot as plt
import statistics
import numpy as np

class Plotter:
    def __init__(self, dimensions):
        self.meanSamples = []



    def updateData(self, data):
        x = []
        y = []

        mean_x = []
        mean_y = []

        colors = []

        altColor = True

        low_x = None
        low_y = None
        high_x = None
        high_y = None

        for targetAngle, samples in data.items():
            if samples is None:
                continue

            ms = list(i[0] for i in samples)
            weight = list(100000 * max(abs(i[1]), 0.001) for i in samples)

            mean = statistics.mean(ms) #np.average(ms,  weights=weight)
            # statistics.mean(weights)
            # x.append(targetAngle)
            
            if low_x is None:
                low_x = targetAngle
                low_y = mean
            high_x = targetAngle
            high_y = mean

            mean_x.append(targetAngle)
            mean_y.append(mean)

            x.append(targetAngle)
            y.append(mean)
            colors.append('black')


            for ms, angleOffset in samples:
                x.append(targetAngle - angleOffset)
                y.append(ms)
                colors.append('red' if altColor else 'green')

            altColor = not altColor

        self.rawPoints = (x, y)
        self.rawPointColor = colors
        self.means = (mean_x, mean_y)
        self.linePoints = ((low_x, high_x), (low_y, high_y))

        self.meanSamples = list(((mean_x[i], mean_y[i]) for i in range(len(mean_x))))


    def plot(self):
        self.ax = plt.axes()
        
        self.ax.scatter(*self.rawPoints, color=self.rawPointColor, s=6)
        self.ax.plot(*self.means)
        self.ax.plot(*self.linePoints)

        plt.show(block=False)
        plt.pause(0.1)
