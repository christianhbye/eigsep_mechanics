import matplotlib.pyplot as plt
import numpy as np


class Mechanics:
    def __init__(self, center=np.zeros(3), tethering_points=3, shape='triangle'):
        self.pointing_accuracy = 5 # degrees
        self.center = center
        self.tethering_points = tethering_points
        self.shape = shape
        self.angle = 2*np.pi/tethering_points  # equal spacing


class Platform(Mechanics):
    def __init__(self, center_dist=3, center=np.zeros(3), tethering_points=3, shape='triangle'):
        """

        :param center_dist: float, length from center to each tethering point along the plane of platform
        :param center: coordinates of center of platform
        """
        super().__init__(center, tethering_points, shape)
        self.center_dist = center_dist
        self.vertices = self.center_dist*np.array([[np.cos(i*self.angle), np.sin(i*self.angle), 0]
                                                   for i in range(tethering_points)])
        if self.shape == 'triangle':  # draw straight lines between tethering points
            self.sides = np.array([[self.vertices[i], self.vertices[(i+1) % tethering_points]] for i in range(tethering_points)])

        self.normal_vector = np.cross(self.vertices[1]-self.vertices[0], self.vertices[2]-self.vertices[0])
        self.polar_angle = np.arccos(self.normal_vector[-1]/np.linalg.norm(self.normal_vector)) * 180/np.pi

    def plot(self, axes=np.array([0, 1])):
        fig = plt.figure()
        ax = fig.add_axes([0, 0, 1, 1])
        ax1, ax2 = axes
        ax.scatter(self.vertices.T[ax1], self.vertices.T[ax2])
        ax.plot(self.sides.T[ax1], self.sides.T[ax2], c='k')
        return fig, ax
#        plt.show()


class Anchor(Mechanics):
    def __init__(self, r=80, h=30, ropes=90, center=np.zeros(3), tethering_points=3, shape='triangle'):
        super().__init__(center, tethering_points, shape)
        self.vertices = np.array([[r*np.cos(i*ang), r*np.sin(i*ang), h] for i in range(tethering_points)]) + center
        #self.ropes = ropes * np.ones(self.vertices.shape[0])

        self.sides = np.array([[self.vertices[i], self.vertices[(i + 1) % tethering_points]]
                               for i in range(tethering_points)])

    #    def attach_platform(self, platform):
#        offsets = self.vertices - platform.vertices
#        z = self.vertices[:, 2] - np.sqrt(self.ropes**2 - (offsets[:, 0]**2 + offsets[:, 1]**2))
    def plot(self, ax, axes=np.array([0, 1])):
        ax1, ax2 = axes
        ax.plot(self.sides.T[ax1], self.sides.T[ax2], c='k')
        return ax
