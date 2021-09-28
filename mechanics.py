import matplotlib.pyplot as plt
import numpy as np


class Platform:
    def __init__(self, tethering_points=3, center_dist=3, center=np.array([0., 0., 0.]), shape='triangle'):
        """

        :param tethering_points: int, number of tethering points
        :param center_dist: float, length from center to each tethering point along the plane of platform
        :param center: coordinates of center of platform
        """
        self.angle = 2*np.pi/3
        self.pointing_accuracy = 5 # degrees
        self.center = center
        self.vertices = center_dist*np.array([[np.cos(i*self.angle), np.sin(i*self.angle), 0]
                                              for i in range(tethering_points)])
        if shape == 'triangle':  # draw straight lines between tethering points
            self.sides = np.array([[self.vertices[i], self.vertices[(i+1) % tethering_points]] for i in range(tethering_points)])

        self.normal_vector = np.cross(self.vertices[1]-self.vertices[0], self.vertices[2]-self.vertices[0])
        self.polar_angle = np.arccos(self.normal_vector[-1]/np.linalg.norm(self.normal_vector)) * 180/np.pi

    def plot(self, axes=[0, 1]):
        fig = plt.figure()
        ax = fig.add_axes([0, 0, 1, 1])
        ax1, ax2 = axes
        ax.scatter(self.vertices.T[ax1], self.vertices.T[ax2])
        ax.plot(self.sides.T[ax1], self.sides.T[ax2], c='k')
        plt.show()
