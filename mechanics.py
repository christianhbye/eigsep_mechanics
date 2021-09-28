import matplotlib.pyplot as plt
import numpy as np


class Platform:
    def __init__(self, tethering_points=3, center_dist=3, center=np.array([0., 0., 0.])):
        """

        :param tethering_points: int, number of tethering points
        :param center_dist: float, length from center to each tethering point along the plane of platform
        :param center: coordinates of center of platform
        """
        self.angle = 2*np.pi/3
        self.pointing_accuracy = 5 * np.pi/180  # radians
        self.center = center
        self.vertices = center_dist*np.array([[np.cos(i*self.angle), np.sin(i*self.angle), 0]
                                              for i in range(tethering_points)])

    def plot(self, projection='xy'):
        fig = plt.figure()
        ax1 = fig.add_axes([0, 0, 1, 1])
        ax1.plot(self.vertices[:, :, 0])
        plt.show()

