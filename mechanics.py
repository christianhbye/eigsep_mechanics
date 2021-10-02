import matplotlib.pyplot as plt
import numpy as np


class Platform:
    """
    Coordinate system has origin at the initial (before attached to the anchors) center of the platform,
    xy-plane is parallel to the ground and to the platform initially
    """
    def __init__(self, r=3, center=np.zeros(3), tethering_points=3, shape='triangle'):
        # quantities that don't change
        self.pointing_accuracy = 5  # degrees
        self.tethering_points = tethering_points
        self.shape = shape
        self.az_angle = 2 * np.pi / tethering_points

        # variables that depend on the positioning of the the tethering points (vertices)
        self.center = center  # coordinates of the center of the platform
        self.vertices = np.array([[r*np.cos(i*self.az_angle+np.pi/2), r*np.sin(i*self.az_angle+np.pi/2), 0]
                                  for i in range(tethering_points)]) + self.center
        if shape == 'triangle':  # draw straight lines between tethering points
            self.sides = np.array([[self.vertices[i], self.vertices[(i+1) % tethering_points]]
                                   for i in range(tethering_points)])

        self.normal_vector = np.cross(self.vertices[1]-self.vertices[0], self.vertices[2]-self.vertices[0])
        self.polar_angle = np.arccos(self.normal_vector[-1]/np.linalg.norm(self.normal_vector)) * 180/np.pi

    def compute_center(self):
        """
        The center of the platform is a function of the vertices and the shape
        """
        

    def translate_z(self, dz):
        # when we translate the coordinates of the vertices, a couple of things change
        self.vertices[:, 2] += dz
        self.normal_vector = np.cross(self.vertices[1]-self.vertices[0], self.vertices[2]-self.vertices[0])
        self.polar_angle = np.arccos(self.normal_vector[-1]/np.linalg.norm(self.normal_vector)) * 180/np.pi
        if self.shape == 'triangle':  # draw straight lines between tethering points and define a center
            self.sides = np.array([[self.vertices[i], self.vertices[(i+1) % self.tethering_points]]
                                   for i in range(self.tethering_points)])




    def plot(self, axes=np.array([0, 1])):
        fig = plt.figure()
        ax = fig.add_axes([0, 0, 1, 1])
        ax1, ax2 = axes
        ax.scatter(self.vertices.T[ax1], self.vertices.T[ax2])
        ax.scatter(self.center[ax1], self.center[ax2])
        ax.plot(self.sides.T[ax1], self.sides.T[ax2], c='k')
        ax.plot([self.center[ax1], self.normal_vector[ax1]], [self.center[ax2], self.normal_vector[ax2]])
        if ax2 == 2:
            ax.axvline(self.center[ax1], ls='--', c='k')
        plt.show()


class Anchor:
    """
    Assumes same coordinate system as Platform class
    """
    def __init__(self, r=80, h=30, rope_length=90, anchor_points=3):

        self.angle = 2 * np.pi/anchor_points
        self.vertices = np.array([[r*np.cos(i*self.angle), r*np.sin(i*self.angle), h]
                                  for i in range(anchor_points)])
        self.ropes = rope_length * np.ones(self.vertices.shape[0])

    def attach_platform(self, platform):
        offsets = self.vertices - platform.vertices
        dz = - np.sqrt(self.ropes**2 - (offsets[:, 0]**2 + offsets[:, 1]**2))  # assumes fixed x and y coordinates
        z = self.vertices[:, 2] + dz

    def plot(self, ax, axes=np.array([0, 1])):
        ax1, ax2 = axes
        return ax
