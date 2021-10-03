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
        The center of the platform is a function of the vertices and the shape. For an equilateral triangle, label the
        vertices by 1, 2, 3. The midpoint on one side is M = P1 + 1/2 (P2-P1). The center (C) is on the line MP3, with
        CM = (x/2) tan(30) = x/(2 sqrt(3)) with x being the side length of the triangle.
        """
        if self.shape == 'triangle':
            x = self.vertices[1] - self.vertices[0]  # side length
            m = self.vertices[0] + x/2  # midpoint on the side
            cm = x/(2*np.sqrt(3))  # length from m to center c
            cm_dir = self.vertices[2] - m  # direction to center from midpoint
            cm_vec = cm_dir/np.linalg.norm(cm_dir) * cm  # make cm_dir a unit vector and multiply by the length
            c = m + cm_vec
            return c

    def translate_z(self, dz):
        # when we translate the coordinates of the vertices, a couple of things change
        self.vertices[:, 2] += dz
        self.normal_vector = np.cross(self.vertices[1]-self.vertices[0], self.vertices[2]-self.vertices[0])
        self.polar_angle = np.arccos(self.normal_vector[-1]/np.linalg.norm(self.normal_vector)) * 180/np.pi
        assert self.polar_angle <= self.pointing_accuracy, \
            'Pointing accuracy requirement violated, polar angle = {:.2f} deg'.format(self.polar_angle)
        if self.shape == 'triangle':  # draw straight lines between tethering points and define a center
            self.sides = np.array([[self.vertices[i], self.vertices[(i+1) % self.tethering_points]]
                                   for i in range(self.tethering_points)])
        c = self.compute_center()
        self.center = c

    def plot(self, axes=np.array([0, 1]), fig=None, ax=None):
        if not fig:
            fig = plt.figure()
        if not ax:
            ax = fig.add_axes([0, 0, 1, 1])
        ax1, ax2 = axes
        ax.scatter(self.vertices.T[ax1], self.vertices.T[ax2])
        ax.scatter(self.center[ax1], self.center[ax2])
        ax.plot(self.sides.T[ax1], self.sides.T[ax2], c='k')
        ax.plot([self.center[ax1], self.normal_vector[ax1]], [self.center[ax2], self.normal_vector[ax2]])
        if ax2 == 2:
            ax.axvline(0, ls='--', c='k')
        ax.set_xlabel(chr(120+ax1))
        ax.set_ylabel(chr(120+ax2))
        return fig, ax


class Anchor:
    """
    Assumes same coordinate system as Platform class
    """
    def __init__(self, r=80, h=30, rope_lengths=np.array([80, 80, 80]), anchor_points=3):

        self.angle = 2 * np.pi/anchor_points
        self.anchor_points = anchor_points
        self.vertices = np.array([[r*np.cos(i*self.angle+np.pi/2), r*np.sin(i*self.angle+np.pi/2), h]
                                  for i in range(anchor_points)])
        self.rope_lengths = rope_lengths

    def attach_platform(self, platform):
        offsets_xy = self.vertices[:, :2] - platform.vertices[:, :2]  # rope must cover this offset to fix xy-coords
        rope_z = np.sqrt(self.rope_lengths**2 - (offsets_xy[:, 0]**2 + offsets_xy[:, 1]**2))  # remaining rope in z-dir
        dz = self.vertices[:, 2] - rope_z - platform.vertices[:, 2]  # delta between end of rope and platform vertices
        platform.translate_z(dz)  # update position of platform

    def plot(self, platform, axes=np.array([0, 1])):
        ax1, ax2 = axes
        fig, ax = platform.plot(axes)
        ax.scatter(self.vertices.T[ax1], self.vertices.T[ax2])
        for i in range(self.anchor_points):
            ropesx = [platform.vertices[i][ax1], self.vertices[i][ax1]]
            ropesy = [platform.vertices[i][ax2], self.vertices[i][ax2]]
            ax.plot(ropesx, ropesy, c='blue')
        newax = fig.add_axes([1, 0.7, 0.3, 0.3])
        fig, newax = platform.plot(axes, fig, newax)
        return fig, [ax, newax]
