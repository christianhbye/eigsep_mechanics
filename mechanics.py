import numpy as np


class Components:
    def __init__(self, r_anchor=100, h_anchor=150, rope_lengths=np.array([80, 80, 80]), anchor_points=3,
                 r_platform=3, tethering_points=3):
        self.pointing_accuracy = 5.  # degrees
        # anchor parameters
        self.r_anchor = r_anchor  # cylindrical r-coordinate for anchors
        self.h_anchor = h_anchor  # z-coordinate for anchors
        self.rope_lengths = rope_lengths  # lengths of ropes
        self.anchor_points = anchor_points  # number of anchors
        self.anchor_angle = 2*np.pi/anchor_points  # azimuth-coordinate for anchors
        self.anchor_vertices = np.array([[r_anchor*np.cos(i*self.anchor_angle + np.pi/2),
                                          r_anchor*np.sin(i*self.anchor_angle + np.pi/2),
                                          h_anchor]
                                         for i in range(anchor_points)])
        # platform parameters
        self.tethering_points = tethering_points  # number of tethering points
        self.platform_angle = 2*np.pi/tethering_points  # azimuth-coordinate for tethering points
        self.r_platform = r_platform  # cylindrical r-coordinate for tethering points
        self.platform_vertices = np.array([[r_platform*np.cos(i*self.platform_angle + np.pi/2),
                                            r_platform*np.sin(i*self.platform_angle + np.pi/2),
                                            0]
                                           for i in range(tethering_points)])  # initialized on the ground (z=0)
        self.normal_vector = np.cross(self.platform_vertices[1] - self.platform_vertices[0],
                                      self.platform_vertices[2] - self.platform_vertices[0])
        self.polar_angle = np.arccos(self.normal_vector[-1]/np.linalg.norm(self.normal_vector))*180./np.pi
        self.platform_sides = np.array(
            [[self.platform_vertices[i], self.platform_vertices[(i + 1) % self.tethering_points]]
             for i in range(self.tethering_points)])

    def translate_platform(self, dz):
        # we fix the x and y coordinates of the platform for now
        # when we translate the coordinates of the vertices, a couple of things change
        self.platform_vertices[:, 2] += dz
        self.normal_vector = np.cross(self.platform_vertices[1] - self.platform_vertices[0],
                                      self.platform_vertices[2] - self.platform_vertices[0])
        self.polar_angle = np.arccos(self.normal_vector[-1] / np.linalg.norm(self.normal_vector)) * 180. / np.pi
        assert self.polar_angle <= self.pointing_accuracy, \
            'Pointing accuracy requirement violated, polar angle = {:.2f} deg'.format(self.polar_angle)
        self.platform_sides = np.array([[self.platform_vertices[i], self.platform_vertices[(i+1)%self.tethering_points]]
                                        for i in range(self.tethering_points)])

    def attach_platform(self):
        offsets_xy = self.anchor_vertices[:, :2] - self.platform_vertices[:, :2]  # part of rope needed to fix x and y
        rope_z = np.sqrt(self.rope_lengths**2 - (offsets_xy[:, 0]**2 + offsets_xy[:, 1]**2))  # remaining rope in z-dir
        dz = self.anchor_vertices[:, 2] - rope_z - self.platform_vertices[:, 2]  # distance between ropes and platform
        self.translate_platform(dz)  # update position of platform

    def plot_platform(self, ax, axes=np.array([0, 1])):
        ax1, ax2 = axes
        ax.scatter(self.platform_vertices.T[ax1], self.platform_vertices.T[ax2])
        ax.plot(self.platform_sides.T[ax1], self.platform_sides.T[ax2], c='k')
        ax.plot([0, self.normal_vector[ax1]], [0, self.normal_vector[ax2]])
        if ax2 == 2:
            ax.axvline(0, ls='--', c='k')
        ax.set_xlabel(chr(120+ax1))
        ax.set_ylabel(chr(120+ax2))

    def plot_anchor(self, ax, axes=np.array([0, 1])):
        ax1, ax2 = axes
        ax.scatter(self.anchor_vertices.T[ax1], self.anchor_vertices.T[ax2])
        for i in range(self.anchor_points):
            ropes_x = [self.platform_vertices[i][ax1], self.anchor_vertices[i][ax1]]
            ropes_y = [self.platform_vertices[i][ax2], self.anchor_vertices[i][ax2]]
            ax.plot(ropes_x, ropes_y, c='blue')

