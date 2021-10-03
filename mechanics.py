import numpy as np


class Components:
    def __init__(self, r_anchor=100, h_anchor=150, rope_lengths=np.array([120, 120, 120]), anchor_points=3,
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
        normal_vector = np.cross(self.platform_vertices[1] - self.platform_vertices[0],
                                      self.platform_vertices[2] - self.platform_vertices[0])
        self.unit_normal = normal_vector/np.linalg.norm(normal_vector)
        self.polar_angle = np.arccos(self.unit_normal[-1])*180./np.pi
        self.platform_sides = np.array(
            [[self.platform_vertices[i], self.platform_vertices[(i + 1) % self.tethering_points]]
             for i in range(self.tethering_points)])
        self.platform_center = np.zeros(3)

    def compute_unit_normal(self):
        normal_vector = np.cross(self.platform_vertices[1] - self.platform_vertices[0],
                                 self.platform_vertices[2] - self.platform_vertices[0])
        norm = np.linalg.norm(normal_vector)
        return normal_vector/norm

    def compute_polar_angle(self):
        nz = self.unit_normal[-1]
        angle_rad = np.arccos(nz)
        return angle_rad*180./np.pi  # in degrees

    def compute_platform_center(self):
        """
        The center of the platform is a function of the vertices and the shape. For an equilateral triangle, label the
        vertices by 1, 2, 3. The midpoint on one side is M = P1 + 1/2 (P2-P1). The center (C) is on the line MP3, with
        CM = (x/2) tan(30) = x/(2 sqrt(3)) with x being the side length of the triangle.
        """
        base = self.platform_vertices[1] - self.platform_vertices[0]  # base of triangle, vector
        x = np.linalg.norm(base)  # base length, scalar
        m = self.platform_vertices[0] + base/2  # midpoint on the base, vector
        cm = x/(2*np.sqrt(3))  # length from m to center c, scalar
        cm_dir = self.platform_vertices[2] - m  # direction to center from midpoint, vector
        cm_vec = cm_dir*cm/np.linalg.norm(cm_dir)  # make cm_dir a unit vector and multiply by the length, vector
        c = m + cm_vec  # center position, vector
        return c

    def translate_platform(self, dz):
        # we fix the x and y coordinates of the platform for now
        # when we translate the coordinates of the vertices, a couple of things change
        self.platform_vertices[:, 2] += dz
        self.platform_sides = np.array([[self.platform_vertices[i],
                                         self.platform_vertices[(i+1) % self.tethering_points]]
                                        for i in range(self.tethering_points)])
        self.platform_center = self.compute_platform_center()
        self.unit_normal = self.compute_unit_normal()
        self.polar_angle = self.compute_polar_angle()
        assert np.abs(self.polar_angle) <= self.pointing_accuracy, \
            'Pointing accuracy requirement violated, polar angle = {:.2f} deg'.format(self.polar_angle)

    def attach_platform(self):
        offsets_xy = self.anchor_vertices[:, :2] - self.platform_vertices[:, :2]  # part of rope needed to fix x and y
        rope_z = np.sqrt(self.rope_lengths**2 - (offsets_xy[:, 0]**2 + offsets_xy[:, 1]**2))  # remaining rope in z-dir
        dz = self.anchor_vertices[:, 2] - rope_z - self.platform_vertices[:, 2]  # distance between ropes and platform
        self.translate_platform(dz)  # update position of platform

    def plot_platform(self, ax, axes=np.array([0, 1])):
        ax1, ax2 = axes
        ax.scatter(self.platform_vertices.T[ax1], self.platform_vertices.T[ax2])
        ax.scatter(self.platform_center[ax1], self.platform_center[ax2])
        ax.plot(self.platform_sides.T[ax1], self.platform_sides.T[ax2], c='k')
        ax.plot([0., self.unit_normal[ax1]]+self.platform_center[ax1],
                [0., self.unit_normal[ax2]]+self.platform_center[ax2])
        if ax2 == 2:
            ax.axvline(self.platform_center[ax1], ls='--', c='k')
        ax.set_xlabel(chr(120+ax1))
        ax.set_ylabel(chr(120+ax2))

    def plot_anchor(self, ax, axes=np.array([0, 1])):
        ax1, ax2 = axes
        ax.scatter(self.anchor_vertices.T[ax1], self.anchor_vertices.T[ax2])
        for i in range(self.anchor_points):
            ropes_x = [self.platform_vertices[i][ax1], self.anchor_vertices[i][ax1]]
            ropes_y = [self.platform_vertices[i][ax2], self.anchor_vertices[i][ax2]]
            ax.plot(ropes_x, ropes_y, c='blue')

