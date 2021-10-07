import numpy as np


class Components:
    def __init__(self, r_anchor=100, h_anchor=150, rope_lengths=np.array([120, 120, 120]), anchor_points=3,
                 r_platform=3, tethering_points=3, platform_mass=50):
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
        self.r_platform = r_platform  # cylindrical r-coordinate for tethering points
        self.platform_angles = [i*2*np.pi/tethering_points + np.pi/2 for i in range(tethering_points)]  # azimuthal
        self.platform_vertices = np.array([[r_platform*np.cos(self.platform_angles[i]),
                                            r_platform*np.sin(self.platform_angles[i]),
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
        self.platform_mass = platform_mass

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


class Forces(Components):

    def gravity(self, rope_density=3.4):
        f_tot = self.platform_mass  # total force (in units of g))
        rope_length_total = np.sum(self.rope_lengths) * 2  # from anchors to platform, *2 for 2 ropes per tethering pt
        rope_length_total += np.sum(self.anchor_vertices[:, -1])  # from platform to ground (assume perfectly vertical)
        # rope_length_total += ropes from pulley to ground ...
        rope_mass = rope_length_total * rope_density
        f_tot += rope_mass
        return f_tot

    def tension_down(self, pulling_tension=10):
        return pulling_tension * self.tethering_points  # tension in each of the vertical ropes going down

    def tensions(self):
        total_force_down = self.gravity() + self.tension_down()

        # angles of ropes
        polar_angles = [np.arccos(self.platform_vertices[i, -1]/np.linalg.norm(self.platform_vertices[i]))
                        for i in range(self.tethering_points)]
        azimuthal_angles = self.platform_angles

        # balance forces (a's are polar, b's are azimuthal)
        # x-equation: T1*sin(a1)*cos(b1) + T2*sin(a2)*cos(b2) + T3*sin(a3)*cos(b3) = 0
        # y-equation: T1*sin(a1)*sin(b1) + T2*sin(a2)*sin(b2) + T3*sin(a3)*sin(b3) = 0
        # z_equation: T1*cos(a1) + T2*cos(a2) + T3*cos(a3) = total force down
        coeff_matrix = np.array([
            [np.sin(polar_angles[i])*np.cos(azimuthal_angles[i]) for i in range(self.tethering_points)],
            [np.sin(polar_angles[i])*np.sin(azimuthal_angles[i]) for i in range(self.tethering_points)],
            [np.cos(polar_angles[i]) for i in range(self.tethering_points)]
        ])
        rhs_vector = np.array([0, 0, total_force_down]).T
        solution = np.linalg.inv(coeff_matrix).dot(rhs_vector)
        t1, t2, t3 = solution  # tensions
        return t1, t2, t3
