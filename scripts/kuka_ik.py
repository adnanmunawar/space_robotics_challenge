# Credits https://github.com/elhussieny/iiwa_ik
from PyKDL import Rotation, Vector, Frame
import math


class KUKA_IK:
    def __init__(self):

        # Set some DH params
        self.d0 = 0.312
        self.d3 = 0.39600
        self.d5 = 0.4059
        self.d7 = 0.0499

        self.joint_limits = [0.0]*7
        self.joint_limits[0] = math.degrees(170.0)
        self.joint_limits[1] = math.degrees(120.0)
        self.joint_limits[2] = math.degrees(170.0)
        self.joint_limits[3] = math.degrees(170.0)
        self.joint_limits[4] = math.degrees(170.0)
        self.joint_limits[5] = math.degrees(120.0)
        self.joint_limits[6] = math.degrees(175.0)

    def compute_ik(self, pos, rot):
        P_7_0 = Vector(pos[0], pos[1], pos[2])
        R_7_0 = None
        if len(rot) == 3:
            # Consider RPY rotation
            R_7_0 = Rotation.RPY(rot[0], rot[1], rot[2])
        elif len(rot) == 4:
            # Consider Quaternion rotation, x, y, z, w
            R_7_0 = Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3])
        else:
            print("ERROR, SPEICFY ROTATION AS A LIST OF RPY (r, p, y) OR QUATERINION (x, y, z, w) ")
            return

        valid_ik = True
        joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        P_6_0 = P_7_0 - self.d7 * R_7_0.UnitZ()
        v = Vector()

        joint_pos[0] = math.atan2(P_6_0[1], P_6_0[0])

        L2_6_0 = P_6_0.Norm() * P_6_0.Norm()
        c3 = (L2_6_0 - self.d3 * self.d3 - self.d5 * self.d5) / (2.0 * self.d3 * self.d5)

        if c3 > 1.0:
            print("IK INVALID AS C3 > 1.0", c3)
            valid_ik = False
            return valid_ik, [0]*7

        s3 = -math.sqrt(1 - c3*c3)

        joint_pos[3] = math.atan2(s3, c3) + math.pi / 2.0

        joint_pos[2] = 0.0

        L_6_0_xy = P_6_0
        L_6_0_xy[3] = 0
        L2_6_0_xy = L_6_0_xy.Norm() * L_6_0_xy.Norm()

        s2 = ((self.d3 + self.d5 * c3) * P_6_0[2] - self.d5 * s3 * L2_6_0_xy) / L2_6_0
        c2 = ((self.d3 + self.d5 * c3) * L2_6_0_xy + self.d5 * s3 * P_6_0[2]) / L2_6_0

        joint_pos[1] = math.atan2(s2, c2)

        cosj0 = math.cos(joint_pos[0])
        sinj0 = math.sin(joint_pos[0])
        cosj1 = math.cos(joint_pos[1])
        sinj1 = math.sin(joint_pos[1])
        cosj3 = math.cos(joint_pos[3])
        sinj3 = math.sin(joint_pos[3])

        R_1_0 = Rotation(cosj0, 0.0, sinj0, sinj0, 0.0, -cosj0, 0.0, 1.0, 0.0)
        R_2_1 = Rotation(cosj1, -sinj1, 0.0, sinj1, cosj1, 0.0, 0.0, 0.0, 1.0)
        R_3_2 = Rotation(cosj3, 0.0, sinj3, sinj3, 0.0, -cosj3, 0.0, 1.0, 0.0)

        R_3_0 = R_1_0 * R_2_1 * R_3_2
        R_6_3 = R_3_0.Inverse() * R_7_0

        joint_pos[4] = math.atan2(R_6_3.UnitY()[2], R_6_3.UnitX()[2])
        joint_pos[5] = math.atan2(math.sqrt(R_6_3.UnitX()[2] * R_6_3.UnitX()[2] + R_6_3.UnitY()[2] * R_6_3.UnitY()[2]),
                                  R_6_3.UnitZ()[2])
        joint_pos[6] = math.atan2(R_6_3.UnitZ()[1], R_6_3.UnitZ()[0])

        if joint_pos[1] < -math.pi / 2.0:
            joint_pos[1] = joint_pos[1] + 3.0 * math.pi / 2.0
        else:
            joint_pos[1] = joint_pos[1] - math.pi / 2.0

        if joint_pos[3] < -math.pi / 2.0:
            joint_pos[3] = joint_pos[3] + 3.0 * math.pi / 2.0
        else:
            joint_pos[3] = joint_pos[3] - math.pi / 2.0

        if joint_pos[6] < 0.0:
            joint_pos[6] = joint_pos[6] + math.pi
        else:
            joint_pos[6] = -math.pi

        for i in range(7):
            if abs(joint_pos[i]) > self.joint_limits[i]:
                print ("IK SOLUTION OUT OF BOUNDS FOR JOINT ", i)
                valid_ik = False

        if not valid_ik:
            return valid_ik, [0.0]*7
        else:
            return valid_ik, joint_pos




