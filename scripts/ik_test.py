from kuka_ik import KUKA_IK
from ambf_client import Client
import time
import rospy
from sensor_msgs.msg import Joy
from PyKDL import Vector, Rotation

msg = Joy()
active = False

def joy_cb(msg0):
    global msg, active
    msg = msg0
    active = True

ik = KUKA_IK()

c = Client()
c.connect()
scale = 0.0005
b = c.get_obj_handle('link7')
m = c.get_obj_handle('Marker')
time.sleep(0.5)


sub = rospy.Subscriber('/joy', Joy, joy_cb, queue_size=5)

pos = Vector(0.1, 0.1, 0.8)
rpy = Vector(0.0, 0.0, 0.0)
cmd_jnt_pos = [0.0]*7

while not rospy.is_shutdown():
    if active:
        if msg.buttons[6]:
            rpy = rpy + Vector(msg.axes[0], msg.axes[1], msg.axes[2]) * scale
        else:
            pos = pos + Vector(-msg.axes[2], -msg.axes[0], -msg.axes[1]) * scale

        # valid, jnt_pos = ik.compute_ik([1.0, 0.0, 0.0], [0, 0, 0])
        # if valid:
        #     cmd_jnt_pos = jnt_pos
        b.set_pos(pos[0], pos[1], pos[2])
        b.set_rpy(rpy[0], rpy[1], rpy[2])
        m.set_pos(pos[0], pos[1], pos[2])
        m.set_rpy(rpy[0], rpy[1], rpy[2])

        # for i in range(7):
            # b.set_joint_pos(i, cmd_jnt_pos[i])
        time.sleep(0.001)



