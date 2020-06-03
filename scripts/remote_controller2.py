import rospy
from PyKDL import Frame, Vector, Rotation
from sensor_msgs.msg import Joy
from ambf_client import Client
import sys
import time
from argparse import ArgumentParser
from tf_conversions import posemath


# Init everything related to JoyStick
class JoyStickDevice:
    # The name should include the full qualified prefix. I.e. '/Geomagic/', or '/omniR_' etc.
    def __init__(self, name):
        joy_str = name + 'joy'

        self._subs_active = False
        self._scale = 0.001
        self.grey_button_pressed = False
        self.white_button_pressed = False

        self._axis_1 = 1
        self._axis_2 = 0
        self._axis_3 = 2
        self._axis_4 = 3

        self._button_a = 0
        self._button_next = 4
        self._button_next = 5
        self._button_rot = 6

        self._joy_msg = Joy()

        self._lin_scale = 0.01
        self._ang_scale = 0.005

        self._obj_pos_cmd = Vector(0, 0, 0)
        self._obj_rpy_cmd = Vector(0, 0, 0)

        self._is_obj_active = False # Flag to check if obj is active
        self._cmd_pos = Vector(0, 0, 0)
        self._cmd_rpy = Vector(0, 0, 0)

        self._pose_sub = rospy.Subscriber(joy_str, Joy, self.joy_cb, queue_size=10)

        self._obj_handles = {}

        self._msg_counter = 0

        self._total_objs = 0
        self._active_obj = None
        self._active_obj_idx = -1

        self.kuka_T_targetINb = Frame(Rotation.RPY(0, 0, 0), Vector(0.1, 0.1, 0.7))
        self.kuka_base_handle = None
        self.marker_handle = None

        # Buttons Configuration
        self._prev_obj_button_idx = 4
        self._next_obj_button_idx = 5

    def bind_objects(self, ambf_client, obj_names):
        for obj_name in obj_names:
            obj_handle = ambf_client.get_obj_handle(obj_name)
            if obj_handle is None:
                print('ERROR, CAN\'T FIND ', obj_names)
                print('EXISTING OBJECTS ARE: ')
                print(ambf_client.get_obj_names())
            else:
                time.sleep(0.5)
                self._obj_handles[obj_name] = obj_handle
                self._total_objs = self._total_objs + 1
                print("Adding Object ", obj_handle.get_name(), "to list")

        # Get the handle to KUKA Base
        self.kuka_base_handle = ambf_client.get_obj_handle('base')

        # Get a handle to the marker for debugging
        self.marker_handle = ambf_client.get_obj_handle('Marker')

        # Start with binding the first object as obj_handle
        self.choose_active_obj(0)

    def choose_active_obj(self, idx):
        if 0 <= idx < self._total_objs:
            self._active_obj = self._obj_handles.values()[idx]
            self._active_obj_idx = idx
            print("Setting ", self._active_obj.get_name(), " as active object")
        else:
            print "Requested Object Idx ", idx, " greater than total objects found"

    def choose_prev_obj(self):
        if 0 < self._active_obj_idx < self._total_objs:
            self.choose_active_obj(self._active_obj_idx - 1)
        else:
            self.choose_active_obj(self._total_objs - 1)

        print('PREV: Choosing Object: ', self._active_obj.get_name())

    def choose_next_obj(self):
        if 0 < self._active_obj_idx < self._total_objs:
            self.choose_active_obj(self._active_obj_idx + 1)
        else:
            self.choose_active_obj(0)

        print('NEXT: Choosing Object: ', self._active_obj.get_name())

    def joy_cb(self, msg):
        self._joy_msg = msg
        self._joy_msg = msg
        self._joy_msg = msg
        self._joy_msg = msg

        if self._joy_msg.buttons[self._prev_obj_button_idx]:
            self.choose_prev_obj()
        elif self._joy_msg.buttons[self._next_obj_button_idx]:
            self.choose_next_obj()

        self._subs_active = True
        pass

    def process_commands(self):
        if self._subs_active and self._active_obj is not None:
            if self._active_obj.is_wd_expired():
                self._is_obj_active = False
            else:
                self._is_obj_active = True

            if self._is_obj_active:
                if self._active_obj.get_name() == '/ambf/env/Chassis':

                    cur_pos = self._active_obj.get_pos()
                    cur_rot = self._active_obj.get_rot()
                    P_oINw = Vector(cur_pos.x, cur_pos.y, cur_pos.z)
                    R_oINw = Rotation.Quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)

                    f_o = Vector(-self._joy_msg.axes[self._axis_1], 0, 0)
                    n_o = Vector(0, 0, self._joy_msg.axes[self._axis_2])
                    f_oINw = R_oINw * f_o
                    f_oINw = f_oINw * 200.0

                    n_oINw = R_oINw * n_o
                    n_oINw = n_oINw * 200.0
                    self._active_obj.set_force(f_oINw[0], f_oINw[1], f_oINw[2])
                    self._active_obj.set_torque(n_oINw[0], n_oINw[1], n_oINw[2])
                    # print f_oINw

                elif self._active_obj.get_name() == '/ambf/env/link7':

                    p = self.kuka_base_handle.get_pos()
                    q = self.kuka_base_handle.get_rot()
                    P_bINw = Vector(p.x, p.y, p.z)

                    R_bINw = Rotation.Quaternion(q.x, q.y, q.z, q.w)

                    T_bINw = Frame(R_bINw, P_bINw)
                    scale = 0.001

                    rpy = self.kuka_T_targetINb.M.GetRPY()
                    rpy = Vector(rpy[0], rpy[1], rpy[2])
                    pos = self.kuka_T_targetINb.p
                    if self._joy_msg.buttons[self._button_rot]:
                        rpy = rpy + Vector(self._joy_msg.axes[self._axis_1],
                                           self._joy_msg.axes[self._axis_2],
                                           self._joy_msg.axes[self._axis_3]) * scale
                    else:
                        pos = pos + Vector(self._joy_msg.axes[self._axis_1],
                                           self._joy_msg.axes[self._axis_2],
                                           self._joy_msg.axes[self._axis_3]) * scale

                    self.kuka_T_targetINb = Frame(Rotation.RPY(rpy[0], rpy[1], rpy[2]), pos)

                    T_targetINb = T_bINw * self.kuka_T_targetINb

                    p = T_targetINb.p
                    rpy = T_targetINb.M.GetRPY()
                    self._active_obj.set_pos(p[0], p[1], p[2])
                    self._active_obj.set_rpy(rpy[0], rpy[1], rpy[2])
                    self.marker_handle.set_pos(p[0], p[1], p[2])
                    self.marker_handle.set_rpy(rpy[0], rpy[1], rpy[2])

            if self._msg_counter % 100 == 0:
                pass
            if self._msg_counter >= 1000:
                self._msg_counter = 0
            self._msg_counter = self._msg_counter + 1

def main():
    # Begin Argument Parser Code
    parser = ArgumentParser()

    parser.add_argument('-d', action='store', dest='joystick_name', help='Specify ros base name of joystick',
                        default='/')
    parser.add_argument('-o', action='store', dest='obj_name', help='Specify AMBF Obj Name', default='Chassis')
    parser.add_argument('-a', action='store', dest='client_name', help='Specify AMBF Client Name',
                        default='client_name')
    parser.add_argument('-p', action='store', dest='print_obj_names', help='Print Object Names',
                        default=False)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print parsed_args

    client = Client(parsed_args.client_name)
    client.connect()

    if parsed_args.print_obj_names:
        print ('Printing Found AMBF Object Names: ')
        print client.get_obj_names()
        exit()

    _pair_one_specified = True

    _joy_stick_ccus = []

    # The publish frequency
    _pub_freq = 500

    joystick_one = JoyStickDevice(parsed_args.joystick_name)
    joystick_one.bind_objects(client, ['Chassis', 'link7'])
    _joy_stick_ccus.append(joystick_one)

    rate = rospy.Rate(_pub_freq)
    msg_index = 0
    _start_time = rospy.get_time()

    while not rospy.is_shutdown():
        for joystick_dev in _joy_stick_ccus:
            joystick_dev.process_commands()

        rate.sleep()
        msg_index = msg_index + 1
        if msg_index % _pub_freq * 3 == 0:
            # Print every 3 seconds as a flag to show that this code is alive
            print('SpaceNav AMBF Node Alive...', round(rospy.get_time() - _start_time, 3), 'secs')
        if msg_index >= _pub_freq * 10:
            # After ten seconds, reset, no need to keep increasing this
            msg_index = 0


if __name__ == '__main__':
    main()