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
    def __init__(self, name, rover, arm, sensor, actuators):
        joy_str = name + 'joy'

        self._subs_active = False
        self._scale = 0.001
        self.grey_button_pressed = False
        self.white_button_pressed = False

        self._rover_handle = rover
        self._arm_handle = arm

        time.sleep(0.5)
        # The vehicle in this case has 6 wheels.
        # Set the 3rd, 4th, 5th and 6th wheel as powered wheels.
        self._rover_handle.set_powered_wheel_indices([2, 3, 4, 5])
        # Set the 1st and 2nd wheel as steerable wheel.
        self._rover_handle.set_steered_wheel_indices([0, 1])
        self._rover_initialized = True

        # Get the actuator and the sensors
        self._sensor = sensor
        self._actuators = actuators
        self._actuator_activated = []
        for i in range(len(self._actuators)):
            self._actuator_activated.append(False)

        self._axis_0 = 1
        self._axis_1 = 0
        self._axis_2 = 2
        self._axis_3 = 3

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

        self._msg_counter = 0

        self.arm_cmd_scale = 0.005
        self.arm_vel_scale = 2.0
        self.base_scale = 50.0

        self._joint_cmds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self._rover_initialized = False
        self._rover_power_scale = 200.0
        self._rover_steering_scale = 0.3

        self._arm_joint_limits = [[-0.5, 0.5], [-0.6, 0.6], [-1.047, 1.047], [-2.094, 2.094], [0, 0.1], [0, 0.1]]

    def joy_cb(self, msg):
        self._joy_msg = msg

        self._subs_active = True
        pass

    def process_commands(self):
        if self._subs_active:
            if not self._rover_initialized:
                # The vehicle in this case has four wheels.
                # Set the 3rd and 4th wheel as powered wheel.
                self._rover_handle.set_powered_wheel_indices([2, 3, 4, 5])
                # Set the 1st and 2nd wheel as steerable wheel.
                self._rover_handle.set_steered_wheel_indices([0, 1])
                self._rover_initialized = True
            else:
                steering = self._joy_msg.axes[self._axis_1]
                if self._joy_msg.buttons[6]:
                    power = 1.0
                else:
                    power = -self._joy_msg.axes[self._axis_0]
                self._rover_handle.set_vehicle_power(self._rover_power_scale * power)
                self._rover_handle.set_vehicle_steering(self._rover_steering_scale * steering)
            arm_jnt_pos = [0.0]*4
            for i in range(4):
                arm_jnt_pos[i] = self._arm_handle.get_joint_pos(i)
                if self._joy_msg.buttons[1]:
                    axis_0_dir = -1
                elif self._joy_msg.buttons[2]:
                    axis_0_dir = 1
                else:
                    axis_0_dir = 0
                if self._joy_msg.buttons[0]:
                    axis_1_dir = 1
                elif self._joy_msg.buttons[3]:
                    axis_1_dir = -1
                else:
                    axis_1_dir = 0
                self._joint_cmds[0] = self._joint_cmds[0] + self.arm_cmd_scale * axis_0_dir
                self._joint_cmds[1] = self._joint_cmds[1] - self.arm_cmd_scale * axis_1_dir
                self._joint_cmds[2] = self._joint_cmds[2] + self.arm_cmd_scale * self._joy_msg.axes[self._axis_3]
                self._joint_cmds[3] = self._joint_cmds[3] - self.arm_cmd_scale * self._joy_msg.axes[self._axis_2]
                for i in range(4):
                    min_lim = self._arm_joint_limits[i][0]
                    max_lim = self._arm_joint_limits[i][1]
                    self._joint_cmds[i] = max(min(self._joint_cmds[i], max_lim), min_lim)
            for i in range(4):
                self._arm_handle.set_joint_pos(i, self._joint_cmds[i])
                # print self._joint_cmds
            if self._joy_msg.buttons[7]:
                # self._joint_cmds[4] = self._arm_joint_limits[4][1]
                # self._joint_cmds[5] = self._arm_joint_limits[5][1]
                # print 'Grasp Button Pressed'
                for i in range(self._sensor.get_count()):
                    if self._sensor.is_triggered(i) and self._actuator_activated[i] is False:
                        obj_name = self._sensor.get_sensed_object(i)
                        print 'Grasping ', obj_name, ' via actuator ', i
                        print type(obj_name)
                        self._actuators[i].actuate(obj_name)
                        self._actuator_activated[i] = True
            else:
                for i in range(self._sensor.get_count()):
                    self._actuators[i].deactuate()
                    if self._actuator_activated[i] is True:
                        print 'Releasing object from actuator ', i
                    self._actuator_activated[i] = False

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

    joy_stick_devs = []

    num_rovers = 2
    num_actuators = 4

    for i in range(num_rovers):
        rover_prefix = 'rover' + str(i+1) + '/'
        rover = client.get_obj_handle(rover_prefix + 'Rover')
        arm = client.get_obj_handle(rover_prefix + 'arm_link1')
        sensor = client.get_obj_handle(rover_prefix + 'Proximity0')
        actuators = []
        for j in range(num_actuators):
            actuator = client.get_obj_handle(rover_prefix + 'Constraint' + str(i))
            actuators.append(actuator)

        # If you have multiple Joysticks, you can pass in different names
        js_device = JoyStickDevice(parsed_args.joystick_name, rover, arm, sensor, actuators)
        joy_stick_devs.append(js_device)

    # The publish frequency
    pub_freq = 60
    rate = rospy.Rate(pub_freq)
    msg_index = 0

    while not rospy.is_shutdown():
        for joystick_dev in joy_stick_devs:
            joystick_dev.process_commands()

        rate.sleep()
        msg_index = msg_index + 1
        if msg_index % pub_freq * 50 == 0:
            # Print every 3 seconds as a flag to show that this code is alive
            # print('Running JoyStick Controller Node...', round(rospy.get_time() - _start_time, 3), 'secs')
            pass
        if msg_index >= pub_freq * 10:
            # After ten seconds, reset, no need to keep increasing this
            msg_index = 0


if __name__ == '__main__':
    main()