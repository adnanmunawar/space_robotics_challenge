import rospy
from PyKDL import Frame, Vector, Rotation
from sensor_msgs.msg import Joy
from ambf_client import Client
import sys
import time
from argparse import ArgumentParser
from itertools import cycle

rover_iterator = cycle(range(2))
active_rover = rover_iterator.next()


# Init everything related to JoyStick
class JoyStickDevice:
    # The name should include the full qualified prefix. I.e. '/Geomagic/', or '/omniR_' etc.
    def __init__(self, name):
        joy_str = name + 'joy'

        self.subs_active = False

        time.sleep(0.5)

        self.axis_0 = 1
        self.axis_1 = 0
        self.axis_2 = 2
        self.axis_3 = 3

        self.joy_msg = Joy()

        self._pose_sub = rospy.Subscriber(joy_str, Joy, self.joy_cb, queue_size=10)

        self._msg_counter = 0

    def joy_cb(self, msg):
        global rover_iterator, active_rover
        self.joy_msg = msg
        if self.joy_msg.buttons[4] == 1:
            active_rover = rover_iterator.next()
            print "Selecting Rover Number ", active_rover
            # print "4 Pressed"

        self.subs_active = True
        pass


class ControlUnit:
    def __init__(self, joystick, rover, arm, sensor, actuators):
        # Get the actuator and the sensors
        self._js_handle = joystick
        self._rover_handle = rover
        self._arm_handle = arm
        self._sensor = sensor
        self._actuators = actuators
        self._grasped = []
        for i in range(len(self._actuators)):
            self._grasped.append(False)

        # The vehicle in this case has 6 wheels.
        # Set the 3rd, 4th, 5th and 6th wheel as powered wheels.
        self._rover_handle.set_powered_wheel_indices([2, 3, 4, 5])
        # Set the 1st and 2nd wheel as steerable wheel.
        self._rover_handle.set_steered_wheel_indices([0, 1])
        self._rover_initialized = True

        self._rover_power_scale = 200.0
        self._rover_steering_scale = 0.3

        self.arm_cmd_scale = 0.02
        self.base_scale = 50.0

        self._joint_cmds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._arm_joint_limits = [[-0.5, 0.5], [-0.6, 0.6], [-1.047, 1.047], [-2.094, 2.094], [0, 0.1], [0, 0.1]]

    def process_commands(self):
        if self._js_handle.subs_active:
            if not self._rover_initialized:
                # The vehicle in this case has four wheels.
                # Set the 3rd and 4th wheel as powered wheel.
                self._rover_handle.set_powered_wheel_indices([2, 3, 4, 5])
                # Set the 1st and 2nd wheel as steerable wheel.
                self._rover_handle.set_steered_wheel_indices([0, 1])
                self._rover_initialized = True

            else:
                steering = self._js_handle.joy_msg.axes[self._js_handle.axis_1]
                if self._js_handle.joy_msg.buttons[6]:
                    power = 1.0
                else:
                    power = -self._js_handle.joy_msg.axes[self._js_handle.axis_0]
                self._rover_handle.set_vehicle_power(self._rover_power_scale * power)
                self._rover_handle.set_vehicle_steering(self._rover_steering_scale * steering)

                if self._js_handle.joy_msg.buttons[1]:
                    axis_0_dir = -1
                elif self._js_handle.joy_msg.buttons[2]:
                    axis_0_dir = 1
                else:
                    axis_0_dir = 0

                if self._js_handle.joy_msg.buttons[0]:
                    axis_1_dir = 1
                elif self._js_handle.joy_msg.buttons[3]:
                    axis_1_dir = -1
                else:
                    axis_1_dir = 0
                self._joint_cmds[0] = self._joint_cmds[0] + self.arm_cmd_scale * axis_0_dir
                self._joint_cmds[1] = self._joint_cmds[1] - self.arm_cmd_scale * axis_1_dir
                self._joint_cmds[2] = self._joint_cmds[2] + self.arm_cmd_scale * self._js_handle.joy_msg.axes[self._js_handle.axis_3]
                self._joint_cmds[3] = self._joint_cmds[3] - self.arm_cmd_scale * self._js_handle.joy_msg.axes[self._js_handle.axis_2]
                for i in range(4):
                    min_lim = self._arm_joint_limits[i][0]
                    max_lim = self._arm_joint_limits[i][1]
                    self._joint_cmds[i] = max(min(self._joint_cmds[i], max_lim), min_lim)
                    self._arm_handle.set_joint_pos(i, self._joint_cmds[i])

            if self._js_handle.joy_msg.buttons[7]:
                # print 'Grasp Button Pressed'
                for i in range(self._sensor.get_count()):
                    if self._sensor.is_triggered(i) and self._grasped[i] is False:
                        obj_name = self._sensor.get_sensed_object(i)
                        print 'Grasping ', obj_name, ' via actuator ', i
                        self._actuators[i].actuate(obj_name)
                        self._grasped[i] = True
            else:
                for i in range(self._sensor.get_count()):
                    self._actuators[i].deactuate()
                    if self._grasped[i] is True:
                        print 'Releasing object from actuator ', i
                    self._grasped[i] = False


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

    control_units = []

    num_rovers = 2
    num_actuators = 4
    # Since we have only one JS for now,
    joystick = JoyStickDevice(parsed_args.joystick_name)

    for i in range(num_rovers):
        rover_prefix = 'rover' + str(i+1) + '/'
        rover = client.get_obj_handle(rover_prefix + 'Rover')
        arm = client.get_obj_handle(rover_prefix + 'arm_link1')
        sensor = client.get_obj_handle(rover_prefix + 'Proximity0')
        actuators = []
        for j in range(num_actuators):
            actuator = client.get_obj_handle(rover_prefix + 'Constraint' + str(j))
            actuators.append(actuator)

        # If you have multiple Joysticks, you can pass in different names
        control_unit = ControlUnit(joystick, rover, arm, sensor, actuators)
        control_units.append(control_unit)

    # The publish frequency
    pub_freq = 60
    rate = rospy.Rate(pub_freq)
    msg_index = 0

    while not rospy.is_shutdown():
        # Control the active Control Unit
        control_units[active_rover].process_commands()

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