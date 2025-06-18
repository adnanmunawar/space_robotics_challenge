## Usage

### Cloning the correct AMBF branch and building
Clone, build and source AMBF based on these [instructions](https://github.com/WPI-AIM/ambf/wiki/Installing-AMBF)

### Launching the simulation
We can use the provided launch file in this repo with the appropriate indices.

```bash
cd <path_to_this_folder>
./ambf_simulator --launch_file ADF/launch.yaml -l 0,1,2,3,4,5,6,7,8
```
You may open the `launch.yaml` file and inspect its contents to figure out what exactly do the ADF files `0-8` represent.

### Connecting a Controller
If you have a game controller, you can use it control the rovers and the attached manipulator in the simulation.

To do so, we first need to spawn the ROS Joy node. Please set it up using these [instructions](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)

As per the instructions above, please determine which `/dev/input/jsX` is your joystick bound to, then open a terminal and type:
```bash
rosparam set joy_node/dev "/dev/input/jsX"
```
replacing `X` by the number found above.

Then run the joy node as:
```bash
rosrun joy joy_node
```
### Starting the Remote Control Script
Navigate to the the `scripts` folder in this package. With your `joy` node and `simulation` running before hand, run the script below:
```bash
python multi_rover_control.py
```
Now you should be able to use your joystick to control the rovers and their attached manipulators. `Button 4` is bound to toggle control between the rovers. To change the button and axes mapping, you can tweak the code in `multi_rover_control.py`.

### Controlling the Simulation Programatically
The scripts folder should have good examples for this. Please refer also to the `multi_rover_control.py` script and modify as necessary.
