## Usage

### Cloning the correct AMBF branch and building
To run the galen robot simulation, first clone the right branch of ambf.

``` bash
cd ambf
git checkout origin/ambf-1.0
git checkout -b ambf-1.0
git pull origin ambf-1.0
```

Make a build directory if it is not already there

``` bash
mkdir build
```

Then build the simulator
``` bash
cd build
cmake ..
make -j8
```
Make sure to source it so ROS can find the packages. You can do it either per terminal or in your .bashrc
``` bash
source devel/setup.bash
```

### Running the Models
We can run the simulation files using the custom launch file and loading the desired
ADF files file by providing their indexes.

```bash
./ambf_simulator --launch_file <path_to_this_shared_folder>/ADF/launch.yaml -l 0,1,2,3,4,5,6,7,8
```
You may open the `launch.yaml` file and inspect its contents to figure out what exactly do the ADF files `0-8` represent.

### Connecting a Controller
If you have a game controller, you can use it control the rovers and the attached manipulator in the simulation.

We must now run the ROS node so that it spawns the JoyStick driver and streams its state data. You can find the details here:
http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

To sum things up. Make sure that you have the ROS `joy` package installed and that your Joystick is connected to the computer via USB/Ethernet/Bluetooth etc.

Use the link above to figure out which `/dev/input/jsX` is your joystick bound to, then open a terminal and type:
```bash
rosparam set joy_node/dev "/dev/input/jsX"
```
replacing `X` by the number found above.

Then run the joy node as:
```bash
rosrun joy joy_node
```
### Starting the Remote Control Script
Head over to the `scripts` folder in this package. With your `joy` node and `simulation` running before hand, you can run the script:

```bash
python multi_rover_control.py
```
Now you should be able to use your joystick to control the rovers and their attached manipulators. `Button 4` is bound to toggle control between the rovers. To change the button and axes mapping, you can tweak the code in `multi_rover_control.py`
