# 612-Project-Testing

To get the repository via command line use the following command in the "src" folder of a catkin workspace:
```
git clone https://github.com/Lilweavs/robot_simulation.git
```
To build the workspace, make sure you are in a catkin workspace directory then run:
```
catkin build
```
Navigate to ~/{your_ws} and source the workspace (you must source your workspace anytime you open a new terminal and you must be in your catkin workspace to source the workspace):
```
source devel/setup.bash
```
All the code is ready to run now. First start a roscore:
```
roscore
```
Now open a new terminal (re-source). Then start a empty gazebo world using:
```
roslaunch gazebo_ros empty_world.launch
```
Open another terminal (re-source). Finally load the models to simulate run:
```
roslaunch robot_simulation test.launch
```

The file config.yaml is where the PID controller gains are adjusted. Open that file using gedit, nano or vi and edit the gains to your liking.

To control the robot arm:

1. Open another terminal (re-source)

2. Run ```rostopic echo -c /simple_model/joint_states```
(This subscribes to that topic so you can view all the current joint states of your robot)

3. Open another terminal (re-source)

4. To control the robot you must publish a desired position value to the controller command topic:
```rostopic pub -1 /simple_model/base_to_first_joint_position_controller/command std_msgs/Float64 "data: {your value here in radians}" ```

5. Now sit back and watch it move

