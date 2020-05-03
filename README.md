# 612-Project-Testing

To get the repository via command line use the following command in the "src" folder of a catkin workspace:

```git clone https://github.com/Lilweavs/robot_simulation.git```

To build the workspace, make sure you are in a catkin workspace directory then run:

```catkin build```

Navigate to ~/{your_ws} and source the workspace (you must source your workspace anytime you open a new terminal and you must be in your catkin workspace to source the workspace):

```source devel/setup.bash```

All the code is ready to run now. First start a roscore:

```roscore```

Now open a new terminal (re-source). Then start a empty gazebo world using:

```roslaunch gazebo_ros empty_world.launch```

Open another terminal (re-source). Finally load the robot in to the Gazebo world using:

```roslaunch robot_simulation robot.launch```

Now open another terminal to load the target and projectile run:

```roslaunch robot_simulation target.launch```

```roslaunch robot_simulation projectile.launch```

The file config.yaml is where the PID controller gains are adjusted. Open that file using gedit, nano or vi and edit the gains to your liking.

To control the robot arm:

1. Run ```rosrun robot_simulation latch_release.py``` , this program determines when to open the latch

2. Run ```rosrun robot_simulation state_solver.py x y z inclination_angle```, and substitute the values of x,y,z and inclination_angle based on the target location
3. Run ```rosrun robot_simulation trajectory_plan.py```, and watch the ball get thrown. After the ball is thrown make sure to Ctrl+C this node so the robot will return to the initial state.

There are weird collision issues with the ball, so if the ball does not spawn in the end effector, follow these steps:
1. Run ```rosservice call gazebo/delete_model '{model_name: my_projectile}', Then
2. Run ```roslaunch robot_simulation projectile.launch```, to respawn the projectile.
