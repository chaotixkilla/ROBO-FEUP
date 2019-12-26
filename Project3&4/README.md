Move files from folders `worlds`, `launch` and `models` to their respective paths inside the `catkin_ws/src/turtlebot3_gazebo` folder.

Inside the `catkin_ws` folder created during gazebo setup, run in a terminal the commands `catkin_make` and `source devel/setup.bash` followed by `roslaunch turtlebot3_gazebo simple_world.launch`.

Inside the `Project3&4` folder, run `catkin_make` and `source devel/setup.bash` followed by `rosrun simple_reactive_robot simple_reactive_robot <operation>`. 

Operation values:
    * 1: Wall-Following
    * 2: Line-Following
    * 3: Object in Front Following