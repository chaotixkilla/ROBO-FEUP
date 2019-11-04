# ROBO-FEUP

Move files in folder `stdr_files` to respective folders in `/opt/ros/kinetic/share/`. e.g. Use `sudo cp map1.yaml /opt/ros/kinetic/share/stdr_resources/maps/`

Run `catkin_make` followed by `source devel/setup.bash` on root folder to compile the project

Run `roscore` in a terminal.

Run `roslaunch stdr_launchers simple-reactive-robot.launch` in another terminal.

Run `rosrun simple_reactive_robot simple_reactive_robot` in another terminal.
