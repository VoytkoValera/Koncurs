Для начала необходимо открыть проект в контейнере
Затем в терминале:
> rosdep install --from-paths src --ignore-src -r -y
>catkin_make
> source devel/setup.bash
> roslaunch panda_gazebo start_reach_world.launch

В новом терминале:
>source devel/setup.bash
>roslaunch panda_gazebo put_robot_in_world.launch

В новом терминале:
>source devel/setup.bash
>python3 final.py
