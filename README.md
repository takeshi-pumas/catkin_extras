# catkin_tutorials
Catkin Must run on top of takeshi catkin
(source /HSRPumas/catkindevel/setup.bash)
(source /opt/ros/...)

clone this git 

$  git clone https://github.com/takeshi-pumas/catkin_extras.git

$ cd catkin_extras

"build the catkin space containing Moveit, and the wrs arenas"

$ catkin_make

Source this new catkin space on top of Takeshi and/or tmc packages.

$ source devel/setup.bash  
Consider adding this line to bashrc file. 
if not , source must be done on every new terminal.

At this point autocomplete should be able to find the moveit launch files and the gazebo tmc wrs worlds.

Try typing

$  roslaunch hsrb_wrs_gazebo_launch

then hit tab for autocomplete you should be able to launch 

or just
$ roslaunch hsrb_wrs_gazebo_launch wrs_practice0_easy_tmc.launch

Gazebo (paused) is launched, hit play  and open a new terminal ( remeber sourcing )

MOVEIT IS NOW INLCUDED IN LAUNCH;

$ python3 /src/smaches/ANY SMACH.py

PUMAS NAVIGATION IS NOW AVAILABLE

Check https://www.youtube.com/watch?v=oE1UKBc-sz4


