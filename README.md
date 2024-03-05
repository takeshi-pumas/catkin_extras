# catkin_tutorials
Catkin Must run on top of takeshi catkin

clone this git 

$  git clone https://github.com/takeshi-pumas/catkin_extras.git

$ cd catkin_extras

"build the catkin space containing Moveit, and the wrs arenas"

$ catkin_make

Source this new catkin space on top of Takeshi and/or tmc packages.

$ source devel/setup.bash  
Consider adding this line to bashrc file. 

"""

alias start_map='rosnode kill /pose_integrator;rosrun hector_mapping hector_mapping _map_size:=512 _map_resolution:=0.05 _pub_map_odom_transform:=true _scan_topic:=/hsrb/base_scan _use_tf_scan_transformation:=true _map_update_angle_thresh:=2.0 _map_update_distance_thresh:=0.10 _scan_subscriber_queue_size:=1 _update_factor_free:=0.39 _update_factor_occupied:=0.85 _base_frame:=base_link'
alias end_map='rosrun map_server map_saver'
"""

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




