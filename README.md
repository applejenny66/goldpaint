> history:  
> pmc_sghero > pmc_sghero_tmp > pmc_sghero_merge (for robot competition) > sghero_demo (for alpha site demo)  

# sghero
Fully ROS-based mobile manipulator in gazebo simulation and real world  
<img src="https://i.imgur.com/eRhjJWh.png" width="600"/>

### Dependence
Manipulator hardware related: ALMO driver
```
cd ELMO_CPP
sudo cp -r MMC* /usr/local/include/
sudo cp libMMC* /usr/local/lib/
```
Sensor related: laserscan merger
```
git clone https://github.com/shannon112/ira_laser_tools.git
catkin_make
```

## Getting Started With sghero!!
Modular standalone function in form of launch files  
<img src="https://i.imgur.com/yNfrUmg.png" width="300"/>

### Robot bringup
```sh
# spawn robot with controller in gazebo world 
roslaunch scorpio_gazebo robot_empty_world.launch rviz:=1 gazebo:=1 rqt:=1
# spawn robot with controller in real world
roslaunch scorpio_bringup amr_bringup.launch rviz:=1 rqt:=1
roslaunch scorpio_bringup amir_bringup.launch rviz:=1 rqt:=1
```
<img src="https://github.com/willie5588912/scorpio/blob/sghero_demo/scorpio_description/img/overview.png?raw=true" height="300"/> <img src="https://github.com/willie5588912/scorpio/blob/sghero_demo/scorpio_description/img/overview2.png?raw=true" height="300"/>

### Moveit demo
```sh
# enable moveit control the manipulator
roslaunch scorpio_moveit_config scorpio_moveit_planning_execution.launch
# open moveit rviz panel
roslaunch scorpio_moveit_config moveit_rviz.launch
```
<img src="https://github.com/willie5588912/scorpio/blob/sghero_demo/scorpio_moveit_config/img/moveit_demo.png?raw=true" width="550"/>

### Gmapping demo
```sh
# enable Gmapping slam on robot
roslaunch scorpio_slam scorpio_slam_execution.launch carto:=0 slam:=1
```
<img src="https://github.com/willie5588912/scorpio/blob/sghero_demo/scorpio_slam/img/gmapping_demo.png?raw=true" width="550"/>

### AMCL demo
```sh
# enable AMCL localization on robot
roslaunch scorpio_slam scorpio_slam_execution.launch carto:=0 slam:=0
```
<img src="https://github.com/willie5588912/scorpio/blob/sghero_demo/scorpio_slam/img/amcl_locali.png?raw=true" width="550"/>

### cartographer slam demo
```sh
# enable cartpgrapher slam on robot
roslaunch scorpio_slam scorpio_slam_execution.launch carto:=1 slam:=1
```
<img src="https://github.com/willie5588912/scorpio/blob/sghero_demo/scorpio_slam/img/carto_slam.png?raw=true" width="550"/>

### cartographer pure localization demo
```sh
# enable cartographer localization on robot
roslaunch scorpio_slam scorpio_slam_execution.launch carto:=1 slam:=0
# give initial pose to cartographer pure localization
roslaunch scorpio_slam scorpio_catrto_init_pose.launch
```
<img src="https://github.com/willie5588912/scorpio/blob/sghero_demo/scorpio_slam/img/carto_locali.png?raw=true" width="550"/>


### move_base navigation 
```sh
# enable global_planner, teb_local_planner navigation on localization mode
roslaunch scorpio_navigation move_base.launch
```
<img src="https://github.com/willie5588912/scorpio/blob/sghero_demo/scorpio_navigation/img/navigation.png?raw=true" width="550"/>
