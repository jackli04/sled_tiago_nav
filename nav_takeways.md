# Navigation Takeaways

- Activate ROS Environment  
`source /opt/ros/noetic/setup.bash`  
`source ~/tiago_public_ws/devel/setup.bash`  

- Access Turbo:  
`cd /nfs/turbo/coe-chaijy/jinlinli`  

- Activate Conda:  
`source /nfs/turbo/coe-chaijy/jinlinli/miniconda3/etc/profile.d/conda.sh`  
`conda activate sled_cv`  

**1. Launch tiago_navigation.launch**  
`roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true lost:=true`  

**2. Localize the robot**  
- Launch predictions  
`rosservice call /global_localization "{}"`  
- Launch key_teleop  
`rosrun key_teleop key_teleop.py`  
- Rotate the robot  
- Clear the costmap  
`rosservice call /move_base/clear_costmaps "{}"`  

**3. Get Odometry**  
`rosrun tiago_nav get_odom.py`  

**4. Move to a point**  
Enter x, y, and theta (orientation) respectively  
`rosrun tiago_nav point_goal.py`  

**5. Goal refiner**  
`roslaunch tiago_nav point_goal.launch x:=0.0 y:=0.0 yaw:=0.0 fov_deg:=120.0`  
 