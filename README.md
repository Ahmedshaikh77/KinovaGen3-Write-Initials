## Summary 
I taught a Kinova Gen3 Lite (in ROS 2 + MoveIt 2 + Gazebo) to “handwrite” my initials, MAN, on a whiteboard. I described each letter as simple 2D points, then mapped them into 3D at a fixed board depth with the pen always facing the surface. To make the writing look smooth and reliable, I used tiny Cartesian steps and, if that plan wasn’t good enough, I automatically fell back to a joint-space move. The very first touch uses a small tap-in so the pen meets the board cleanly, and between strokes I lift off a couple centimeters to avoid smearing. I visualized the path live in RViz as a green line, which made debugging scale and ordering dead simple. I documented three key poses (End of M, Start of A, Start of N) with their joint angles, forward kinematics, and Jacobians. Early hiccups on hardware (segments overlapping) disappeared once I made every motion synchronous and from there the sim and real behavior matched nicely.




## How to run solutions
Have four terminals open all connected to the docker
- In terminal 1 run
```
- ros2 launch kortex_bringup kortex_sim_control.launch.py\
   sim_gazebo:=true\
   robot_type:=gen3_lite\
   gripper:=gen3_lite_2f\
   robot_name:=gen3_lite\
   dof:=6\
   use_sim_time:=true\
   launch_rviz:=false\
   robot_controller:=joint_trajectory_controller
```
- In terminal 2 run
``` 
  - source /opt/ros/$ROS_DISTRO/setup.bash
  - ros2 launch kinova_gen3_lite_moveit_config sim.launch.py use_sim_time:=true
```
- In terminal 3, add the scene
```  
  - ros2 launch whiteboard_setup spawn_whiteboard.launch.py
  - ros2 run whiteboard_setup attach_pen
```
- In terminal 4 in the ros2 workspace run the draw_initials task
```  
  - colcon build --symlink-install
  - source install/setup.bash
  - ros2 run initials_initials draw_initials
```

## Image
<img width="581" height="474" alt="Screenshot 2025-10-12 at 3 58 36 PM" src="https://github.com/user-attachments/assets/042c55d2-1e8c-45a9-a07a-aea36fd825ba" />


![IMG_0588](https://github.com/user-attachments/assets/917045f6-5421-43e5-a1a9-0bc5de7ad8bd)

## Simulation
https://github.com/user-attachments/assets/15fa1ca4-7764-486b-971b-5368cc5f26cd

## Physical Implementation 




https://github.com/user-attachments/assets/d488f909-c91f-47a4-ac85-f958479386f9


