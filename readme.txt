# TURTLEBOT CARTOGRAPHY USING HUMAN POSE NAVIGATION

The aim of this project is to showcase the use of human pose estimation in 
helping the robots create SLAM maps of the environment just by following the humans.
This can reduce the tedious task of manually running the robot for mapping.

1) Create Workspace of turtlebot3 as mentioned in the ROS2 documentation.
2) Change the camera angle if needed. This can be found in the sdf file of the turtlebot models.
3) Download the tensorflow lite model for pose estimation.
4) After setting the ROS2 workspace for turtlebot3 run the human_cam_follower_project.py script and open the RVIZ cartographer.

NOTE: a) The algorithm used is a simple form of PD controller. This works well in this case but the bot sometimes has a problem as seen in the end of the video so please do tell me what other methods can be used to make a better controller for future iterations.
b) Furthermore the tflite model is also a bit less accurate compared to other available models so that can also be changed to improve the bot navigation.
TURTLEBOT LICENSE> https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/LICENSE
