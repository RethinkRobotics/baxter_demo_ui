1.1.1 (2015-5-15)
---------------------------------
- Added Torso Navigators to control UI in demo mode
For recorded trajectory playback:
- Fixed gripper playback synchronization issue with arm movement (baxter_examples/joint_trajectory_file_playback.py)
- Slowed down arm movement to initial position (baxter_examples/joint_trajectory_file_playback.py)
- Prevented trajectory stopping before completion
 

1.1.0 (2015-1-2)
---------------------------------
- Updated baxter_demo_ui to ROS Indigo
- Indigo robot required a reworking of how cameras power **on** / **off** - Three cameras can no longer be powered at the same time, and closing cameras turns power **off** to the specified camera and **on** to the other two
- Updated Python cv dependancy to require cv2 instead
- Updated all cv2 functions to use numpy for image storage

1.0.0 (2014-5-1)
---------------------------------
- Creation of baxter_demo_ui repository
- Initial demo mode implementation showcasing joint_torque_springs, joint_velocity_puppet, camera display and more.
- Adds generic UI, and Baxter start/stop processes classes
