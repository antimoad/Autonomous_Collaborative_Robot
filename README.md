# Autonomous_Collaborative_Robot
Autonomous and Collaborative Robot for Sorting applications.
Panda robotic arm with fixed camera, capable of sorting different cubes based on their visual appeareance, and a neural network that counts user's fingers via webcam to decide where to put outlier cubes (1 finger = Red bin, 2 = Green bin, 3 = Blue bin).
To use this projecto, clone this repository in your Destktop, build the workspace and source it, them run the folloging commands, you're going to need 2 terminals, 1 for each command:

roslaunch panda_moveit_config demo_gazebo.launch

rosrun joint_talker my_final2.py
