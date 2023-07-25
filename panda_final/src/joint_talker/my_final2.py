#!/usr/bin/env python3
# Initial imports
from __future__ import print_function
#from ctypes.wintypes import WPARAM
import random
from shutil import move
from time import sleep
from turtle import position
from gazebo_msgs.srv import SpawnModel
from rospy_tutorials.srv import AddTwoInts
from geometry_msgs.msg import Point, Pose, Quaternion
from franka_gripper.msg import GraspAction, GraspGoal

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import numpy as np
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from fingers_classifier import FingersClassifier

cube_folder="cube3"

class MoveGroupPythonInterface(object):
    # Global variable to set initial joint goals to None
    initial_joint_goal = None
    
    # Initial setup
    # Initializes the RobotCommander, PlanningSceneInterface, MoveGroupCommander objects, and sets up Publisher
    # Prints initial states
    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm" # Group name for Panda arm, not including gripper
        move_group = moveit_commander.MoveGroupCommander(group_name)
        group_name_hand = "panda_hand" # Group name for Panda gripper
        move_group_hand = moveit_commander.MoveGroupCommander(group_name_hand)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)
        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)
        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())
        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        # Misc variables
        self.box_name = "cube"
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.move_group_hand = move_group_hand
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.image = None
        self.br = CvBridge()     
        self.isWritten = None   
        self.finger_classifier = FingersClassifier()    
    
    # Opens the gripper to a desired width with a given force
    def open_gripper(self):
        client = actionlib.SimpleActionClient('franka_gripper/grasp',GraspAction)
        client.wait_for_server()
        action = GraspGoal(width=0.079,speed=0.0001,force=95)
        client.send_goal(action)
        wait = client.wait_for_result()
        return client.get_result()
        
    # Closes the gripper to a desired width
    # Takes in a force value with which to close the gripper with
    def close_gripper(self,f):
        client = actionlib.SimpleActionClient('franka_gripper/grasp',GraspAction)
        client.wait_for_server()
        action = GraspGoal(width=0.059,speed=0.1,force=f)
        action.epsilon.inner=0.005
        action.epsilon.outer=0.005
        client.send_goal(action)
        timeout = rospy.Duration(secs=3)
        wait = client.wait_for_result(timeout)
        return client.get_result()
    
    def close_gripper2(self):
        gripper = self.move_group_hand
        gripper_joint_values = {}
        gripper_joint_values['panda_finger_joint1'] = 0# = gripper.get_current_joint_values()
        gripper_joint_values['panda_finger_joint2'] = 0
        #close_joint_value = (pi / 11 + pi / 12) / 2
        close_joint_value = 0.0042
        #for i in range(len(gripper_joint_values)):
        #    gripper_joint_values[i] = close_joint_value
        gripper_joint_values['panda_finger_joint1'] = 0.001
        gripper_joint_values['panda_finger_joint2'] = 0.001
        
        gripper.go(gripper_joint_values, wait=True)
        gripper.stop()

    def attachBox(self,box_name):
        #TO DO: Close the gripper and call the service that releases the box
        try:
            touch_links = self.robot.get_link_names(self.move_group_hand)
            self.scene.attach_box(self.eef_link, box_name, touch_links=touch_links)

            self.close_gripper2()

            attach = rospy.ServiceProxy('AttachObject', AttachObject)
            attach(1, box_name)

            return self.wait_for_state_update(box_name, box_is_known=True, box_is_attached=True)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

            return False

    # Spawns a cube by calling the spawn model service with ServiceProxy
    # Takes in a name, x and y coordinates for position, and a color to spawn with (only supports Red, Blue, and Green)
    def spawn_cube(self,name,x,y,color):
        #color = "Blue"
        cubeColor = 'model' + color + '.sdf'
        cube_path = os.path.join(os.path.expanduser('~'),'Desktop', 'panda_final','src','resources',cube_folder, cubeColor)
        client = rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)
        client(
            model_name=name,
            model_xml=open(cube_path,"r").read(),
            robot_namespace="/moveit_commander",
            initial_pose=Pose(position= Point(x,y,0),
            orientation=Quaternion(0,0,0,0)),
            reference_frame='world'
        )
        self.box_name = name
        sleep(1)
        return True
    
    # Spawns a bin
    # Takes in a name, x and y coordinates for position, and a color to spawn with (only supports Red, Blue, and Green)
    def spawn_bin(self,name,x,y,color):
        binColor = 'model' + color + '.sdf'
        bin_path = os.path.join(os.path.expanduser('~'),'Desktop', 'panda_final','src','resources','bin', binColor)
        client = rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)
        #add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        client(
            model_name=name,
            model_xml=open(bin_path,"r").read(),
            robot_namespace="/moveit_commander",
            initial_pose=Pose(position= Point(x,y,0),
            orientation=Quaternion(0,0,0,0)),
            reference_frame='world'
        )
        return True

    # Spawns the camera at the position (0.5, 0.8, 1) and orients it facing the ground
    # It also sets up the camera to publish and subscribe from the '/camera/color/image_raw' topic
    # This topic was created in the sdf of the camera itself
    def spawn_camera(self,name):
        camera_path = os.path.join(os.path.expanduser('~'),'Desktop', 'panda_final','src','resources','camera','model.sdf')
        client = rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)
        client(
            model_name=name,
            model_xml=open(camera_path,"r").read(),
            robot_namespace="/moveit_commander",
            initial_pose=Pose(position= Point(0.5,0.8,1),
            orientation=Quaternion(-0.5**0.5,0,0.5**0.5,0)),
            reference_frame='world'
        )
        self.camera_name = name
        self.pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        return True

    # Global variable to check if camera has written to the desired path or not
    isWritten = False

    # Callback function tries to read the image from the camera using a cv_bridge
    # Checks isWritten so the image is only written once
    # cv_bridge converts ROS image messages to OpenCV images
    def callback(self, msg): 
        try:
            if not self.isWritten:
                self.image = self.br.imgmsg_to_cv2(msg)
                path = os.path.join(os.path.expanduser('~'), 'Desktop', 'panda_final', 'src', 'joint_talker','images','camera_image.jpeg')
                self.isWritten = cv2.imwrite(path, cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR)) # Convert RGB image to BGR for CV2
                print("Image written")
        except CvBridgeError as e:
            print(e)
            return
    
    # Global variable sets up a empty list, used for image processing
    col = []
    
    # Calls the camera subscriber and initiates the callback function
    def camera_result(self):
        image_topic = "/camera/color/image_raw"
        try:
            rospy.Subscriber(image_topic, Image, self.callback)
            return
        except CvBridgeError as e:
            print(e)
    
    # Reads in the image written by the camera
    # Finds the boxes in order from top to bottom and determines the color of each box
    # Sets global variable col to be a list of the colors of the blocks in order
    def find_colors(self):
        ap = argparse.ArgumentParser()
        image = cv2.imread(os.path.join(os.path.expanduser('~'), 'Desktop', 'panda_final', 'src', 'joint_talker','images','camera_image.jpeg'))
        #image = cv2.resize(image, (300,400))
        # hsv color range values
        boundaries = [
	    ([0,50,50], [10, 255, 255], "red" ),
        ([170,50,50], [179, 255, 255], "red" ),
	    ([70,20,20], [130, 255, 255], "blue"),
	    ([45, 50, 50], [70, 255, 255], "green"),
        ([25, 50, 50], [35, 255, 255], "other")
        ]
        # position of pixels to look at 
        #posit = [[115,90],[150,90],[190,90],[230,90],[270,90]]
        posit = [[240,60],[315,60],[385,60],[460,60],[530,60]]
        for pos in posit:
            print(image[pos[0], pos[1]])
        # variable to store the colors
        col = [0,0,0,0,0]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # loop over the boundaries
        i = 0
        for (lower, upper, color) in boundaries:
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            mask = cv2.inRange(hsv, lower, upper)
            output = cv2.bitwise_and(hsv, hsv, mask = mask)
            color2 = color
            if i == 0:
                color2 = "red_low"
            elif i == 1:
                color2 = "red high"
            cv2.imwrite(str(os.path.join(os.path.expanduser('~'), 'Desktop', 'panda_final', str('img_' + color2 + '.png'))), cv2.cvtColor(output, cv2.COLOR_HSV2BGR))
            i += 1
            count = 0
            for loc in posit:
                if( np.any(output[loc] != [0,0,0])):
                    col[count] = color
                count = count+1
        print(col)
        self.col = col
        return True

    def find_colored_cubes(self):
        ap = argparse.ArgumentParser()
        image = cv2.imread(os.path.join(os.path.expanduser('~'), 'Desktop', 'panda_final', 'src', 'joint_talker','images','camera_image.jpeg'))
        image = cv2.resize(image, (300,400))
        # hsv color range values
        boundaries = [
	    ([0,50,50], [30, 255, 255], "red" ),
	    ([70,20,20], [130, 255, 255], "blue"),
	    ([30, 50, 50], [65, 255, 255], "green")
        ]
        # position of pixels to look at 
        posit = [[115,90],[150,90],[190,90],[230,90],[270,90]]
        # variable to store the colors
        col = [0,0,0,0,0]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # loop over the boundaries
        for (lower, upper, color) in boundaries:
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            mask = cv2.inRange(hsv, lower, upper)
            output = cv2.bitwise_and(hsv, hsv, mask = mask)
            count = 0
            sift = cv2.SIFT_create()
            #ret2,in_image = cv2.threshold(output[:, :, 2],0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            #kp = sift.detect(in_image, None)
            kp = sift.detect(output[:, :, 2], None)
            new_img = cv2.drawKeypoints(image=output[:, :, 2], keypoints=kp, outImage=image)
            for loc in posit:
                if( np.any(output[loc] != [0,0,0])):
                    col[count] = color
                count = count+1
        print(col)
        self.col = col
        return True

    # Moves the end effector over a cube at position (x,y) and picks it up with the gripper
    # Takes in coordinates x and y, which is the position of a cube
    def move_to_box_and_attach(self,x,y,timeout=4):
        # box located at x,y 
        # set the end effector pointing down
        if cube_folder == "cube4":
            y-=0.03
            x-=0.03
        move_group = self.move_group
        move_group_hand = self.move_group_hand
        box_name = self.box_name
        # open the gripper
        self.open_gripper()
        sleep(1.5)
        # point the gripper down
        joint_goal = move_group.get_current_joint_values()
         
        # move over box
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = 0.5
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0,
            avoid_collisions=True
        )
        constraint = moveit_msgs.msg.Constraints()
        joint_constraint = moveit_msgs.msg.JointConstraint()
        joint_constraint.joint_name = "panda_joint7"
        joint_constraint.position = joint_goal[-1]
        joint_constraint.tolerance_above = 0.001
        joint_constraint.tolerance_below = 0.001
        joint_constraint.weight = 1
        constraint.joint_constraints.append(copy.deepcopy(joint_constraint))
        joint_constraint.joint_name = "panda_joint6"
        joint_constraint.position = joint_goal[-2]
        joint_constraint.tolerance_above = 0.001
        joint_constraint.tolerance_below = 0.001
        joint_constraint.weight = 1
        constraint.joint_constraints.append(copy.deepcopy(joint_constraint))
        success = move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        current_joints = move_group.get_current_joint_values()
        sleep(1)


        # move to box
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = 0.13
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0,
            avoid_collisions=True
        )
        success = move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        current_joints = move_group.get_current_joint_values()

        # close the gripper
        sleep(0.5)
        res = self.close_gripper(30)
        #self.close_gripper2()
        #self.attachBox("cube0")
        print(f"Result of closing gripper: {res}" )
        #sleep(0.5)

        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = 0.5

        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0,
            avoid_collisions=True
        )
        success = move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        current_joints = move_group.get_current_joint_values()
        print("============ Joint values: ", current_joints)
        return True

    def move_to_box_and_attach2(self,x,y,timeout=4):
        # box located at x,y 
        # set the end effector pointing down
        #y-=0.005
        move_group = self.move_group
        move_group_hand = self.move_group_hand
        box_name = self.box_name
        # open the gripper
        self.open_gripper()
        #sleep(1.5)
        # point the gripper down
        joint_goal = move_group.get_current_joint_values()
         
        # move over box
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = 0.5
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0,
            avoid_collisions=True
        )
        constraint = moveit_msgs.msg.Constraints()
        joint_constraint = moveit_msgs.msg.JointConstraint()
        joint_constraint.joint_name = "panda_joint7"
        joint_constraint.position = joint_goal[-1]
        joint_constraint.tolerance_above = 0.001
        joint_constraint.tolerance_below = 0.001
        joint_constraint.weight = 1
        constraint.joint_constraints.append(copy.deepcopy(joint_constraint))
        joint_constraint.joint_name = "panda_joint6"
        joint_constraint.position = joint_goal[-2]
        joint_constraint.tolerance_above = 0.001
        joint_constraint.tolerance_below = 0.001
        joint_constraint.weight = 1
        constraint.joint_constraints.append(copy.deepcopy(joint_constraint))
        success = move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        current_joints = move_group.get_current_joint_values()
        sleep(1)


        # move to box
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = 0.145
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0,
            avoid_collisions=True
        )
        success = move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        current_joints = move_group.get_current_joint_values()

        # close the gripper
        sleep(0.5)
        move_group_hand.set_named_target('close')
        plan2 = move_group_hand.go()
        sleep(0.5)

        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = 0.5
        move_group.set_pose_target(wpose)
        move_group.go()
        move_group.clear_pose_targets()
        current_joints = move_group.get_current_joint_values()
        print("============ Joint values: ", current_joints)
        return True
    # Moves the end effector over a bin at location (x,y) and drops the cube in the bin
    # Takes in coordinates x and y, which is the position of a bin
    def move_to_bin(self,x,y,timeout=4):
        # box located at x,y 
        move_group = self.move_group
        move_group_hand = self.move_group_hand
        box_name = self.box_name

        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = 0.5
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0,
            avoid_collisions=True
        )
        success = move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        current_joints = move_group.get_current_joint_values()

        self.open_gripper()
        rospy.sleep(3)
        # allow 5 seconds to ensure cube is fully dropped
        self.close_gripper(60)
        #rospy.sleep(1)
        return True
    
    # Spawns cubes with a random assortment of colors
    def spawn_random_color_box(self):
        pos = 0.2
        colors = ["Blue","Red","Green", "Yellow"]
        color = colors[0]
        self.spawn_cube('cube0', pos, 0.3, color)
        pos += 0.1
        color = colors[1]
        self.spawn_cube('cube1', pos, 0.3, color)
        pos += 0.1
        color = colors[2]
        self.spawn_cube('cube2', pos, 0.3, color)
        pos += 0.1
        # spawns 5 cubes
        for i in range(3,4):
            color = random.choice(colors)
            # randomly select a color and store it in color
            self.spawn_cube('cube' + str(i), pos, 0.3, color)
            pos = pos + 0.1
        color = colors[3]
        self.spawn_cube('cube4', pos, 0.3, color)
        pos += 0.1
        return True

    # Sorts the cubes into bins of the correct color
    # Reads the global variable col to determine the order of colors
    # Blocks and bins are in predetermined locations
    # Uses the move_to_box_and_attach and move_to_bin methods to plan motion to cubes and bins
    def sort_cubes(self):
        colorOrder = self.col
        xPos = 0.2299
        yPos = 0.3299
        redBinX = 0.45
        redBinY = 0.0
        greenBinX = 0.70
        greenBinY = 0.0
        blueBinX = 0.575
        blueBinY = -0.25
        # repeats this loop for each value in the list colorOrder (each block found by the image processing)
        for elem in colorOrder:
            if elem == "red": # if block at certain location is red
                print("Moving Red Cube to Red Bin")
                self.move_to_box_and_attach(xPos,yPos)
                self.move_to_bin(redBinX,redBinY)
            elif elem == "green": # if block at certain location is green
                print("Moving Green Cube to Green Bin")
                self.move_to_box_and_attach(xPos,yPos)
                self.move_to_bin(greenBinX,greenBinY)
            elif elem == "blue": # if block at certain location is blue
                print("Moving Blue Cube to Blue Bin")
                self.move_to_box_and_attach(xPos,yPos)
                self.move_to_bin(blueBinX,blueBinY)
            elif elem == "other":# if block at certain location can't be classified by the robot
                print("Unknown Cube detected: asking user where to place it")
                self.move_to_box_and_attach(xPos,yPos)
                fingers = 0
                while fingers not in (1, 2, 3):
                    fingers = self.finger_classifier.classify()
                if fingers == 1:
                    self.move_to_bin(redBinX,redBinY)
                elif fingers == 2:
                    self.move_to_bin(greenBinX,greenBinY)
                else:
                    self.move_to_bin(blueBinX,blueBinY)
            xPos = xPos + 0.1 # move to the next block, each block is at the same y coordinate, 0.1 apart in x direction
        return True

def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Smart Robotics Project --- Riccardo Salami 20/07/2023")
        print("----------------------------------------------------------")
        panda = MoveGroupPythonInterface()
        # Spawn bins in predetermined locations
        panda.spawn_bin("bin1",0.575,-0.25, "Blue")
        panda.spawn_bin("bin2",0.45,0, "Red")
        panda.spawn_bin("bin3",0.70,0, "Green")
        # Colors can only be "Red", "Green", "Blue"
        # Spawn 5 cubes of random color
        panda.spawn_random_color_box()
        # Sleep ensures all cubes are correctly spawned before moving on
        sleep(3)
        # Spawn camera
        panda.spawn_camera("camera")
        # Sleep ensures the camera is correctly spawned before moving on
        sleep(1)
        # Write image from camera
        panda.camera_result()
        # Sleep ensures camera image is written before moving on
        while panda.isWritten == None or panda.isWritten == False:
            sleep(0.5)
        # Use image processing to determine order of colors
        panda.find_colors()
        # Sort cubes into correct bins
        panda.sort_cubes()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    


if __name__ == "__main__":
    main()
