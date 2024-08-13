# Initial imports
import random
from time import sleep
from turtle import position
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

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

    # Spawns a cube by calling the spawn model service with ServiceProxy
    # Takes in a name, x and y coordinates for position, and a color to spawn with (only supports Red, Blue, and Green)
    def spawn_cube(self,name,x,y,color):
        cubeColor = 'model' + color + '.sdf'
        cube_path = os.path.join('/Koncurs','src','panda-gazebo','panda_gazebo','resources','models','cube', cubeColor)
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
        bin_path = os.path.join('/Koncurs','src','panda-gazebo','panda_gazebo','resources','models','bin', binColor)
        client = rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)
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
        camera_path = os.path.join('/Koncurs','src','panda-gazebo','panda_gazebo','resources','models','camera','model.sdf')
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
        return True
    
    # Spawns cubes with a random assortment of colors
    def spawn_random_color_box(self):
        pos = 0.2
        colors = ["Blue","Red","Green"]
        # spawns 5 cubes
        for i in range(0,5):
            color = random.choice(colors)
            # randomly select a color and store it in color
            self.spawn_cube('cube' + str(i), pos, 0.3, color)
            pos = pos + 0.1
        return True


def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to our Final Project")
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
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    rospy.spin()


if __name__ == "__main__":
    main()
