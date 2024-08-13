#!/usr/bin/env python3
"""Small node that spins up a dynamic reconfigure server that can be used to change the
panda arm joint efforts and gripper width.
"""
import actionlib
import rospy
from dynamic_reconfigure.server import Server
from franka_gripper.msg import MoveAction, MoveGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from panda_gazebo.cfg import JointEffortsConfig

# Constants for topic names
ARM_TOPIC = "/panda_arm_joint_effort_controller/command"
JOINT_STATES_TOPIC = "joint_states"
GRIPPER_ACTION_NAME = "franka_gripper/move"


class JointEffortsDynamicReconfigureServer:
    """A small node that spins up a dynamic reconfigure server that can be used to
    change the panda arm joint efforts and gripper width.
    """

    def __init__(self):
        """Initialise JointEffortsDynamicReconfigureServer object."""
        rospy.loginfo("Starting dynamic reconfigure server...")
        self.srv = Server(JointEffortsConfig, self.callback)

        # Create joint efforts publisher.
        self.arm_pub = rospy.Publisher(ARM_TOPIC, Float64MultiArray, queue_size=10)

        # Create gripper width publisher.
        self.gripper_move_client = actionlib.SimpleActionClient(
            GRIPPER_ACTION_NAME, MoveAction
        )
        self.gripper_connected = self.gripper_move_client.wait_for_server(
            timeout=rospy.Duration(secs=5)
        )

    def callback(self, config, level):
        """Dynamic reconfigure callback function.

        Args:
            config (dict): Dictionary containing the new configuration.
            level (int): Level of the dynamic reconfigure server. Represents the
                variable that was changed or if -1 that the server was just started.
        """
        if level == -1:
            self._initialize_joint_states(config)
        else:
            self._log_reconfigure_request(config)
            if level < 7:
                self._publish_joint_efforts(config)
            elif level in [7, 8]:
                if self.gripper_connected:
                    self._send_gripper_command(config)
                else:
                    rospy.logwarn_once(
                        "Gripper commands not applied since the gripper command "
                        "action was not found."
                    )
        return config

    def _initialize_joint_states(self, config):
        """Set initial joint states.

        Args:
            config (dict): Dictionary containing the new configuration.
        """
        rospy.loginfo("Waiting for initial joint states...")
        joint_states = rospy.wait_for_message(JOINT_STATES_TOPIC, JointState)
        position_dict = dict(zip(joint_states.name, joint_states.position))
        effort_dict = dict(zip(joint_states.name, joint_states.effort))
        set_values = list(effort_dict.values())[:-2]
        set_values.append(list(position_dict.values())[-1] * 2)
        config.update(
            **dict(zip([key for key in config.keys() if key != "groups"], set_values))
        )
        rospy.loginfo("Initial joint states retrieved.")
        rospy.loginfo("Dynamic reconfigure server started.")

    def _log_reconfigure_request(self, config):
        """Log reconfigure request.

        Args:
            config (dict): Dictionary containing the new configuration.
        """
        rospy.loginfo(
            (
                "Reconfigure Request: {joint1_effort}, {joint2_effort}, "
                "{joint3_effort}, {joint4_effort}, {joint5_effort}, {joint6_effort}, "
                "{joint7_effort}, {width}, {speed}"
            ).format(**config)
        )

    def _publish_joint_efforts(self, config):
        """Publish joint efforts.

        Args:
            config (dict): Dictionary containing the new configuration.
        """
        self.arm_pub.publish(Float64MultiArray(data=list(config.values())[:7]))

    def _send_gripper_command(self, config):
        """Send gripper command.

        Args:
            config (dict): Dictionary containing the new configuration.
        """
        move_goal = MoveGoal(width=config["width"], speed=config["speed"])
        self.gripper_move_client.send_goal(move_goal)
        result = self.gripper_move_client.wait_for_result()
        if not result:
            rospy.logerr("Something went wrong while setting the gripper commands.")


if __name__ == "__main__":
    rospy.init_node("joint_efforts_reconfig_server", anonymous=False)

    try:
        JointEffortsDynamicReconfigureServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr(f"Unexpected error occurred: {e}")
