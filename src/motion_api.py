#!/marwan/bin/env python3

import rospy
import subprocess
import os
from sensor_msgs.msg import JointState  # Message type for robot joint states
from trajectory_msgs.msg import JointTrajectory  # Message type for joint trajectory
from trac_ik_python.trac_ik import IK  # TRAC-IK is used for IK solving

class RobotMotionAPI:
    def __init__(self):
        rospy.init_node('robot_motion_api', anonymous=True)

        # Publisher for joint trajectory
        self.joint_pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)

        # Subscriber for joint states
        self.joint_state = None
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

        # IK Solver "base_link" and "wrist_3_link" UR5 configuration
        self.ik_solver = IK("base_link", "wrist_3_link") 
    
    def joint_state_callback(self, msg):
        """Callback to update the robot's joint state."""
        self.joint_state = msg

    def get_robot_state(self):
        """Return the current joint positions, velocities, and efforts."""
        if self.joint_state:
            return {
                "positions": self.joint_state.position,
                "velocities": self.joint_state.velocity,
                "efforts": self.joint_state.effort
            }
        else:
            rospy.logwarn("Joint state not available yet.")
            return None

    def run_sine_motion(self):
        subprocess.run(['python3', os.path.join(file_directory, 'sine_wave_publisher.py')])

    def run_joint_space_motion(self):
        subprocess.run(['python3', os.path.join(file_directory, 'joint_space_motion.py')])

    def run_cartesian_motion(self):
        subprocess.run(['python3', os.path.join(file_directory, 'cartesian_motion.py')])

if __name__ == '__main__':
    # Path to motion scripts
    file_directory = os.path.expanduser('~/ur5_ws/src/Gazebo_ur5_ros_Marwan/src')
    
    api = RobotMotionAPI()
    
    # Allow time for the node to start receiving joint states
    rospy.sleep(1)
    
    # Get robot state
    state = api.get_robot_state()
    if state:
        print(f"Current Joint Positions: {state['positions']}")
        print(f"Current Joint Velocities: {state['velocities']}")
        print(f"Current Joint Efforts: {state['efforts']}")

    # run motion (you can change flags as needed)
    use_sine_wave = False
    use_joint_space_motion = False
    use_cartesian_motion = True

    if use_sine_wave:
        rospy.loginfo("Running sine wave motion...")
        api.run_sine_motion()
    elif use_joint_space_motion:
        rospy.loginfo("Running joint space motion...")
        api.run_joint_space_motion()
    elif use_cartesian_motion:
        rospy.loginfo("Running Cartesian motion...")
        api.run_cartesian_motion()
