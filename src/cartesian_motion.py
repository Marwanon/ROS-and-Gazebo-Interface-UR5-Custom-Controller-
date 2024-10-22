#!/marwan/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from trac_ik_python.trac_ik import IK
import tf.transformations

def generate_cartesian_motion(pose1, pose2, steps=100):
    trajectory = []
    for i in range(steps):
        ratio = i / float(steps)
        intermediate_pose = [(1 - ratio) * p1 + ratio * p2 for p1, p2 in zip(pose1, pose2)]
        trajectory.append(intermediate_pose)
    return trajectory

def convert_cartesian_to_joint_angles(cartesian_trajectory, ik_solver):
    joint_trajectory = []
    for pose in cartesian_trajectory:
        x, y, z, roll, pitch, yaw = pose
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        qx, qy, qz, qw = quaternion
        
        joint_angles = ik_solver.get_ik(
            [0, 0, 0, 0, 0, 0],
            x, y, z, qx, qy, qz, qw
        )
        
        if joint_angles:
            joint_trajectory.append(joint_angles)
        else:
            rospy.logwarn(f"Failed to find IK solution for pose: {pose}")
    
    return joint_trajectory

def publish_trajectory(joint_trajectory, velocity=0.1, acceleration=0.1):
    pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    
    traj_msg = JointTrajectory()
    traj_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    for positions in joint_trajectory:
        traj_point = JointTrajectoryPoint()
        traj_point.positions = positions
        traj_point.velocities = [velocity] * len(positions)
        traj_point.accelerations = [acceleration] * len(positions)
        traj_point.time_from_start = rospy.Duration(0.1)
        traj_msg.points = [traj_point]
        pub.publish(traj_msg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('cartesian_motion', anonymous=True)

    ik_solver = IK("base_link", "wrist_3_link")
    
    pose1 = [0.5, 0.2, 0.3, 0, 1.57, 0]
    pose2 = [0.6, 0.2, 0.3, 0, 1.57, 0]
    
    rospy.loginfo("Generating Cartesian space motion...")
    cartesian_trajectory = generate_cartesian_motion(pose1, pose2)
    
    rospy.loginfo("Converting Cartesian poses to joint angles using IK...")
    joint_trajectory = convert_cartesian_to_joint_angles(cartesian_trajectory, ik_solver)
    
    rospy.loginfo("Publishing Cartesian motion as joint trajectory...")
    try:
        publish_trajectory(joint_trajectory, velocity=0.01, acceleration=0.01)
    except rospy.ROSInterruptException:
        pass

