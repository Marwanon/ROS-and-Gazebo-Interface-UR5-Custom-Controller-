#!/marwan/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def generate_joint_motion(point1, point2, velocity=0.1, acceleration=0.1, steps=100):
    trajectory = []
    for i in range(steps):
        ratio = i / float(steps)
        intermediate_point = [(1 - ratio) * p1 + ratio * p2 for p1, p2 in zip(point1, point2)]
        trajectory.append(intermediate_point)
    return trajectory

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
    rospy.init_node('joint_space_motion', anonymous=True)
    
    point1 = [0, 0, 0, 0, 0, 0]
    point2 = [1, 1, 1, 1, 1, 1]
    
    rospy.loginfo("Generating joint space motion...")
    joint_trajectory = generate_joint_motion(point1, point2, velocity=0.1, acceleration=0.1)
    
    rospy.loginfo("Publishing joint space motion...")
    try:
        publish_trajectory(joint_trajectory, velocity=0.1, acceleration=0.1)
    except rospy.ROSInterruptException:
        pass



