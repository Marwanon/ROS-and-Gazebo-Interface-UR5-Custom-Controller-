#!/marwan/bin/env python3

import rospy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def publish_sine_wave_joint_angles(velocity=0.1, acceleration=0.1):
    pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    traj_msg = JointTrajectory()
    traj_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    while not rospy.is_shutdown():
        t = rospy.get_time()
        traj_point = JointTrajectoryPoint()
        traj_point.positions = [math.sin(t + i) for i in range(6)]
        traj_point.velocities = [velocity for _ in range(6)]
        traj_point.accelerations = [acceleration for _ in range(6)]
        traj_point.time_from_start = rospy.Duration(0.1)
        traj_msg.points = [traj_point]
        pub.publish(traj_msg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('sine_wave_motion', anonymous=True)
    try:
        velocity = 0.2  # velocity value
        acceleration = 0.05  # acceleration value
        publish_sine_wave_joint_angles(velocity, acceleration)
    except rospy.ROSInterruptException:
        pass
