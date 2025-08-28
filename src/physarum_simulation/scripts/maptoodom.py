#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('map_to_odom_tf')

    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    ROBOTS = ['robot_0', 'robot_1', 'robot_2']
    TRANSFORMS = {
        'robot_0': (-7.0, -7.0, 0.0),  # x, y, theta
        'robot_1': (-2.0, -7.0, 0.0),
        'robot_2': (-7.0, -1.0, 0.0),
 
    }

    transforms = []
    for robot in ROBOTS:
        x, y, yaw = TRANSFORMS[robot]
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = f"{robot}/odom"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0  # sem rotação
        transforms.append(t)

    tf_broadcaster.sendTransform(transforms)
    rospy.spin()

