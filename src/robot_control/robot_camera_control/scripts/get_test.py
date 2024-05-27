#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
import numpy as np


def get_transform():
    rospy.init_node('tf_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(
                'base_link_rm', 'camera_color_optical_frame', rospy.Time(0))

            # Extract the translation
            translation = trans.transform.translation
            tx, ty, tz = translation.x, translation.y, translation.z

            # Extract the rotation (quaternion)
            rotation = trans.transform.rotation
            qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w

            # Convert quaternion to rotation matrix
            R = np.array([
                [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
                [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
            ])

            # Construct the 4x4 transformation matrix
            T = np.eye(4)
            T[0:3, 0:3] = R
            T[0:3, 3] = [tx, ty, tz]
            # target_position = np.dot(camera2robot[0:3, 0:3], target) + camera2robot[0:3, 3:]
            # target_position = target_position[0:3, 0]
            # Output the transformation matrix
            rospy.loginfo(
                "Transformation matrix from base_link to camera_link:")
            rospy.loginfo("\n" + str(T))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF Exception")
            rate.sleep()
            continue

        rate.sleep()


if __name__ == '__main__':
    # try:
    #     get_transform()
    # except rospy.ROSInterruptException:
    #     pass
    import numpy as np

    # 定义4x4的转换矩阵
    camera_pose = np.array([
        [0.01387119, -0.99947549, -0.02926496, 0.04179944],
        [0.99981123, 0.01346563, 0.01400994, -0.04704901],
        [-0.01360852, -0.02945377, 0.9994735, 0.87734411],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # 将矩阵写入文件
    np.savetxt('camera_pose.txt', camera_pose, delimiter=' ')
