import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_matrix
import numpy as np
from math import pi
import os
from std_msgs.msg import Float32

offset_transform = np.array([[0,-1, 0, 0],
                            [1, 0, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

transformtion_file_path = '/home/yara/camera_ws/src/graspnet-baseline/doc/example_data/transformation_matrix.npy'
width_path = r"/home/yara/camera_ws/src/graspnet-baseline/doc/example_data/width.txt"

def rotate_4x4_rotation_matrix_x(rotation_matrix_4x4, angle = pi/4):
    # Extract the upper-left 3x3 rotation submatrix
    rotation_submatrix = rotation_matrix_4x4[:3, :3]
    
    # Define the rotation matrix around the x-axis
    rotation_x = np.array([[1, 0, 0],
                           [0, np.cos(angle), -np.sin(angle)],
                           [0, np.sin(angle), np.cos(angle)]])
    
    # Perform rotation on the upper-left 3x3 rotation submatrix
    rotated_rotation_submatrix = np.dot(rotation_submatrix, rotation_x.T)
    
    # Update the upper-left 3x3 rotation submatrix in the 4x4 rotation matrix
    rotated_rotation_matrix_4x4 = np.copy(rotation_matrix_4x4)
    rotated_rotation_matrix_4x4[:3, :3] = rotated_rotation_submatrix
    
    return rotated_rotation_matrix_4x4

def get_transform():
    # First transformation
    transform_msg = TransformStamped()
    transform_msg.header.frame_id = "camera_link_temp"
    transform_msg.child_frame_id = "obj_grasp"

    # Populate the transformation matrix for the first transform
    transform_matrix = np.load(transformtion_file_path)

    transform_matrix = np.dot(transform_matrix, offset_transform)
    transform_matrix = rotate_4x4_rotation_matrix_x(transform_matrix)
    
    quaternion = quaternion_from_matrix(transform_matrix)
    transform_msg.transform.rotation.x = quaternion[0]
    transform_msg.transform.rotation.y = quaternion[1]
    transform_msg.transform.rotation.z = quaternion[2]
    transform_msg.transform.rotation.w = quaternion[3]
    transform_msg.transform.translation.x = transform_matrix[0][3]
    transform_msg.transform.translation.y = transform_matrix[1][3]
    transform_msg.transform.translation.z = transform_matrix[2][3]

    return transform_msg

def get_width():
    with open(width_path, 'r') as width_file:
        width = float(width_file.read())

    width_msg = Float32()
    width_msg.data = width
    return width_msg

if __name__ == '__main__':
    try:
        # Initialize a ROS node
        rospy.init_node('transform_publisher')

        # Create a TransformBroadcaster
        broadcaster = tf2_ros.TransformBroadcaster()
        width_publisher = rospy.Publisher('grasp_width', Float32, queue_size=10)

        # Publish the transformations continuously
        rate = rospy.Rate(10)  # Adjust the publishing rate as needed
        while not rospy.is_shutdown():
            if (os.path.exists(transformtion_file_path) and os.path.exists(width_path)):
                current_time = rospy.Time.now()
                transform_msg = get_transform()
                width_msg = get_width()
                
                transform_msg.header.stamp = current_time
                broadcaster.sendTransform(transform_msg)
                width_publisher.publish(width_msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        os.remove(transformtion_file_path)
        os.remove(width_path)
        pass
