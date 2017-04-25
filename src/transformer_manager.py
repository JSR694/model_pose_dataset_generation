
import tf
import rospy
from time import sleep


class TransformerManager():
    def __init__(self):
        self.transformer = tf.TransformerROS(True, rospy.Duration(10.0))

    def add_transform(self, pose_in_world_frame, frame_id, child_frame_id):
        transform_msg = tf.msg.geometry_msgs.msg.TransformStamped()
        transform_msg.transform.translation.x = pose_in_world_frame.position.x
        transform_msg.transform.translation.y = pose_in_world_frame.position.y
        transform_msg.transform.translation.z = pose_in_world_frame.position.z

        transform_msg.transform.rotation.x = pose_in_world_frame.orientation.x
        transform_msg.transform.rotation.y = pose_in_world_frame.orientation.y
        transform_msg.transform.rotation.z = pose_in_world_frame.orientation.z
        transform_msg.transform.rotation.w = pose_in_world_frame.orientation.w
        transform_msg.child_frame_id = child_frame_id
        transform_msg.header.frame_id = frame_id

        self.transformer.setTransform(transform_msg)
        sleep(.1)


    def transform_pose(self, pose, old_frame, new_frame):

        transform_msg = tf.msg.geometry_msgs.msg.PoseStamped()
        transform_msg.pose.position.x = pose.position.x
        transform_msg.pose.position.y = pose.position.y
        transform_msg.pose.position.z = pose.position.z

        transform_msg.pose.orientation.x = pose.orientation.x
        transform_msg.pose.orientation.y = pose.orientation.y
        transform_msg.pose.orientation.z = pose.orientation.z
        transform_msg.pose.orientation.w = pose.orientation.w

        transform_msg.header.frame_id = old_frame

        result = self.transformer.transformPose(new_frame, transform_msg)
        sleep(0.1)
        return result
