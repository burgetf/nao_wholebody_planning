#!/usr/bin/env python  

import roslib
roslib.load_manifest('rrt_connect_planner_example')
import rospy
import math
import tf
import visualization_msgs

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, PointStamped, PolygonStamped

global outfile
global listener

def callback(data):
  global outfile
  global listener
  pos = data.pose.position
  rospy.loginfo("Reveiced: " + str(pos.x) + " " + str(pos.y) + " " + str(pos.z));
  p = PoseStamped()
  p.pose = data.pose
  p.header = data.header
  listener.waitForTransform("r_sole", "r_gripper", p.header.stamp, rospy.Duration(0.5))
  com_pose = listener.transformPose('r_sole', p)
  zero = PointStamped()
  zero.header.frame_id = "r_gripper"
  zero.header.stamp = data.header.stamp
  gripper_point = listener.transformPoint('r_sole', zero)
  
  com_pos = com_pose.pose.position
  rospy.loginfo("Writing: %f %f %f %f",com_pos.x, com_pos.y, gripper_point.point.x, gripper_point.point.y);
  outfile.write("%f %f %f %f\n" % (com_pos.x, com_pos.y, gripper_point.point.x, gripper_point.point.y))
  
def callbackPoly(data):
  global listener
  polyfile = open("polygon_log.txt", 'w')
  listener.waitForTransform(data.header.frame_id, "r_sole", data.header.stamp, rospy.Duration(0.5))
  for p in data.polygon.points:
    p_s = PointStamped()
    p_s.point = p
    p_s.header = data.header
    p_st = listener.transformPoint('r_sole',p_s)
    polyfile.write("%f %f\n" % (p_st.point.x, p_st.point.y))
    rospy.loginfo("Writing Poly: %f %f",p_st.point.x, p_st.point.y);
  
  polyfile.close()
  

if __name__ == '__main__':
  rospy.init_node('eval_trajectories')
  global listener
  listener = tf.TransformListener()
  
  global outfile
  outfile = open("trajectory_log.txt", 'w')
  rospy.Subscriber("/test_stability/projected_com", Marker, callback)
  rospy.Subscriber("/test_stability/support_polygon", PolygonStamped, callbackPoly)
  
  rospy.spin()
  




    #rate = rospy.Rate(10.0)
    #while not rospy.is_shutdown():
        #try:
            #(trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #continue

        #angular = 4 * math.atan2(trans[1], trans[0])
        #linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        #turtle_vel.publish(turtlesim.msg.Velocity(linear, angular))

        #rate.sleep() 
