#!/usr/bin/env python
import numpy as np
import rospy 
import math 
from tf.transformations import *
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion

if __name__ == '__main__':
  rospy.init_node('vicon_vins_listener')

  tfBuffer = tf2_ros.Buffer()
  listener = tf2_ros.TransformListener(tfBuffer)

  rate = rospy.Rate(10.0)
  vicon_ready = False 
  vins_ready = False 
  broadcaster = tf2_ros.StaticTransformBroadcaster()
  static_transformStamped = geometry_msgs.msg.TransformStamped()
  while not rospy.is_shutdown():
    try:
      trans = tfBuffer.lookup_transform('vicon/world', 'vicon/visquad1/visquad1', rospy.Time())
      print(trans)
      vicon_ready = True 
      if vicon_ready: # and  vins_ready:
        
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "vicon/world"
        static_transformStamped.child_frame_id = "world"

        tf_pos = trans.transform.translation;
        tf_quat = trans.transform.rotation;
	tf_quat_np = np.array([tf_quat.x, tf_quat.y, tf_quat.z, tf_quat.w])

	print("tf_quat: ", tf_quat_np)

        #roll_90 = quaternion_from_euler(math.pi, 0,0)
	roll_90 = np.array([0, 0, -np.sqrt(2)/2, np.sqrt(2)/2])
	print("roll90: ", roll_90)

        new_quat = quaternion_multiply(roll_90, tf_quat_np)
        print("new quat", new_quat)

	static_transformStamped.transform.translation = tf_pos;
	static_transformStamped.transform.rotation.x = new_quat[0];
	static_transformStamped.transform.rotation.y = new_quat[1];
	static_transformStamped.transform.rotation.z = new_quat[2];
	static_transformStamped.transform.rotation.w = new_quat[3];


        res = broadcaster.sendTransform(static_transformStamped)
	print(res)
	print(static_transformStamped)
        rospy.spin()
    	break
 
    except Exception as e:
      print(e)
     
      continue
       
    # try:
    #   trans_vins = tfBuffer.lookup_transform('world', 'body', rospy.Time())
    #   print(trans_vins)
    #   vins_ready = True 

    except Exception as e:
      print(e)
      continue
 
    rate.sleep()




