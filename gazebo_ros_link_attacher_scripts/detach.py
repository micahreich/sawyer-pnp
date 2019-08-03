#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


if __name__ == '__main__':

    rospy.init_node('detach')
    attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Attach)
    attach_srv.wait_for_service()

    # Remove static link between cube and sawyer EEF
    rospy.loginfo("Detaching cube1 and sawyer end effector")
    req = AttachRequest()
    req.model_name_1 = "cube1"
    req.link_name_1 = "link"
    req.model_name_2 = "sawyer"
    req.link_name_2 = "right_l6"

    attach_srv.call(req)
