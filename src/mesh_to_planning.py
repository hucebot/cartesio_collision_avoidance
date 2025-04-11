#!/usr/bin/env python3

import rospy
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest

srv = None

def collision_object_cb(msg):
    global srv
    msg.header.frame_id = "world"
    print(msg)
    rospy.wait_for_service("cartesian/collision_avoidance/apply_planning_scene")
    req = ApplyPlanningSceneRequest()
    req.scene.is_diff = True
    req.scene.world.collision_objects.append(msg)
    srv(req)

def main():
    global srv
    rospy.init_node("ps_pub", anonymous=True)
    rospy.Subscriber("cartesian/collision_objects", CollisionObject, collision_object_cb)
    srv = rospy.ServiceProxy("cartesian/collision_avoidance/apply_planning_scene", ApplyPlanningScene)
    rospy.spin()

if __name__ == "__main__":
    main()
