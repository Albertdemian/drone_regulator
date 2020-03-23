#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseStamped, AccelWithCovarianceStamped, TwistStamped, PoseArray
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from tf import transformations
from dynamic_reconfigure.msg import Config
import numpy as np



mavros.set_namespace()

drone_pose = PoseStamped()
def pose_state(my_drone_pos):
    global drone_pose 
    drone_pose = my_drone_pos


obstacle_pose = PoseArray()
def obstacle(my_obstacle_pose):
    global obstacle_pose
    obstacle_pose= my_obstacle_pose
    #print(obstacle_pose)



def rep_force(dist_to_obs, c_rep,r_field):
    if dist_to_obs<r_field:
        field = c_rep / dist_to_obs - c_rep / r_field
    else:
        field=0

    return field

def distance_from_obs(obstacle_pose, drone_pose):
    drone_pose_x = drone_pose.pose.position.x 
    drone_pose_y = drone_pose.pose.position.y 
    drone_pose_z = drone_pose.pose.position.z 

    curr_drone_pos = [drone_pose_x,drone_pose_y,drone_pose_z]

    num_obs = len(obstacle_pose.poses)
    dist_to_obs = []
    for i in range(num_obs):
        xx = obstacle_pose.poses[i].position.x
        yy = obstacle_pose.poses[i].position.y
        zz = obstacle_pose.poses[i].position.z
        obs_pose = [xx,yy,zz]
        pos_vec = [drone_pose_x-xx, drone_pose_y-yy, drone_pose_z-zz]

        dist_to_obs_sq = pos_vec[0]**2 + pos_vec[1]**2 +pos_vec[2]**2 
        dist_to_obs.append( np.sqrt(dist_to_obs_sq))
    return dist_to_obs


curr_pose_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),PoseStamped,pose_state)
obstacles = rospy.Subscriber('obstacles', PoseArray, obstacle)

rep_velocity_pub = rospy.Publisher('field', TwistStamped, queue_size=10)

def rep_field():
    rospy.init_node('rep_field', anonymous=True)
    rate = rospy.Rate(20.0)
    velocity = TwistStamped()

    #initial values for repulsive field
    c_rep = 0.0001
    r_field = 1.5
    dist_to_obs = 0.0001

    while not rospy.is_shutdown():

        
        obstacle_distance = distance_from_obs(obstacle_pose, drone_pose)
        dist_to_obs = np.min(obstacle_distance)
        repulsion = rep_force(dist_to_obs, c_rep,r_field)

        print(obstacle_distance)
        print( dist_to_obs)
        print(obstacle_pose.poses[1].position)

        velocity.twist.linear.x = repulsion
        velocity.twist.linear.y = repulsion
        velocity.twist.linear.z = repulsion


        velocity.header.stamp = rospy.Time.now()

        print(velocity)
        #rep_velocity_pub.publish(velocity)
        rate.sleep()


if __name__ == '__main__':
    try:
        rep_field()
    except rospy.ROSInterruptException:
        pass