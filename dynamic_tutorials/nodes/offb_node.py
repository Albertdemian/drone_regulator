#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseStamped, AccelWithCovarianceStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from tf import transformations
from dynamic_reconfigure.msg import Config



mavros.set_namespace()

# callback method for state sub
current_state = State() 
offb_set_mode = SetMode
def state_cb(state):
    global current_state
    current_state = state

# callback method for drone position
drone_pose = PoseStamped()
def pose_state(my_drone_pos):
    global drone_pose 
    drone_pose = my_drone_pos
    #print(drone_pos)

# callback method for drone velocity
drone_vel = TwistStamped()
def vel_state(my_drone_vel):
    global drone_vel
    drone_vel = my_drone_vel
    #print(drone_vel)

# callback method for drone setpoint position 
drone_goal = PoseStamped()
def goal(my_drone_goal):
    global drone_goal
    drone_goal = my_drone_goal
    #rospy.loginfo(pose)



#Tuner callback function 
Tuner = Config()
def Tuner_call(config):
    global Tuner
    #rospy.loginfo("Config set to {velocity_x}, {velocity_y}, {velocity_z}, {velocity_phi}".format(**config))
    Tuner = config
    #print(Tuner)

#Twist callback function
Twister = Config()
def Twist_call(config):
    global Twister
    Twister = config
    print(Twister.doubles[0])


#publishers 
local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
velocity_pub = rospy.Publisher(mavros.get_topic('setpoint_velocity', 'cmd_vel'), TwistStamped, queue_size=10)

#subscribers
state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode) 
curr_pose_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),PoseStamped,pose_state)
curr_vel_sub = rospy.Subscriber(mavros.get_topic('local_position', 'velocity_local'),TwistStamped,vel_state)
goal_sub = rospy.Subscriber("/goal", PoseStamped, goal)
Tuner_sub = rospy.Subscriber("/Tuner/parameter_updates", Config, Tuner_call)
Twist_sub = rospy.Subscriber("/Twist/parameter_updates", Config, Twist_call)

#clients 
#client = dynamic_reconfigure.client.Client("dynamic_tutorials", timeout=30, config_callback=client_call)

#setting up initial pose for drone to go to when it's armed with Offboard controller
pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 2





def position_control():
    rospy.init_node('offb_node', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    
    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()

    # controller parameters 
    Kp = 1.25
    Kd = 15
    

    #to store error values of step(i-1)
    old_error_x = 0
    old_error_y = 0
    old_error_z = 0
    old_error_rz = 0 
    #numerical differentiation step h 
    h = 1

    #velocity variable to be controlled  
    velocity = TwistStamped()


    last_request = rospy.get_rostime()

    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = now 
        else:
            if not current_state.armed and (now - last_request > rospy.Duration(5.)):
               arming_client(True)
               last_request = now 

        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.armed != current_state.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)
        if prev_state.mode != current_state.mode: 
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state

        try:
            if len(Tuner.doubles)>0:
                Kp = Tuner.doubles[0].value
                Kd = Tuner.doubles[1].value

            # extracting marker (set point) position components
            drone_goal_x = drone_goal.pose.position.x
            drone_goal_y = drone_goal.pose.position.y
            drone_goal_z = drone_goal.pose.position.z 

            drone_goal_rz= drone_goal.pose.orientation.z 

            #extracting current position components
            drone_pose_x = drone_pose.pose.position.x 
            drone_pose_y = drone_pose.pose.position.y 
            drone_pose_z = drone_pose.pose.position.z 

            drone_pose_rz= drone_pose.pose.orientation.z

            #calculating errors 
            error_x = drone_goal_x - drone_pose_x  
            error_y = drone_goal_y - drone_pose_y 
            error_z = drone_goal_z - drone_pose_z 

            #converting from Quanternion to euler angles 
            drone_pose_quan = (
                drone_pose.pose.orientation.x,
                drone_pose.pose.orientation.y,
                drone_pose.pose.orientation.z,
                drone_pose.pose.orientation.w
            )

            drone_goal_quan = (
                drone_goal.pose.orientation.x,
                drone_goal.pose.orientation.y,
                drone_goal.pose.orientation.z,
                drone_goal.pose.orientation.w
            )

            drone_pose_eul = transformations.euler_from_quaternion(drone_pose_quan)
            drone_goal_eul = transformations.euler_from_quaternion(drone_goal_quan)
            #print(drone_pose_eul)

            error_rz = drone_goal_eul[2] - drone_pose_eul[2] 

            #controller output for velocity
            velocity.twist.linear.x =  (error_x*Kp) + ((old_error_x-error_x)/h * Kd) + Twister.doubles[0].value
            velocity.twist.linear.y =  (error_y*Kp) + ((old_error_y-error_y)/h * Kd) + Twister.doubles[1].value
            velocity.twist.linear.z =  (error_z*Kp) + ((old_error_z-error_z)/h * Kd) + Twister.doubles[2].value

            velocity_out_rz = (error_rz*Kp) + ((old_error_rz-error_rz)/h * Kd) + Twister.doubles[3].value
            
            

            velocity.twist.angular.x = 0
            velocity.twist.angular.y = 0
            velocity.twist.angular.z = velocity_out_rz
            
            
            #position setpoint 
            pose.pose.position.x = drone_goal_x
            pose.pose.position.y = drone_goal_y 
            pose.pose.position.z = drone_goal_z
            pose.pose.orientation.z = drone_goal_rz

            #forcing limit speed condition 
            if (velocity.twist.linear.x > 2):
                velocity.twist.linear.x = 2
            elif (velocity.twist.linear.x < -2):
                velocity.twist.linear.x = -2
            
            if (velocity.twist.linear.y > 2):
                velocity.twist.linear.y = 2
            elif (velocity.twist.linear.y < -2):
                velocity.twist.linear.y = -2

            if (velocity.twist.linear.z > 1.5):
                velocity.twist.linear.y = 1.5
            elif (velocity.twist.linear.y < -1.5):
                velocity.twist.linear.y = -1.5

            if (velocity.twist.angular.z > 1):
                velocity.twist.angular.z = 1
            elif(velocity.twist.angular.z <-1):
                velocity.twist.angular.z = -1

            print(Kp, Kd)
            #print(Tuner.doubles)
            #print(Twister.doubles)

            old_error_x = error_x
            old_error_y = error_y
            old_error_z = error_z
            old_error_rz = error_rz
        except rospy.ROSInterruptException:
            pass

        #synchronizing timestamps 
        pose.header.stamp = rospy.Time.now()
        drone_goal.header.stamp = rospy.Time.now()
        velocity.header.stamp = rospy.Time.now()

        #Publishing desired position and controller velocity 
        local_pos_pub.publish(pose)
        velocity_pub.publish(velocity)

        rate.sleep()

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass