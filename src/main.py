#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import random
import turtlesim.srv


class TurtleBot:

    def __init__(self):
        rospy.init_node('catch_and_catch', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

        self.r = rospy.get_param('~rayon')
        self.t= rospy.get_param('~tour')
        self.f = rospy.get_param('~freq')
        self.d_t = rospy.get_param('~distance_tolerance')
    
    def update_pose (self, data):
        self.pose = data
        self.pose.x = round(self.pose.x,6)
        self.pose.y = round(self.pose.y,6)


    def make_circle(self):
        
        r =  float(self.r)
        f =  float(self.f)
        n =  float(self.t)
        counter = 0
        
        goal_pose = Pose()

        self.Turtle_spawn(goal_pose,'turtle1','turtle2')
        vel_msg = Twist()
        self.rate.sleep()
        t0 = float(rospy.Time.now().to_sec())
        distance = 0
        
        angle = self.pose.theta
        
        while (2*3.1416*r*n) >= distance:
            vel_msg.linear.x = 2*3.1416*f*r
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
    
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 2*3.1416*f
            
            t1=float(rospy.Time.now().to_sec())
            distance = vel_msg.linear.x * (t1-t0)
            
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        

        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)
        self.Turtle_kill('turtle2')
       

    def move2goal(self): 
        goal_pose = Pose()

        while(True):
    
            
            goal_pose.x=random.randrange(2,10)
            goal_pose.y=random.randrange(2,10)
            

            self.Turtle_spawn(goal_pose,'turtle1','turtle2')

            vel_msg = Twist()

            while self.euclidean_distance(goal_pose) >= float(self.d_t):

                vel_msg.linear.x = self.linear_vel(goal_pose)
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_vel(goal_pose)

                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()

        
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)

            self.Turtle_kill('turtle2')  

    def Turtle_spawn(self,goal,killer_name,spawn_name): 
        self.spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        self.turtle_name = rospy.get_param(killer_name, spawn_name)
        self.spawner(goal.x, goal.y, 0, self.turtle_name) 
        

    def Turtle_kill(self,killer_name): 
        self.killer=rospy.ServiceProxy('kill',turtlesim.srv.Kill)
        self.killer(killer_name)


    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=8):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)    
        
        
    def main(self):
        self.make_circle()
        self.move2goal() 

    

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
