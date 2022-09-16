#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt


class TurtleBot:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)

        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

        self.r = rospy.get_param('~rayon')
        self.t= rospy.get_param('~tour')
        self.f = rospy.get_param('~freq')

    def update_pose(self, data):       
    
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def make_circle(self):
        
        r =  float(self.r)
        f =  float(self.f)
        n =  float(self.t)
        counter = 0
        
        goal_pose = Pose()
        vel_msg = Twist()
        self.rate.sleep()
        t0 = float(rospy.Time.now().to_sec())
        distance = 0
        
        angle = self.pose.theta
        
        while (2*3.1416*r*n) >= distance:
            vel_msg.linear.x = 2*3.1416*f*r
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
    
                #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 2*3.1416*f
            #print(self.pose.x)
            t1=float(rospy.Time.now().to_sec())
            distance = vel_msg.linear.x * (t1-t0)
            print("vel mesage ")
            rospy.loginfo(vel_msg)
            print(" goal pose")
            rospy.loginfo(goal_pose)
            print('No. of turnd = '+ str( distance/(2*3.1416*r)))
            
    
        #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)
        #print(distance//(2*3.1416*r))
        

    def move2goal(self):
        goal_pose = Pose()


        goal_pose.x = 8
        goal_pose.y = 5

        distance_tolerance = 0.5
        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            self.velocity_publisher.publish(vel_msg)
            print("move goal")
            rospy.loginfo(vel_msg)
            self.rate.sleep()

       
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()
        x.make_circle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
