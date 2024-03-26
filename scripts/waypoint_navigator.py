#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import queue
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import numpy as np


class WaypointNavigator(Node):

    '''
    In this code, by waypoint I am referring to (latitude, longitude) representation and by
    goal I am referring to the point in world coordinates of the bot.
    transform : publishes to topic which NavSat Transform Node is listening to
    '''

    def __init__(self):
        super().__init__('waypoint_nav')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.waypoint_queue = queue.Queue()

        initial_pose = self.navigator.initial_pose.pose.position
        self.current_goal = [initial_pose.x,initial_pose.y]

        # radius around the gps waypoint to reach until next point is given
        self.thresh = 2

        # Publishers
        self.navsat_pub = self.create_publisher(NavSatFix, 'vidyut/navsat', 10)

        # Subscribers
        self.waypoint_sub = self.create_subscription(
            NavSatFix,
            'vidyut/waypoints',
            self.add_to_queue,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'vidyut/odom',
            self.check_proximity,
            10)
        
        self.goal_sub = self.create_subscription(
            Odometry,
            'navsat_transformer/transformed_waypoint',
            self.navigate_to_goal,
            10)
        
    def add_to_queue(self, data: NavSatFix):

        fix = NavSatFix()
        
        '''
        TODO:
            Properly add all parameters like frame_id etc to NavSatFix msg
        '''
        
        fix.latitude  = data.latitude
        fix.longitude = data.longitude
        print("Waypoint Added to Queue!\nlatitude =",data.latitude," longitude =",data.longitude)

        self.waypoint_queue.put(fix)
    
    def check_proximity(self, data: Odometry):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        d = np.sqrt(((x-self.current_goal[0])**2)+((y-self.current_goal[1])**2))

        if d < self.thresh and not self.goal_queue.empty():
            # publish to topic listened to by navsat transform node
            self.navsat_pub.publish(self.waypoint_queue.get())
        

    def navigate_to_goal(self, data: Odometry):
        self.current_goal[0] = data.pose.pose.position.x
        self.current_goal[1] = data.pose.pose.position.y

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'odom'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = data.pose.pose.position.x
        goal_pose.pose.position.y = data.pose.pose.position.y

        self.navigator.cancelTask()
        self.navigator.goToPose(goal_pose)
        print("Navigating to Goal!\nx =",data.pose.pose.position.x," y =",data.pose.pose.position.y)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            print("Distance to goal:",feedback.distance_remaining,"meters")
         
            if feedback.navigation_duration > 600:
                self.navigator.cancelTask()

                

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')






        
    