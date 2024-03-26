#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import csv

from sensor_msgs.msg import NavSatFix 
import time

class GenerateWaypoints(Node):
    def __init__(self):
        super().__init__('waypoint_gen')
        self.waypoint_pub = self.create_publisher(NavSatFix, 'vidyut/waypoints', 10)
        
    def publish_waypoint(self,lat,long):
        fix = NavSatFix()
        fix.latitude = float(lat)
        fix.longitude = float(long)
        self.waypoint_pub.publish(fix)

def main(args=None):
    rclpy.init(args=args)
    
    generator = GenerateWaypoints()

    file = open('/home/joel/igvc_ws/src/global_nav/routes/test_route.csv') 
    csvreader = csv.reader(file)
    header = next(csvreader) 
    for row in csvreader:
        generator.publish_waypoint(row[0],row[1])
        print("Published latitude =",row[0]," longitude =",row[1])
        time.sleep(0.5)

    generator.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
