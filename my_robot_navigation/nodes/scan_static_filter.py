#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Point
from scipy.spatial import KDTree
import copy

class StaticScanFilter:
    def __init__(self):
        rospy.init_node('scan_static_filter', anonymous=True)
        
        # Parameters
        self.THRESHOLD_DYNAMIC = rospy.get_param('~threshold_dynamic', 0.15)  # Meters
        self.ODOM_FRAME = rospy.get_param('~odom_frame', 'odom')
        self.LASER_FRAME = rospy.get_param('~laser_frame', 'sensor_laser_link')  # Updated to match URDF
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(20.0))  # Increased buffer to 20 seconds
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publishers and subscribers
        self.pub_static = rospy.Publisher('/scan_static', LaserScan, queue_size=10)
        self.pub_dynamic = rospy.Publisher('/scan_dynamic', LaserScan, queue_size=10)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=10)
        
        # Data storage
        self.prev_scan_points_odom = None

    def transform_scan_to_points(self, scan_msg):
        """Converts LaserScan to a list of 2D points in the sensor frame."""
        points = []
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if scan_msg.range_min < r < scan_msg.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append((x, y))
            angle += scan_msg.angle_increment
        return np.array(points)

    def scan_callback(self, scan_msg):
        current_time = scan_msg.header.stamp
        laser_frame = scan_msg.header.frame_id

        try:
            # Use the latest available transform to avoid extrapolation errors
            trans = self.tf_buffer.lookup_transform(self.ODOM_FRAME, laser_frame, rospy.Time(0), timeout=rospy.Duration(2.0))
            transform_time = trans.header.stamp

            # Convert laser points to odom frame
            points_laser = self.transform_scan_to_points(scan_msg)
            if points_laser.size == 0:
                self.pub_static.publish(scan_msg)
                self.pub_dynamic.publish(LaserScan(header=scan_msg.header))
                return

            points_odom = []
            for x, y in points_laser:
                p_laser = PointStamped()
                p_laser.header.frame_id = laser_frame
                p_laser.header.stamp = transform_time  # Use transform time to avoid extrapolation
                p_laser.point = Point(x, y, 0.0)
                p_odom = tf2_geometry_msgs.do_transform_point(p_laser, trans)
                points_odom.append((p_odom.point.x, p_odom.point.y))
            current_scan_points_odom = np.array(points_odom)

            # Create masks for static and dynamic points
            is_static = np.ones(len(points_odom), dtype=bool)
            is_dynamic = np.zeros(len(points_odom), dtype=bool)

            # Check for dynamic points using previous scan
            if self.prev_scan_points_odom is not None and self.prev_scan_points_odom.size > 0:
                kdtree = KDTree(self.prev_scan_points_odom)
                dist, _ = kdtree.query(current_scan_points_odom)
                is_dynamic = dist > self.THRESHOLD_DYNAMIC
                is_static = ~is_dynamic

            # Publish filtered scans
            self.publish_filtered_scans(scan_msg, is_static, is_dynamic)

            # Update previous scan
            self.prev_scan_points_odom = current_scan_points_odom

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF Exception in scan filter: %s", e)
            self.pub_static.publish(scan_msg)
            self.pub_dynamic.publish(LaserScan(header=scan_msg.header))

    def publish_filtered_scans(self, original_scan, is_static, is_dynamic):
        """Publishes filtered scans."""
        static_scan = copy.deepcopy(original_scan)
        dynamic_scan = copy.deepcopy(original_scan)
        static_ranges = list(original_scan.ranges)
        dynamic_ranges = list(original_scan.ranges)

        point_idx = 0
        for i, r in enumerate(original_scan.ranges):
            if original_scan.range_min < r < original_scan.range_max:
                if not is_static[point_idx]:
                    static_ranges[i] = float('inf')
                if not is_dynamic[point_idx]:
                    dynamic_ranges[i] = float('inf')
                point_idx += 1
            else:
                static_ranges[i] = r
                dynamic_ranges[i] = r

        static_scan.ranges = static_ranges
        dynamic_scan.ranges = dynamic_ranges
        self.pub_static.publish(static_scan)
        self.pub_dynamic.publish(dynamic_scan)

if __name__ == '__main__':
    try:
        StaticScanFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
