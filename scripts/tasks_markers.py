#!/usr/bin/env python3
import os, yaml, rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray

class TasksMarkers(Node):
    def __init__(self):
        super().__init__('tasks_markers')
        tasks_path = os.path.join(get_package_share_directory('mrs_auction'), 'config', 'tasks.yaml')
        tasks = yaml.safe_load(open(tasks_path)).get('tasks', [])
        self.pub = self.create_publisher(MarkerArray, '/auction/tasks_markers', 10)

        self.arr = MarkerArray()
        for idx, t in enumerate(tasks):
            m = Marker()
            m.header.frame_id = 'world'
            m.ns = 'tasks'
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(t['location'][0])
            m.pose.position.y = float(t['location'][1])
            m.pose.position.z = 0.0
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.r, m.color.g, m.color.b, m.color.a = (0.1, 0.6, 1.0, 0.9)
            m.lifetime.sec = 0
            self.arr.markers.append(m)

            # optional label
            label = Marker()
            label.header.frame_id = 'world'
            label.ns = 'task_labels'
            label.id = 1000 + idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = m.pose.position.x
            label.pose.position.y = m.pose.position.y
            label.pose.position.z = 0.25
            label.scale.z = 0.15
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 0.9
            label.text = t.get('id', f'T{idx+1}')
            self.arr.markers.append(label)

        # Publish periodically to ensure subscribers see it
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(f'Publishing {len(tasks)} task markers')

    def publish_markers(self):
        # Update timestamp
        now = self.get_clock().now().to_msg()
        for m in self.arr.markers:
            m.header.stamp = now
        self.pub.publish(self.arr)

def main(args=None):
    rclpy.init(args=args)
    node = TasksMarkers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()