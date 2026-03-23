"""
Graph Visualizer Node
Publish RViz MarkerArray for the semantic room graph and exploration plan.
  - Colored spheres at each region centroid
  - Text labels with region ID / semantic label
  - Lines for graph edges
  - Numbered arrows for exploration plan order
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import Int32MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from semantic_map_msgs.msg import SemanticGraph

_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


PALETTE = [
    (0.12, 0.47, 0.71),
    (1.00, 0.50, 0.05),
    (0.17, 0.63, 0.17),
    (0.84, 0.15, 0.16),
    (0.58, 0.40, 0.74),
    (0.55, 0.34, 0.29),
    (0.89, 0.47, 0.76),
    (0.50, 0.50, 0.50),
    (0.74, 0.74, 0.13),
    (0.09, 0.75, 0.81),
]


class GraphVisualizerNode(Node):
    def __init__(self):
        super().__init__('graph_visualizer_node')

        self.graph = None
        self.plan = []

        self.sub_graph = self.create_subscription(
            SemanticGraph, '/semantic_map/graph_with_poses', self._on_graph, _LATCHED_QOS
        )
        self.sub_topo = self.create_subscription(
            SemanticGraph, '/semantic_map/topological_graph', self._on_topo, _LATCHED_QOS
        )
        self.sub_plan = self.create_subscription(
            Int32MultiArray, '/semantic_map/exploration_plan', self._on_plan, _LATCHED_QOS
        )

        self.pub_markers = self.create_publisher(
            MarkerArray, '/semantic_map/markers', 10
        )

        self.timer = self.create_timer(2.0, self._publish)
        self.get_logger().info('GraphVisualizerNode ready.')

    def _on_graph(self, msg: SemanticGraph):
        self.graph = msg

    def _on_topo(self, msg: SemanticGraph):
        if self.graph is None:
            self.graph = msg

    def _on_plan(self, msg: Int32MultiArray):
        self.plan = list(msg.data)

    def _publish(self):
        if self.graph is None:
            return

        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        del_marker.header.frame_id = 'map'
        del_marker.header.stamp = stamp
        ma.markers.append(del_marker)

        for i, region in enumerate(self.graph.regions):
            c = PALETTE[i % len(PALETTE)]

            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = stamp
            sphere.ns = 'regions'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = region.entry_pose
            sphere.pose.position.z = 0.3
            sphere.scale.x = 0.6
            sphere.scale.y = 0.6
            sphere.scale.z = 0.6
            sphere.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=0.85)
            ma.markers.append(sphere)

            label = region.semantic_label or region.region_type_geom
            txt = Marker()
            txt.header.frame_id = 'map'
            txt.header.stamp = stamp
            txt.ns = 'labels'
            txt.id = i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose = region.entry_pose
            txt.pose.position.z = 0.9
            txt.scale.z = 0.35
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = f'R{region.region_id}: {label}'
            ma.markers.append(txt)

        for j, edge in enumerate(self.graph.edges):
            src = self._find_region(edge.source_region_id)
            tgt = self._find_region(edge.target_region_id)
            if src is None or tgt is None:
                continue

            line = Marker()
            line.header.frame_id = 'map'
            line.header.stamp = stamp
            line.ns = 'edges'
            line.id = j
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.08
            line.color = ColorRGBA(r=0.9, g=0.9, b=0.2, a=0.7)
            p1 = Point(
                x=src.entry_pose.position.x,
                y=src.entry_pose.position.y,
                z=0.2,
            )
            p2 = Point(
                x=tgt.entry_pose.position.x,
                y=tgt.entry_pose.position.y,
                z=0.2,
            )
            line.points = [p1, p2]
            ma.markers.append(line)

        for k, rid in enumerate(self.plan):
            region = self._find_region(rid)
            if region is None:
                continue
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = stamp
            arrow.ns = 'plan'
            arrow.id = k
            arrow.type = Marker.TEXT_VIEW_FACING
            arrow.action = Marker.ADD
            arrow.pose = region.entry_pose
            arrow.pose.position.z = 1.4
            arrow.scale.z = 0.4
            arrow.color = ColorRGBA(r=0.0, g=1.0, b=0.4, a=1.0)
            arrow.text = f'#{k+1}'
            ma.markers.append(arrow)

        self.pub_markers.publish(ma)

    def _find_region(self, region_id):
        if self.graph is None:
            return None
        for r in self.graph.regions:
            if r.region_id == region_id:
                return r
        return None


def main(args=None):
    rclpy.init(args=args)
    node = GraphVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
