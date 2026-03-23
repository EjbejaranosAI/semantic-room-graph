"""
Graph Store Node
Persist the semantic graph as JSON and support load/save operations.
Subscribes to the latest semantic graph and writes it to disk.
Provides a service to reload the graph from file.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import json
import os

from std_srvs.srv import Trigger
from semantic_map_msgs.msg import SemanticGraph

_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


class GraphStoreNode(Node):
    def __init__(self):
        super().__init__('graph_store_node')

        self.declare_parameter('output_dir', '/tmp/semantic_map')
        self.declare_parameter('filename', 'semantic_graph.json')
        self.declare_parameter('auto_save', True)

        self.output_dir = self.get_parameter('output_dir').value
        self.filename = self.get_parameter('filename').value
        self.auto_save = self.get_parameter('auto_save').value

        os.makedirs(self.output_dir, exist_ok=True)
        self.filepath = os.path.join(self.output_dir, self.filename)
        self.latest_graph = None

        self.sub_graph = self.create_subscription(
            SemanticGraph, '/semantic_map/semantic_graph', self._on_graph, _LATCHED_QOS
        )

        self.pub_loaded_graph = self.create_publisher(
            SemanticGraph, '/semantic_map/semantic_graph', _LATCHED_QOS
        )

        self.srv_save = self.create_service(
            Trigger, '/semantic_map/save_graph', self._save_cb
        )
        self.srv_load = self.create_service(
            Trigger, '/semantic_map/load_graph', self._load_cb
        )

        self.get_logger().info(
            f'GraphStoreNode ready. Path: {self.filepath}'
        )

    def _on_graph(self, msg: SemanticGraph):
        self.latest_graph = msg
        if self.auto_save:
            self._save_to_disk(msg)

    def _save_to_disk(self, graph: SemanticGraph):
        data = {
            'map_frame': graph.map_frame,
            'map_resolution': graph.map_resolution,
            'regions': [],
            'edges': [],
        }

        for r in graph.regions:
            region_dict = {
                'region_id': r.region_id,
                'region_type_geom': r.region_type_geom,
                'status': r.status,
                'semantic_label': r.semantic_label,
                'label_confidence': r.label_confidence,
                'entry_pose': {
                    'x': r.entry_pose.position.x,
                    'y': r.entry_pose.position.y,
                    'z': r.entry_pose.position.z,
                    'qx': r.entry_pose.orientation.x,
                    'qy': r.entry_pose.orientation.y,
                    'qz': r.entry_pose.orientation.z,
                    'qw': r.entry_pose.orientation.w,
                },
                'inspection_pose': {
                    'x': r.inspection_pose.position.x,
                    'y': r.inspection_pose.position.y,
                    'z': r.inspection_pose.position.z,
                    'qx': r.inspection_pose.orientation.x,
                    'qy': r.inspection_pose.orientation.y,
                    'qz': r.inspection_pose.orientation.z,
                    'qw': r.inspection_pose.orientation.w,
                },
                'ocr_text': list(r.ocr_text),
                'neighbors': list(r.neighbors),
            }
            data['regions'].append(region_dict)

        for e in graph.edges:
            edge_dict = {
                'source_region_id': e.source_region_id,
                'target_region_id': e.target_region_id,
                'transition_pose': {
                    'x': e.transition_pose.position.x,
                    'y': e.transition_pose.position.y,
                    'z': e.transition_pose.position.z,
                    'qx': e.transition_pose.orientation.x,
                    'qy': e.transition_pose.orientation.y,
                    'qz': e.transition_pose.orientation.z,
                    'qw': e.transition_pose.orientation.w,
                },
                'transition_width': e.transition_width,
            }
            data['edges'].append(edge_dict)

        with open(self.filepath, 'w') as f:
            json.dump(data, f, indent=2)

        self.get_logger().debug(f'Graph saved: {len(data["regions"])} regions')

    def _save_cb(self, request, response):
        if self.latest_graph is not None:
            self._save_to_disk(self.latest_graph)
            response.success = True
            response.message = f'Saved to {self.filepath}'
        else:
            response.success = False
            response.message = 'No graph received yet'
        return response

    def _load_cb(self, request, response):
        if not os.path.exists(self.filepath):
            response.success = False
            response.message = f'File not found: {self.filepath}'
            return response

        try:
            with open(self.filepath, 'r') as f:
                data = json.load(f)

            graph = SemanticGraph()
            graph.header.stamp = self.get_clock().now().to_msg()
            graph.header.frame_id = data.get('map_frame', 'map')
            graph.map_frame = data.get('map_frame', 'map')
            graph.map_resolution = data.get('map_resolution', 0.05)

            from semantic_map_msgs.msg import RegionNode, RegionEdge

            for rd in data.get('regions', []):
                node = RegionNode()
                node.region_id = rd['region_id']
                node.region_type_geom = rd.get('region_type_geom', '')
                node.status = rd.get('status', 'unexplored')
                node.semantic_label = rd.get('semantic_label', '')
                node.label_confidence = rd.get('label_confidence', 0.0)

                ep = rd.get('entry_pose', {})
                node.entry_pose.position.x = ep.get('x', 0.0)
                node.entry_pose.position.y = ep.get('y', 0.0)
                node.entry_pose.position.z = ep.get('z', 0.0)
                node.entry_pose.orientation.x = ep.get('qx', 0.0)
                node.entry_pose.orientation.y = ep.get('qy', 0.0)
                node.entry_pose.orientation.z = ep.get('qz', 0.0)
                node.entry_pose.orientation.w = ep.get('qw', 1.0)

                ip = rd.get('inspection_pose', {})
                node.inspection_pose.position.x = ip.get('x', 0.0)
                node.inspection_pose.position.y = ip.get('y', 0.0)
                node.inspection_pose.position.z = ip.get('z', 0.0)
                node.inspection_pose.orientation.x = ip.get('qx', 0.0)
                node.inspection_pose.orientation.y = ip.get('qy', 0.0)
                node.inspection_pose.orientation.z = ip.get('qz', 0.0)
                node.inspection_pose.orientation.w = ip.get('qw', 1.0)

                node.ocr_text = rd.get('ocr_text', [])
                node.neighbors = rd.get('neighbors', [])
                graph.regions.append(node)

            for ed in data.get('edges', []):
                edge = RegionEdge()
                edge.source_region_id = ed['source_region_id']
                edge.target_region_id = ed['target_region_id']

                tp = ed.get('transition_pose', {})
                edge.transition_pose.position.x = tp.get('x', 0.0)
                edge.transition_pose.position.y = tp.get('y', 0.0)
                edge.transition_pose.position.z = tp.get('z', 0.0)
                edge.transition_pose.orientation.x = tp.get('qx', 0.0)
                edge.transition_pose.orientation.y = tp.get('qy', 0.0)
                edge.transition_pose.orientation.z = tp.get('qz', 0.0)
                edge.transition_pose.orientation.w = tp.get('qw', 1.0)

                edge.transition_width = ed.get('transition_width', 0.0)
                graph.edges.append(edge)

            self.latest_graph = graph
            self.pub_loaded_graph.publish(graph)

            response.success = True
            response.message = (
                f'Loaded {len(graph.regions)} regions from {self.filepath}'
            )
        except Exception as e:
            response.success = False
            response.message = f'Load error: {e}'

        return response


def main(args=None):
    rclpy.init(args=args)
    node = GraphStoreNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
