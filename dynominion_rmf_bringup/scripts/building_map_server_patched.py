#!/usr/bin/env python3
import errno
import gzip
import json
import math
import os
import sys
import yaml

from numpy import inf

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.node import Node

from rmf_building_map_msgs.srv import GetBuildingMap
from rmf_building_map_msgs.msg import BuildingMap
from rmf_building_map_msgs.msg import Level
from rmf_building_map_msgs.msg import Graph
from rmf_building_map_msgs.msg import GraphNode
from rmf_building_map_msgs.msg import GraphEdge
from rmf_building_map_msgs.msg import Place
from rmf_building_map_msgs.msg import AffineImage
from rmf_building_map_msgs.msg import Door
from rmf_building_map_msgs.msg import Lift
from rmf_building_map_msgs.msg import Param

from rmf_site_map_msgs.msg import SiteMap

from building_map.building import Building
from building_map.transform import Transform

class BuildingMapServerPatched(Node):
    def __init__(self, map_path):
        super().__init__('building_map_server')

        self.get_logger().info('loading map path: {}'.format(map_path))

        if not os.path.isfile(map_path):
            raise FileNotFoundError(
                errno.ENOENT, os.strerror(errno.ENOENT), map_path)
        self.map_dir = os.path.dirname(map_path)  # for calculating image paths

        if map_path.endswith('.building.yaml'):
            self.load_building_yaml(map_path)
        elif map_path.endswith('.gpkg'):
            self.load_geopackage(map_path)
        elif map_path.endswith('.geojson'):
            self.load_geojson(map_path)
        elif map_path.endswith('.geojson.gz'):
            self.load_geojson(map_path, True)
        else:
            self.get_logger().fatal('unknown filename suffix')
            sys.exit(1)

        self.get_building_map_srv = self.create_service(
            GetBuildingMap, 'get_building_map', self.get_building_map)

        qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        self.building_map_pub = self.create_publisher(
            BuildingMap, 'map', qos_profile=qos)

        self.site_map_pub = self.create_publisher(
            SiteMap, 'site_map', qos_profile=qos)

        self.get_logger().info('publishing map...')
        self.building_map_pub.publish(self.map_msg)
        self.site_map_pub.publish(self.site_map_msg)

        self.get_logger().info(
            'ready to serve map: "{}"  Ctrl+C to exit...'.format(
                self.map_msg.name))

    def load_building_yaml(self, map_path):
        with open(map_path, 'r') as f:
            self.yaml_dict = yaml.load(f, Loader=yaml.CLoader)
            building = Building(self.yaml_dict, 'yaml')

        self.create_map_msg(building)

        self.site_map_msg = SiteMap()
        uncompressed = building.generate_geojson()
        if 'features' in uncompressed and len(uncompressed['features']):
            data_str = json.dumps(uncompressed, sort_keys=True)
            data_gzip = gzip.compress(bytes(data_str, 'utf-8'))

            self.get_logger().info(f'compressed GeoJSON: {len(data_gzip)} B')
            self.site_map_msg.encoding = SiteMap.MAP_DATA_GEOJSON_GZ
            self.site_map_msg.data = data_gzip
        else:
            self.get_logger().info(f'unable to generate GeoJSON for this map.')

    def create_map_msg(self, building):
        self.map_msg = BuildingMap()
        self.map_msg.name = building.name
        for _, level_data in building.levels.items():
            self.map_msg.levels.append(self.level_msg(level_data))
        for _, lift_data in building.lifts.items():
            self.map_msg.lifts.append(self.lift_msg(lift_data))

    def load_geopackage(self, map_path):
        with open(map_path, 'rb') as f:
            self.site_map_msg = SiteMap()
            self.site_map_msg.encoding = SiteMap.MAP_DATA_GPKG
            self.site_map_msg.data = f.read()
        self.get_logger().info(f'read {len(self.site_map_msg.data)} byte GPKG')

    def load_geojson(self, map_path, compressed=False):
        self.site_map_msg = SiteMap()

        with open(map_path, 'rb') as f:
            json_bytes = f.read()

        self.site_map_msg.data = json_bytes
        self.get_logger().info(f'read {len(self.site_map_msg.data)} bytes')

        if compressed:
            self.site_map_msg.encoding = SiteMap.MAP_DATA_GEOJSON_GZ
            json_str = gzip.decompress(json_bytes)
            self.get_logger().info(f'decompressed to {len(json_str)} bytes')
            json_node = json.loads(json_str)
        else:
            self.site_map_msg.encoding = SiteMap.MAP_DATA_GEOJSON
            json_node = json.loads(json_bytes.decode('utf-8'))

        building = Building(json_node, 'geojson')
        self.create_map_msg(building)

    def level_msg(self, level):
        msg = Level()
        msg.name = level.name
        msg.elevation = float(level.elevation)
        if level.drawing_name:
            image = AffineImage()
            image_filename = level.drawing_name
            image_path = os.path.join(self.map_dir, image_filename)

            if os.path.exists(image_path):
                self.get_logger().info(f'opening: {image_path}')
                with open(image_path, 'rb') as image_file:
                    image.data = image_file.read()
                self.get_logger().info(f'read {len(image.data)} byte image')
                image.name = image_filename.split('.')[0]
                image.encoding = image_filename.split('.')[-1]
                
                # PATCHED: Extract image scale directly from yaml to avoid transform corruption
                # when coordinate_system: cartesian_meters is used.
                yaml_scale = None
                if hasattr(self, 'yaml_dict') and 'levels' in self.yaml_dict:
                    if level.name in self.yaml_dict['levels']:
                        level_dict = self.yaml_dict['levels'][level.name]
                        if 'drawing' in level_dict and 'scale' in level_dict['drawing']:
                            yaml_scale = float(level_dict['drawing']['scale'])

                if yaml_scale is not None:
                    image.scale = yaml_scale
                elif hasattr(level.transform, 'scale'):
                    image.scale = float(level.transform.scale)
                else:
                    image.scale = 1.0
                
                if hasattr(level.transform, 'translation'):
                    image.x_offset = float(level.transform.translation[0])
                    image.y_offset = float(level.transform.translation[1])
                else:
                    image.x_offset = float(level.transform.x)
                    image.y_offset = float(level.transform.y)
                
                image.yaw = float(level.transform.rotation)
                
                # DEBUG: Log types and values to catch the Assertion 'PyFloat_Check' failure
                self.get_logger().info(f"--- AffineImage Debug [{image.name}] ---")
                self.get_logger().info(f"  x_offset: {image.x_offset} ({type(image.x_offset)})")
                self.get_logger().info(f"  y_offset: {image.y_offset} ({type(image.y_offset)})")
                self.get_logger().info(f"  yaw:      {image.yaw} ({type(image.yaw)})")
                self.get_logger().info(f"  scale:    {image.scale} ({type(image.scale)})")
                self.get_logger().info(f"  encoding: {image.encoding} ({type(image.encoding)})")
                self.get_logger().info(f"  data len: {len(image.data)}")
                
                msg.images.append(image)
            else:
                self.get_logger().error(f'unable to open image: {image_path}')

        if len(level.doors):
            for door in level.doors:
                door_msg = Door()
                door_msg.name = door.params['name'].value
                door_msg.v1_x = float(level.transformed_vertices[door.start_idx].x)
                door_msg.v1_y = float(level.transformed_vertices[door.start_idx].y)
                door_msg.v2_x = float(level.transformed_vertices[door.end_idx].x)
                door_msg.v2_y = float(level.transformed_vertices[door.end_idx].y)
                door_msg.motion_range = math.pi * float(
                    door.params['motion_degrees'].value) / 180.0
                door_msg.motion_direction = int(door.params[
                    'motion_direction'].value)
                door_type = door.params['type'].value
                if door_type == 'sliding':
                    door_msg.door_type = door_msg.DOOR_TYPE_SINGLE_SLIDING
                elif door_type == 'hinged':
                    door_msg.door_type = door_msg.DOOR_TYPE_SINGLE_SWING
                elif door_type == 'double_sliding':
                    door_msg.door_type = door_msg.DOOR_TYPE_DOUBLE_SLIDING
                elif door_type == 'double_hinged':
                    door_msg.door_type = door_msg.DOOR_TYPE_DOUBLE_SWING
                else:
                    door_msg.door_type = door_msg.DOOR_TYPE_UNDEFINED
                msg.doors.append(door_msg)

        for i in range(0, 9):
            g = level.generate_nav_graph(i, always_unidirectional=False)
            if not g['lanes']:
                continue
            graph_msg = Graph()
            graph_msg.name = str(i)
            for v in g['vertices']:
                gn = GraphNode()
                gn.x = float(v[0])
                gn.y = float(v[1])
                gn.name = v[2]['name']

                for str_param in ["dock_name",
                                  "pickup_dispenser",
                                  "dropoff_ingestor"]:
                    if (str_param in v[2]):
                        p = Param()
                        p.name = str_param
                        p.type = p.TYPE_STRING
                        p.value_string = v[2][str_param]
                        gn.params.append(p)

                for bool_param in ["is_charger",
                                   "is_cleaning_zone",
                                   "is_holding_point",
                                   "is_parking_spot"]:
                    if (bool_param in v[2]):
                        p = Param()
                        p.name = bool_param
                        p.type = p.TYPE_BOOL
                        p.value_bool = v[2][bool_param]
                        gn.params.append(p)

                graph_msg.vertices.append(gn)

            for l in g['lanes']:
                ge = GraphEdge()
                ge.v1_idx = l[0]
                ge.v2_idx = l[1]
                if l[2]['is_bidirectional']:
                    ge.edge_type = GraphEdge.EDGE_TYPE_BIDIRECTIONAL
                else:
                    ge.edge_type = GraphEdge.EDGE_TYPE_UNIDIRECTIONAL
                if "speed_limit" in l[2]:
                    p = Param()
                    p.name = "speed_limit"
                    p.type = p.TYPE_DOUBLE
                    p.value_float = float(l[2]["speed_limit"])
                    ge.params.append(p)
                graph_msg.edges.append(ge)
            msg.nav_graphs.append(graph_msg)

        wall_graph = level.generate_wall_graph()
        msg.wall_graph.name = "WallGraph"
        for v in wall_graph['vertices']:
            gn = GraphNode()
            gn.x = v[0]
            gn.y = v[1]
            gn.name = v[2]['name']
            msg.wall_graph.vertices.append(gn)

        for w in wall_graph['walls']:
            ge = GraphEdge()
            ge.edge_type = GraphEdge.EDGE_TYPE_BIDIRECTIONAL
            ge.v1_idx = w[0]
            ge.v2_idx = w[1]

            for param_name, param_obj in w[2].items():
                p = Param()
                p.name = param_name
                p.type = param_obj.type

                if p.type == Param.TYPE_STRING:
                    p.value_string = str(param_obj.value)
                elif p.type == Param.TYPE_INT:
                    p.value_int = int(param_obj.value)
                elif p.type == Param.TYPE_DOUBLE:
                    p.value_float = float(param_obj.value)
                elif p.type == Param.TYPE_BOOL:
                    p.value_bool = bool(param_obj.value)

                ge.params.append(p)

            msg.wall_graph.edges.append(ge)

        return msg

    def lift_msg(self, lift):
        msg = Lift()
        msg.ref_x, msg.ref_y = lift.x, lift.y
        msg.name = lift.name
        msg.levels = lift.level_names

        msg.ref_yaw = lift.yaw
        msg.width = lift.width
        msg.depth = lift.depth
        for door in lift.doors:
            door_msg = Door()
            door_msg.name = door.name
            door_msg.door_type = door.door_type
            v1_x = -0.5*door.width
            v1_y = 0.0
            v2_x = 0.5*door.width
            v2_y = 0.0
            transform = Transform()
            transform.set_rotation(door.motion_axis_orientation)
            transform.set_translation(door.x, door.y)
            v1_x, v1_y = transform.transform_point([v1_x, v1_y])
            v2_x, v2_y = transform.transform_point([v2_x, v2_y])
            transform.set_rotation(lift.yaw)
            transform.set_translation(msg.ref_x, msg.ref_y)
            v1_x, v1_y = transform.transform_point([v1_x, v1_y])
            v2_x, v2_y = transform.transform_point([v2_x, v2_y])
            door_msg.v1_x = v1_x
            door_msg.v1_y = v1_y
            door_msg.v2_x = v2_x
            door_msg.v2_y = v2_y
            door_msg.motion_range = 1.571
            door_msg.motion_direction = -1
            msg.doors.append(door_msg)
        return msg

    def get_building_map(self, request, response):
        self.get_logger().info('get_building_map()')
        response.building_map = self.map_msg
        return response

def main():
    if len(sys.argv) > 1:
        map_path = sys.argv[1]
    elif 'RMF_MAP_PATH' in os.environ:
        map_path = os.environ['RMF_MAP_PATH']
    else:
        print('map path must be provided in command line or RMF_MAP_PATH env')
        sys.exit(1)

    rclpy.init()
    n = BuildingMapServerPatched(map_path)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    sys.exit(main())
