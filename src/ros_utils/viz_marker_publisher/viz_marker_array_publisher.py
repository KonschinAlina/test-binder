#!/usr/bin/env python
# -*- coding: utf-8 -*-

import tf
import time
import rospy
import inflect
import threading
from std_msgs.msg import ColorRGBA
import urdf_parser_py.urdf as parser
from geometry_msgs.msg import Vector3, Pose
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from urdf_parser_py.urdf import URDF, Box, Cylinder, Sphere, Mesh



# Publishes an Array of visualization marker which represent objects.
class VizMarkerArrayPublisher:

    def __init__(self, topic_name="/pycram/viz_marker", interval=0.1):
        self.topic_name = topic_name
        self.interval = interval
        self.pub = rospy.Publisher(self.topic_name, MarkerArray, queue_size=10)
        #self.thread = threading.Thread(target=self._publish)
        #self.kill_event = threading.Event()

    
    # Publishes the highlight MarkerArray once.
    def _publish(self, urdf_file:URDF, yaml_data:dict, obj_name:str, trans) -> None:
        marker_array = self.create_marker_array(urdf_file, yaml_data, obj_name, trans)
        self.pub.publish(marker_array)
        return marker_array

    
    # Stops the publishing of the MarkerArray by setting the kill event.
    def _stop_publishing(self, marker_array: MarkerArray):
        
        for m in marker_array.markers:
            m.action = Marker.DELETE
        
        self.pub.publish(marker_array)
        #self.thread.join()

    # Creates a MarkerArray 
    def create_marker_array(self, urdf_file:URDF, yaml_data:dict, obj_name:str, trans):
        marker_array = MarkerArray()
        mid = 0

        # wenn es sich um einen der anderen tables handelt --> obj_name = table
        # aber obj.get('name', '') == long_table == obj_name
        # if popcorn_table --> obj_name
        # if obj_name other aber ends with ... --> obj_name = table
        if obj_name.endswith(("table", "counter", "island")): #and "popcorn_table" not in obj_name:
            #if "popcorn_table" in obj_name:
                #obj_name = "table"
        # popcorn table geht hier nicht rein !!!
            
            msg = Marker()
            msg.header.frame_id = "map"
            msg.header.stamp = rospy.Time.now()
            msg.ns = f"{obj_name}_highlight"
            msg.id = mid
            mid += 1
            msg.type = Marker.CUBE
            msg.action = Marker.ADD
    
    
            # yaml + pose of center link  not obj[x_pos] etc because for tables thats a corner link
            for key, obj in yaml_data['tables'].items():
                if not isinstance(obj, dict):
                    continue
                if obj.get('name', '') == obj_name:
                    msg.scale = Vector3(obj['depth'], obj['width'], obj['height'])
                    msg.pose.position.x = trans[0]
                    msg.pose.position.y = trans[1]
                    msg.pose.position.z = obj['height'] / 2.0
                    q = quaternion_from_euler(0, 0, obj['z_rot'])
                    msg.pose.orientation.x = q[0]
                    msg.pose.orientation.y = q[1]
                    msg.pose.orientation.z = q[2]
                    msg.pose.orientation.w = q[3]
    
            msg.color = ColorRGBA(1.0, 1.0, 0, 1.0)
            marker_array.markers.append(msg)            

        #elif "dishwasher" in obj_name:
            
            
        else:
            for link in urdf_file.links:
                if obj_name not in link.name:
                    continue
    
                if not link.visual:
                    continue
    
                vis_list = link.visual if isinstance(link.visual, list) else [link.visual]
                for vis in vis_list:
                    msg = Marker()
                    msg.header.frame_id = "map"
                    msg.header.stamp = rospy.Time.now()
                    msg.ns = f"{obj_name}_highlight"
                    msg.id = mid
                    mid += 1
                    msg.action = Marker.ADD
             
                    geom = vis.geometry
                    if isinstance(geom, Box):
                        msg.type = Marker.CUBE
                        msg.scale = Vector3(geom.size[0],
                                            geom.size[1],
                                            geom.size[2])
                    elif isinstance(geom, Cylinder):
                         msg.type = Marker.CYLINDER
                         msg.scale = Vector3(geom.radius * 2,
                                             geom.radius * 2,
                                             geom.length)
    
                    elif isinstance(geom, Sphere):
                         msg.type = Marker.SPHERE
                         msg.scale = Vector3(geom.radius * 2,
                                             geom.radius * 2,
                                             geom.radius * 2)
                        
                    elif isinstance(geom, Mesh):
                         msg.type = Marker.MESH_RESOURCE
                         msg.mesh_resource = "file://" + geom.mesh.file_name
                         scale = geom.scale if hasattr(geom, "scale") else [1,1,1]
                         msg.scale = Vector3(scale[0],scale[1],scale[2])
    

                # Get coordinates from yaml file  
                p = inflect.engine()#
                if obj_name != "couch":
                        yaml_key = p.plural(obj_name)
                        for _, obj in yaml_data[yaml_key].items():
                            if not isinstance(obj, dict):
                                continue  # skip strings: perception_postfix, amount
                            if obj.get('name', '') == obj_name:
                                msg.pose.position.x = obj['x_pos']
                                msg.pose.position.y = obj['y_pos']
                                msg.pose.position.z = obj['height'] / 2.0
                                q = quaternion_from_euler(0, 0, obj['z_rot'])
                                msg.pose.orientation.x = q[0]
                                msg.pose.orientation.y = q[1]
                                msg.pose.orientation.z = q[2]
                                msg.pose.orientation.w = q[3]

                else:
                    yaml_key = "couch"
                    for _, obj in yaml_data[yaml_key].items():
                        if not isinstance(obj, dict):
                            continue  # skip strings: perception_postfix, amount
                        if obj.get('name', '') == obj_name:
                            msg.pose.position.x = obj['x_pos']
                            msg.pose.position.y = obj['y_pos']
                            msg.pose.position.z = obj['height'] / 2.0
                            q = quaternion_from_euler(0, 0, obj['z_rot'])
                            msg.pose.orientation.x = q[0]
                            msg.pose.orientation.y = q[1]
                            msg.pose.orientation.z = q[2]
                            msg.pose.orientation.w = q[3]

                # Set highlight color
                msg.color = ColorRGBA(1.0, 1.0, 0, 1.0)
                marker_array.markers.append(msg)
                
            
        return marker_array

                
                    
