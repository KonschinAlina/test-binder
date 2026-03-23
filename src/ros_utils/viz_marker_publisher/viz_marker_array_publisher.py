#!/usr/bin/env python
# -*- coding: utf-8 -*-

import tf
import time
import rospy
import numpy as np 
import threading      
from std_msgs.msg import ColorRGBA
import urdf_parser_py.urdf as parser
from geometry_msgs.msg import Vector3, Pose
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from urdf_parser_py.urdf import URDF, Box, Cylinder, Sphere, Mesh
import tf.transformations as tft


# This class publishes an Array of visualization marker which represent objects.

class VizMarkerArrayPublisher:

    def __init__(self, topic_name="/pycram/viz_marker", interval=0.1):
        self.topic_name = topic_name
        self.interval = interval
        self.pub = rospy.Publisher(self.topic_name, MarkerArray, queue_size=10)

    
    # Publishes the highlight MarkerArray once
    def _publish(self, obj_name:str, target_link_obj, trans, rot) -> MarkerArray:
        marker_array = self.create_marker_array(obj_name, target_link_obj, trans, rot)
        self.pub.publish(marker_array)
        return marker_array

    
    # Stops the publishing of the MarkerArray by setting the kill event
    def _stop_publishing(self, marker_array: MarkerArray):
        for m in marker_array.markers:
            m.action = Marker.DELETE
        
        self.pub.publish(marker_array)

    # Creates a MarkerArray 
    def create_marker_array(self, obj_name:str, target_link_obj, trans, rot):
        marker_array = MarkerArray()
        mid = 0

        # Checks for visual tags of a link
        if not target_link_obj or not target_link_obj.visual:
            rospy.logwarn(f"No visual link found for {obj_name}. Cannot highlight without visual link.")
            return marker_array

        # Retrieves the visual link(s)
        vis_list = target_link_obj.visual if isinstance(target_link_obj.visual, list) else [target_link_obj.visual]

        # Creates a Marker for every visual link
        for vis in vis_list:
            msg = Marker()
            msg.header.frame_id = "map"
            msg.header.stamp = rospy.Time.now()
            msg.ns = f"{obj_name}_highlight"
            msg.id = mid
            mid += 1
            msg.action = Marker.ADD
    
            # Determines the geometry and scale  
            geom = vis.geometry
            if isinstance(geom, Box):
                msg.type = Marker.CUBE
                msg.scale = Vector3(*geom.size)
                
            elif isinstance(geom, Cylinder):
                msg.type = Marker.CYLINDER
                msg.scale = Vector3(geom.radius * 2, geom.radius * 2, geom.length)
                
            elif isinstance(geom, Mesh):
                msg.type = Marker.MESH_RESOURCE
                msg.mesh_resource = geom.filename
                scale = getattr(geom, "scale", [1.0, 1.0, 1.0]) or [1.0, 1.0, 1.0] # take original size
                msg.scale = Vector3(*scale)

                mesh_path = geom.filename
                
    
            # Calculates the rotation via rotation matrix and determines the pose
            if vis.origin:
                local_offset = vis.origin.xyz

                rot_matrix = tft.quaternion_matrix(rot)
                local_vec = np.array([local_offset[0], local_offset[1], local_offset[2], 1.0])

                rotated_offset = np.dot(rot_matrix, local_vec)
                                        
                msg.pose.position.x = trans[0] + rotated_offset[0]
                msg.pose.position.y = trans[1] + rotated_offset[1]
                msg.pose.position.z = trans[2] + rotated_offset[2]

                if vis.origin.rpy:
                    q_vis = tft.quaternion_from_euler(*vis.origin.rpy)
                    combined_rot = tft.quaternion_multiply(rot, q_vis)
                    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = combined_rot
                else:
                    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = rot
                       
            else:
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = trans
                msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = rot
    
            msg.color = ColorRGBA(1.0, 1.0, 0, 0.6)
            marker_array.markers.append(msg)

        
        return marker_array

                
                    
