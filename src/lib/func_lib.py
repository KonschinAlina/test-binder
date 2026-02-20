#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
func_lib.py

A lib that provides simple functions used for querying and highlighting and creating trajectories on a sem map.
"""

import re 
import os
import tf
import sys
import yaml
import math
import types
import rospy
import rosnode
import trimesh
import logging
import warnings
import tempfile
import subprocess
from subprocess import DEVNULL
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import MarkerArray, Marker
sys.path.append("/home/jovyan/work/src/")
import ros_utils.viz_marker_publisher.viz_marker_array_publisher as MA

from owlready2 import *

ma = MA.VizMarkerArrayPublisher()
 
#########################################################################################################
class FuncLib:

    def __init__(self, topic_name="/pycram/viz_marker"):
        self.topic_name = topic_name
        self.pub = rospy.Publisher(self.topic_name, MarkerArray, queue_size=10)
        if not rospy.core.is_initialized():
            rospy.init_node('func_lib', anonymous=True)
             
#########################################################################################################
# Check whether map server is running
#
    def map_server_running(self):
        nodes = rosnode.get_node_names()
        if '/map_server' in nodes:
            return True
        else:
            return False

########################################################################################################
# POSE
#
# get the pose of an object with tf
#
# Input: object frame 
# Output: pose of object frame
#
#
    def object_pose(self, object_frame, ref_frame="map"):
        # 0. Check whether map is loaded
        # if not self.map_server_running():
        #     rospy.logwarn(f"No semantic map loaded!")
        #     return
        #else: 
            listener = tf.TransformListener()
            rospy.sleep(1.0)
            try:
                trans, rot = listener.lookupTransform(ref_frame, object_frame, rospy.Time(0))
                return [ref_frame, [trans[0], trans[1], trans[2]], [rot[0], rot[1],rot[2], rot[3]]]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return None

########################################################################################################
# HIGHLIGHT
#
# creates an MarkerArray on top of a object to highlight it
#
# Input: name of individual to be highlighted
# Output: object gets highlighted and notice with highlighting duration
#
#
    def highlight(self, indiv_name: str, ref_frame="map"):
        # 0. Check whether map is loaded
        # if not self.map_server_running():
        #     rospy.logwarn(f"No semantic map loaded!")
        #     return
    
        # 1. Get link-individual matching
        # [{"link": "...", "urdf_obj": "..."}, ...]
        matching = self.link_indiv_matching()
        #print(matching)
                        
        # 2. Get name of target_link and urdf_obj matched to individual
        if indiv_name in matching:
                target_link_data = matching[indiv_name]
                #print(f"indiv_name: '{indiv_name}'")
                #print(f"target_link_data: '{target_link_data}'")
            
                target_link = target_link_data["link_name"]
                target_link_obj = target_link_data["urdf_obj"]
            
        else:
            rospy.logwarn(f"Could not find a link for individual '{indiv_name}'")
            return
        
       
        # 3. Get pose of target_link
        target_frame = f"iai_kitchen/{target_link}"
        #print(f"target_frame:{target_frame}")
        target_link_pose = self.object_pose(target_frame, ref_frame)
        #print(f"target_link_pose: '{target_link_pose}'")
        
        if target_link_pose is None:
            print(f"Could not get pose of target_link: '{target_link}'!")
            return

        trans = target_link_pose[1]
        rot = target_link_pose[2]
    
        # 4. Publish markers (with flashing effect)
        flash_interval = 0.5
        for _ in range(2):
            # create marker
            markers = ma._publish(indiv_name, target_link_obj, trans, rot) 
            rospy.sleep(flash_interval)
            ma._stop_publishing(markers)
            rospy.sleep(flash_interval)
    
        # 5. Highlight for 4 seconds
        markers = ma._publish(indiv_name, target_link_obj, trans,rot)
        duration = 4.0
        rospy.sleep(duration)
        ma._stop_publishing(markers)
        
        return f"Successfully highlighted {indiv_name} using link: '{target_link}'."
        

########################################################################################################
# HIGHLIGHT A LIST OF OBJECTS
#
# create MarkerArray on top of list of objects    
#
# Input: the name of a class; all objects of this class are to be highlighted
# Output: objects get highlighted and notice with highlighting duration
#
#
    def highlight_list(self, selected_class, ref_frame="map", duration=4.0):

        # 0. Check whether map is loaded
        # if not self.map_server_running():
        #     rospy.logwarn(f"No semantic map loaded!")
        #     return
        
        # 1. Get link to individual matching
        matching = self.link_indiv_matching()

        # 2. Load ontology and get individuals from onto
        onto = get_ontology('/home/jovyan/work/prolog/BA-class_extraction1.owl').load()

        results = onto.search(iri=f"*{selected_class.capitalize()}*")

        if results:
            target_class = results[0]
            indivs = target_class.instances()
            indivs_names = [indiv.name for indiv in indivs]

        all_markers = MarkerArray()
        listen = tf.TransformListener()
        rospy.sleep(0.5)

        global_id = 0
        
        # 3. For each object: 
        for indiv_name in indivs_names:
            if indiv_name not in matching:
                rospy.logwarn(f"Could not find a link for individual '{indiv_name}' in matching")
                continue

            targets_data = matching[indiv_name]
            target_link_obj = targets_data["urdf_obj"]
            target_link = targets_data["link_name"]
                
            if not target_link_obj.visual:
                continue
                
            target_frame = f"iai_kitchen/{target_link}"
            target_link_pose = self.object_pose(target_frame, ref_frame)

            if target_link_pose is None:
                print(f"Could not get pose of target_link: '{target_link}'!")
                return
            
            trans = target_link_pose[1]
            rot = target_link_pose[2]
            
            markers = ma.create_marker_array(
                indiv_name,
                target_link_obj,
                trans,
                rot
            )

            for m in markers.markers:
                m.id = global_id
                global_id += 1
    
            # append to combined MarkerArray of all objects
            all_markers.markers.extend(markers.markers)

        if not all_markers.markers:
            return "No object list found for highlighting!"
            
        # 4. effect: flashing
        flash_interval = 0.5
        for _ in range(2):
            # create markers 
            for m in all_markers.markers:
               m.action = Marker.ADD
            ma.pub.publish(all_markers)
            rospy.sleep(flash_interval)
        
            # delete markers
            for m in all_markers.markers:
                m.action = Marker.DELETE
            ma.pub.publish(all_markers)
            rospy.sleep(flash_interval)
        
        # publish for a few seconds
        for m in all_markers.markers:
            m.action = Marker.ADD
        ma.pub.publish(all_markers)
        rospy.sleep(duration)
        
        # delete markers
        ma._stop_publishing(all_markers)
               
        return f"Highlighted {len(indivs_names)} objects of class {selected_class} for {duration} seconds!"


########################################################################################################
## HIGHLIGHT A LIST OF OBJECTS
#
# A function that creates Markers on top of list of objects    
#
# Input: a list of individual names
# Output: objects get highlighted and notice with highlighting duration
#
#
    def highlight_indiv_list(self, indivs_names, ref_frame="map", duration=4.0):

        # 0. Check whether map is loaded
        # if not self.map_server_running():
        #     rospy.logwarn(f"No semantic map loaded!")
        #     return

        if not isinstance(indivs_names, list):
            indivs_names = [indivs_names]
        
        # 1. Get link to individual matching
        matching = self.link_indiv_matching()


        all_markers = MarkerArray()
        listen = tf.TransformListener()
        rospy.sleep(0.5)

        global_id = 0
        
        # 2. For each object: 
        for indiv_name in indivs_names:
            if indiv_name not in matching:
                rospy.logwarn(f"Could not find a link for individual '{indiv_name}' in matching")
                continue

            targets_data = matching[indiv_name]
            target_link_obj = targets_data["urdf_obj"]
            target_link = targets_data["link_name"]
                
            if not target_link_obj.visual:
                continue
                
            target_frame = f"iai_kitchen/{target_link}"
            target_link_pose = self.object_pose(target_frame, ref_frame)

            if target_link_pose is None:
                print(f"Could not get pose of target_link: '{target_link}'!")
                return
            
            trans = target_link_pose[1]
            rot = target_link_pose[2]
            
            markers = ma.create_marker_array(
                indiv_name,
                target_link_obj,
                trans,
                rot
            )

            for m in markers.markers:
                m.id = global_id
                global_id += 1
    
            # append to combined MarkerArray of all objects
            all_markers.markers.extend(markers.markers)

        if not all_markers.markers:
            return "No object list found for highlighting!"
            
        # 4. effect: flashing
        flash_interval = 0.5
        for _ in range(2):
            # create markers 
            for m in all_markers.markers:
               m.action = Marker.ADD
            ma.pub.publish(all_markers)
            rospy.sleep(flash_interval)
        
            # delete markers
            for m in all_markers.markers:
                m.action = Marker.DELETE
            ma.pub.publish(all_markers)
            rospy.sleep(flash_interval)
        
        # publish for a few seconds
        for m in all_markers.markers:
            m.action = Marker.ADD
        ma.pub.publish(all_markers)
        rospy.sleep(duration)
        
        # delete markers
        ma._stop_publishing(all_markers)
               
        return f"Highlighted {len(indivs_names)} individual(s) for {duration} seconds!"
########################################################################################################
# TRAJECTORY
#
# trajectories show how doors and drawers are opended based on their joint
#
# Input: name of child link of a joint = an indiv from database
# Output: trajectory is shown and notice with duration
#
#
    def trajectory(self, child_link, ref_frame="map", duration=4.0):
        
        # 0. Check whether map is loaded
        # if not self.map_server_running():
        #     rospy.logwarn(f"No semantic map loaded!")
        #     return

        # 1. get urdf from param server
        urdf_string = rospy.get_param("/kitchen_description")
        urdf = URDF.from_xml_string(urdf_string)

        # 2. Load ontology and get individuals from onto
        onto = get_ontology('/home/jovyan/work/prolog/BA-class_extraction1.owl').load()

        
        # 3. Get the joint where child_link is child link
        # 3.5. Get joint type
        joint_type = None
        joint_name = None
        for joint in urdf.joints:
            if joint.child == child_link:
                # get name and type of target joint 
                joint_name = joint.name.lower()
                joint_type = joint.type.lower()
                break

        
        if joint_type is None:
            rospy.logwarn(f"No joint found where joint.child is {child_link}")
            return

        point_list = []
        
        # 4. Create trajectory based on joint type
########### FIXED ### 
        if joint_type == "fixed":
            rospy.logwarn(f"No trajectory for fixed joints!")
            return
            
            
########### REVOLUTE ###         
        if joint_type == "revolute" or joint_type == "continuous":

            # Get the individual object to child_link
            indiv_obj = onto.search_one(iri=f"*{child_link}")
            

            # Get individuals that do not open left/right
            is_knob = any("Knob" in cls.name for cls in indiv_obj.is_a) if indiv_obj else False

            # a door that belongs to a dishwasher or oven 
            is_vertical_door = any(x in child_link.lower() for x in ["oven", "dishwasher", "dish_washer"]) and not is_knob

            # a door that opens horizontally e.g. door, fridge
            is_horizontal_door = any(x in child_link.lower() for x in ["fridge", "door"]) and not is_knob and not is_vertical_door
            
            # Get the radius at the height of the handle and axis
            if is_knob:
                radius = 0.03
            else:  
                mesh_width = self.get_door_width_from_mesh(joint.child, urdf)

                if mesh_width:
                    radius = mesh_width
                    #print(f"radius mesh: {radius}")
                else:
                    radius = self.get_radius_from_handle(joint.child, urdf) or 0.5
                    #print(f"radius else from handle: {radius}")

            # Get the axis and rotation of the joint
            axis = joint.axis or [0, 0, 1] 
            r,p,y = joint.origin.rpy if joint.origin else [0, 0, 0]

            # Get the upper and lower joint limit
            limit = getattr(joint, "limit", None)

            # in case both limits are 0.0 
            lower_limit = limit.lower if (limit and limit.lower != limit.upper) else 0.0
            upper_limit = limit.upper if (limit and limit.lower != limit.upper) else 1.57 # 90°

            #lower_limit = -1.57
            #upper_limit = 0.0
            
            
            ## old version, works only if handle names are consistent
            ## i.e. coffee_table_handle_joint
            # handle_joint_name = f"{child_link}_handle_joint"
            # handle_joint = next((j for j in urdf.joints if j.name == handle_joint_name), None)
            #-----------------------------

            handle_joint = None
            handle_link_name = None
            
            # Check for parts of the individual that are handles
            if indiv_obj and hasattr(indiv_obj, "hasPart"):
                for part in indiv_obj.hasPart:
                    if any(cls.name == "Handle" for cls in part.is_a):
                        handle_link_name = part.name
            
            # Get the handle joints where the child link is a handle link
            if handle_link_name:
                handle_joint = next((j for j in urdf.joints if j.child == handle_link_name), None)
                pass
            else:
                rospy.logwarn("No handles found for this individual!")
               

            # Handle height used for "else" in prismatic 
            handle_z = 0.0

            
            if handle_joint:
                handle_origin = handle_joint.origin.xyz
                handle_z = handle_joint.origin.xyz[2] if joint.origin else 0.0
                   
            else:
                print(f"No handle joints found that have child: {child_link}).")
                
                #handle_z = joint.origin.xyz[2] if joint.origin else 0.0

                if radius and radius > 0.05:
                    handle_origin = [radius, 0,0]
                    
                else:
                    handle_origin = [0.4,0,0]
                    
                if is_knob:
                    handle_origin = [0.02,0,0]
                
            
            # handle transformation 
            h_x_trans = handle_origin[0]
            h_y_trans = handle_origin[1] * math.cos(r) - handle_origin[2] * math.sin(r)
            h_z_trans = handle_origin[1] * math.sin(r) + handle_origin[2] * math.cos(r)

            # Define params based on door type/ joint type
            if is_horizontal_door: #or abs(axis[2]) > 0.5:
                #print(f"l.392: if is horizontal")
               #start_angle = math.atan2(h_y_trans, h_x_trans)
                
                # in case yaw is 3.14 (180 degree rotation)
                if abs(y-3.14) < 0.1:
                    direction = -1.0 * (1.0 if axis[2] >= 0 else -1.0)
                else:
                    direction = 1.0 if axis[2] >= 0 else -1.0

                h_y_trans = -h_y_trans if lower_limit < -0.1 else h_y_trans

                if not radius or radius <= 0.5:
                    #print(f"Using calculated radius.")
                    radius = math.sqrt(h_x_trans**2 + h_y_trans**2)
                else:
                    print(f"Using width of door for radius.")
                    
                               
            else:
                #print(f"l.401: not horizontal: else")
                if not radius or radius == 0.5:
                    #print(f"Using calculated radius.")
                    radius = math.sqrt(h_x_trans**2 + h_z_trans**2)
                    #print(f"radius formel: {radius}")

            # Get joint origin 
            joint_origin = joint.origin.xyz if joint.origin else [0, 0, 0]
            
            # Create points and calcuate their poses
            point_count = 20
            for i in range (point_count +1):
                angle = lower_limit + (i * (upper_limit - lower_limit) / point_count)
                p = Point()
                
                # Calculate the rotation depending on the axis (sideways vs. up/down)
                if is_knob:
                    
                    p.x = 0.0
                    p.y = radius * math.sin(angle)
                    p.z = radius * math.cos(angle)
                    
                # y-rotation i.e dishwasher, oven
                elif is_vertical_door or abs(axis[1]) > 0.5:
                    
                    p.x = joint_origin[0] + h_x_trans + (radius * math.sin(angle))
                    p.y = joint_origin[1] + h_y_trans
                    p.z = joint_origin[2] + h_z_trans - radius * (1 - math.cos(angle))
                
                # z-rotation i.e. door
                elif is_horizontal_door: #or abs(axis[2]) > 0.5:
                    start_angle = math.atan2(h_y_trans, h_x_trans)
                    
                    # in case yam rotated by 180 degree
                    if abs(y-3.14) < 0.1:
                        current_arc_pos = y + start_angle - (angle * direction)
                    else:
                        current_arc_pos = y + start_angle + (angle * direction)
                    
                    # p.x = joint_origin[0] + h_x_trans + (radius * math.cos(angle))
                    # p.y = joint_origin[1] + h_y_trans + (radius * math.sin(angle))
                    # p.z = joint_origin[2] + h_z_trans 

                    p.x = joint_origin[0] + (radius * math.cos(current_arc_pos))
                    p.y = joint_origin[1] + (radius * math.sin(current_arc_pos))
                    p.z = joint_origin[2] + h_z_trans 

                else:
                    dir_angle = angle* (axis[2] if axis[2] != 0 else 1.0)
                    p.x = radius * math.cos(dir_angle)
                    p.y = radius * math.sin(dir_angle)
                    p.z = handle_z 

                point_list.append(p)
                
                # reverse point_list if door opens from right to left
                if is_horizontal_door or abs(axis[2]) > 0.5:
                    if abs(lower_limit) > abs(upper_limit):
                       point_list.reverse()
                        
########### PRISMATIC ###         
        # If indiv is drawer, get joint limit
        elif joint_type == "prismatic":

            handle_link_name = None
            
            # get indiv_obj of child_link
            indiv_obj = onto.search_one(iri=f"*{child_link}")

            # search for hasPart handle
            if indiv_obj and hasattr(indiv_obj, "hasPart"):
                for part in indiv_obj.hasPart:
                    if any(cls.name == "Handle" for cls in part.is_a):
                        handle_link_name = part.name
                        break          

            # Get drawer origin from urdf
            drawer_origin = joint.origin.xyz if joint.origin else [0.0, 0.0, 0.0]
            #print(f"drawer_origin:{drawer_origin}")

            r,p,y = joint.origin.rpy if joint.origin else [0,0,0]
            
            # Get handle offset, calculate points starting from handle of indiv
            handle_offset = [0,0,0]
            if handle_link_name:
                handle_joint = next((j for j in urdf.joints if j.child == handle_link_name), None)
                #print(f"handle_link_name: {handle_link_name}")
                #print(f"Found handle_joint: {handle_joint.name}")
                
                if handle_joint:
                    handle_offset = handle_joint.origin.xyz
                    #print(handle_offset)
            else:
                print(f"No handle link name found!")
                return
                
            # Get joint limit and axis
            limit = getattr(joint, "limit", None)
            axis = getattr(joint, "axis", [1, 0, 0])

            if limit is not None:
                max_limit = limit.upper

                # create point list
                point_list = []
                point_count = 10
                
                for i in range(point_count):
                    distance = (max_limit / point_count) * i

                    p = Point()

                    v_x = handle_offset[0] + (axis[0] * distance)
                    v_y = handle_offset[1] + (axis[1] * distance)
                    v_z = handle_offset[2] + (axis[2] * distance)

                    rot_x = v_x * math.cos(y) - v_y * math.sin(y)
                    rot_y = v_x * math.sin(y) + v_y * math.cos(y)
                    rot_z = v_z # Z bleibt bei Z-Rotation gleich

                    p_obj = Point()
                    p_obj.x = drawer_origin[0] + rot_x
                    p_obj.y = drawer_origin[1] + rot_y
                    p_obj.z = drawer_origin[2] + rot_z
                    
                    point_list.append(p_obj)


        if not point_list:
            print(f"No pointlist!")

        
        # Get frame 
        tf_prefix = "iai_kitchen"
        parent_frame = joint.parent

        # add tf prefix if it is missing
        if not parent_frame.startswith(tf_prefix):
            parent_frame = f"{tf_prefix}/{parent_frame}"
    
        
        # 5. Create Marker 
        marker = Marker()
        marker.header.frame_id = parent_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = f"trajectory"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.scale.x = 0.07
        marker.scale.y = 0.07
        marker.scale.z = 0.07
        marker.action = Marker.ADD
        marker.color = ColorRGBA(1.0,1.0,0,1.0)

        # create MarkerArray
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        
        #effect: spheres appear in order
        for p_obj in point_list:
            marker.points.append(p_obj)
            ma.pub.publish(marker_array)
            rospy.sleep(0.2)

        rospy.sleep(duration)
        
        # delete markers
        ma._stop_publishing(marker_array)
        
        return f"Published trajectory for {child_link} for {duration} seconds."

#######################################################################################
## trajectory for individual list

    def trajectory_list(self, indiv_names):
        
        if not isinstance(indiv_names, list):
            indiv_names = [indiv_names]
            
        for name in indiv_names:
            self.trajectory(name)

        return f"Finished"

    
#######################################################################################
# HIGHLIGHT TRAJECTORY
# a trajectory gets highlighted 
#  
# Input: list of points
# Output: points get highlighted and notice with highlighting duration
#
#
    def highlight_trajectory(self, indiv_list, ref_frame="map", duration=5.0):
        
        # 0. Check whether map is loaded
        # if not self.map_server_running():
        #     rospy.logwarn(f"No semantic map loaded!")
        #     return
        #else: 
            
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = f"highlight_trajectory"
            marker.id = 0
            marker.type = Marker.SPHERE_LIST
            marker.scale.x = 0.07
            marker.scale.y = 0.07
            marker.scale.z = 0.07
            marker.action = Marker.ADD
            marker.color = ColorRGBA(1.0,1.0,0,1.0)
    
            # create MarkerArray
            marker_array = MarkerArray()
            marker_array.markers.append(marker)
            
            # add Points
            for (x, y, z) in point_list:
                p = Point()
                p.x, p.y, p.z = x, y, z
                marker.points.append(p)
    
            # effect: flashing 
            flashes = 3
            flash_interval = 0.5
            for i in range(flashes):
                for m in marker_array.markers:
                   m.action = Marker.ADD
                ma.pub.publish(marker_array)
                rospy.sleep(flash_interval)
            
                # delete markers
                for m in marker_array.markers:
                    m.action = Marker.DELETE
                ma.pub.publish(marker_array)
                rospy.sleep(flash_interval)
            
            # publish for a few seconds
            for m in marker_array.markers:
                m.action = Marker.ADD
            ma.pub.publish(marker_array)
            rospy.sleep(duration)
            
            # delete markers
            ma._stop_publishing(marker_array)
                   
            return f"Highlighted trajectory with {len(point_list)} points for {duration} seconds"


#######################################################################################
## Helper function for highlighting
#
# finds the corresponding indiv urdf obj for the target link
#  
    def link_indiv_matching(self, param_name="/kitchen_description"):
        # 0. Check map server
        # if not self.map_server_running():
        #     rospy.logwarn("No semantic map loaded!")
        #     return {}
            
        # 1. Get urdf 
        urdf_string = rospy.get_param(param_name)
        urdf = URDF.from_xml_string(urdf_string)
        
        # 2. Get individuals from ontology 
        onto = get_ontology("/home/jovyan/work/prolog/BA-class_extraction1.owl").load()
        indiv_names = [i.name.lower() for i in onto.individuals()]
    
        matches = {}
    
        # 3. Get names of urdf links
        for link_obj in urdf.links:
            link_obj_name = link_obj.name.lower()
            
            
            # check if name in onotology
            # create matching 
            if link_obj_name in indiv_names:
                matches[link_obj_name] = {
                    "link_name": link_obj.name, 
                    "urdf_obj": link_obj 
                }
    
        return matches
        
#######################################################################################
#######################################################################################
## URDF Parser
# first parse for links 
# match links to classes in ontology
# save links as individuals 
# create joint classes and object properties
# parse for joints
# save joints as individuals 
# save parent link as hasParentLink and child link as hasChildLink
# save hasParentLink hasPart hasChildLink
#
# Input: -
# Output: -
#
#
    def parse_urdf(self, param_name="/kitchen_description"):
        
        # 0. Check whether map is loaded
        # if not self.map_server_running():
        #     rospy.logwarn(f"No semantic map loaded!")
        #     return
        
        # 1. get urdf from param server
        rospy.loginfo(f"Read URDF from ROS param server '{param_name}' ....")
        urdf_string = rospy.get_param(param_name)
        urdf = URDF.from_xml_string(urdf_string)

        # get matching of links to classes from database
        matching = self.link_class_matching()
        #print (f"l.663: matching link to class:{matching}")

        if matching is None:
            return f"No matchings found, parsing stopped!"
        else:
            
            # load ontology
            onto = get_ontology("/home/jovyan/work/prolog/BA-class_extraction.owl").load()
            
            # create new empty ontology
            # not .load() because when starting parser again, this ontology is not empty
            #  do not use the information in ontology 
            new_onto = get_ontology("http://test.org/BA-class_extraction1.owl")
            
            # get all classes from onto written in lower case          
            lower_class_names = {claass.name.lower(): claass for claass in onto.classes()}
            ##print(f"class_lowercase: {lower_class_names}")
            
            # get already existing indivs from onto
            indivs = [indivs.name for indivs in onto.individuals()]
            ##print(f"individuals: {indivs}")
         
            # add links as individuals
            for item in matching:
                link_name = item["link"]
                class_name = item["class"]
                
                # add indiv if class of indiv exists
                # 'fridge' --> 'Fridge'
                claass = onto[class_name.capitalize()]
                #print(f"l.637: claass: {claass}")
                ##print(f" l.690: class cap, {claass}")

                if claass is None:
                    continue
                
                # if indiv with link_name already exists, ignore
                if link_name in indivs:
                    print(f"already existing link name:{link_name}")
                    ind = onto[link_name]
                    print(f"{link_name} already existing as {ind}")
            
                else:
                    # add link as individual
                    with new_onto: 
                        link_indiv = claass(link_name)
                        #print(f"l.654: link_indiv: {link_indiv}")
            
      
###############
### JOINTS ###
        # define object properties and Joint class
        with new_onto:
            if new_onto["Joint"] is None:
                class Joint(Thing): pass
            else:
                Joint = new_onto["Joint"]

            if new_onto["hasParentLink"] is None:
                class hasParentLink(ObjectProperty):
                    domain = [Joint]
                    range  = [Thing]
            
            if new_onto["hasChildLink"] is None:
                class hasChildLink(ObjectProperty):
                    domain = [Joint]
                    range  = [Thing]

            if new_onto["hasPart"] is None:
                class hasPart(ObjectProperty, TransitiveProperty): pass

            if new_onto["isPartOf"] is None:
                class isPartOf(ObjectProperty, TransitiveProperty):
                    inverse_property = new_onto["hasPart"]
                    
    
        # get joints and joint info
        for joint in urdf.joints:
            joint_type = joint.type.lower() 
            joint_name = joint.name.lower() # long_table:l_table:table_origin_joint
            #print(joint_name, joint_type)
            
            # get parent and child link 
            parent_link = joint.parent.lower()
            child_link = joint.child.lower()
            #print(f"parent_link:{parent_link}")
            #print(f"child_link: {child_link}")
            
            # form name of joint class like in onto i.e "Prismatic_Joint"
            joint_class_name = f"{joint_type.capitalize()}_Joint"
            #print(f"joint_class_name: {joint_class_name}")
            
            # check, if joint class exists in onto 
            if joint_class_name.lower() not in lower_class_names:
                print(f"Joint class {joint_class_name.lower()} does not exist!")
                continue
    
            
            with new_onto:
                # create individual for joint if one does not already exist
                if new_onto[joint_name] is None:
                    # set "Joint" class as parent
                    # i.e. BA-class_extraction.Prismatic_Joint
                    joint_class = types.new_class(joint_class_name, (Joint,))
                    #print(f"joint_class: {joint_class}")
                    joint_indiv = joint_class(joint_name)
                    # joint_indiv = BA-class_extraction.long_table:l_table:table_origin_joint
                    #print(f"joint_indiv: {joint_indiv}")
                else:
                    joint_class = new_onto[joint_class_name]
                    joint_indiv = new_onto[joint_name]
                    #print(f"{joint_name} already exists as {joint_indiv}!")

                # get indiv of parent and child link 
                parent_indiv = new_onto[parent_link]
                child_indiv = new_onto[child_link]
                #print(f"parent_indiv: {parent_indiv}")
                #print(f"child_indiv: {child_indiv}")

                # if parent and child link exist in onto, get them 
                # get links from matching
                # check if parent and child in this list or already saved as indivs
                check_links = [item["link"] for item in matching]
                #print(check_links)

                if parent_link in check_links and child_link in check_links:
                
                    # add relations for hasParentLink and hasChildLink in onto
                    if parent_indiv and child_indiv:
                        new_onto.hasParentLink[joint_indiv] = [parent_indiv]
                        new_onto.hasChildLink[joint_indiv] = [child_indiv]
                        #joint_indiv.hasParentLink = [parent_indiv]
                        #joint_indiv.hasChildLink = [child_indiv]
                   

                        # add hasPart relation if does not already exist
                        if hasattr(parent_indiv, "hasPart"):
                            if child_indiv not in parent_indiv.hasPart:
                                parent_indiv.hasPart.append(child_indiv)
                                #print(f"parent_indiv[0]: {parent_indiv[0]}")
                                #print(f"child_indiv[0]: {child_indiv[0]}"))
                                #print(f"saved hasPart")
                        else:
                            print(f"No new hasPart relations added into ontology!")
                            #print(parent_indiv.name)
                    else: 
                        print(f"No parent_indiv or no child_indiv!")
                        print(f"parent_link:{parent_link}")
                        print(f"child_link: {child_link}")

        # save onto 
        new_onto.save(file="/home/jovyan/work/prolog/BA-class_extraction1.owl", format="rdfxml")
    
        return f"Parsed URDF file successfully!"

#######################################################################################
## Helper function for parse_urdf
#
# matches links from urdf to existing classes (later: take classes from SOMA) 
# 
    def link_class_matching(self, param_name="/kitchen_description"):
        # 0. Check whether map is loaded
        # if not self.map_server_running():
        #     rospy.logwarn(f"No semantic map loaded!")
        #     return
            
        # 1. Get urdf from param server
        urdf_string = rospy.get_param(param_name)
        urdf = URDF.from_xml_string(urdf_string)

        # 2. Get all links
        urdf_links = [link.name.lower() for link in urdf.links]

        # 3. Load ontology
        onto = get_ontology('/home/jovyan/work/prolog/BA-class_extraction.owl').load()

        # 4. Get all classes from ontology as lower case
        lower_class_name = [claass.name.lower() for claass in onto.classes()]
        #print(f"lower_class_name: {lower_class_name}")

        # 5. Look through classes and match link to class
        # remove suffixes = clean_links
        link_to_class_matching = []
        remove_suffix = ["_main", "_link", "_center", "_origin", "_footprint"]

        for link in urdf_links:
            full_link_name = link.lower()
            matched_class = None

            # coffee_machine --> coffeemachine
            compact_name = full_link_name.replace("_", "")
            compact_name = compact_name.replace("coffe", "coffee")
            
            # first check for priority_classes
            # because of some weird naming such as: drawer_oven_board0
            # order is specific, handle before drawer!!
            priority_classes = ["handle", "knob", "door", "drawer", "room", "cabinet"]
        
            for prio_cls in priority_classes:
                if prio_cls in full_link_name:
                    matched_class = prio_cls
                    break
                    
            
            # if no priority_class in full link name, split 
            # begin from right side so with the last cut word
            if not matched_class:
                clean_link = link.split(":")[0] if ":" in link else link
                for token in remove_suffix:
                    if clean_link.endswith(token):
                        clean_link = clean_link.replace(token, "")

                words = clean_link.split('_')

                for cut in reversed(words):
                    # if e.g. cabinet10
                    cut_clean = ''.join([i for i in cut if not i.isdigit()])

                    # if e.g. link name = windows but class window
                    singular_word = self.to_singular(cut_clean)
                    
                    if singular_word in lower_class_name:
                       matched_class = singular_word 
                       break
                        
                    elif singular_word == "washer":
                        matched_class = "dishwasher"
                        break
                        
                    elif singular_word in ["island", "counter"]:
                        matched_class = "table"
                        break

                    elif compact_name in lower_class_name:
                        matched_class = compact_name 
                        break
                    
            if matched_class:
                link_to_class_matching.append({
                    "link": link,
                    "class": matched_class
                })
        
        return link_to_class_matching if link_to_class_matching else None

########################################################################################################
##  Helper funtion for trajectory (revolute joints)
#
# get radius for doors to show trajectory
# get handle of indiv from database and calculate distance: handle - joint 
#
    def get_radius_from_handle(self, indiv_name, urdf):
        
        handle_link_name = None
        
        # Load the ontology 
        onto = get_ontology('/home/jovyan/work/prolog/BA-class_extraction1.owl').load()
        
        # 1. Get the individual object
        indiv_obj = onto.search_one(iri=f"*{indiv_name}")

        # 2. Find the handle linked to the individual
        if indiv_obj and hasattr(indiv_obj, "hasPart"):
            for part in indiv_obj.hasPart:
                if (cls.name == "Handle" for cls in part.is_a):
                    handle_link_name = part.name
                    break

        # 2. Get distance = radius from urdf
        if handle_link_name:
            # search for the joint that has the handle as child link
            joint_to_handle = next((j for j in urdf.joints if j.child == handle_link_name), None)
            
            if joint_to_handle and joint_to_handle.origin: 
                x, y, z = joint_to_handle.origin.xyz
                # euclidean distance: r = sqrt(x² + y²)
            else: 
                x, y, z = 0.0, 0.0 , 0.0
                radius = math.sqrt(x**2 + y**2)
                return radius

        if "knob" in indiv_name.lower():
            return 0.05 # 5 cm 
    
        # if no handle found, return None
        return None

########################################################################################################
## helper function for link_class_matching
# in case link is named e.g. windows 

    def to_singular(self, word):
        
        if word.endswith('ies'): # e.g. batteries -> battery
            return word[:-3] + 'y'
        if word.endswith('ves'): # e.g. shelves -> shelf
            return word[:-3] + 'f'
        if word.endswith('s') and not word.endswith('ss'): # e.g. windows -> window
            return word[:-1]
        return word

########################################################################################################
## URDF Parser v2
# receives urdf file 
#
# Input: path to urdf file
# Output: -
#
#
    def parse_urdf_1(self, urdf_source):
        
        # Check whether the map is loaded
        # if not self.map_server_running():
        #     rospy.logwarn(f"No semantic map loaded!")
        #     return

        # Get urdf from param server
        rospy.loginfo(f"Read URDF '{urdf_source}' ....")
        urdf_string = None

        if os.path.exists(urdf_source):
            rospy.loginfo(f"Reading URDF from local file: {urdf_source}")
            try:
                with open(urdf_source, 'r') as f:
                    urdf_string = f.read()
            except Exception as e:
                rospy.logerr(f"Could not read URDF file: {e}")
                return
        else:
            # Otherwise, treat it as a ROS parameter
            rospy.loginfo(f"Read URDF from ROS param server '{urdf_source}' ....")
            if rospy.has_param(urdf_source):
                urdf_string = rospy.get_param(urdf_source)
            else:
                rospy.logerr(f"Parameter '{urdf_source}' not found and not a valid file path!")
                return

        # Parse the string into the URDF object
        try:
            urdf = URDF.from_xml_string(urdf_string)
        except Exception as e:
            rospy.logerr(f"Failed to parse URDF XML: {e}")
            return
        
        # Get the matching of links to classes
        matching = self.link_class_matching()

        if matching is None:
            return f"No matchings found, parsing stopped!"
        else:
            
            # Load the ontology for class extractions
            onto = get_ontology("/home/jovyan/work/prolog/BA-class_extraction.owl").load()
            
            # Create a new empty ontology
            new_onto = get_ontology("http://test.org/BA-class_extraction2.owl")
            
            # Get all classes from the ontology written in lower case          
            lower_class_names = {claass.name.lower(): claass for claass in onto.classes()}
            
            # Get already existing individuals from the ontology
            indivs = [indivs.name for indivs in onto.individuals()]
         
            # Add all links as individuals into the ontology
            for item in matching:
                link_name = item["link"]
                class_name = item["class"]
                
                # Check, whether the class from the matching exists in the ontology
                claass = onto[class_name.capitalize()]

                if claass is None:
                    continue
                
                # If the individual with link_name already exists, ignore
                if link_name in indivs:
                    rospy.loginfo(f"Already existing link name:{link_name}")
                    ind = onto[link_name]
                    rospy.loginfo(f"{link_name} already existing as: {ind}")
            
                else:
                    # Add the link as an individual into the ontology
                    with new_onto: 
                        link_indiv = claass(link_name)
            
      
###############
### JOINTS ###
        # Define the Joint class and object properties
        with new_onto:
            if new_onto["Joint"] is None:
                class Joint(Thing): pass
            else:
                Joint = new_onto["Joint"]

            if new_onto["hasParentLink"] is None:
                class hasParentLink(ObjectProperty):
                    domain = [Joint]
                    range  = [Thing]
            
            if new_onto["hasChildLink"] is None:
                class hasChildLink(ObjectProperty):
                    domain = [Joint]
                    range  = [Thing]

            if new_onto["hasPart"] is None:
                class hasPart(ObjectProperty, TransitiveProperty): pass

            if new_onto["isPartOf"] is None:
                class isPartOf(ObjectProperty, TransitiveProperty):
                    inverse_property = new_onto["hasPart"]
                    
    
        # Get all joints from the URDF file and joint info
        for joint in urdf.joints:
            joint_type = joint.type.lower() 
            joint_name = joint.name.lower() # i.e. long_table:l_table:table_origin_joint
            
            # Get the parent link and child link 
            parent_link = joint.parent.lower()
            child_link = joint.child.lower()
            
            # Form the name of joint class like in the ontology i.e "Prismatic_Joint"
            joint_class_name = f"{joint_type.capitalize()}_Joint"
            
            # Check, if joint class exists in the ontology 
            if joint_class_name.lower() not in lower_class_names:
                print(f"Joint class {joint_class_name.lower()} does not exist!")
                continue
    
            
            with new_onto:
                # Create an individual for the joint if one does not already exist
                if new_onto[joint_name] is None:
                    joint_class = types.new_class(joint_class_name, (Joint,)) # set "Joint" class as parent
                    joint_indiv = joint_class(joint_name) #BA-class_extraction.long_table:l_table:table_origin_joint
        
                else:
                    joint_class = new_onto[joint_class_name]
                    joint_indiv = new_onto[joint_name]
                    #print(f"{joint_name} already exists as {joint_indiv}!")

                # Get the individual of parent link and child link 
                parent_indiv = new_onto[parent_link]
                child_indiv = new_onto[child_link]
                
                # Get all links from link class matching 
                check_links = [item["link"] for item in matching]

                # Check whether parent and child are in this list
                if parent_link in check_links and child_link in check_links:
                
                    # Add relations for hasParentLink and hasChildLink in the ontology
                    if parent_indiv and child_indiv:
                        new_onto.hasParentLink[joint_indiv] = [parent_indiv]
                        new_onto.hasChildLink[joint_indiv] = [child_indiv]

                    # Add hasPart relation if it does not already exist
                    if child_indiv and child_indiv not in parent_indiv.hasPart:
                        parent_indiv.hasPart.append(child_indiv)

        # Save the ontology 
        new_onto.save(file="/home/jovyan/work/prolog/BA-class_extraction2.owl", format="rdfxml")
    
        return f"Parsed URDF file successfully!"


#########################################################################
## Starts a launch file with existing urdf as input
# 
# Input: the urdf_path, the name of the recorded ground map
# Output: - (starting of launch file)
#
    def start_env(self, urdf_path, map_name):

        # Check, whether urdf_path is valid input 
        if not os.path.exists(urdf_path):
            rospy.logwarn(f"Could not find: {urdf_path}")
            return

        # Launch the bringup.launch file 
        rospy.loginfo(f"Starting BA-envi_bringup.launch")
        subprocess.Popen([
            'roslaunch', 'suturo_bringup', 'BA-envi_bringup.launch',
            f'urdf_path:={urdf_path}',
            f'map_name:={map_name}',
            'use_rviz:=true'
        ], stdout=DEVNULL, stderr=DEVNULL)     

        time.sleep(5)
        rospy.loginfo(f"Setup complete.")
        

###################################################################################
## A function that delivers infos about the way an furniture item can be opended
# 
# Input: the name of the individual
# Output: the door(s) that need to be opened and the door handles
#
    def open_door(self, indiv_name):

        # Get the URDF file 
        urdf_string = rospy.get_param("/kitchen_description")
        urdf = URDF.from_xml_string(urdf_string)
        
        # Load the Ontology
        # Search for the indiv_obj using indiv_name
        onto = get_ontology('/home/jovyan/work/prolog/BA-class_extraction1.owl').load()
        indiv_obj = onto.search_one(iri=f"*{indiv_name}")

        
        if not indiv_obj:
            rospy.logwarn(f"Cannot find any indivdual named: {indiv_name}")
            return

        # Highlight the individual 
        rospy.loginfo(f"Highlighting the individual: {indiv_name}")
        self.highlight(indiv_name)

        # Get all parts of an individual (transitive) 
        def get_all_parts(indiv_obj):
            parts = set()
            if hasattr(indiv_obj, "hasPart"):
                for part in indiv_obj.hasPart:
                    parts.add(part)
                    parts.update(get_all_parts(part))

            return parts

        all_parts_of_indiv_obj = get_all_parts(indiv_obj)
        #print(all_parts_of_indiv_obj)

        # Search for handle in parts list
        handle_link_names = []
        for part in all_parts_of_indiv_obj:
            if any(cls.name == "Handle" for cls in part.is_a):
                     handle_link_names.append(part.name)
                
        
        if handle_link_names:
            for handle_link_name in handle_link_names:

                # Highlight the handle 
                rospy.loginfo(f"Highlighting the handle: {handle_link_name}")
                self.highlight(handle_link_name)

                # Build handle frame name
                full_handle_frame = f"iai_kitchen/{handle_link_name}"
            
                # Get the pose of the handle
                pose_data = self.object_pose(full_handle_frame, ref_frame="map")

                # Print pose data 
                if pose_data:
                    frame, trans, rot = pose_data
                    rospy.loginfo(f"Handle pose in '{frame}': x={trans[0]:.2f}, y={trans[1]:.2f}, z={trans[2]:.2f}")
                else:
                    rospy.logwarn(f"Could not find frame named: {full_handle_frame}")
            
                # Get the handle_joint that has handle as child link
                handle_joint = next((j for j in urdf.joints if j.child == handle_link_name), None)

                # If a handle joint exists call trajectory with child link 
                if handle_joint:
                    rospy.loginfo(f"Trajectory for: {handle_joint.parent}")
                    result = self.trajectory(handle_joint.parent)
                    #print(result)
                    
                else:
                    rospy.warn(f"No handle joint found where child link is {handle_link_name}.")
                
        else:
            rospy.logwarn(f"No handle_link_name found. Therefore no full_handle_frame.")

        return f"Finish."

#####################################################################################################
## A function that takes a individual name as input and returns doors and handles to open the object
#
# Input: the name of an individual
# Output: the info about doors and handles for opening a furniture item
#
    def opening_information(self, indiv_name):

        # Get the URDF file 
        urdf_string = rospy.get_param("/kitchen_description")
        urdf = URDF.from_xml_string(urdf_string)
        
        # Load the Ontology
        # Search for the indiv_obj using indiv_name
        onto = get_ontology('/home/jovyan/work/prolog/BA-class_extraction1.owl').load()
        indiv_obj = onto.search_one(iri=f"*{indiv_name}")

        if not indiv_obj:
            rospy.logwarn(f"Cannot find any indivdual named: {indiv_name}")
            return [], []

        # Get all parts of an individual (transitive) 
        def get_all_parts(indiv_obj):
            parts = set()
            if hasattr(indiv_obj, "hasPart"):
                for part in indiv_obj.hasPart:
                    parts.add(part)
                    parts.update(get_all_parts(part))

            return parts

        all_parts_of_indiv_obj = get_all_parts(indiv_obj)

        # Search for handle in parts list
        handle_link_names = []
        for part in all_parts_of_indiv_obj:
            if any(cls.name == "Handle" for cls in part.is_a):
                     handle_link_names.append(part.name)

        doors = []
        if handle_link_names:
            for handle_link_name in handle_link_names:

                 # Get the handle_joint that has handle as child link
                handle_joint = next((j for j in urdf.joints if j.child == handle_link_name), None)

                # Collect doors and handles
                doors.append(handle_joint.parent)
        
            return [doors, handle_link_names]
        
############################################
## helper function 
# 
    def get_door_width_from_mesh(self, link_name, urdf):
        # 1. Search for link in urdf 
        link = next((l for l in urdf.links if l.name == link_name), None)
        
        if not link or not link.visual:
            return None
    
        # 2. Get mesh path
        full_mesh_path = link.visual.geometry.filename
        #print(full_mesh_path)

        mesh_path = full_mesh_path.replace("package://", "/opt/ros/overlay_ws/src/iai_maps/")
    
        try:
            mesh = trimesh.load(mesh_path)
            # extents gibt [Breite, Höhe, Tiefe] (x, y, z) zurück
            # Meistens ist die Breite bei Türen die X- oder Y-Achse
            # get the width 
            dims = sorted(mesh.extents)
            door_width = dims[1]
            
            return door_width
            
        except Exception as e:
            print(f"Mesh not found: {e}")
            return None
























