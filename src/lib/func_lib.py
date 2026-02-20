#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
func_lib.py

A library that provides simple functions used for parsing a URDF file, highlighting, querying and creating trajectories on object in a semantic map.
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

## This function defines the class constructor
#
    def __init__(self, topic_name="/pycram/viz_marker"):
        self.topic_name = topic_name
        self.pub = rospy.Publisher(self.topic_name, MarkerArray, queue_size=10)
        if not rospy.core.is_initialized():
            rospy.init_node('func_lib', anonymous=True)
             
#########################################################################################################
# This function checks whether the param server is running
#
    def param_server_running(self):
        return rospy.has_param('/kitchen_description')

########################################################################################################
## Start the Environment 
#
# This function starts a launch file with an existing URDF file as input.
# 
# Input: the urdf_path
# Output: - (starting of launch file)
#
#
    def start_env(self, urdf_path, map_name=""):

        # Checks whether urdf_path is valid input 
        if not os.path.exists(urdf_path):
            rospy.logwarn(f"Could not find: {urdf_path}")
            return

        # Launches the bringup.launch file 
        rospy.loginfo(f"Starting BA-envi_bringup.launch")
        subprocess.Popen([
            'roslaunch', 'suturo_bringup', 'BA-envi_bringup.launch',
            f'urdf_path:={urdf_path}',
            f'map_name:={map_name}',
            'use_rviz:=true'
        ], stdout=DEVNULL, stderr=DEVNULL)     

        time.sleep(5)
        rospy.loginfo(f"Setup complete.")
        
########################################################################################################
## URDF Parser
# 
# This function parses a URDF file and creates a ontology with the collected information.
#
# Input: -
# Output: -
#
#
    def parse_urdf(self, param_name="/kitchen_description"):
        
        # 0. Checks whether the map is loaded
        if not self.param_server_running():
            rospy.logwarn(f"No semantic map loaded!")
            return
        
        # Retrieves the urdf from param server
        rospy.loginfo(f"Read URDF from ROS param server '{param_name}' ....")
        urdf_string = rospy.get_param(param_name)
        urdf = URDF.from_xml_string(urdf_string)

        # Retrieves the matching of links to classes(from SOMA-HOME)
        matching = self.link_class_matching()

        if matching is None:
            return f"No matchings found, parsing stopped!"
        else:
            
            # Loads the ontology ontology for class extraction (=SOMA-HOME)
            onto = get_ontology("/home/jovyan/work/prolog/BA-class_extraction.owl").load()
            #onto = get_ontology("/home/jovyan/work/ease_     /SOMA-HOME.owl").load()
            
            # Creates a new empty ontology
            new_onto = get_ontology("http://test.org/BA-class_extraction1.owl")
            
            # Retrieves all classes from the ontology written in lower case          
            lower_class_names = {claass.name.lower(): claass for claass in onto.classes()}
            
            # Retrieves already existing individuals from the ontology
            indivs = [indivs.name for indivs in onto.individuals()]
         
            # Retrieves the link_name and the class_name from the matching
            for item in matching:
                link_name = item["link"]
                class_name = item["class"]
                
                # Adds individuals into the new ontology if corresponding class exists
                claass = onto[class_name.capitalize()]

                if claass is None:
                    continue
                
                # Checks for individuals with name: link_name already exist
                # if they already exist, ignore
                if link_name in indivs:
                    print(f"already existing link name:{link_name}")
                    ind = onto[link_name]
                    print(f"{link_name} already existing as {ind}")
            
                else:
                    # Adds link as individual into the  new ontology
                    with new_onto: 
                        link_indiv = claass(link_name)
                        #print(f"l.654: link_indiv: {link_indiv}")
            
      
###############
### JOINTS ###
        # Defines the Joint class and object properties
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
                    
    
        # Retrieves all joints and information about them from the urdf
        for joint in urdf.joints:
            joint_type = joint.type.lower() 
            joint_name = joint.name.lower() # long_table:l_table:table_origin_joint
            
            # Retrieves the parent link and child link 
            parent_link = joint.parent.lower()
            child_link = joint.child.lower()
            
            # Forms the name of the joint class i.e "Prismatic_Joint"
            joint_class_name = f"{joint_type.capitalize()}_Joint"
            
            # Checks if the joint class exists in the ontology 
            if joint_class_name.lower() not in lower_class_names:
                print(f"Joint class {joint_class_name.lower()} does not exist!")
                continue
    
            
            with new_onto:
                # Creates an individual for the joint if one does not already exist
                if new_onto[joint_name] is None:
                    # Sets "Joint" class as parent
                    joint_class = types.new_class(joint_class_name, (Joint,))
                    joint_indiv = joint_class(joint_name)
                    
                else:
                    joint_class = new_onto[joint_class_name]
                    joint_indiv = new_onto[joint_name]

                # Retrieves the individual of the corresponding parent link and child link 
                parent_indiv = new_onto[parent_link]
                child_indiv = new_onto[child_link]

                # Retrieves the links from the matching 
                check_links = [item["link"] for item in matching]

                # If parent link and child link exist in the ontology,
                #  add object properties
                if parent_link in check_links and child_link in check_links:
                
                    if parent_indiv and child_indiv:
                        new_onto.hasParentLink[joint_indiv] = [parent_indiv]
                        new_onto.hasChildLink[joint_indiv] = [child_indiv]

                        if hasattr(parent_indiv, "hasPart"):
                            if child_indiv not in parent_indiv.hasPart:
                                parent_indiv.hasPart.append(child_indiv)
                               
                        else:
                            print(f"No new hasPart relations added into ontology!")
                        
                    else: 
                        print(f"No parent_indiv or no child_indiv!")
                        print(f"parent_link:{parent_link}")
                        print(f"child_link: {child_link}")

        # Saves the new ontology 
        new_onto.save(file="/home/jovyan/work/prolog/BA-class_extraction1.owl", format="rdfxml")
    
        return f"Parsed URDF file successfully!"

########################################################################################################
## POSE
#
# This function retrieves the pose of an object using the tf framework.
#
# Input: the object frame 
# Output: the pose of object frame
#
#
    def object_pose(self, object_frame, ref_frame="map"):
        
        # Checks whether the map is loaded
        if not self.param_server_running():
            rospy.logwarn(f"No semantic map loaded!")
            return
        else: 
            # Retrieves the pose of the object
            listener = tf.TransformListener()
            rospy.sleep(1.0)
            try:
                trans, rot = listener.lookupTransform(ref_frame, object_frame, rospy.Time(0))
                return [ref_frame, [trans[0], trans[1], trans[2]], [rot[0], rot[1],rot[2], rot[3]]]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return None

########################################################################################################
## HIGHLIGHTING
#
# This function creates an MarkerArray on top of a object to highlight it.
#
# Input: name of the individual to be highlighted
# Output: - (the object is being highlighted)
#
#
    def highlight(self, indiv_name: str, ref_frame="map"):
        
        # Checks whether the map is loaded
        if not self.param_server_running():
            rospy.logwarn(f"No semantic map loaded!")
            return
    
        # Retrieves the link to individual matching
        matching = self.link_indiv_matching()

        # Retrieves the link name and URDF object associated to the individual
        if indiv_name in matching:
                indiv_data = matching[indiv_name]
            
                target_link = indiv_data["link_name"]
                target_link_obj = indiv_data["urdf_obj"]
            
        else:
            rospy.logwarn(f"Could not find a link for individual '{indiv_name}'")
            return
        
       
        # Retrieves the pose of the individual
        target_frame = f"iai_kitchen/{target_link}"
        target_link_pose = self.object_pose(target_frame, ref_frame)
        
        if target_link_pose is None:
            print(f"Could not get pose in '{ref_frame}' frame of target_link: '{target_link}'!")
            return

        trans = target_link_pose[1]
        rot = target_link_pose[2]
    
        # Publishes markers (with flashing effect)
        flash_interval = 0.5
        for _ in range(2):
            # create marker
            markers = ma._publish(indiv_name, target_link_obj, trans, rot) 
            rospy.sleep(flash_interval)
            ma._stop_publishing(markers)
            rospy.sleep(flash_interval)
    
        # Highlights the individuals for 4 seconds
        markers = ma._publish(indiv_name, target_link_obj, trans,rot)
        duration = 4.0
        rospy.sleep(duration)
        ma._stop_publishing(markers)
        
        return f"Successfully highlighted {indiv_name} using link: '{target_link}'."
        

########################################################################################################
## HIGHLIGHT ALL OBJECTS OF A CLASS
#
# This function creates MarkerArrays on top of a list of objects.   
#
# Input: the name of a class; all objects of this class are to be highlighted
# Output: objects get highlighted and notice with highlighting duration
#
#
    def highlight_list(self, selected_class, ref_frame="map", duration=4.0):

        # Checks whether the map is loaded
        if not self.param_server_running():
            rospy.logwarn(f"No semantic map loaded!")
            return
        
        # Retrieves the link to individual matching
        matching = self.link_indiv_matching()

        # Loads the ontology for class extraction
        onto = get_ontology('/home/jovyan/work/prolog/BA-class_extraction1.owl').load()

        results = onto.search(iri=f"*{selected_class.capitalize()}*")

        # Retrieves all instances (individuals) of the selected class
        if results:
            target_class = results[0]
            indivs = target_class.instances()
            indivs_names = [indiv.name for indiv in indivs]

        all_markers = MarkerArray()
        listen = tf.TransformListener()
        rospy.sleep(0.5)

        global_id = 0
        
        # For each individual: 
        for indiv_name in indivs_names:
            if indiv_name not in matching:
                rospy.logwarn(f"Could not find a link for individual '{indiv_name}' in matching")
                continue

            # Retrieves the link name and URDF object associated to the individual
            indiv_data = matching[indiv_name]
            target_link_obj = indiv_data["urdf_obj"]
            target_link = indiv_data["link_name"]
                
            if not target_link_obj.visual:
                continue

            # Retrieve the pose of the individual
            target_frame = f"iai_kitchen/{target_link}"
            target_link_pose = self.object_pose(target_frame, ref_frame)

            if target_link_pose is None:
                print(f"Could not get pose of target_link: '{target_link}'!")
                return
            
            trans = target_link_pose[1]
            rot = target_link_pose[2]

            # Creates Markers
            markers = ma.create_marker_array(
                indiv_name,
                target_link_obj,
                trans,
                rot
            )

            for m in markers.markers:
                m.id = global_id
                global_id += 1
    
            # Appends the Markers to a combined MarkerArray of all objects
            all_markers.markers.extend(markers.markers)

        if not all_markers.markers:
            return "No object list found for highlighting!"
            
        # Highlights the objects (effect: flashing)
        flash_interval = 0.5
        for _ in range(2):
            # Creates markers 
            for m in all_markers.markers:
               m.action = Marker.ADD
            ma.pub.publish(all_markers)
            rospy.sleep(flash_interval)
        
            # Deletes markers
            for m in all_markers.markers:
                m.action = Marker.DELETE
            ma.pub.publish(all_markers)
            rospy.sleep(flash_interval)
        
        # Publishes markers for a few seconds
        for m in all_markers.markers:
            m.action = Marker.ADD
        ma.pub.publish(all_markers)
        rospy.sleep(duration)
        
        # Deletes markers
        ma._stop_publishing(all_markers)
               
        return f"Highlighted {len(indivs_names)} objects of class {selected_class} for {duration} seconds!"


########################################################################################################
## HIGHLIGHT A LIST OF INDIVIDUALS
#
# A function that creates Markers on top of list of objects.    
#
# Input: a list of individual names
# Output: objects get highlighted and notice with highlighting duration
#
#
    def highlight_indiv_list(self, indivs_names, ref_frame="map", duration=4.0):

        # Checks whether the map is loaded
        if not self.param_server_running():
            rospy.logwarn(f"No semantic map loaded!")
            return

        if not isinstance(indivs_names, list):
            indivs_names = [indivs_names]
        
        # Retrieves the link to individual matching
        matching = self.link_indiv_matching()


        all_markers = MarkerArray()
        listen = tf.TransformListener()
        rospy.sleep(0.5)

        global_id = 0
        
        # For each individual: 
        for indiv_name in indivs_names:
            if indiv_name not in matching:
                rospy.logwarn(f"Could not find a link for individual '{indiv_name}' in matching")
                continue

            # Retrieves the link name and URDF object associated to the individual
            targets_data = matching[indiv_name]
            target_link_obj = targets_data["urdf_obj"]
            target_link = targets_data["link_name"]
                
            if not target_link_obj.visual:
                continue

            # Retrieve the pose of the individual
            target_frame = f"iai_kitchen/{target_link}"
            target_link_pose = self.object_pose(target_frame, ref_frame)

            if target_link_pose is None:
                print(f"Could not get pose of target_link: '{target_link}'!")
                return
            
            trans = target_link_pose[1]
            rot = target_link_pose[2]

            # Creates Markers
            markers = ma.create_marker_array(
                indiv_name,
                target_link_obj,
                trans,
                rot
            )

            for m in markers.markers:
                m.id = global_id
                global_id += 1
    
            # Appends the Markers to a combined MarkerArray of all objects
            all_markers.markers.extend(markers.markers)

        if not all_markers.markers:
            return "No object list found for highlighting!"
            
        # Highlights the objects (effect: flashing)
        flash_interval = 0.5
        for _ in range(2):
            # Creates markers 
            for m in all_markers.markers:
               m.action = Marker.ADD
            ma.pub.publish(all_markers)
            rospy.sleep(flash_interval)
        
            # Deletes markers
            for m in all_markers.markers:
                m.action = Marker.DELETE
            ma.pub.publish(all_markers)
            rospy.sleep(flash_interval)
        
        # Publishes markers for a few seconds
        for m in all_markers.markers:
            m.action = Marker.ADD
        ma.pub.publish(all_markers)
        rospy.sleep(duration)
        
        # Deletes markers
        ma._stop_publishing(all_markers)
               
        return f"Highlighted {len(indivs_names)} individual(s) for {duration} seconds!"
########################################################################################################
## TRAJECTORY
#
# This function visualizes the opening trajectories for doors and drawers based on their joint types.
#
# Input: the child link of a joint / an indiv from the ontology
# Output: visualisation of the trajectory
#
#
    def trajectory(self, child_link, ref_frame="map", duration=4.0):
        
        # Checks whether map is loaded
        if not self.param_server_running():
            rospy.logwarn(f"No semantic map loaded!")
            return

        # Retrieves the URDF file from the param server
        urdf_string = rospy.get_param("/kitchen_description")
        urdf = URDF.from_xml_string(urdf_string)

        # Loads the ontology
        onto = get_ontology('/home/jovyan/work/prolog/BA-class_extraction1.owl').load()

        
        # Retrieves all joints in the URDF file where child_link is the child link
        joint_type = None
        joint_name = None
        for joint in urdf.joints:
            if joint.child == child_link:
                # Retrieves the name and type of the target joint 
                joint_name = joint.name.lower()
                joint_type = joint.type.lower()
                break

        
        if joint_type is None:
            rospy.logwarn(f"No joint found where joint.child is {child_link}")
            return

        point_list = []
        
        # Creates a trajectory based on the joint type
########### FIXED ### 
        if joint_type == "fixed":
            rospy.logwarn(f"No trajectory for fixed joints!")
            return
            
            
########### REVOLUTE ###         
        if joint_type == "revolute" or joint_type == "continuous":

            # Retrieves the individual object associated to child_link
            indiv_obj = onto.search_one(iri=f"*{child_link}")
            

            # Retrieves the individuals that do not open left/right
            is_knob = any("Knob" in cls.name for cls in indiv_obj.is_a) if indiv_obj else False

            # a door that belongs to a dishwasher or oven 
            is_vertical_door = any(x in child_link.lower() for x in ["oven", "dishwasher", "dish_washer"]) and not is_knob

            # a door that opens horizontally e.g. door, fridge
            is_horizontal_door = any(x in child_link.lower() for x in ["fridge", "door"]) and not is_knob and not is_vertical_door
            
            # Retrieves the radius at the height of the handle and axis
            if is_knob:
                radius = 0.03
            else:  
                mesh_width = self.get_door_width_from_mesh(joint.child, urdf)

                if mesh_width:
                    radius = mesh_width
                    
                else:
                    radius = self.get_radius_from_handle(joint.child, urdf) or 0.5

            # Retrieves the axis and rotation of the joint
            axis = joint.axis or [0, 0, 1] 
            r,p,y = joint.origin.rpy if joint.origin else [0, 0, 0]

            # Retrieves the upper and lower joint limit
            limit = getattr(joint, "limit", None)

            # in case both limits are 0.0 
            lower_limit = limit.lower if (limit and limit.lower != limit.upper) else 0.0
            upper_limit = limit.upper if (limit and limit.lower != limit.upper) else 1.57 # 90°

            handle_joint = None
            handle_link_name = None
            
            # Checks for parts of the individual that are handles
            if indiv_obj and hasattr(indiv_obj, "hasPart"):
                for part in indiv_obj.hasPart:
                    if any(cls.name == "Handle" for cls in part.is_a):
                        handle_link_name = part.name
            
            # Retrieves the handle joints where the child link is a handle link
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
                

                if radius and radius > 0.05:
                    handle_origin = [radius, 0,0]
                    
                else:
                    handle_origin = [0.4,0,0]
                    
                if is_knob:
                    handle_origin = [0.02,0,0]
                
            
            # Calculates the transformation of the handle's pose
            h_x_trans = handle_origin[0]
            h_y_trans = handle_origin[1] * math.cos(r) - handle_origin[2] * math.sin(r)
            h_z_trans = handle_origin[1] * math.sin(r) + handle_origin[2] * math.cos(r)

            # Defines params based on door type/ joint type
            if is_horizontal_door: #or abs(axis[2]) > 0.5:
                
                # In case yaw is 3.14 (180 degree rotation)
                if abs(y-3.14) < 0.1:
                    direction = -1.0 * (1.0 if axis[2] >= 0 else -1.0)
                    
                else:
                    direction = 1.0 if axis[2] >= 0 else -1.0

                h_y_trans = -h_y_trans if lower_limit < -0.1 else h_y_trans

                if not radius or radius <= 0.5:
                    radius = math.sqrt(h_x_trans**2 + h_y_trans**2)
                    
                else:
                    print(f"Using width of door for radius.")
                    
                               
            else:
                if not radius or radius == 0.5:
                    radius = math.sqrt(h_x_trans**2 + h_z_trans**2)

            # Retrieves the joint's origin 
            joint_origin = joint.origin.xyz if joint.origin else [0, 0, 0]
            
            # Creates points and calcuates their poses
            point_count = 20
            for i in range (point_count +1):
                angle = lower_limit + (i * (upper_limit - lower_limit) / point_count)
                p = Point()
                
                # Calculates the rotation depending on the axis (sideways vs. up/down)
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
                    
                    # in case yaw is rotated by 180 degree
                    if abs(y-3.14) < 0.1:
                        current_arc_pos = y + start_angle - (angle * direction)
                    else:
                        current_arc_pos = y + start_angle + (angle * direction)
                     

                    p.x = joint_origin[0] + (radius * math.cos(current_arc_pos))
                    p.y = joint_origin[1] + (radius * math.sin(current_arc_pos))
                    p.z = joint_origin[2] + h_z_trans 

                else:
                    dir_angle = angle* (axis[2] if axis[2] != 0 else 1.0)
                    p.x = radius * math.cos(dir_angle)
                    p.y = radius * math.sin(dir_angle)
                    p.z = handle_z 

                point_list.append(p)
                
                # Reverses the point_list if the door opens from right to left
                if is_horizontal_door or abs(axis[2]) > 0.5:
                    if abs(lower_limit) > abs(upper_limit):
                       point_list.reverse()
                        
########### PRISMATIC ###         
        elif joint_type == "prismatic":

            handle_link_name = None
            
            # Retrieves the indiv_obj of child_link
            indiv_obj = onto.search_one(iri=f"*{child_link}")

            # Searches for parts that are handles
            if indiv_obj and hasattr(indiv_obj, "hasPart"):
                for part in indiv_obj.hasPart:
                    if any(cls.name == "Handle" for cls in part.is_a):
                        handle_link_name = part.name
                        break          

            # Retrieves the drawer origin from the URDF file
            drawer_origin = joint.origin.xyz if joint.origin else [0.0, 0.0, 0.0]

            r,p,y = joint.origin.rpy if joint.origin else [0,0,0]
            
            # Retrieves the handle offset
            # Calculates points starting from the handle of individual
            handle_offset = [0,0,0]
            if handle_link_name:
                handle_joint = next((j for j in urdf.joints if j.child == handle_link_name), None)
                
                if handle_joint:
                    handle_offset = handle_joint.origin.xyz
                    
            else:
                print(f"No handle link name found!")
                return
                
            # Retrieves the joint limit and axis
            limit = getattr(joint, "limit", None)
            axis = getattr(joint, "axis", [1, 0, 0])

            if limit is not None:
                max_limit = limit.upper

                # Creates the point list
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
                    rot_z = v_z

                    p_obj = Point()
                    p_obj.x = drawer_origin[0] + rot_x
                    p_obj.y = drawer_origin[1] + rot_y
                    p_obj.z = drawer_origin[2] + rot_z
                    
                    point_list.append(p_obj)


        if not point_list:
            print(f"No pointlist!")

        
        # Retrieves the frame of the parent 
        tf_prefix = "iai_kitchen"
        parent_frame = joint.parent

        # Adds the tf prefix if it is missing
        if not parent_frame.startswith(tf_prefix):
            parent_frame = f"{tf_prefix}/{parent_frame}"
    
        
        # Creates Markers
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

        # Creates the MarkerArray
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        
        # Creates the effect: spheres appear in order
        for p_obj in point_list:
            marker.points.append(p_obj)
            ma.pub.publish(marker_array)
            rospy.sleep(0.2)

        rospy.sleep(duration)
        
        # Deletes markers
        ma._stop_publishing(marker_array)
        
        return f"Published trajectory for {child_link} for {duration} seconds."

#######################################################################################
## TRAJECTORY for individual list
#
# This function is a wrapper for trajectory and enables to call trajectory for a list of individuals.
#
# Input: the list of individual names
# Output: the visualisation of the trajectories and a notice

    def trajectory_list(self, indiv_names):

        # Creates a list if the input is not a list
        if not isinstance(indiv_names, list):
            indiv_names = [indiv_names]

        # Calls the trajectory function for each individual
        for name in indiv_names:
            self.trajectory(name)

        return f"Finished"

#######################################################################################
## Link to Individual Matching 
#
# This is a helper function for highlighting and matches the links form the URDF file to the 
#  individuals from the ontology, savinf this in a dictionary.
#
# Input: -
# Output: A dictionary with the link to individual matching
#  
#
    def link_indiv_matching(self, param_name="/kitchen_description"):
        
        # Checks if the map is loaded
        if not self.param_server_running():
            rospy.logwarn("No semantic map loaded!")
            return {}
            
        # Retrieves the URDF file 
        urdf_string = rospy.get_param(param_name)
        urdf = URDF.from_xml_string(urdf_string)
        
        # Retrieves all individuals from the ontology 
        onto = get_ontology("/home/jovyan/work/prolog/BA-class_extraction1.owl").load()
        indiv_names = [i.name.lower() for i in onto.individuals()]
    
        matches = {}
    
        # Retrieves the names of the URDF links
        for link_obj in urdf.links:
            link_obj_name = link_obj.name.lower()
            
            
            # Checks if the link name can be found in the onotology
            # Creates the matching 
            if link_obj_name in indiv_names:
                matches[link_obj_name] = {
                    "link_name": link_obj.name, 
                    "urdf_obj": link_obj 
                }
    
        return matches
        

##########################################################################################################
## Link to Class matching 
#
# This is a helper function for parse_urdf and matches all links from URDF file to classes in SOMA-HOME
# 
# Input: -
# Output: A matching of links to classes 
#
#
    def link_class_matching(self, param_name="/kitchen_description"):
        
        # Cheks whether the map is loaded
        if not self.param_server_running():
            rospy.logwarn(f"No semantic map loaded!")
            return
            
        # Retrieves the URDF file from the param server
        urdf_string = rospy.get_param(param_name)
        urdf = URDF.from_xml_string(urdf_string)

        # Retrieves all links from the URDF file
        urdf_links = [link.name.lower() for link in urdf.links]

        # Loads the ontology
        onto = get_ontology('/home/jovyan/work/prolog/BA-class_extraction.owl').load()
        #onto = get_ontology('/SOMA-HOME.owl').load()

        # Retrieves all classes from the ontology as lower case
        lower_class_name = [claass.name.lower() for claass in onto.classes()]
        #print(f"lower_class_name: {lower_class_name}")

        # Looks through all classes and matches the links to a class
        # remove suffixes = clean_links
        link_to_class_matching = []
        remove_suffix = ["_main", "_link", "_center", "_origin", "_footprint"]

        for link in urdf_links:
            full_link_name = link.lower()
            matched_class = None

            # i.e. coffee_machine --> coffeemachine
            compact_name = full_link_name.replace("_", "")
            compact_name = compact_name.replace("coffe", "coffee")
            
            # Checks for priority_classes
            # because of naming such as: drawer_oven_board0
            # order is specific, handle before drawer!!
            priority_classes = ["handle", "knob", "door", "drawer", "room", "cabinet"]
        
            for prio_cls in priority_classes:
                if prio_cls in full_link_name:
                    matched_class = prio_cls
                    break
                    
            
            # Splits the link and goes through cuts from right to left 
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
##  Calculate the radius with help of a handle
#
# This function is a helper function for trajectory (revolute joints) and calculates
#   the radius for doors for the trajectory and 
#   the retrieves the handle of an individual to calculate the distance: handle - joint 
#
# Input: the name of the individual, the URDF file
# Output: the calculated radius
#
#
    def get_radius_from_handle(self, indiv_name, urdf):
        
        handle_link_name = None
        
        # Loads the ontology 
        onto = get_ontology('/home/jovyan/work/prolog/BA-class_extraction1.owl').load()
        
        # Retrieves the individual object
        indiv_obj = onto.search_one(iri=f"*{indiv_name}")

        # Finds the handle linked to the individual
        if indiv_obj and hasattr(indiv_obj, "hasPart"):
            for part in indiv_obj.hasPart:
                if (cls.name == "Handle" for cls in part.is_a):
                    handle_link_name = part.name
                    break

        # Retrievs thes distance = radius from the URDF file
        if handle_link_name:
            # Searches for the joint that has the handle as child link
            joint_to_handle = next((j for j in urdf.joints if j.child == handle_link_name), None)
            
            if joint_to_handle and joint_to_handle.origin: 
                x, y, z = joint_to_handle.origin.xyz
                
            else: 
                # Using the euclidean distance: r = sqrt(x² + y²)
                x, y, z = 0.0, 0.0 , 0.0
                radius = math.sqrt(x**2 + y**2)
                return radius

        if "knob" in indiv_name.lower():
            return 0.05
    
        # If no handle is found, return None
        return None

########################################################################################################
## Retrieve the radius from the door width of an object
#
# This is a helper function that calculates teh radius by using a doors width.
#
# Input: the name of the link
# Output: the calculated radius
# 
    def get_door_width_from_mesh(self, link_name, urdf):
        
        # Searches for the link in the URDF file 
        link = next((l for l in urdf.links if l.name == link_name), None)
        
        if not link or not link.visual:
            return None
    
        # Retrieves the mesh path
        full_mesh_path = link.visual.geometry.filename

        mesh_path = full_mesh_path.replace("package://", "/opt/ros/overlay_ws/src/iai_maps/")
    
        try:
            # Using trimesh to retrieve the width of the door
            mesh = trimesh.load(mesh_path)
            dims = sorted(mesh.extents)
            door_width = dims[1]
            
            return door_width
            
        except Exception as e:
            print(f"Mesh not found: {e}")
            return None

########################################################################################################
## Singular
#
# This is a helper function for link_class_matching and forms the singular form of a noun.
#
# Input: a noun
# Output: the changed grammatical form (singular)
#
#
    def to_singular(self, word):
        
        if word.endswith('ies'): # e.g. batteries -> battery
            return word[:-3] + 'y'
            
        if word.endswith('ves'): # e.g. shelves -> shelf
            return word[:-3] + 'f'
            
        if word.endswith('s') and not word.endswith('ss'): # e.g. windows -> window
            return word[:-1]
        return word


###################################################################################
## Open Door
#
# This is a function that delivers information about the way an furniture item can be opended.
# 
# Input: the name of the individual
# Output: the door(s) that need to be opened and the door handles
#
#
    def open_door(self, indiv_name):

        # Retrieves the URDF file 
        urdf_string = rospy.get_param("/kitchen_description")
        urdf = URDF.from_xml_string(urdf_string)
        
        # Loads the Ontology
        # Searches for the indiv_obj using indiv_name
        onto = get_ontology('/home/jovyan/work/prolog/BA-class_extraction1.owl').load()
        indiv_obj = onto.search_one(iri=f"*{indiv_name}")

        if not indiv_obj:
            rospy.logwarn(f"Cannot find any indivdual named: {indiv_name}")
            return

        # Highlights the individual 
        rospy.loginfo(f"Highlighting the individual: {indiv_name}")
        self.highlight(indiv_name)

        # Retrieves all parts of an individual (transitive) 
        def get_all_parts(indiv_obj):
            parts = set()
            if hasattr(indiv_obj, "hasPart"):
                for part in indiv_obj.hasPart:
                    parts.add(part)
                    parts.update(get_all_parts(part))

            return parts

        all_parts_of_indiv_obj = get_all_parts(indiv_obj)

        # Searches for handles in the parts list
        handle_link_names = []
        for part in all_parts_of_indiv_obj:
            if any(cls.name == "Handle" for cls in part.is_a):
                     handle_link_names.append(part.name)
                
        
        if handle_link_names:
            for handle_link_name in handle_link_names:

                # Highlights the handle 
                rospy.loginfo(f"Highlighting the handle: {handle_link_name}")
                self.highlight(handle_link_name)

                # Builds the handle frame name
                full_handle_frame = f"iai_kitchen/{handle_link_name}"
            
                # Retrieves the pose of the handle
                pose_data = self.object_pose(full_handle_frame, ref_frame="map")

                # Retrieves the pose data of the handle
                if pose_data:
                    frame, trans, rot = pose_data
                    rospy.loginfo(f"Handle pose in '{frame}': x={trans[0]:.2f}, y={trans[1]:.2f}, z={trans[2]:.2f}")
                else:
                    rospy.logwarn(f"Could not find frame named: {full_handle_frame}")
            
                # Retrieves the handle_joint that has handle as child link
                handle_joint = next((j for j in urdf.joints if j.child == handle_link_name), None)

                # If a handle joint exists, it calls trajectory with child link 
                if handle_joint:
                    rospy.loginfo(f"Trajectory for: {handle_joint.parent}")
                    result = self.trajectory(handle_joint.parent)
                    
                else:
                    rospy.warn(f"No handle joint found where child link is {handle_link_name}.")
                
        else:
            rospy.logwarn(f"No handle_link_name found. Therefore no full_handle_frame.")

        return f"Finish."

#####################################################################################################
## Opening Information
#
# This is a function for querying and delivers information about doors and handles linked to an individual.
# This information is used to understand how a furniture item can be opened.
#
# Input: the name of an individual
# Output: information about doors and handles for opening a furniture item
#
    def opening_information(self, indiv_name):

        # Retrieves the URDF file 
        urdf_string = rospy.get_param("/kitchen_description")
        urdf = URDF.from_xml_string(urdf_string)
        
        # Loads the Ontology
        # Searches for the indiv_obj using indiv_name
        onto = get_ontology('/home/jovyan/work/prolog/BA-class_extraction1.owl').load()
        indiv_obj = onto.search_one(iri=f"*{indiv_name}")

        if not indiv_obj:
            rospy.logwarn(f"Cannot find any indivdual named: {indiv_name}")
            return [], []

        # Retrieves all parts of an individual (transitive) 
        def get_all_parts(indiv_obj):
            parts = set()
            if hasattr(indiv_obj, "hasPart"):
                for part in indiv_obj.hasPart:
                    parts.add(part)
                    parts.update(get_all_parts(part))

            return parts

        all_parts_of_indiv_obj = get_all_parts(indiv_obj)

        # Searches for handle in parts list
        handle_link_names = []
        for part in all_parts_of_indiv_obj:
            if any(cls.name == "Handle" for cls in part.is_a):
                     handle_link_names.append(part.name)

        doors = []
        if handle_link_names:
            for handle_link_name in handle_link_names:

                 # Retrieves the handle_joint that has handle as child link
                handle_joint = next((j for j in urdf.joints if j.child == handle_link_name), None)

                # Collects doors and handles
                doors.append(handle_joint.parent)
        
            return [doors, handle_link_names]
        
























