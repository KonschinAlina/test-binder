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
import tempfile
import subprocess
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import MarkerArray, Marker
sys.path.append("/home/jovyan/work/src/")
import ros_utils.viz_marker_publisher.viz_marker_array_publisher as MA

from owlready2 import *

ma = MA.VizMarkerArrayPublisher()

round_obj = {"lamp", "round_armchair", "round_table", "trashcan"}
box_obj = {"bed", "bucket", "chair", "coathanger","couch", "drawer", "simple_couch", "table", "long_table"}

mappings = {
    "room": {
        "center_x": "center_x",
        "center_y": "center_y",
        "parent": "parent",
        "name": "name"
    },
    "shelf": {
        "depth": "size_x",
        "width": "size_y",
        "height": "size_z",
        "floors_height": "floors_height",
        "number_of_floors": "number_of_floors",
        "door": "door",
        "parent": "parent",
        "name": "name"
    },
    "shelf_openable": {
        "depth": "size_x",
        "width": "size_y",
        "height": "size_z",
        "floors_height": "floors_height",
        "number_of_floors": "number_of_floors",
        "door": "door",
        "wall_thickness": "wall_thickness",
        "handle_from_bottom": "handle_from_bottom",
        "handle_from_side": "handle_from_side",
        "handle_height": "handle_height",
        "handle_width": "handle_width",
        "handle_depth": "handle_depth",
        "parent": "parent",
        "name": "name"
    },
    "shelf_billy": {
        "depth": "size_x",
        "width": "size_y",
        "height": "size_z",
        "floors_height": "floors_height",
        "number_of_floors": "number_of_floors",
        "door": "door",
        "door_height": "door_height",
        "door_length": "door_length",
        "wall_thickness": "wall_thickness",
        "handle_from_bottom": "handle_from_bottom",
        "handle_from_side_left": "handle_from_side_left",
        "handle_from_side_right": "handle_from_side_right",
        "handle_diameter": "handle_diameter",
        "handle_depth": "handle_depth",
        "parent": "parent",
        "name": "name"
    },
    "simple_drawer": {
        "depth": "size_x",
        "width": "size_y",
        "height": "size_z",
        "handle_x": "handle_x",
        "handle_z": "handle_z",
        "handle_size_x": "handle_size_x",
        "handle_size_y": "handle_size_y",
        "handle_size_z": "handle_size_z",
        "parent": "parent",
        "name": "name"
    },
    "dishwasher": {
        "table_size_x": "table_size_x",
        "table_size_y": "table_size_y",
        "table_size_z": "table_size_z",
        "height": "height",
        "table_height": "table_height",
        "handle_x": "handle_x",
        "handle_y": "handle_y",
        "handle_z": "handle_z",
        "parent": "parent",
        "name": "name"
    },
    "door": {
        "height": "height",
        "width": "width",
        "depth": "depth",
        "x": "x",
        "y": "y",
        "z": "z",
        "zrot": "zrot",
        "door_turn_limit_lower": "door_turn_limit_lower",
        "door_turn_limit_max": "door_turn_limit_max",
        "handle_x": "handle_x",
        "handle_z": "handle_z",
        "parent": "parent",
        "name": "name"
    }
}

def get_param_mapping(object_name: str):
    if object_name in round_obj:
        return {
            "radius": "radius",
            "length": "length",
            "parent": "parent",
            "name": "name"
        }
    elif object_name in box_obj or object_name.endswith("island") or  object_name.endswith("counter"):
        return {
            "depth": "size_x",
            "width": "size_y",
            "height": "size_z",
            "parent": "parent",
            "name": "name"
        }
    elif object_name in mappings:
        return mappings[object_name]
    else:
        rospy.logwarn(f"Kein Mapping für {object_name} definiert")
        return {}
        
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

#########################################################################################################
# Pose
# get the pose of an object with tf
#
# Input: object frame 
# Output: pose of object frame
#
#
    def object_pose(self, object_frame, ref_frame="map"):
        # 0. Check whether map is loaded
        if not self.map_server_running():
            rospy.logewarn(f"No semantic map loaded!")
            return
        else: 
            listener = tf.TransformListener()
            rospy.sleep(1.0)
            try:
                trans, rot = listener.lookupTransform(ref_frame, object_frame, rospy.Time(0))
                return [ref_frame, [trans[0], trans[1], trans[2]], [rot[0], rot[1],rot[2], rot[3]]]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return None

        
#########################################################################################################
# HIGHLIGHT 
# create an MarkerArray on top of a object to highlight it 
#    
# Input: name of object to be highlighted
# Output: object gets highlighted and notice with highlighting duration
#
#
    def highlight(self, indiv_name:str, ref_frame="map"):

        # 0. Check whether map is loaded
        if not self.map_server_running():
            rospy.logewarn(f"No semantic map loaded!")
            return
            
        else: 
            # 1. Load yaml file of object and get object params
            yaml_path = f"/opt/ros/overlay_ws/src/suturo_resources/suturo_resources/urdf/yaml/ms3_room_04.yaml"
            with open(yaml_path, "r") as f:
                 yaml_data = yaml.safe_load(f)
            
            # 2. Parse .urdf from .xacro file
            #obj_urdf = self.parse_urdf_from_xacro(xacro_path, indiv_name, yaml_data)
    
            # 2. (new) Get urdf info from param server 
            urdf_from_kit_des = rospy.get_param("/kitchen_description")
            urdf_new = URDF.from_xml_string(urdf_from_kit_des)
            rospy.loginfo(f"Loaded URDF for '{indiv_name}' from parameter server.")

            # 
            if indiv_name not in self.link_indiv_matching():
                rospy.logwarn(f"No matching URDF link found for '{indiv_name}'")
                return
                
                
            # 3. Get pose of center_link and give to create_marker
            # 3.1 frame ending depends on furniture item type
            suffix = {
                "chair": "chair_center",
                "couch": "place_one",
                "kitchen_island": "table_center",
                "long_table": "table_center",
                "popcorn_table": "table_center",
                "couch_table": "table_center"
            }
            
            # 3.2 dishwasher and kitchen_island have no "center_link" link 
            listen = tf.TransformListener()
            rospy.sleep(1.0)
            if indiv_name == "dishwasher":
                target_frame = f"iai_kitchen/sink_area_dish_washer_main"

                #target_frame = f"iai_kitchen/dishwasher_table:d_table:{suffix.get(indiv_name)}"
                #target_frame = f"iai_kitchen/{indiv_name}_main"
            
            # VORÜBERGEHEN, bis Änderung der knowledge role 
            elif indiv_name == "kitchen_island":
                target_frame = f"iai_kitchen/kitchen_island"
            elif indiv_name == "long_table":
                target_frame = f"iai_kitchen/long_table:l_table:{suffix.get(indiv_name)}"
            elif indiv_name == "popcorn_table":
                target_frame = f"iai_kitchen/popcorn_table:p_table:{suffix.get(indiv_name)}"
            else:
                target_frame = f"iai_kitchen/{indiv_name}:{indiv_name}:{suffix.get(indiv_name)}"
                
            # 3.3 get pose 
            trans, rot = listen.lookupTransform(ref_frame, target_frame, rospy.Time(0))
    
            # 4. Create and publish MarkerArray for a certain duration
            # 4.5 effect: flashing 
            flashes = 2
            flash_interval = 0.5
            for _ in range(flashes):
                # _publish() also creates markers 
                markers = ma._publish(urdf_new, yaml_data, indiv_name, trans)
                rospy.sleep(flash_interval)
                ma._stop_publishing(markers)
                rospy.sleep(flash_interval)
        
            markers = ma._publish(urdf_new, yaml_data, indiv_name, trans)
            duration= 4.0
            rospy.sleep(duration)
            ma._stop_publishing(markers)
             
            return f"Highlighted {indiv_name} for {duration} seconds."

#########################################################################################################
# HIGHLIGHT A LIST OF OBJECTS
# create MarkerArray on top of list of objects    
#
# Input: the name of a class; all objects of this class are to be highlighted
# Output: objects get highlighted and notice with highlighting duration
#
#
    def highlight_list(self, selected_class, ref_frame="map", duration=4.0):

        # 0. Check whether map is loaded
        if not self.map_server_running():
            rospy.logewarn(f"No semantic map loaded!")
            return
        else: 
            # 1. Load yaml file of objects and get param values of objects (e.g. height = 0.8)
            # which bringup was launched?
            # get .yaml file
            yaml_name  = f"ms3_room_04.yaml"
            yaml_path  = f"/opt/ros/overlay_ws/src/suturo_resources/suturo_resources/urdf/yaml/{yaml_name}"
            yaml_path1 = f"/opt/ros/overlay_ws/src/suturo_resources/suturo_resources/urdf/yaml/ms3_room_04.yaml"
            
    
            # 2. Get/Build target frame
            # frame ending depending on furniture item
            # !!! add: get center 
            suffix = {
                "chair": "chair_center",
                "couch": "place_one",
                "kitchen_island": "table_center",
                "long_table": "table_center",
                "popcorn_table": "table_center",
                "couch_table": "table_center"
            }

            
            onto = get_ontology('/home/jovyan/work/prolog/BA-ont.owl').load()
            indivs = onto[selected_class].instances()
            indivs_names = [indiv.name for indiv in indivs]

            # VORÜBERGEHEN, bis Änderung der knowledge role 
            target_frames = []
            for obj_name in indivs_names:
                if obj_name   == "long_table":
                    target_frame = f"iai_kitchen/long_table:l_table:{suffix.get(obj_name)}"
                elif obj_name == "popcorn_table":
                    target_frame = f"iai_kitchen/popcorn_table:p_table:{suffix.get(obj_name)}"
                elif obj_name == "kitchen_island":
                    target_frame = f"iai_kitchen/kitchen_island"
                else:
                    target_frame = f"iai_kitchen/{obj_name}:{obj_name}:{suffix.get(obj_name)}"
                        
                target_frames.append(target_frame)
    
            all_markers = MarkerArray()
            
            # 3. For each object: 
            for i in range(len(indivs_names)):
                obj = indivs_names[i]
                yp = yaml_path
                tf_target = target_frames[i]
        
                # parse yaml 
                with open(yp, "r") as f:
                    yaml_data = yaml.safe_load(f)
        
                # 3.5. (new) Get urdf info from param server 
                urdf_from_kit_des = rospy.get_param("/kitchen_description")
                urdf_new = URDF.from_xml_string(urdf_from_kit_des)
                rospy.loginfo(f"Loaded URDF for {obj} from parameter server.")
    
                # get pose of target frame
                listen = tf.TransformListener()
                rospy.sleep(1.0)  # wait for TF
                trans, rot = listen.lookupTransform(ref_frame, tf_target, rospy.Time(0))
        
                # create marker array for this object
                markers = ma.create_marker_array(urdf_new, yaml_data, obj, trans)
        
                # append to combined MarkerArray of all objects
                all_markers.markers.extend(markers.markers)
    
            # 4. effect: flashing
            flashes = 2
            flash_interval = 0.5
            for i in range(flashes):
                # add markers again 
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
                   
            return f"Highlighted {len(indivs_names)} objects simultaneously for {duration} seconds"

#########################################################################################################
# TRAJECTORY
# a trajectory of an action is illustrated such as how a drawer is opened
#
# Input: action name and list with points
# Output: points are shown and notice with duration
#
#
    def trajectory(self, action, point_list, frame_id="map", duration=8.0):
        
        # 0. Check whether map is loaded
        if not self.map_server_running():
            rospy.logewarn(f"No semantic map loaded!")
            return
        else: 
            
            marker = Marker()
            marker.header.frame_id = frame_id
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
            
            # effect: spheres appear in order
            for (x,y,z) in point_list:
                p = Point()
                p.x, p.y, p.z = x, y, z
                marker.points.append(p)
                ma.pub.publish(marker_array)
                rospy.sleep(0.3)
    
            rospy.sleep(duration)
            
            # delete markers
            ma._stop_publishing(marker_array)
    
            return f"Published {len(point_list)} points for {duration} seconds"

#######################################################################################
# HIGHLIGHT TRAJECTORY
# a trajectory gets highlighted 
#  
# Input: list of points
# Output: points get highlighted and notice with highlighting duration
#
#
    def highlight_trajectory(self, point_list, frame_id="map", duration=8.0):
        
        # 0. Check whether map is loaded
        if not self.map_server_running():
            rospy.logewarn(f"No semantic map loaded!")
            return
        else: 
            
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
# matches links from an urdf to individuals in database 
#
# Input: -
# Output: a dict with link-individual matchings
#
#
    def link_indiv_matching(self, param_name="/kitchen_description"):
    
     # 0. Check whether map is loaded
        if not self.map_server_running():
            rospy.logwarn(f"No semantic map loaded!")
            return
        else: 
            # 1. get urdf from param server
            rospy.loginfo(f"Read URDF from ROS param server '{param_name}' ....")
            
            urdf_string = rospy.get_param(param_name)
            urdf = URDF.from_xml_string(urdf_string)

            # get all links from the urdf
            link_list = [link.name.lower() for link in urdf.links]
            #print(f"link_list:{link_list}")
                
            # get all individuals from the data base
            onto = get_ontology("/home/jovyan/work/prolog/BA-ont.owl").load()
            indivs = list(onto.individuals())
            indiv_names = [i.name.lower() for i in indivs]
            #print(f"indiv_names: {indiv_names}")

            # get only relevant links with shortened name
            # relevant links example: center_link fot tables 
            urdf_objs = self.extract_relevant_links(link_list)
            urdf_name = list(urdf_objs.keys())
            #print(f"urdf_objs: {urdf_objs}")
            
            # names with some suffix cut from link name and no duplicates
            simple_link_names_from_urdf = [self.simplify_link_name(n) for n in urdf_objs]
            simple_link_names_from_urdf = list(set(simple_link_names_from_urdf))
            #print(f"simple_link_names_from_urdf:{simple_link_names_from_urdf}")

            # matches between shortened link name and individual name 
            matches = {}
            for link in simple_link_names_from_urdf:
                if link in indiv_names:
                  matches[link] = link
                else:
                    matches[link] = None
                    # create class?
        
        
            # only return matches where match not None
            # valid_matches = {matches[link] for ind in matches.values() if ind is not None}
            return matches

    # extract only relevant links such as for table the center_link only 
    def extract_relevant_links(self, frame_list):
        suffix = {
            "table": "center",
            "door": "center",
            "drawer": "main",
            "handle": "handle",
            "couch": "place_two",
            "shelf": "center",
            "fridge": "area",
            "kitchen_island":"kitchen_island"
        }
        objects = {}
        for link in frame_list:
            # split link at ':' and get part before split
            link_name = link.split(':')[0]
            for key, suffixword in suffix.items():
                # if key in split part and suffix in link then add link to dict with split name
                if key in link_name and suffixword in link:
                    objects[link_name] = link
        return objects

    # simplify link_name 
    # keep suffix "main" for drawers for now (see suffix in extract_relevant_links)
    def simplify_link_name(self, name:str):
        remove_ending = ["_main", "_link", "_center", "_origin", "_footprint"]
        for token in remove_ending:
            name = name.replace(token, "")
        return name.lower()

#######################################################################################
## URDF Parser
# first parse for links 
# match links to classes in ontology
# save links as individuals 
# parse for joints
# save joints as individuals 
# save parent link as hasParentLink and child link as hasChildLink
# save hasParentLink hasPart hasChildLink
# Input: -
# Output: -
#
#
    def parse_urdf(self, param_name="/kitchen_description"):
        
        # 0. Check whether map is loaded
        if not self.map_server_running():
            rospy.logwarn(f"No semantic map loaded!")
            return

        # 1. get urdf from param server
        rospy.loginfo(f"Read URDF from ROS param server '{param_name}' ....")
        urdf_string = rospy.get_param(param_name)
        urdf = URDF.from_xml_string(urdf_string)

        # get matching of links to classes from database
        matching = self.link_class_matching()
        ##print (f"l.663: matching link to class:{matching}")

        if matching is None:
            return f"No matchings found, can not proceed with parsing"
        else:
            
            # load ontology
            onto = get_ontology("/home/jovyan/work/prolog/BA-class_extraction.owl").load()
            
            # create new empty onotlogy
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
                ##print(f" l.690: class cap, {claass}")
                
                # if indiv with link_name already exists, ignore
                #existing_indiv = onto1.search_one(iri=f"*{link_name}")
                if link_name in indivs:
                    print(f"already existing link name:{link_name}")
                    ind = onto[link_name]
                    print(f"{link_name} already existing as {ind}")
            
                else:
                    # add link as individual
                    with new_onto: 
                        link_indiv = claass(link_name)
                        print(f"{link_indiv}")
            
      
######################
### JOINTS ###
        with new_onto:
            if new_onto["Joint"] is None:
                class Joint(Thing): pass
            else:
                Joint = new_onto["Joint"]
                
            class hasParentLink(ObjectProperty):
                domain = [Joint]
                range = [Thing]
                            
            class hasChildLink(ObjectProperty):
                domain = [Joint]
                range = [Thing]
    
            class hasPart(ObjectProperty):
                pass

            class isPartOf(ObjectProperty):
                inverse_property = hasPart
       
        # get joints and joint info
        for joint in urdf.joints:
            joint_type = joint.type.lower() 
            joint_name = joint.name.lower() # long_table:l_table:table_origin_joint

            if ':' in joint_name:
                print(f"cut: {joint_name.split(':')[-1]}")
            
           
            # get parent and child link 
            parent_link = joint.parent.lower()
            child_link = joint.child.lower()
            
            # form name of joint class like in onto i.e "Prismatic_Joint"
            joint_class_name = f"{joint_type.capitalize()}_Joint"

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
                    joint_indiv = joint_class(joint_name) #BA-class_extraction.long_table:l_table:table_origin_joint
                    indivs.append(joint_name)
                else:
                    joint_class = new_onto[joint_class_name]
                    joint_indiv = new_onto[joint_name]
                    #print(f"{joint_name} already exists as {joint_indiv}!")

                # get indiv of parent and child link 
                parent_indiv = new_onto[parent_link]
                child_indiv = new_onto[child_link]

                # if parent and child link exist in onto, get them 
                # get links from matching for checking if parent and child in this list/ already saved as indivs
                check_links = [item["link"] for item in matching]
                #print(check_links)

                if parent_link in check_links and child_link in check_links:
                    # parent_indiv = onto.search(iri="*"+ str(parent_link))
                    # child_indiv  = onto.search(iri="*"+ str(child_link))
                    #print(f"parent_indiv: {parent_indiv}")

                    
                    # add relations for hasParentLink and hasChildLink in onto
                    if parent_indiv and child_indiv:
                        joint_indiv.hasParentLink = [parent_indiv]
                        joint_indiv.hasChildLink = [child_indiv]

                    # add hasPart relation if does not already exist
                    if child_indiv not in parent_indiv.hasPart:
                        #parent_indiv[0].hasPart = [child_indiv[0]]
                        parent_indiv.hasPart.append(child_indiv)
                        #print(f"parent_indiv[0]: {parent_indiv[0]}")
                        #print(f"child_indiv[0]: {child_indiv[0]}"))

        # save onto 
        new_onto.save(file="/home/jovyan/work/prolog/BA-class_extraction1.owl", format="rdfxml")
    
        return f"Parsed URDF file successfully!"

#######################################################################################
# matches links from urdf to existing classes 
# 
    def link_class_matching(self, param_name="/kitchen_description"):
        
        #  # 0. Check whether map is loaded
        # if not self.map_server_running():
        #     rospy.logwarn(f"No semantic map loaded!")
        #     return
        
        # 1. Get urdf from param server
        rospy.loginfo(f"Read URDF from ROS param server '{param_name}' ....")
        
        urdf_string = rospy.get_param(param_name)
        urdf = URDF.from_xml_string(urdf_string)

        # 2. Get all links
        urdf_links = [link.name.lower() for link in urdf.links]

        # 3. Cut certain suffixes
        # split link at ':' if exists
        # create list with clean link names
        clean_urdf_links = []
        #remove_suffix = ["_main", "_link", "_center", "_origin", "_footprint"]
        remove_suffix = ["_origin"]
        ## do not remove suffix, as full link name needed for hasChildLink and hasParentLink assignment
        for link in urdf_links:
            link_name = link.split(":")[0] if ":" in link else link
            print(f"link_name: {link_name}")
            for token in remove_suffix:
                if link_name.endswith(token):
                    link_name = link_name.replace(token, "")
            clean_urdf_links.append(link_name)

        clean_urdf_links = list(set(clean_urdf_links))
        print(f"clean_urdf_links: {clean_urdf_links}")
        # Load ontology
        onto = get_ontology('/home/jovyan/work/prolog/BA-class_extraction.owl').load()

        # Get all classes from ontology
        # [Couch, Dishwasher, Door, Drawer, Fridge, Handle, Shelf, Table]
        classes = [claass.name for claass in onto.classes()]
        #print(f"classes: {classes}")
        lower_class_name = [claass.lower() for claass in classes]
        
        # 4. Look through classes and match link to class
        # split link into single words and try matching these to one class 
        # begin from right side so with the last cut word
        # save matching
        link_to_class_matching = []
        for link in clean_urdf_links:
            words = link.split('_')
            for cut in reversed(words):
                if cut in lower_class_name:
                    link_to_class_matching.append({
                        "link": link,
                        "class": cut
                    })
                    break
                    
                elif cut == "island" or cut == "counter":
                    link_to_class_matching.append({
                        "link": link,
                        "class": "table"
                    })

        if link_to_class_matching is not None:
            return link_to_class_matching
        else:
            return None
            










