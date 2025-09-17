#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
func_lib.py

A lib that provides simple functions used for querying and highlighting and creating trajectories on a sem map.
"""


import os
import tf
import sys
import yaml 
import rospy
import tempfile
import subprocess
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import MarkerArray, Marker
sys.path.append("/home/jovyan/work/src/")
import ros_utils.viz_marker_publisher.viz_marker_array_publisher as MA

ma = MA.VizMarkerArrayPublisher()


class FuncLib:

    def __init__(self, topic_name="/pycram/viz_marker"):
        self.topic_name = topic_name
        self.pub = rospy.Publisher(self.topic_name, MarkerArray, queue_size=10)
        if not rospy.core.is_initialized():
            rospy.init_node('func_lib', anonymous=True)
             
        

#########################################################################################################
# Pose
# get the pose with tf
    def object_pose(self, object_frame, ref_frame="map"):
        listener = tf.TransformListener()
        rospy.sleep(1.0)
        try:
            trans, rot = listener.lookupTransform(ref_frame, object_frame, rospy.Time(0))
            return [ref_frame, [trans[0], trans[1], trans[2]], [rot[0], rot[1],rot[2], rot[3]]]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

        
#########################################################################################################
# Highlight
# create MarkerArray on top of object to highlight it  
    def highlight(self, xacro_path:str, object_name:str, ref_frame="map"):
      
        # 1. Load yaml file of object and get object params
        yaml_path ="/opt/ros/overlay_ws/src/suturo_resources/suturo_resources/urdf/yaml/ms3_room_04.yaml"
        with open(yaml_path, "r") as f:
             yaml_data = yaml.safe_load(f)
        
        # 2. Parse .urdf from .xacro file
        obj_urdf = self._parse_urdf_from_xacro(xacro_path, object_name, yaml_data)
        
        # 3. Check whether map is loaded
        # add rospy.logwarn(f"map not loaded")
        
        # 4. Get pose of center_link and give to create_marker
        # frame ending depends on furniture item type
        suffix = {
            "chair": "chair_center",
            "couch": "place_one",
            "kitchen_island": "table_center",
            "long_table": "table_center",
            "popcorn_table": "table_center"
            #"dishwasher": "table_center"
        }

        # 5. get pose of center_link
        listen = tf.TransformListener()
        rospy.sleep(1.0)
        if object_name == "dishwasher":
            #target_frame = f"iai_kitchen/dishwasher_table:d_table:{suffix.get(object_name)}"
            target_frame = f"iai_kitchen/{object_name}_main"
        else:
            target_frame = f"iai_kitchen/{object_name}:{object_name}:{suffix.get(object_name)}"
        
        trans, rot = listen.lookupTransform(ref_frame, target_frame, rospy.Time(0))

        # 6. Create and publish MarkerArray for a certain duration
        # 6.5 effect: flashing 
        flashes = 2
        flash_interval = 0.5
        for _ in range(flashes):
            # _publish() also creates markers 
            markers = ma._publish(obj_urdf, yaml_data, object_name, trans)
            rospy.sleep(flash_interval)
            ma._stop_publishing(markers)
            rospy.sleep(flash_interval)
    
        markers = ma._publish(obj_urdf, yaml_data, object_name, trans)
        duration= 4.0
        rospy.sleep(duration)
        ma._stop_publishing(markers)
         
        return f"Highlighted {object_name} for {duration} seconds."

#########################################################################################################
# Highlight
# create MarkerArray on top of list of objects    
    def highlight_list(self, xacro_paths, object_list, ref_frame="map", duration=4.0):

        # 1. Load yaml file of objects and get params of objects
        yaml_path = "/opt/ros/overlay_ws/src/suturo_resources/suturo_resources/urdf/yaml/ms3_room_04.yaml"
        
        if not (len(xacro_paths) == len(object_list)):
            roslog.warn("All input lists must have the same length")

        # 2. Get/Build target frame
        # frame ending depending on furniture item
        # !!! add: get center 
        suffix = {
            "chair": "chair_center",
            "couch": "place_one",
            "kitchen_island": "table_center",
            "long_table": "table_center",
            "popcorn_table": "table_center"
        }
        
        target_frames = [
                f"iai_kitchen/{obj_name}:{obj_name}:{suffix.get(obj_name)}"
                for obj_name in object_list
            ]

        all_markers = MarkerArray()
        
        # 3. For each object: 
        for i in range(len(object_list)):
            xf = xacro_paths[i]
            obj = object_list[i]
            yp = yaml_path
            tf_target = target_frames[i]
    
            # parse yaml 
            with open(yp, "r") as f:
                yaml_data = yaml.safe_load(f)
    
            # build .urdf wrapper and parse
            obj_urdf = self._parse_urdf_from_xacro(xf, obj, yaml_data)

            # get pose of target frame
            listen = tf.TransformListener()
            rospy.sleep(1.0)  # wait for TF
            trans, rot = listen.lookupTransform(ref_frame, tf_target, rospy.Time(0))
    
            # create marker array for this object
            markers = ma.create_marker_array(obj_urdf, yaml_data, obj, trans)
    
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
               
        return f"Highlighted {len(object_list)} objects simultaneously for {duration} seconds"

##########################################################################################################
    def _parse_urdf_from_xacro(self, xacro_path: str, object_name: str, yaml_data: dict) -> URDF:
        
        # Search in yaml key:name for object_name i.e "name: popcorn_table"
        obj_entry = None
        for _, objs in yaml_data.items():
            #print(yaml_data.items())
            #print(objs)
            if not isinstance(objs, dict):
                continue
            
            # # some objects are named "suturo_object"
            # if object_name.startswith("suturo_"):
            #     base_name = object_name[len("suturo_"):]  # remove "suturo_"
            # else:
            #     base_name = object_name
                
            for key, entry in objs.items():
                #print(key, entry)
                if isinstance(entry, dict):
                    entry_name = entry.get("name", "")
                    if entry_name == object_name or entry_name == f"suturo_{object_name}":
                        obj_entry = entry
                        #print(f"obj_entry: {obj_entry}")
                        break
                    
            if obj_entry:
                break
    
        if obj_entry is None:
            rospy.logwarn(f"No yaml entry found for object '{object_name}'")
            return None

        
        # 2. Build wrapper urdf file, with macro instantiation
        # Deviance: popcorn_table because the macro is smaller (i.e. no depth aka size_x)
        if object_name == "popcorn_table":
            macro_name = "iai_popcorn_table"
            wrapper_urdf = f"""
            <robot name="{object_name}_wrapper" xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:include filename="{xacro_path}"/>
                <xacro:{macro_name} 
                    name="{obj_entry['name']}:{obj_entry['knowledge_role']}"
                    parent="{obj_entry['parent']}">
                    <origin xyz="{obj_entry['x_pos']} {obj_entry['y_pos']} 0" rpy="0 0 {obj_entry['z_rot']}" />
                </xacro:{macro_name}>
            </robot>
            """
            #print(f"wrapper_urdf_if: {wrapper_urdf}")
            
        elif object_name.endswith("table") or object_name.endswith("counter") or object_name.endswith("island"):
            macro_name = "suturo_table"
            wrapper_urdf = f"""
            <robot name="{object_name}_wrapper" xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:include filename="{xacro_path}"/>
                <xacro:{macro_name} 
                    name="{obj_entry['name']}:{obj_entry['knowledge_role']}"
                    parent="{obj_entry['parent']}"
                    size_x="{obj_entry['depth']}"
                    size_y="{obj_entry['width']}"
                    size_z="{obj_entry['height']}">
                    <origin xyz="{obj_entry['x_pos']} {obj_entry['y_pos']} 0" rpy="0 0 {obj_entry['z_rot']}" />
                </xacro:{macro_name}>
            </robot>
            """
            #print(f"wrapper_urdf_elif: {wrapper_urdf}")

        elif "dishwasher" == object_name:
            macro_name = "suturo_dishwasher"
            wrapper_urdf = f"""
            <robot name="{object_name}_wrapper" xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:include filename="{xacro_path}"/>
                <xacro:{macro_name} 
                    name="{obj_entry['name']}:{obj_entry['knowledge_role']}"
                    parent="{obj_entry['parent']}"
                    table_size_x="{obj_entry['table_size_x']}"
                    table_size_y="{obj_entry['table_size_y']}"
                    table_size_z="{obj_entry['table_size_z']}"
                    height = "{obj_entry['height']}"
                    table_height = "{obj_entry['table_height']}"
                    handle_x = "{obj_entry['handle_x']}"
                    handle_y = "{obj_entry['handle_y']}"
                    handle_z = "{obj_entry['handle_z']}">
                    <origin xyz="{obj_entry['x_pos']} {obj_entry['y_pos']} 0" rpy="0 0 {obj_entry['z_rot']}" />
                </xacro:{macro_name}>
            </robot>
            """
            #print(f"wrapper_urdf_if: {wrapper_urdf}")
        
        else:
            macro_name = f"suturo_{object_name}"
            wrapper_urdf = f"""
            <robot name="{object_name}_wrapper" xmlns:xacro="http://www.ros.org/wiki/xacro">
                <xacro:include filename="{xacro_path}"/>
                <xacro:{macro_name}
                    name="{obj_entry['name']}:{obj_entry['knowledge_role']}"
                    parent="{obj_entry['parent']}"
                    size_x="{obj_entry['depth']}"
                    size_y="{obj_entry['width']}"
                    size_z="{obj_entry['height']}">
                    <origin xyz="{obj_entry['x_pos']} {obj_entry['y_pos']} 0" rpy="0 0 {obj_entry['z_rot']}" />
                </xacro:{macro_name}>
            </robot>
            """
            #print(f"wrapper_urdf_else: {wrapper_urdf}")
            
    
        # 3. generate temporate file aka a urdf file
        with tempfile.NamedTemporaryFile(delete=False, suffix=".xacro") as tmp:
            tmp.write(wrapper_urdf.encode("utf-8"))
            tmp_path = tmp.name
        try:
            # build xml string from .urdf.xacro file content
            xml_str = subprocess.check_output(["xacro", tmp_path])
            # print(xml_string)
        finally:
            os.remove(tmp_path)
    
        # 4. return parsed .urdf file
        return URDF.from_xml_string(xml_str)

####################################################################
# trajectory

    def trajectory(self, action, point_list, frame_id="map", duration=8.0):
        
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "trajectory"
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

####################################################################
# highlight trajectory

    def highlight_trajectory(self, point_list, frame_id="map", duration=8.0):
        
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "highlight_trajectory"
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

    
