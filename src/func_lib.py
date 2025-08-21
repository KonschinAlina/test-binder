#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
func_lib.py

A lib that provides simple functions used for querying and highlighting a sem map.
"""

import rospy
import xml.etree.ElementTree as ET
import time


def highlight(object_name, color=(1,1,1,1), duration=180):
    urdf_str = rospy.get_param("/robot_description")
    tree = ET.ElementTree(ET.fromstring(urdf_str))
    root = tree.getroot()

    found = False
    for link in root.findall(".//link"):
    	if object_name in link.attrib.get("name", ""):
   	 mat = link.find("visual/material")
    	 if mat is not None:
            color_tag = mat.find("color")
            if color_tag is not None:
               rgba_str = " ".join(map(str, color))
               color_tag.set("rgba", rgba_str)
               found = True

    if not found:
	rospy.logwarn(f"Objekt '{object_name}' nicht im URDF gefunden.")
	return

    rospy.set_param("/robot_description", ET.tostring(root, encoding="unicode"))
    rospy.loginfo(f"Objekt '{object_name}' hervorgehoben.")

    time.sleep(duration)
    rospy.set_param("/robot_description", urdf_str)
    rospy.loginfo(f"Highlight für '{object_name}' zurückgesetzt.")	
	

