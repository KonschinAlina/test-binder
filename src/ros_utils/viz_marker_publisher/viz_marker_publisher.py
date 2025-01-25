import atexit
import threading
import time
from typing import List, Optional, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from datastructures.dataclasses import BoxVisualShape, CylinderVisualShape, MeshVisualShape, SphereVisualShape, Color
from datastructures.enums import AxisIdentifier, ObjectType
from datastructures.pose import Pose, Transform
#from designator import ObjectDesignatorDescription
from datastructures.world import World


class VizMarkerPublisher:
    """
    Publishes an Array of visualization marker which represent the situation in the World
    """

    def __init__(self, topic_name="/pycram/viz_marker", interval=0.1):
        """
        The Publisher creates an Array of Visualization marker with a Marker for each link of each Object in the
        World. This Array is published with a rate of interval.

        :param topic_name: The name of the topic to which the Visualization Marker should be published.
        :param interval: The interval at which the visualization marker should be published, in seconds.
        """
        self.topic_name = topic_name
        self.interval = interval

        self.pub = rospy.Publisher(self.topic_name, MarkerArray, queue_size=10)

        self.thread = threading.Thread(target=self._publish)
        self.kill_event = threading.Event()
        self.main_world = World.current_world if not World.current_world.is_prospection_world else World.current_world.world_sync.world

        self.thread.start()
        atexit.register(self._stop_publishing)

    def _publish(self) -> None:
        """
        Constantly publishes the Marker Array. To the given topic name at a fixed rate.
        """
        while not self.kill_event.is_set():
            marker_array = self._make_marker_array()

            self.pub.publish(marker_array)
            time.sleep(self.interval)

    def _make_marker_array(self) -> MarkerArray:
        """
        Creates the Marker Array to be published. There is one Marker for link for each object in the Array, each Object
        creates a name space in the visualization Marker. The type of Visualization Marker is decided by the collision
        tag of the URDF.

        :return: An Array of Visualization Marker
        """
        obj_coloring = False
        marker_array = MarkerArray()
        for obj in self.main_world.objects:
            if obj.obj_type == ObjectType.ROBOT or obj.name == "floor" and not obj.name == "rollin_justin":
                continue
            if obj.obj_type == ObjectType.GENERIC_OBJECT:
                obj_coloring = True
            for link in obj.link_name_to_id.keys():
                geom = obj.get_link_geometry(link)
                if not geom:
                    continue
                msg = Marker()
                msg.header.frame_id = "map"
                msg.ns = obj.name
                msg.id = obj.link_name_to_id[link]
                msg.type = Marker.MESH_RESOURCE
                msg.action = Marker.ADD
                link_pose = obj.get_link_transform(link)
                if obj.get_link_origin(link) is not None:
                    link_origin = obj.get_link_origin_transform(link)
                else:
                    link_origin = Transform()
                link_pose_with_origin = link_pose * link_origin
                msg.pose = link_pose_with_origin.to_pose().pose
                if obj_coloring:
                    colors = {
                        "orange": (1, 0.75, 0, 1),
                        "cucumber": (0, 1, 0, 1),
                        "banana": (1, 1, 0, 1),
                        "lemon": (1, 1, 0, 1),
                        "citron": (1, 1, 0, 1),
                        "lime": (0.75, 1.0, 0.0, 1),
                        "apple": (1, 0, 0, 1),
                        "tomato": (1, 0, 0, 1),
                        "peach": (1.0, 0.8, 0.64, 1),
                        "kiwi": (0.76, 0.88, 0.52, 1),
                        "avocado": (0.0, 0.5, 0.0, 1),
                        "bowl": (1, 0, 0, 1),
                        "jeroen_cup":  (0, 0, 1, 1)
                    }
                    color = colors.get(obj.name, [1, 1, 1, 1])

                else:
                    color = [1, 1, 1, 1] if obj.link_name_to_id[link] == -1 else obj.get_link_color(link).get_rgba()

                msg.color = ColorRGBA(*color)
                msg.lifetime = rospy.Duration(1)

                if isinstance(geom, MeshVisualShape):
                    msg.type = Marker.MESH_RESOURCE
                    msg.mesh_resource = "file://" + geom.file_name
                    if hasattr(geom, "scale") and geom.scale:
                        msg.scale = Vector3(geom.scale[0], geom.scale[1], geom.scale[2])
                    else:
                        msg.scale = Vector3(1, 1, 1)
                    msg.mesh_use_embedded_materials = True
                elif isinstance(geom, CylinderVisualShape):
                    msg.type = Marker.CYLINDER
                    msg.scale = Vector3(geom.radius * 2, geom.radius * 2, geom.length)
                elif isinstance(geom, BoxVisualShape):
                    msg.type = Marker.CUBE
                    msg.scale = Vector3(*geom.size)
                elif isinstance(geom, SphereVisualShape):
                    msg.type = Marker.SPHERE
                    msg.scale = Vector3(geom.radius * 2, geom.radius * 2, geom.radius * 2)

                marker_array.markers.append(msg)
        return marker_array

    def _stop_publishing(self) -> None:
        """
        Stops the publishing of the Visualization Marker update by setting the kill event and collecting the thread.
        """
        self.kill_event.set()
        self.thread.join()

#
#class ManualMarkerPublisher:
#    """
#    Class to manually add and remove marker of objects and poses.
#    """

#    def __init__(self, topic_name: str = '/pycram/manual_marker', interval: #float = 0.1):
#        """
#        The Publisher creates an Array of Visualization marker with a marker for a pose or object.
#        This Array is published with a rate of interval.

#        :param topic_name: Name of the marker topic
#        :param interval: Interval at which the marker should be published
#        """
#        self.start_time = None
#        self.marker_array_pub = rospy.Publisher(topic_name, MarkerArray, queue_size=10)

#        self.marker_array = MarkerArray()
#        self.marker_overview = {}
#        self.current_id = 0

#        self.interval = interval
#        self.log_message = None

#    def publish(self, pose: Pose, color: Optional[List] = None, bw_object: Optional[ObjectDesignatorDescription] = None,
#                name: Optional[str] = None):
#        """
#        Publish a pose or an object into the MarkerArray.
#        Priorities to add an object if possible

#        :param pose: Pose of the marker
#        :param color: Color of the marker if no object is given
#        :param bw_object: Object to add as a marker
#        :param name: Name of the marker
#        """

#        if color is None:
#            color = [1, 0, 1, 1]

#        self.start_time = time.time()
#        thread = threading.Thread(target=self._publish, args=(pose, bw_object, name, color))
#        thread.start()
        # rospy.loginfo(self.log_message)
#        thread.join()

#    def _publish(self, pose: Pose, bw_object: Optional[ObjectDesignatorDescription] = None, name: Optional[str] = None,
#                 color: Optional[List] = None):
#        """
#        Publish the marker into the MarkerArray
#        """
#        stop_thread = False
#        duration = 2

#        while not stop_thread:
#            if time.time() - self.start_time > duration:
#                stop_thread = True
#            if bw_object is None:
#                self._publish_pose(name=name, pose=pose, color=color)
#            else:
#                self._publish_object(name=name, pose=pose, bw_object=bw_object)

#            rospy.sleep(self.interval)

#    def _publish_pose(self, name: str, pose: Pose, color: Optional[List] = None):
#        """
#        Publish a Pose as a marker

#        :param name: Name of the marker
#        :param pose: Pose of the marker
#        :param color: Color of the marker
#        """

#        if name is None:
#            name = 'pose_marker'

#        if name in self.marker_overview.keys():
#            self._update_marker(self.marker_overview[name], new_pose=pose)
#            return

#        color_rgba = ColorRGBA(*color)
#        self._make_marker_array(name=name, marker_type=Marker.ARROW, marker_pose=pose,
#                                marker_scales=(0.05, 0.05, 0.05), color_rgba=color_rgba)
#        self.marker_array_pub.publish(self.marker_array)
#        self.log_message = f"Pose '{name}' published"

#    def _publish_object(self, name: Optional[str], pose: Pose, bw_object: ObjectDesignatorDescription):
#        """
#        Publish an Object as a marker

#        :param name: Name of the marker
#        :param pose: Pose of the marker
#        :param bw_object: ObjectDesignatorDescription for the marker
#        """

#        bw_real = bw_object.resolve()

#        if name is None:
#            name = bw_real.name

#        if name in self.marker_overview.keys():
#            self._update_marker(self.marker_overview[name], new_pose=pose)
#            return

#        path = bw_real.world_object.root_link.geometry.file_name

#        self._make_marker_array(name=name, marker_type=Marker.MESH_RESOURCE, marker_pose=pose,
#                                path_to_resource=path)

#        self.marker_array_pub.publish(self.marker_array)
#        self.log_message = f"Object '{name}' published"

#    def _make_marker_array(self, name, marker_type: int, marker_pose: Pose, #marker_scales: Tuple = (1.0, 1.0, 1.0),
#                           color_rgba: ColorRGBA = ColorRGBA(*[1.0, 1.0, 1.0, 1.0]),
#                           path_to_resource: Optional[str] = None):
#        """
#        Create a Marker and add it to the MarkerArray

#        :param name: Name of the Marker
#        :param marker_type: Type of the marker to create
#        :param marker_pose: Pose of the marker
#        :param marker_scales: individual scaling of the markers axes
#        :param color_rgba: Color of the marker as RGBA
#        :param path_to_resource: Path to the resource of a Bulletworld object
#        """

#        frame_id = marker_pose.header.frame_id
#        new_marker = Marker()
#        new_marker.id = self.current_id
#        new_marker.header.frame_id = frame_id
#        new_marker.ns = name
#        new_marker.header.stamp = rospy.Time.now()
#        new_marker.type = marker_type
#        new_marker.action = Marker.ADD
#        new_marker.pose = marker_pose.pose
#        new_marker.scale.x = marker_scales[0]
#        new_marker.scale.y = marker_scales[1]
#        new_marker.scale.z = marker_scales[2]
#        new_marker.color.a = color_rgba.a
#        new_marker.color.r = color_rgba.r
#        new_marker.color.g = color_rgba.g
#        new_marker.color.b = color_rgba.b

#        if path_to_resource is not None:
#            new_marker.mesh_resource = 'file://' + path_to_resource

#        self.marker_array.markers.append(new_marker)
#        self.marker_overview[name] = new_marker.id
#        self.current_id += 1

#    def _update_marker(self, marker_id: int, new_pose: Pose) -> bool:
#        """
#        Update an existing marker to a new pose

#        :param marker_id: id of the marker that should be updated
#        :param new_pose: Pose where the updated marker is set

#        :return: True if update was successful, False otherwise
#        """

        # Find the marker with the specified ID
#        for marker in self.marker_array.markers:
#            if marker.id == marker_id:
                # Update successful
#                marker.pose = new_pose
#                self.log_message = f"Marker '{marker.ns}' updated"
#                self.marker_array_pub.publish(self.marker_array)
#               return True

        # Update was not successful
        # rospy.logwarn(f"Marker {marker_id} not found for update")
#        return False

#    def remove_marker(self, bw_object: Optional[ObjectDesignatorDescription] = None, name: Optional[str] = None):
#        """
#        Remove a marker by object or name

#        :param bw_object: Object which marker should be removed
#        :param name: Name of object that should be removed
#        """

#        if bw_object is not None:
#            bw_real = bw_object.resolve()
#            name = bw_real.name

#        if name is None:
            # rospy.logerr('No name for object given, cannot remove marker')
#            return

#        marker_id = self.marker_overview.pop(name)

#        for marker in self.marker_array.markers:
#            if marker.id == marker_id:
#                marker.action = Marker.DELETE

#        self.marker_array_pub.publish(self.marker_array)
#        self.marker_array.markers.pop(marker_id)
        # rospy.loginfo(f"Removed Marker '{name}'")

#    def clear_all_marker(self):
#        """
#        Clear all existing markers
#        """
#        for marker in self.marker_array.markers:
#            marker.action = Marker.DELETE

#        self.marker_overview = {}
#        self.marker_array_pub.publish(self.marker_array)

        # rospy.loginfo('Removed all markers')


class AxisMarkerPublisher:
    def __init__(self, topic='/pycram/axis_marker', frame_id='map'):

        self.marker_pub = rospy.Publisher(topic, MarkerArray, queue_size=10)

        self.marker_array = MarkerArray()
        self.marker_overview = {}
        self.current_id = 0
        self.frame_id = frame_id

        self.length = None
        self.duration = None
        self.poses = None
        self.axis = None
        self.colorclass = Color()
        self.color = None

        self.thread = threading.Thread(target=self._publish)

    def publish(self, poses: List[Pose], duration=15.0, length=0.1, name=None):
        """
        Publish a MarkerArray with given pose and axis.
        Duration, length and color of the line are optional.

        :param poses: List of Poses to be visualized
        :param axis: Orientation for the Line
        :param duration: Duration of the marker
        :param length: Length of the line
        :param color: Color of the line if it should be personalized
        """

        self.name = name
        self.poses = poses
        self.duration = duration
        self.length = length
        color = self.colorclass

        for pose in self.poses:
            self._create_line(pose, AxisIdentifier.X.value, self.duration, self.length,
                              color.get_color_from_string('red'))
            self._create_line(pose, AxisIdentifier.Y.value, self.duration, self.length,
                              color.get_color_from_string('green'))
            self._create_line(pose, AxisIdentifier.Z.value, self.duration, self.length,
                              color.get_color_from_string('blue'))

        self.thread.start()
        # rospy.loginfo("Publishing axis visualization")
        self.thread.join()
        # rospy.logdebug("Stopped Axis visualization")

    def _publish(self):
        if self.name in self.marker_overview.keys():
            self._update_marker(self.marker_overview[self.name], new_pose=self.pose)
            return

        stop_thread = False
        duration = 1
        frequency = 0.2
        start_time = time.time()

        while not stop_thread:
            if time.time() - start_time > duration:
                stop_thread = True

            # Publish the MarkerArray
            self.marker_pub.publish(self.marker_array)

            rospy.sleep(frequency)

    def _get_color(self, axis):
        """
        Get the color of the given Axis

        :param axis: Used axis
        """
        color = self.colorclass
        if axis == AxisIdentifier.X.value:
            axis_color = color.get_color_from_string('red')
        elif axis == AxisIdentifier.Y.value:
            axis_color = color.get_color_from_string('green')
        elif axis == AxisIdentifier.Z.value:
            axis_color = color.get_color_from_string('blue')
        else:
            # rospy.logwarn(f'Axis {str(axis)} is not valid')
            axis_color = color.get_color_from_string('red')

        return axis_color

    def _create_line(self, pose, axis, duration, length, color):
        """
        Create a line marker to add to the marker array.

        :param pose: Starting pose of the line
        :param axis: Axis along which the line is set
        :param duration: Duration of the line marker
        :param length: Length of the line
        :param color: Optional color for the Line
        """

        def normalize_quaternion(q):
            norm = np.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
            if norm > 0:
                return q.x / norm, q.y / norm, q.z / norm, q.w / norm
            return q.x, q.y, q.z, q.w

        def quaternion_multiply(q1, q2):
            x1, y1, z1, w1 = q1
            x2, y2, z2, w2 = q2
            return (
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2,
                w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2,
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            )

        def rotate_axis_by_quaternion(axis, quaternion):
            # Normalize the quaternion to avoid distortions
            qx, qy, qz, qw = normalize_quaternion(quaternion)

            # Represent axis as quaternion (x, y, z, 0)
            axis_quat = (*axis, 0)

            # Quaternion components
            q = (qx, qy, qz, qw)

            # Compute the inverse (conjugate for unit quaternion)
            q_conjugate = (-qx, -qy, -qz, qw)

            # Rotate the vector
            rotated_quat = quaternion_multiply(quaternion_multiply(q, axis_quat), q_conjugate)

            # The rotated vector is the vector part of the resulting quaternion
            return rotated_quat[:3]

        # Create a line marker for the axis
        line_marker = Marker()
        line_marker.header.frame_id = self.frame_id
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = f'axis_visualization_{self.current_id}'
        line_marker.id = self.current_id
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.01  # Line width
        line_marker.color = color
        line_marker.lifetime = rospy.Duration(duration)

        # Start point at the position specified by the pose (translation part)
        start_point = Point()
        start_point.x = pose.position.x
        start_point.y = pose.position.y
        start_point.z = pose.position.z

        quaternion = pose.orientation
        rotated_axis = rotate_axis_by_quaternion(axis, quaternion)

        # Calculate the end point by adding the rotated axis vector (scaled by length)
        end_point = Point()
        end_point.x = pose.position.x + (rotated_axis[0] * length)
        end_point.y = pose.position.y + (rotated_axis[1] * length)
        end_point.z = pose.position.z + (rotated_axis[2] * length)

        line_marker.points.append(start_point)
        line_marker.points.append(end_point)

        # Add the line marker to the MarkerArray
        self.marker_array.markers.append(line_marker)
        self.marker_overview[f"{self.name}_{self.current_id}"] = line_marker.id
        self.current_id += 1

    def _update_marker(self, marker_id, new_pose):
        """
        Update an existing marker to a new pose

        :param marker_id: id of the marker that should be updated
        :param new_pose: Pose where the updated marker is set
        """

        # Find the marker with the specified ID
        for marker in self.marker_array.markers:
            if marker.id == marker_id:
                # Update successful
                marker.pose = new_pose
                # rospy.logdebug(f"Marker {marker_id} updated")
                self.marker_pub.publish(self.marker_array)
                return True

        # Update was not successful
        # rospy.logwarn(f"Marker {marker_id} not found for update")
        return False


class CostmapPublisher:
    """
    Class to manually add and remove marker of objects and poses.
    """

    def __init__(self, topic_name: str = '/pycram/costmap_marker', interval: float = 0.1):
        """
        The Publisher creates an Array of Visualization marker with a marker for a pose or object.
        This Array is published with a rate of interval.

        :param topic_name: Name of the marker topic
        :param interval: Interval at which the marker should be published
        """
        self.start_time = None
        self.marker_array_pub = rospy.Publisher(topic_name, Marker, queue_size=10)

        self.marker = Marker()
        self.marker_overview = {}
        self.current_id = 0

        self.interval = interval
        self.log_message = None

    def publish(self, poses: List[Pose], size: float = None, name: Optional[str] = None, scale: Optional[float] = 0.4):
        """
        Publish a pose or an object into the MarkerArray.
        Priorities to add an object if possible

        :param poses: List of Pose of the Costmap
        :param scale: Scale of the z-axis of the costmap
        :param name: Name of the marker
        """

        self.start_time = time.time()
        thread = threading.Thread(target=self._publish, args=(poses, name, size, scale))
        thread.start()
        # rospy.loginfo(self.log_message)
        thread.join()

    def _publish(self, poses: List[Pose], name: Optional[str] = None,
                 size=None, scale=0.4):
        """
        Publish the marker into the MarkerArray
        """
        stop_thread = False
        duration = 2

        while not stop_thread:
            if time.time() - self.start_time > duration:
                stop_thread = True
            self._publish_costmap(name=name, poses=poses, size=size, scale=scale)

            rospy.sleep(self.interval)

    def _publish_costmap(self, name: str, poses: List[Pose], size=None, scale=0.4):
        """
        Publish a Pose as a marker

        :param name: Name of the marker
        :param pose: Pose of the marker
        :param scale: Scale of the z-axis of the costmap
        """

        if name is None:
            name = 'costmap_marker'

        if name in self.marker_overview.keys():
            self._update_marker(self.marker_overview[name], new_poses=poses)
            return

        self._make_marker_array(name=name, marker_type=Marker.POINTS, costmap_poses=poses,
                                marker_scales=(size, size, size), z_scale=scale)
        self.marker_array_pub.publish(self.marker)
        self.log_message = f"Pose '{name}' published"

    def _make_marker_array(self, name, marker_type: int, costmap_poses: List[Pose],
                           marker_scales: Tuple = (1.0, 1.0, 1.0), z_scale: float = 0.4):
        """
        Create a Marker and add it to the MarkerArray

        :param name: Name of the Marker
        :param marker_type: Type of the marker to create
        :param marker_pose: Pose of the marker
        :param marker_scales: individual scaling of the markers axes
        :param scale: Scale of the z-axis of the costmap
        """

        frame_id = "map"
        new_marker = Marker()
        new_marker.id = self.current_id
        new_marker.header.frame_id = frame_id
        new_marker.ns = name
        new_marker.header.stamp = rospy.Time.now()
        new_marker.type = marker_type
        new_marker.action = Marker.ADD
        for costmap_pose in costmap_poses:
            color_rgba = Color.gaussian_color_map(costmap_pose.position.z, 0, z_scale)
            new_marker.scale.x = marker_scales[0]
            new_marker.scale.y = marker_scales[1]
            new_marker.scale.z = marker_scales[2]
            new_marker.colors.append(color_rgba)

            point = Point()
            point.x = costmap_pose.position.x
            point.y = costmap_pose.position.y
            point.z = costmap_pose.position.z
            new_marker.points.append(point)

        self.marker = new_marker
        self.marker_overview[name] = new_marker.id
        self.current_id += 1

    def _update_marker(self, marker_id: int, new_poses: List[Pose]) -> bool:
        """
        Update an existing marker to a new pose

        :param marker_id: id of the marker that should be updated
        :param new_pose: Pose where the updated marker is set

        :return: True if update was successful, False otherwise
        """

        # Find the marker with the specified ID
        if self.marker.id == marker_id:
            # Update successful
            new_points = []
            for new_pose in new_poses:
                self.marker.points = []
                point = Point()
                point.x = new_pose.position.x
                point.y = new_pose.position.y
                point.z = new_pose.position.z
                new_points.append(point)
            self.marker.points = new_points
            self.log_message = f"Marker '{self.marker.ns}' updated"
            self.marker_array_pub.publish(self.marker)
            return True

        # Update was not successful
        # rospy.logwarn(f"Marker {marker_id} not found for update")
        return False

    def remove_marker(self, name: Optional[str] = None):
        """
        Remove a marker by object or name

        :param bw_object: Object which marker should be removed
        :param name: Name of object that should be removed
        """

        if name is None:
            # rospy.logerr('No name for object given, cannot remove marker')
            return

        marker_id = self.marker_overview.pop(name)

        if self.marker.id == marker_id:
            self.marker.action = Marker.DELETE

        self.marker_array_pub.publish(self.marker)
        # rospy.loginfo(f"Removed Marker '{name}'")

    def clear_all_marker(self):
        """
        Clear all existing markers
        """
        self.marker.action = Marker.DELETE

        self.marker_overview = {}
        self.marker_array_pub.publish(self.marker)

        # rospy.loginfo('Removed all markers')
