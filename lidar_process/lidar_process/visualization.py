from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from builtin_interfaces.msg import Duration

from . import utils

class IDManager(object):
    def __init__(self, start_id=0):
        self.id= start_id - 1
        return

    def get_new_id(self):
        self.id += 1
        return self.id

colors = utils.Colors()
id_manager = IDManager()

def create_entity_marker(x, y, id, header, ns):
    # Returns
    # A colored sphere marker showing the position of the entity
    # A text next to the sphere marker displaying the id

    marker = Marker()
    marker.header = header
    marker.header.frame_id = "map"
    marker.ns = ns
    marker.id = id_manager.get_new_id()
    marker.type = 2 # sphere
    marker.action = 0 # add/modify

    marker_pose = Pose()
    marker_position = Point()
    marker_orientation = Quaternion()
    marker_position.x = x
    marker_position.y = y
    marker_position.z = 0.0
    marker_orientation.x = 0.0
    marker_orientation.y = 0.0
    marker_orientation.z = 0.0
    marker_orientation.w = 1.0
    marker_pose.position = marker_position
    marker_pose.orientation = marker_orientation
    marker.pose = marker_pose

    marker_size = 0.5
    marker_scale = Vector3()
    marker_scale.x = marker_size
    marker_scale.y = marker_size
    marker_scale.z = marker_size
    marker.scale = marker_scale

    marker.color = colors.get_color(id)
    marker_lifetime = Duration()
    marker_lifetime.sec = 0
    marker_lifetime.nanosec = int(15e7)
    marker.lifetime = marker_lifetime
    marker.frame_locked = False

    text_marker = Marker()
    text_marker.header = header
    marker.header.frame_id = "map"
    text_marker.ns = ns
    text_marker.id = id_manager.get_new_id()
    text_marker.type = 9 # text
    text_marker.action = 0
    text_marker.pose = Pose()
    text_marker.pose.position.x = x + 1.0
    text_marker.pose.position.y = y
    text_marker.pose.orientation.w = 1.0
    text_marker.scale = marker.scale
    text_marker.color = colors.get_color(id, fixed=0)
    text_marker.lifetime = marker.lifetime
    text_marker.frame_locked = False
    text_marker.text = str(id)
    return marker, text_marker

def create_entity_marker_with_velocity(x, y, vx, vy, id, header, ns):
    # Returns
    # A colored sphere marker showing the position of the entity
    # A text next to the sphere marker displaying the id
    # An arrow on the sphere marker showing the velocity of the entity
    marker, text_marker = create_entity_marker(x, y, id, header, ns)

    vel_marker = Marker()
    vel_marker.header = header
    vel_marker.header.frame_id = "map"
    vel_marker.ns = ns
    vel_marker.id = id_manager.get_new_id()
    vel_marker.type = 4 # line
    vel_marker.action = 0
    vel_marker.color = colors.get_color(id, fixed=0)
    vel_marker_scale = Vector3()
    vel_marker_scale.x = 0.1
    vel_marker.scale = vel_marker_scale
    vel_marker_pt_1 = Point()
    vel_marker_pt_1.x = x
    vel_marker_pt_1.y = y
    vel_marker_pt_1.z = 0.0
    vel_marker_pt_2 = Point()
    if vx == 0:
        vx = 0.0001
    if vy == 0:
        vy = 0.0001
    vel_scale = 1.0
    vel_marker_pt_2.x = x + vx * vel_scale
    vel_marker_pt_2.y = y + vy * vel_scale
    vel_marker_pt_2.z = 0.0
    vel_marker.points = [vel_marker_pt_1, vel_marker_pt_2]
    vel_marker.lifetime = marker.lifetime
    vel_marker.frame_locked = False

    return marker, text_marker, vel_marker