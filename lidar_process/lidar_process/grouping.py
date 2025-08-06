import sys
import time
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull

DETAIL_MODE = False
    
def get_groups(data, threshold, core_samples):
    # performs DBSCAN and clusters according to threshold and 
    # number of sample points for the cluster cores
    #
    # returns the indexed labels with the same size as data
    clusters = DBSCAN(eps=threshold, min_samples=core_samples).fit_predict(data)
    return clusters

def DBScan_grouping(labels, properties, standard):
    # DBSCAN clustering by performing clustering in subgroups.
    # Inputs:
    # labels: the input labels. This will be destructively updated to 
    #         reflect the group memberships after DBSCAN.
    # properties: the input that clustering is based on.
    #             Could be positions, velocities or orientation.
    # standard: the threshold value for clustering.

    max_lb = max(labels)
    for lb in range(max_lb + 1):
        sub_properties = []
        sub_idxes = []
        # Only perform DBSCAN within groups (i.e. have the same membership id)
        for i in range(len(labels)):
            if labels[i] == lb:
                sub_properties.append(properties[i])
                sub_idxes.append(i)
    
        # If there's only 1 person then no need to further group
        if len(sub_idxes) > 1:
            sub_labels = get_groups(sub_properties, standard, 1)
            max_label = max(labels)

            # db.fit_predict always return labels starting from index 0
            # we can add these to the current biggest id number to create 
            # new group ids.
            for i, sub_lb in enumerate(sub_labels):
                if sub_lb > 0:
                    labels[sub_idxes[i]] = max_label + sub_lb
    return labels

def pedestrian_grouping(position_array, velocity_array, param_pos, param_spd, param_ori, static_threshold=0.5):
    # Performs grouping given a position array and a cooresponding velocity array
    # The grouping is performed based on three levels of parameters
    # First orientation, then speed, finally position

    if len(position_array) != len(velocity_array):
        print("[grouping] Error: pos array and vel array sohuld be the same size!")
        sys.exit(0)

    num_people = len(position_array)
    if (num_people == 0):
        return []

    vel_orientation_array = []
    vel_magnitude_array = []
    for [vx, vy] in velocity_array:
        velocity_magnitude = np.sqrt(vx ** 2 + vy ** 2)
        if velocity_magnitude < static_threshold: # static pedestrian
            # if too slow, then treated as being stationary
            vel_orientation_array.append((0.0, 0.0))
            vel_magnitude_array.append((0.0, 0.0))
        else:
            vel_orientation_array.append((vx / velocity_magnitude, vy / velocity_magnitude))
            vel_magnitude_array.append((0.0, velocity_magnitude)) # Add 0 to fool DBSCAN
    # grouping in current frame (three passes, each on different criteria)
    labels = [0] * num_people
    labels = DBScan_grouping(labels, vel_orientation_array, param_ori)
    labels = DBScan_grouping(labels, vel_magnitude_array, param_spd)
    labels = DBScan_grouping(labels, position_array, param_pos)
    return labels

def identify_edge_points(positions, robo_pose):
    # This function takes the position and velocity information of pointclouds
    # and their group label information. (At a specific time for a specific group)
    # Then it identifies the edge points for the group.

    left_idx, right_idx = find_left_right_edge(positions, robo_pose)
    dists = np.linalg.norm(positions[:, :2] - robo_pose[:2], axis=1)
    center_idx = np.argmin(dists)

    return left_idx, center_idx, right_idx

def find_left_right_edge(points, target_pt):
    # This function performs a scan of all the points and finds
    # the leftmost and rightmost visible points to the robot
    left_ang = -np.inf
    left_idx = None
    right_ang = np.inf
    right_idx = None
    num_pts = np.shape(points)[0]

    # correction for groups that cross pi (or the 180 degree line w.r.t. the coordinates origin)
    if points[0][0] < target_pt[0]:
        on_left_side = True
    else:
        on_left_side = False

    angs = np.arctan2(points[:, 1] - target_pt[1], points[:, 0] - target_pt[0])
    if on_left_side:
        angs[angs < 0] += 2 * np.pi
    left_idx = np.argmax(angs)
    right_idx = np.argmin(angs)

    return left_idx, right_idx


def vertices_from_edge_pts(robo_pos, edge_pos, edge_vel, increments=16, const=None, offset=1.0):
    # This function takes all the edge points, draws social spaces
    # And then extract the group representation vertices
    #
    # edge_pos and edge_vel must have multiples of 3 entries
    # For each group of entries, 0-left point, 1-cneter point, 2-right point

    # Expand and account for personal spaces
    robo_pos = np.array(robo_pos)
    vertices = []
    num_pts = np.shape(edge_pos)[0]
    if not ((num_pts % 3) == 0):
        raise Exception("num_pts not a multiplier of 3!")

    # left first, close second, right third
    if DETAIL_MODE:
        # In detail mode, each proxemic shape is an asymmetric 2D gaussian egg shape
        for i in range(num_pts):
            pos = edge_pos[i]
            vel = edge_vel[i]
            if const is None:
                candidate_vts = draw_social_shapes([pos], [vel], increments)
            else:
                candidate_vts = draw_social_shapes([pos], [vel], increments, const)
            if (i % 3) == 1:
                pt_choice = None
                dists = np.linalg.norm(candidate_vts[:, :2] - robo_pos[:2], axis=1)
                min_idx = np.argmin(dists)
                pt_choice = candidate_vts[min_idx]
            else:
                pt_choice = None
                left_idx, right_idx = find_left_right_edge(candidate_vts, robo_pos)
                if (i % 3) == 0:
                    pt_choice = candidate_vts[left_idx]
                elif (i % 3) == 2:
                    pt_choice = candidate_vts[right_idx]
                else:
                    raise Exception("i mod 3 cannot be 1 here!")
                pt_choice = np.array([pt_choice[0], pt_choice[1]])
            vertices.append(pt_choice)
    else:
        # In simple mode, each proxemic shape is a circle
        circle_radius = 0.5
        for i in range(num_pts):
            pos = edge_pos[i]
            dist = np.linalg.norm(pos - robo_pos)
            if (i % 3) == 1:
                # center point
                pt_choice = pos - (pos - robo_pos) * circle_radius / dist
            else:
                off_angle = np.arcsin(circle_radius / dist)
                tang_dist = dist * np.cos(off_angle)
                rel_angle = np.arctan2(pos[1] - robo_pos[1], pos[0] - robo_pos[0])
                if (i % 3) == 0:
                    # left point
                    pt_choice = robo_pos + np.array([tang_dist * np.cos(rel_angle - off_angle),
                                                     tang_dist * np.sin(rel_angle - off_angle)])
                elif (i % 3) == 2:
                    # right point
                    pt_choice = robo_pos + np.array([tang_dist * np.cos(rel_angle + off_angle),
                                                     tang_dist * np.sin(rel_angle + off_angle)])
                else:
                    raise Exception("i mod 3 cannot be 1 here!")
            vertices.append(pt_choice)

    # add offsets
    expanded_vertices = []
    num_pts = int(num_pts / 3)
    epsilon = 1e-5
    for i in range(num_pts):
        left_pt = vertices[i * 3]
        center_pt = vertices[i * 3 + 1]
        right_pt = vertices[i * 3 + 2]
        if ((np.linalg.norm(center_pt - left_pt) < epsilon) or
           (np.linalg.norm(center_pt - right_pt) < epsilon)):
            center_pt = (left_pt + right_pt) / 2
        left_offset_pt, right_offset_pt = construct_offset(left_pt, right_pt, robo_pos, offset)
        group_vertices = {}
        group_vertices['left'] = left_pt
        group_vertices['center'] = center_pt
        group_vertices['right'] = right_pt
        group_vertices['left_offset'] = left_offset_pt
        group_vertices['right_offset'] = right_offset_pt
        expanded_vertices.append(group_vertices)
    return expanded_vertices

def construct_offset(left_pt, right_pt, center_pt, offset=1.0):
    # This function constructs a rectangular offset block behind the edge
    # to accomdate possible occlusions
    ang1 = np.arctan2(left_pt[1] - right_pt[1], left_pt[0] - right_pt[0]) + np.pi/2
    ang2 = ang1 - np.pi
    offset1 = np.array([offset * np.cos(ang1), offset * np.sin(ang1)])
    offset2 = np.array([offset * np.cos(ang2), offset * np.sin(ang2)])
    left_pt_cd1 = left_pt + offset1
    left_pt_cd2 = left_pt + offset2
    right_pt_cd1 = right_pt + offset1
    right_pt_cd2 = right_pt + offset2

    dist1 = np.linalg.norm(center_pt - left_pt_cd1)
    dist2 = np.linalg.norm(center_pt - left_pt_cd2)
    if dist1 > dist2:
        offset_pt_l = left_pt_cd1
        offset_pt_r = right_pt_cd1
    else:
        offset_pt_l = left_pt_cd2
        offset_pt_r = right_pt_cd2
    return offset_pt_l, offset_pt_r

def boundary_dist(velocity, rel_ang, const=0.354163):
    # This function determines the proxemic distance based on
    # speed, angle, and a scaling factor

    # Parameters from Rachel Kirby's thesis
    front_coeff = 1.0
    side_coeff = 2.0 / 3.0
    rear_coeff = 0.5
    safety_dist = 0.5
    velocity_x = velocity[0]
    velocity_y = velocity[1]

    velocity_magnitude = np.sqrt(velocity_x ** 2 + velocity_y ** 2)
    variance_front = max(0.5, front_coeff * velocity_magnitude)
    variance_side = side_coeff * variance_front
    variance_rear = rear_coeff * variance_front

    rel_ang = rel_ang % (2 * np.pi)
    flag = int(np.floor(rel_ang / (np.pi / 2)))
    if flag == 0:
        prev_variance = variance_front
        next_variance = variance_side
    elif flag == 1:
        prev_variance = variance_rear
        next_variance = variance_side
    elif flag == 2:
        prev_variance = variance_rear
        next_variance = variance_side
    else:
        prev_variance = variance_front
        next_variance = variance_side

    dist = np.sqrt(const / ((np.cos(rel_ang) ** 2 / (2 * prev_variance)) + (np.sin(rel_ang) ** 2 / (2 * next_variance))))
    dist = max(safety_dist, dist)

    #laser = False
    #if laser:
    #    dist = dist - 0.5 + 1e-9

    return dist

def draw_social_shapes(position, velocity, increments=16, const=0.354163):
    # This function draws social group shapes
    # given the positions and velocities of the pedestrians.

    total_increments = increments # controls the resolution of the blobs
    quater_increments = total_increments / 4
    angle_increment = 2 * np.pi / total_increments

    # Draw a personal space for each pedestrian within the group
    contour_points = []
    for i in range(len(position)):
        center_x, center_y = position[i]
        velocity_x, velocity_y = velocity[i]
        velocity_angle = np.arctan2(velocity_y, velocity_x)

        # Draw four quater-ovals with the axis determined by front, side and rear "variances"
        # The overall shape contour does not have discontinuities.
        for j in range(total_increments):

            rel_ang = angle_increment * j
            value = boundary_dist(velocity[i], rel_ang, const)
            addition_angle = velocity_angle + rel_ang
            x = center_x + np.cos(addition_angle) * value
            y = center_y + np.sin(addition_angle) * value
            contour_points.append((x, y))

    # Get the convex hull of all the personal spaces
    contour_points = np.array(contour_points)
    #hull = ConvexHull(contour_points)
    #convex_hull_vertices = contour_points[hull.vertices]
    """
    convex_hull_vertices = []
    hull = ConvexHull(np.array(contour_points))
    for i in hull.vertices:
        hull_vertice = (contour_points[i][0], contour_points[i][1])
        convex_hull_vertices.append(hull_vertice)
    """

    return contour_points


