import sys
import numpy as np
from sklearn.cluster import DBSCAN
    
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

def pedestrian_grouping(position_array, velocity_array, param_pos, param_spd, param_ori):
    # Performs grouping given a position array and a cooresponding velocity array
    # The grouping is performed based on three levels of parameters
    # First orientation, then speed, finally position

    if len(position_array) != len(velocity_array):
        print("[grouping] Error: pos array and vel array sohuld be the same size!")
        sys.exit(0)

    num_people = len(position_array)
    vel_orientation_array = []
    vel_magnitude_array = []
    for [vx, vy] in velocity_array:
        velocity_magnitude = np.sqrt(vx ** 2 + vy ** 2)
        if velocity_magnitude < params['velocity_ignore_threshold']:
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
