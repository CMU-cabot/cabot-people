#!/usr/bin/env python3

# Copyright (c) 2025  Carnegine Mellon University and IBM Corporation, and others
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import argparse
import csv
from pathlib import Path
import os
import yaml

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from geometry_msgs.msg import PointStamped
from people_msgs.msg import People
import rclpy
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from scipy.optimize import linear_sum_assignment
import tf2_geometry_msgs
import tf2_ros


def people_positions_to_numpy(people):
    positions = []
    for p in people.people:
        positions.append([p.position.x, p.position.y])
    return np.array(positions)

def pad_cost_matrix(cost_matrix, pad_value=1e+9):
    n_rows, n_cols = cost_matrix.shape
    if n_rows == n_cols:
        return cost_matrix
    size = max(n_rows, n_cols)
    padded = np.full((size, size), pad_value)
    padded[:n_rows, :n_cols] = cost_matrix
    return padded

def output_detail_csv(output_file, times, distances, mean_distances):
    max_num_people = 0
    for d in distances:
        if len(d) > max_num_people:
            max_num_people = len(d)

    with open(output_file, 'w', newline='', encoding='utf-8') as csvfile:
        csv_header = ['Time', 'Average Distance']
        csv_header += [f"Distance {i}" for i in range(max_num_people)]

        writer = csv.writer(csvfile)
        writer.writerow(csv_header)

        for time, distance, mean_distance in zip(times, distances, mean_distances):
            while len(distance) < max_num_people:
                distance.append("")

            csv_row = [time, mean_distance]
            csv_row += distance

            writer.writerow(csv_row)

def output_summary_csv(output_file, mean_distances):
    with open(output_file, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Number of Frame', 'Average Distance'])
        writer.writerow([len(mean_distances), np.mean(mean_distances)])

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', required=True, help='ROS bag directory')
    parser.add_argument('-o', '--output', required=True, help='output log directory')
    parser.add_argument('-a', '--anim', action='store_true', help='plot with animation')
    args = parser.parse_args()

    # prepare node and tf2 buffer
    rclpy.init()
    node = rclpy.create_node('plot_people_error')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    # open ROS bag
    input_path = Path(args.input)
    input_metadata = yaml.safe_load(open(input_path / 'metadata.yaml'))
    storage_options = rosbag2_py.StorageOptions(
        uri=str(input_path),
        storage_id=input_metadata['rosbag2_bagfile_information']['storage_identifier'])

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')

    compression = input_metadata['rosbag2_bagfile_information']['compression_mode'] != ''
    if compression:
        reader = rosbag2_py.SequentialCompressionReader()
    else:
        reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    # read ROS bag
    data = {
        '/people': [],
        '/test/people': []
    }
    while reader.has_next():
        (topic, msg_data, time) = reader.read_next()
        time_sec = time * 1e-9
        print(F'read topic {topic} at time {time_sec}')

        msg_type = get_message(type_map[topic])
        msg = deserialize_message(msg_data, msg_type)

        if topic == '/tf':
            for t in msg.transforms:
                tf_buffer.set_transform(t, 'default_authority')
        elif topic == '/tf_static':
            for t in msg.transforms:
                tf_buffer.set_transform_static(t, 'default_authority')
        elif topic == '/people' or topic == '/test/people':
            if len(msg.people) > 0:
                if msg.header.frame_id != 'map':
                    # if people topic is not in map frame, transform the position
                    for person in msg.people:
                        pos_stamped = PointStamped()
                        pos_stamped.header = msg.header
                        pos_stamped.point = person.position

                        vel_stamped = PointStamped()
                        vel_stamped.header = msg.header
                        vel_stamped.point = person.velocity

                        try:
                            transformed_pos_stamped = tf_buffer.transform(pos_stamped, 'map', timeout=Duration(seconds=1.0))
                            transformed_vel_stamped = tf_buffer.transform(vel_stamped, 'map', timeout=Duration(seconds=1.0))

                            person.position = transformed_pos_stamped.point
                            person.velocity = transformed_vel_stamped.point
                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                            print(F'Could not find tf to map')

                data[topic].append((time_sec, msg))

    # find common time that have both /people, /test/people
    times_people = [t for t, _ in data['/people']]
    times_test = [t for t, _ in data['/test/people']]
    times = sorted(set(times_people) & set(times_test))

    # calculate distances between /people, /test/people for all time steps
    people_points = [] # dimension : (time, number of people, 2)
    test_people_points = [] # dimension : (time, number of people, 2)
    distances = [] # dimension : (time, number of people)
    mean_distances = [] # dimension : (time)
    for t in times:
        msg_people = next(msg for time, msg in data['/people'] if time == t)
        msg_test_people = next(msg for time, msg in data['/test/people'] if time == t)

        np_positions = people_positions_to_numpy(msg_people)
        np_test_positions = people_positions_to_numpy(msg_test_people)

        cost = np.linalg.norm(np_positions[:, np.newaxis] - np_test_positions[np.newaxis, :], axis=2)
        padded_cost = pad_cost_matrix(cost)
        row_ind, col_ind = linear_sum_assignment(padded_cost)

        people_points.append([])
        test_people_points.append([])
        distances.append([])
        for i, j in zip(row_ind, col_ind):
            if i < np_positions.shape[0] and j < np_test_positions.shape[0]:
                people_points[-1].append(np_positions[i].tolist())
                test_people_points[-1].append(np_test_positions[j].tolist())
                distances[-1].append(cost[i, j])
        mean_distances.append(np.mean(distances[-1]))

    # output CSV
    if not os.path.exists(args.output):
        os.makedirs(args.output)
        print(f'Created output directory: {args.output}')
    else:
        print(f'Output directory already exists: {args.output}')
    output_detail_csv_file = os.path.join(args.output, "detail.csv")
    output_detail_csv(output_detail_csv_file, times, distances, mean_distances)

    output_summary_csv_file = os.path.join(args.output, "summary.csv")
    output_summary_csv(output_summary_csv_file, mean_distances)

    # plot results
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 10), gridspec_kw={'height_ratios': [2, 1]})

    ax1.set_title("All Corresponding Points")
    ax1.set_xlabel("X")
    ax1.set_ylabel("Y")
    ax1.axis('equal')
    ax1.grid(True)

    ax2.set_title("Average Distances Between Corresponding Points")
    ax2.set_xlabel("Time Step")
    ax2.set_ylabel("Distance")
    ax2.grid(True)

    if args.anim:
        plot_people_points = ax1.plot([], [], '-o')[0]
        plot_test_people_points = ax1.plot([], [], '-s')[0]
        plot_line = ax1.plot([], [], 'k--', linewidth=1)[0]
        plot_mean_distances = ax2.plot([], [], '-o', color='tab:red')[0]

        np_people_points = np.array(people_points)
        np_test_people_points = np.array(test_people_points)
        min_x = min((np.min(np_people_points[:, :, 0]), np.min(np_test_people_points[:, :, 0])))
        max_x = max((np.max(np_people_points[:, :, 0]), np.max(np_test_people_points[:, :, 0])))
        min_y = min((np.min(np_people_points[:, :, 1]), np.min(np_test_people_points[:, :, 1])))
        max_y = max((np.max(np_people_points[:, :, 1]), np.max(np_test_people_points[:, :, 1])))
        ax1.set_xlim(min_x - 1.0, max_x + 1.0)
        ax1.set_ylim(min_y - 1.0, max_y + 1.0)

        ax2.set_xlim(times[0], times[-1])
        ax2.set_ylim(0, max(mean_distances) + 1.0)

        def init():
            plot_people_points.set_data([], [])
            plot_test_people_points.set_data([], [])
            plot_line.set_data([], [])
            plot_mean_distances.set_data([], [])
            return plot_people_points, plot_test_people_points, plot_line, plot_mean_distances

        def update(frame):
            print(F'Update plot {frame}')

            # plot all points for /people, /test/people
            for p in range(len(people_points[frame])):
                plot_people_points.set_data(people_points[frame][p][0], people_points[frame][p][1])
                plot_test_people_points.set_data(test_people_points[frame][p][0], test_people_points[frame][p][1])
                plot_line.set_data([people_points[frame][p][0], test_people_points[frame][p][0]], [people_points[frame][p][1], test_people_points[frame][p][1]])

            # plot mean distances between /people, /test/people points
            plot_mean_distances.set_data(times[frame], mean_distances[frame])

            return plot_people_points, plot_test_people_points, plot_line, plot_mean_distances,

        ani = animation.FuncAnimation(fig, update, frames=len(times), init_func=init, blit=True, repeat=True, interval=500)

        output_gif_file = os.path.join(args.output, "plot.gif")
        ani.save(output_gif_file, writer="pillow")
    else:
        # plot all points for /people, /test/people
        for t in range(len(times)):
            for p in range(len(people_points[t])):
                ax1.plot(people_points[t][p][0], people_points[t][p][1], '-o')
                ax1.plot(test_people_points[t][p][0], test_people_points[t][p][1], '-s')
                ax1.plot([people_points[t][p][0], test_people_points[t][p][0]], [people_points[t][p][1], test_people_points[t][p][1]], 'k--', linewidth=1)

        # plot mean distances between /people, /test/people points
        ax2.plot(times, mean_distances, '-o', color='tab:red')

        output_png_file = os.path.join(args.output, "plot.png")
        plt.savefig(output_png_file)

if __name__ == '__main__':
    main()
