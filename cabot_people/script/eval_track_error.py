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
import math
import os
from pathlib import Path
import yaml

from geometry_msgs.msg import PointStamped, Vector3Stamped
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from scipy.optimize import linear_sum_assignment
from tf2_geometry_msgs import do_transform_point, do_transform_vector3
import tf2_ros


def people_positions_to_numpy(people):
    positions = []
    for p in people.people:
        positions.append([p.position.x, p.position.y])
    return np.array(positions)


def people_velocities_to_numpy(people):
    velocities = []
    for p in people.people:
        velocities.append([p.velocity.x, p.velocity.y])
    return np.array(velocities)


def output_detail_csv(output_file, times, distances, mean_distances, velocity_errors, mean_velocity_errors):
    assert len(distances) == len(velocity_errors), 'distances, velocity_errors should have same time length'

    max_num_people = 0
    for d, v in zip(distances, velocity_errors):
        assert len(d) == len(v), 'distances, velocity_errors should have same number of people'
        if len(d) > max_num_people:
            max_num_people = len(d)

    with open(output_file, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)

        csv_header = ['Time', 'Mean Distance for All People', 'Mean Velocity Error for All People']
        csv_header += [f'Distance for Person {i}' for i in range(max_num_people)]
        csv_header += [f'Velocity Error for Person {i}' for i in range(max_num_people)]
        writer.writerow(csv_header)

        for time, distance, mean_distance, velocity_error, mean_velocity_error in zip(times, distances, mean_distances, velocity_errors, mean_velocity_errors):
            while len(distance) < max_num_people:
                distance.append('')
            while len(velocity_error) < max_num_people:
                velocity_error.append('')

            csv_row = [time, mean_distance, mean_velocity_error]
            csv_row += distance
            csv_row += velocity_error
            writer.writerow(csv_row)


def output_summary_csv(output_file, pos_data, vel_data):
    assert len(pos_data) == len(vel_data), 'position, velocity should have same time length'

    if len(pos_data) > 0:
        median_pos_data = np.median(pos_data)
        mean_pos_data = np.mean(pos_data)
        std_pos_data = np.std(pos_data)
        min_pos_data = np.min(pos_data)
        max_pos_data = np.max(pos_data)
    else:
        median_pos_data = mean_pos_data = std_pos_data = min_pos_data = max_pos_data = None

    if len(vel_data) > 0:
        median_vel_data = np.median(vel_data)
        mean_vel_data = np.mean(vel_data)
        std_vel_data = np.std(vel_data)
        min_vel_data = np.min(vel_data)
        max_vel_data = np.max(vel_data)
    else:
        median_vel_data = mean_vel_data = std_vel_data = min_vel_data = max_vel_data = None

    with open(output_file, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Number of Frame', 'Pos. Median', 'Pos. Mean', 'Pos. Std', 'Pos. Min', 'Pos. Max',
                        'Vel. Median', 'Vel. Mean', 'Vel. Std', 'Vel. Min', 'Vel. Max'])
        writer.writerow([len(pos_data), median_pos_data, mean_pos_data, std_pos_data, min_pos_data, max_pos_data,
                        median_vel_data, mean_vel_data, std_vel_data, min_vel_data, max_vel_data])


def eval_track_error(input_file, output_dir, anim, tf_buffer, eval_topic):
    input_path = Path(input_file)

    # open ROS bag
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

    # read ROS bag (check /people, /test/people for people tracking, /obstacles, /test/obstacles for obstacle tracking)
    groundtruth_topic = '/' + eval_topic
    estimate_topic = '/test/' + eval_topic

    data = {
        groundtruth_topic: [],
        estimate_topic: []
    }
    while reader.has_next():
        (topic, msg_data, time) = reader.read_next()

        msg_type = get_message(type_map[topic])
        msg = deserialize_message(msg_data, msg_type)
        time_sec = time * 1e-9

        if topic == '/tf':
            for t in msg.transforms:
                tf_buffer.set_transform(t, 'default_authority')
        elif topic == '/tf_static':
            for t in msg.transforms:
                tf_buffer.set_transform_static(t, 'default_authority')
        elif topic == groundtruth_topic or topic == estimate_topic:
            if len(msg.people) > 0:
                if msg.header.frame_id != 'map':
                    # if people topic is not in map frame, transform the position
                    for person in msg.people:
                        pos_stamped = PointStamped()
                        pos_stamped.header = msg.header
                        pos_stamped.point = person.position

                        vel_stamped = Vector3Stamped()
                        vel_stamped.header = msg.header
                        vel_stamped.vector.x = person.velocity.x
                        vel_stamped.vector.y = person.velocity.y
                        vel_stamped.vector.z = person.velocity.z

                        try:
                            transform = tf_buffer.lookup_transform('map', msg.header.frame_id, msg.header.stamp)
                            transformed_pos_stamped = do_transform_point(pos_stamped, transform)
                            transformed_vel_stamped = do_transform_vector3(vel_stamped, transform)

                            person.position = transformed_pos_stamped.point
                            person.velocity.x = transformed_vel_stamped.vector.x
                            person.velocity.y = transformed_vel_stamped.vector.y
                            person.velocity.z = transformed_vel_stamped.vector.z
                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                            print('Could not find tf to map')
                            continue

                data[topic].append((time_sec, msg))

    # calculate distances between corresponding points for all time steps
    times = []  # dimension : (time)
    people_points = []  # dimension : (time, number of people/obstacles, 2)
    test_people_points = []  # dimension : (time, number of people/obstacles, 2)
    distances = []  # dimension : (time, number of people/obstacles)
    mean_distances = []  # dimension : (time)
    velocity_errors = []  # dimension : (time, number of people/obstacles)
    mean_velocity_errors = []  # dimension : (time)
    for time_test_people, msg_test_people in data[estimate_topic]:
        if len(data[groundtruth_topic]) == 0:
            continue

        # find msg which have the closest time
        time_people, msg_people = min(data[groundtruth_topic], key=lambda x: abs(x[0] - time_test_people))

        np_positions = people_positions_to_numpy(msg_people)
        np_test_positions = people_positions_to_numpy(msg_test_people)

        cost = np.linalg.norm(np_positions[:, np.newaxis] - np_test_positions[np.newaxis, :], axis=2)
        # if cost matrix is not square matrix, fill with large values to find matching by the linear sum assignment
        num_rows, num_cols = cost.shape
        if num_rows < num_cols:
            # fill rows by large values
            cost = np.vstack([cost, np.full((num_cols - num_rows, num_cols), 1e+9)])
        elif num_cols < num_rows:
            # fill columns by large values
            cost = np.hstack([cost, np.full((num_rows, num_rows - num_cols), 1e+9)])
        row_ind, col_ind = linear_sum_assignment(cost)

        np_velocities = people_velocities_to_numpy(msg_people)
        np_test_velocities = people_velocities_to_numpy(msg_test_people)

        times.append(time_test_people)
        people_points.append([])
        test_people_points.append([])
        distances.append([])
        velocity_errors.append([])
        for i, j in zip(row_ind, col_ind):
            if i < np_positions.shape[0] and j < np_test_positions.shape[0]:
                people_points[-1].append(np_positions[i].tolist())
                test_people_points[-1].append(np_test_positions[j].tolist())
                distances[-1].append(cost[i, j])
                velocity_errors[-1].append(np.linalg.norm(np_velocities[i] - np_test_velocities[j]))
        mean_distances.append(np.mean(distances[-1]))
        mean_velocity_errors.append(np.mean(velocity_errors[-1]))

    # output CSV
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f'Created output directory: {output_dir}')
    else:
        print(f'Output directory already exists: {output_dir}')
    output_detail_csv_file = os.path.join(output_dir, eval_topic + '-detail.csv')
    output_detail_csv(output_detail_csv_file, times, distances, mean_distances, velocity_errors, mean_velocity_errors)

    output_summary_csv_file = os.path.join(output_dir, eval_topic + '-summary.csv')
    output_summary_csv(output_summary_csv_file, mean_distances, mean_velocity_errors)

    # plot results
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 10), gridspec_kw={'height_ratios': [2, 1, 1]})

    ax1.set_title('All Corresponding Points')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.grid(True)

    ax2.set_title('Average Distances Between Corresponding Points')
    ax2.set_xlabel('Time Step')
    ax2.set_ylabel('Distance')
    ax2.grid(True)

    ax3.set_title('Average Velocity Errors Between Corresponding Points')
    ax3.set_xlabel('Time Step')
    ax3.set_ylabel('Velocity Errors')
    ax3.grid(True)

    if anim and len(times) > 0:
        plot_people_points = ax1.plot([], [], '-o')[0]
        plot_test_people_points = ax1.plot([], [], '-s')[0]
        plot_line = ax1.plot([], [], 'k--', linewidth=1)[0]
        plot_mean_distances = ax2.plot([], [], '-o', color='tab:red')[0]
        plot_mean_velocity_errors = ax3.plot([], [], '-o', color='tab:red')[0]

        min_x = math.inf
        max_x = -math.inf
        min_y = math.inf
        max_y = -math.inf
        for t in range(len(times)):
            for p in range(len(people_points[t])):
                if people_points[t][p][0] < min_x:
                    min_x = people_points[t][p][0]
                if test_people_points[t][p][0] < min_x:
                    min_x = test_people_points[t][p][0]

                if people_points[t][p][0] > max_x:
                    max_x = people_points[t][p][0]
                if test_people_points[t][p][0] > max_x:
                    max_x = test_people_points[t][p][0]

                if people_points[t][p][1] < min_y:
                    min_y = people_points[t][p][1]
                if test_people_points[t][p][1] < min_y:
                    min_y = test_people_points[t][p][1]

                if people_points[t][p][1] > max_y:
                    max_y = people_points[t][p][1]
                if test_people_points[t][p][1] > max_y:
                    max_y = test_people_points[t][p][1]

        ax1.set_xlim(min_x - 0.1, max_x + 0.1)
        ax1.set_ylim(min_y - 0.1, max_y + 0.1)

        ax2.set_xlim(times[0], times[-1])
        ax2.set_ylim(0, max(mean_distances) + 0.1)

        ax3.set_xlim(times[0], times[-1])
        ax3.set_ylim(0, max(mean_velocity_errors) + 0.1)

        def init():
            plot_people_points.set_data([], [])
            plot_test_people_points.set_data([], [])
            plot_line.set_data([], [])
            plot_mean_distances.set_data([], [])
            plot_mean_velocity_errors.set_data([], [])
            return plot_people_points, plot_test_people_points, plot_line, plot_mean_distances, plot_mean_velocity_errors

        def update(frame):
            print(F'Update plot {frame}')

            # plot all points for /people, /test/people
            for p in range(len(people_points[frame])):
                plot_people_points.set_data(people_points[frame][p][0], people_points[frame][p][1])
                plot_test_people_points.set_data(test_people_points[frame][p][0], test_people_points[frame][p][1])
                plot_line.set_data([people_points[frame][p][0], test_people_points[frame][p][0]], [people_points[frame][p][1], test_people_points[frame][p][1]])

            # plot mean distances between /people, /test/people points
            plot_mean_distances.set_data(times[frame], mean_distances[frame])

            # plot mean velocity errors between /people, /test/people points
            plot_mean_velocity_errors.set_data(times[frame], mean_velocity_errors[frame])

            return plot_people_points, plot_test_people_points, plot_line, plot_mean_distances, plot_mean_velocity_errors

        func_anim = animation.FuncAnimation(fig, update, frames=len(times), init_func=init, blit=True, repeat=True, interval=500)
        fig.tight_layout()

        output_gif_file = os.path.join(output_dir, eval_topic + '-plot.gif')
        func_anim.save(output_gif_file, writer='pillow')
    else:
        # plot all points for /people, /test/people
        for t in range(len(times)):
            for p in range(len(people_points[t])):
                ax1.plot(people_points[t][p][0], people_points[t][p][1], '-o')
                ax1.plot(test_people_points[t][p][0], test_people_points[t][p][1], '-s')
                ax1.plot([people_points[t][p][0], test_people_points[t][p][0]], [people_points[t][p][1], test_people_points[t][p][1]], 'k--', linewidth=1)

        # plot mean distances between /people, /test/people points
        ax2.plot(times, mean_distances, '-o', color='tab:red')

        # plot mean velocity errors between /people, /test/people points
        ax3.plot(times, mean_velocity_errors, '-o', color='tab:red')

        plt.tight_layout()

        output_png_file = os.path.join(output_dir, eval_topic + '-plot.png')
        plt.savefig(output_png_file)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', required=True, help='ROS bag directory')
    parser.add_argument('-o', '--output', required=True, help='output log directory')
    parser.add_argument('-a', '--anim', action='store_true', help='plot with animation')
    args = parser.parse_args()

    # prepare node and tf2 buffer
    rclpy.init()
    node = rclpy.create_node('eval_track_error')

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)

    eval_track_error(args.input, args.output, args.anim, tf_buffer, 'people')
    eval_track_error(args.input, args.output, args.anim, tf_buffer, 'obstacles')


if __name__ == '__main__':
    main()
