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
import glob
import os
import shutil
import signal
import subprocess
import sys
import yaml

import numpy as np


def read_csv(input_file):
    data = []
    with open(input_file, newline='') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or all(cell.strip() == '' for cell in row):
                continue
            data.append([cell.strip() for cell in row])
    return data

def output_summary_csv(output_file, pos_data, vel_data):
    assert len(pos_data)==len(vel_data), 'position, velocity should have same time length'

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
        writer.writerow(['Number of Frame', 'Pos. Median', 'Pos. Mean', 'Pos. Std', 'Pos. Min', 'Pos. Max', \
            'Vel. Median', 'Vel. Mean', 'Vel. Std', 'Vel. Min', 'Vel. Max'])
        writer.writerow([len(pos_data), median_pos_data, mean_pos_data, std_pos_data, min_pos_data, max_pos_data, \
            median_vel_data, mean_vel_data, std_vel_data, min_vel_data, max_vel_data])

def output_summary_log(output_dir, eval_topic):
    # create summary of all tests
    pos_data = []
    vel_data = []
    csv_files = glob.glob(os.path.join(output_dir, '*', 'eval_track_error', eval_topic + '-detail.csv'))
    for csv_file in csv_files:
        print(f'Read test result CSV file: {csv_file}')
        data = read_csv(csv_file)

        # skip header, and collect mean distance and velocity error for all corresponding points in each frame
        pos_data += [float(row[1]) for row in data[1:]]
        vel_data += [float(row[2]) for row in data[1:]]

    output_file = os.path.join(output_dir, eval_topic + '-summary.csv')
    print(f'Write all test result CSV file: {output_file}')
    output_summary_csv(output_file, pos_data, vel_data)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', required=True, help='input yaml file')
    parser.add_argument('-N', '--navigation', required=True, help='cabot-navigation directory path')
    parser.add_argument('-O', '--output', required=True, help='output directory path')
    parser.add_argument('-a', '--anim', action='store_true', help='plot with animation')
    parser.add_argument('-d', '--dev', action='store_true', help='development')
    parser.add_argument('-v', '--verbose', action='store_true', help='verbose option')
    args = parser.parse_args()

    # prepare output directory
    if os.path.isabs(args.output):
        output_dir = args.output
    else:
        output_dir = os.path.abspath(args.output)

    if os.path.exists(output_dir):
        print(f'Output directory already exists: {output_dir}')
        sys.exit(1)
    os.makedirs(output_dir)

    # load input test list
    with open(args.input, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    # copy input test list to output directory
    shutil.copy(args.input, os.path.join(output_dir, os.path.basename(args.input)))

    # run all tests
    script_dir = os.path.dirname(os.path.abspath(__file__))
    test_script = os.path.join(script_dir, 'run_test.sh')

    for test in data['tests']:
        module = 'tests'
        if 'module' in test:
            module = test['module']
        print('Execute test, cabot_site=' + test['site'] + ', module=' + module + ', func=' + test['func'])
        test_args = [test_script, '-N', args.navigation, '-S', test['site'], '-T', module, '-f', test['func'], '-O', output_dir, '-n', module + '_' + test['func']]
        if args.anim:
            test_args.append('-a')
        if args.dev:
            test_args.append('-d')
        if args.verbose:
            test_args.append('-v')
        proc = subprocess.Popen(test_args, preexec_fn=os.setsid)
        try:
            proc.wait()
        except KeyboardInterrupt:
            print(f'Stop test')
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except Exception as e:
                print(f'Failed to stop test: {e}')
            finally:
                sys.exit(1)

    output_summary_log(output_dir, 'people')
    output_summary_log(output_dir, 'obstacles')

if __name__ == '__main__':
    main()
