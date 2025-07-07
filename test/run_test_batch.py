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
import os
import signal
import subprocess
import sys
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', required=True, help='input yaml file')
    parser.add_argument('-N', '--navigation', required=True, help='cabot-navigation directory path')
    parser.add_argument('-d', '--dev', action='store_true', help='development')
    parser.add_argument('-v', '--verbose', action='store_true', help='verbose option')
    args = parser.parse_args()

    with open(args.input, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    test_script = os.path.join(script_dir, 'run_test.sh')
    for test in data['tests']:
        print('Execute test, cabot_site=' + test['site'] + ', name=' + test['name'])
        test_args = [test_script, '-N', args.navigation, '-S', test['site'], '-f', test['name'], '-n', test['name']]
        if args.dev:
            test_args.append('-d')
        if args.verbose:
            test_args.append('-v')
        proc = subprocess.Popen(test_args, preexec_fn=os.setsid)
        try:
            proc.wait()
        except KeyboardInterrupt:
            os.killpg(proc.pid, signal.SIGINT)

if __name__ == '__main__':
    main()