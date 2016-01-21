#!/usr/bin/python
'''
This script is an example of an issue-specific invocation of dronekit-la

So you write a new analyzer and you want to find which of the files you have show that issue.  A short piece of shell scripting to iterate over the logs will find you the problem logs quickly.

It is also possible to use this as part of a "git bisect", assuming the problem's presence can be determined from a log.
'''

from __future__ import print_function
import os
import json
import difflib
import subprocess
import string
import argparse
import sys
import re

parser = argparse.ArgumentParser()
parser.add_argument('--verbose',
                    help='print more messages',
                    dest='verbose',
                    action="store_true")
parser.add_argument('filepath',
                    help='log to check')
args = parser.parse_args()

def spew_string_to_file(some_string, output_filepath):
    handle = open(output_filepath,"w+")
    handle.write(some_string)
    handle.close()

def diff_json(expected, new):
    expected_string = json.dumps(expected, indent=2, sort_keys=True)
    new_string = json.dumps(new, indent=2, sort_keys=True)
    mydiff = difflib.unified_diff(expected_string.splitlines(1), new_string.splitlines(1))
    return "".join(mydiff)

def json_from_filepath(filepath):
    contents = open(filepath).read()
    return json.loads(contents)

def test_log(filepath_log):
    if args.verbose:
        print("Testing log %s" % filepath_log)

    test_success = True
    try:
        command = ["./dronekit-la",
                   '-m', 'copter', # assume a quadcopter for the time being
                   '-f', 'QUAD',
                   '-a', 'sAcc Issue',
                   filepath_log]
        if args.verbose:
            print("Command: %s" % command)

        p = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        analysis_string,analysis_stderr = p.communicate()
        try:
            analysis_json = json.loads(analysis_string)
        except Exception as e:
            print("Failed to load (%s)" % (analysis_string))
            print ("STDERR was: (%s)" % analysis_stderr)
            raise e

        if analysis_json['evilness']:
            return False; # Bad
        return True

    except subprocess.CalledProcessError as E:
        test_success = False
        print("""
FAIL: %s
subprocess exception (%s)
""" % (filepath_log, str(E),))

    return test_success

success = True
if not test_log(args.filepath):
    print("sAcc problem is present")
    success = False

if not success:
    sys.exit(1)

sys.exit(0)
