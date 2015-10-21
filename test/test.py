#!/usr/bin/python

from __future__ import print_function
import os
import json
import difflib
import subprocess
import string
import argparse
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--accept-all',
                    help='accept all new results',
                    dest='accept_all',
                    action="store_true")
parser.add_argument('--valgrind',
                    help='run analyzer through valgrind',
                    dest='valgrind',
                    action="store_true")
args = parser.parse_args()

def filter_analysis_json(json_stuff, depth):
    if isinstance(json_stuff, dict):
        for key in json_stuff.keys():
            if depth == 0:
                if key == "timestamp":
                    json_stuff[key] = "123456789"
                if key == "duration":
                    json_stuff[key] = "987654321"
                if key == "git_version":
                    json_stuff[key] = "some-hash"
        if (isinstance(json_stuff[key], list) or
            isinstance(json_stuff[key], dict)):
            filter_analysis_json(json_stuff[key], depth+1)
        return;
    if isinstance(json_stuff, list):
        print("list");
        return;

    print("unknown (%s)" % json_stuff)
    

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
    test_success = True
    try:
        command = ["./dronekit-la", filepath_log]
        if args.valgrind:
            correctish_valgrind_logpath = filepath_log + "-memcheck"
            command[:0] = ['valgrind',
                           '--track-fds=yes',
                           '--read-inline-info=yes',
                           '--read-var-info=yes',
                           '--track-origins=yes',
                           '--log-file=%s' % (correctish_valgrind_logpath,)]
        analysis_string = subprocess.check_output(command);
        analysis_json = json.loads(analysis_string)
        filter_analysis_json(analysis_json, 0) # modifies in place
        correctish_json_filepath = filepath_log + "-expected-json"
        correctish_json = json_from_filepath(correctish_json_filepath)

        new_json_filepath = "/tmp/" + string.replace(filepath_log,"/","-") + "-new-json"
        spew_string_to_file(json.dumps(analysis_json, indent=2, sort_keys=True), new_json_filepath)
        mydiff = diff_json(correctish_json, analysis_json)
        if len(mydiff):
            test_success = False
            (correctish_json_dirpath,correctish_json_filename) = os.path.split(correctish_json_filepath)
            accept_command = "cp '%s' '%s'; pushd '%s'; git add '%s'; popd" % (new_json_filepath, correctish_json_filepath, correctish_json_dirpath, correctish_json_filename)
            print("""
FAIL: %s
---------diff-----------------
%s
---------diff-----------------
Accept new result: %s
""" % (filepath_log, mydiff, accept_command))
            if args.accept_all:
                print("Accepting automatically")
                check_me = subprocess.check_output(accept_command, shell=True, executable='/bin/bash');

        else:
            print("PASS: %s" % (filepath_log,))

    except subprocess.CalledProcessError as E:
        test_success = False
        print("""
FAIL: %s 
subprocess exception (%s)
""" % (filepath_log, str(E),))

    return test_success

log_dirpath = os.getenv("LOGANALYZE_DIRPATH_LOG", "test/logs");

# print("Log dirpath: " + log_dirpath);
success = True
for dirname, dirnames, filenames in os.walk(log_dirpath):
    for filename in filenames:
        if filename.endswith(".tlog") or filename.endswith(".BIN"):
            filepath = os.path.join(dirname, filename)
            if not test_log(filepath):
                success = False

if not success:
    sys.exit(1)

sys.exit(0)
