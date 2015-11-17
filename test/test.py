#!/usr/bin/python

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

def diff_valgrind(expected, new):
    expected_string = open(expected).read()
    new_string = open(new).read()

    strip_pids = re.compile("^==\d+==", re.MULTILINE)
    expected_string = strip_pids.sub("==PID==", expected_string)
    new_string = strip_pids.sub("==PID==", new_string)

    strip_parent_pid = re.compile("Parent PID: \d+")
    expected_string = strip_parent_pid.sub("PPID", expected_string);
    new_string = strip_parent_pid.sub("PPID", new_string);

    strip_heap_usage = re.compile("in use at exit: .*");
    expected_string = strip_heap_usage.sub("in use at exit: Yes!", expected_string);
    new_string = strip_heap_usage.sub("in use at exit: Yes!", new_string);

    strip_heap_usage2 = re.compile("total heap usage: .*");
    expected_string = strip_heap_usage2.sub("total heap usage: Yes!", expected_string);
    new_string = strip_heap_usage2.sub("total heap usage: Yes!", new_string);


    mydiff = difflib.unified_diff(expected_string.splitlines(1), new_string.splitlines(1))
    return "\n".join(mydiff)

def json_from_filepath(filepath):
    contents = open(filepath).read()
    return json.loads(contents)

def handle_output_diff(mydiff, correctish_json_filepath, new_json_filepath):
    (correctish_json_dirpath,correctish_json_filename) = os.path.split(correctish_json_filepath)
    accept_command = "cp '%s' '%s'; pushd '%s'; git add '%s'; popd" % (new_json_filepath, correctish_json_filepath, correctish_json_dirpath, correctish_json_filename)
    print("""
---------output diff-----------------
%s
---------end output diff-------------
Accept new result: %s
""" % (mydiff, accept_command))
    if args.accept_all:
        print("Accepting automatically")
        check_me = subprocess.check_output(accept_command, shell=True, executable='/bin/bash');


def handle_valgrind_diff(valdiff, correctish_filepath, new_filepath):
    (correctish_dirpath,correctish_filename) = os.path.split(correctish_filepath)
    accept_command = "cp '%s' '%s'; pushd '%s'; git add '%s'; popd" % (new_filepath, correctish_filepath, correctish_dirpath, correctish_filename)
    print("""
---------valgrind diff-----------------
%s
---------end valgrind diff-------------
Accept new result: %s
""" % (valdiff, accept_command))
    if args.accept_all:
        print("Accepting automatically")
        check_me = subprocess.check_output(accept_command, shell=True, executable='/bin/bash');
            
def test_log(filepath_log):
    test_success = True
    try:
        command = ["./dronekit-la",
                   '-m', 'copter', # assume a quadcopter for the time being
                   '-f', 'QUAD',
                   filepath_log]
        if args.valgrind:
            correctish_valgrind_logpath = filepath_log + "-memcheck"
            new_valgrind_logpath = "/tmp/" + string.replace(correctish_valgrind_logpath,"/","-") + "-new-valgrind"
            command[:0] = ['valgrind',
#                           '--track-fds=yes',
                           '--leak-check=no',
                           '--read-inline-info=yes',
                           '--read-var-info=yes',
                           '--track-origins=yes',
                           '--log-file=%s' % (new_valgrind_logpath,)]
        p = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        analysis_string,analysis_stderr = p.communicate()
        analysis_json = json.loads(analysis_string)
        filter_analysis_json(analysis_json, 0) # modifies in place
        correctish_json_filepath = filepath_log + "-expected-json"
        correctish_json = json_from_filepath(correctish_json_filepath)

        valdiff = ""
        if args.valgrind:
            valdiff = diff_valgrind(correctish_valgrind_logpath, new_valgrind_logpath)

        new_json_filepath = "/tmp/" + string.replace(filepath_log,"/","-") + "-new-json"
        spew_string_to_file(json.dumps(analysis_json, indent=2, sort_keys=True), new_json_filepath)

        mydiff = diff_json(correctish_json, analysis_json)

        if len(mydiff) or len(analysis_stderr) or len(valdiff):
            test_success = False
            print("FAIL: %s" % (filepath_log,))
        else:
            print("PASS: %s" % (filepath_log,))

        if len(mydiff):
            handle_output_diff(mydiff, correctish_json_filepath, new_json_filepath)

        if len(analysis_stderr):
            print("""
---------stderr-----------------
%s
---------stderr-----------------
""" % (analysis_stderr,))

        if len(valdiff):
            handle_valgrind_diff(valdiff, correctish_valgrind_logpath, new_valgrind_logpath)

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
