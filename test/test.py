#!/usr/bin/python

from __future__ import print_function
import os
import json
import difflib
import subprocess
import string

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
    try:
        analysis_string = subprocess.check_output(["./loganalyzer", filepath_log]);
        analysis_json = json.loads(analysis_string)
        filter_analysis_json(analysis_json, 0) # modifies in place
        correctish_json_filepath = filepath_log + "-expected-json"
        correctish_json = json_from_filepath(correctish_json_filepath)
        
        new_json_filepath = "/tmp/" + string.replace(filepath_log,"/","-") + "-new-json"
        spew_string_to_file(json.dumps(analysis_json, indent=2, sort_keys=True), new_json_filepath)
        mydiff = diff_json(correctish_json, analysis_json)
        if len(mydiff):
            (correctish_json_dirpath,correctish_json_filename) = os.path.split(correctish_json_filepath)
            print("""
FAIL: %s
---------diff-----------------
%s
---------diff-----------------
Accept new result: cp '%s' '%s'; pushd '%s'; git add '%s'; popd
""" % (filepath_log, mydiff, new_json_filepath, correctish_json_filepath, correctish_json_dirpath, correctish_json_filename))
        else:
            print("PASS: %s" % (filepath_log,))

    except subprocess.CalledProcessError as E:
        print("""
FAIL: %s 
subprocess exception (%s)
""" % (filepath_log, str(E),))


log_dirpath = os.getenv("LOGANALYZE_DIRPATH_LOG", "test/logs");

# print("Log dirpath: " + log_dirpath);
for dirname, dirnames, filenames in os.walk(log_dirpath):
    for filename in filenames:
        if filename.endswith(".tlog") or filename.endswith(".BIN"):
            filepath = os.path.join(dirname, filename)
            test_log(filepath)
        
