"""
dronekit-la-doc-validation.py

This script generates a summary file that can be used to compare for changes between builds. The summary
includes information about the version of dronekit-la used, the available analysers and the description and 
reason fields for the different tests. It does not go down to lower level detail because we're primarily aiming
to catch high level changes.

The script needs to be run against a big set of logs. By default, the logs used are the private 3DR logs
in https://github.com/dronekit/dronekit-la-testdata-3dr (these cannot be shared publically for legal reasons).


Issues:

- Currently this forces QUAD and copter when generating the output.
"""


import os
import sys
#rootdir = sys.argv[1]

from subprocess import check_output #to call dronekit-la

import json

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Get summary log for dronekit-la, including summary test descriptions/reasons.')
parser.add_argument('--target_directory', help="Path to test log directory")
parser.add_argument('--output_file', default='analysis_summary.txt',
                   help="Path to test log directory")
args = parser.parse_args()

print "target directory: %s" % args.target_directory


def getfiles(root):
    #print "DEBUG: getfiles: %s" % root
    relevant_files=set()
    for root, directories, filenames in os.walk(root):
        for filename in filenames:
            #print "DEBUG: filename: %s" % filename
            file_extension=filename.split('.')[-1].lower()
            if file_extension=='tlog' or file_extension=='bin' or file_extension=='log':
                filename = os.path.join(root,filename)
                relevant_files.add(filename)
    return relevant_files
    

def process_files(filelist):
    """
    Generate dict containing description/reason information for tests
    """
    tests_summary=dict()
    for filename in filelist:
        print "Processing: %s" % filename
        analysed_file=check_output(["dronekit-la", "-f QUAD", "-mcopter" , filename])
        #print analysed_file
        data = json.loads(analysed_file)
        for name_test, testcontents in data['tests'].iteritems():
            #print name_test
            #print testcontents
            if not name_test in tests_summary:
                tests_summary[name_test]=dict()
                tests_summary[name_test]['description']=testcontents['description']
                for result in testcontents['results']:
                    #print result['status']
                    if not result['status'] in tests_summary[name_test]:
                        #print "made it"
                        tests_summary[name_test][result['status']]=set() #set of warning reasons
                    if 'reason' in result:
                        tests_summary[name_test][result['status']].add(result['reason'])
                    else:
                        tests_summary[name_test][result['status']].add("ISSUE: NO REASON GIVEN")
                    #print 'RESULT: %s' %result

    return tests_summary

def generate_logs_summary(tests_summary):

    output_string=''
    for test_name, contents in tests_summary.iteritems():
        output_string+= "\n\nTEST: %s"  % test_name
        output_string+= "\nDescription: %s"  % contents['description']

        if 'FAIL' in contents:
	    for description in contents['FAIL']:
	        output_string+= "\n* Fail: %s" % description

        if 'WARN' in contents:
	    for description in contents['WARN']:
	        output_string+= "\n* Warn: %s" % description
	    
        if 'PASS' in contents:
	    for description in contents['PASS']:
	        output_string+= "\n* Pass: %s" % description

    return output_string


def generate_dk_la_summary():
    """
    Generates DroneKit-LA version and analyser information for log.
    """



    dk_la_version=check_output(["dronekit-la", "-V"])
    output_string='DK-LA Version: %s' % dk_la_version

    dk_la_version=check_output(["dronekit-la", "-k"])
    output_string+='\nDK-LA HELP:\n\n %s' % dk_la_version

    dk_la_list_analysers=check_output(["dronekit-la", "-l"])
    output_string+='\nAnalyser List: \n\n%s' % dk_la_list_analysers
    return output_string


#Generate information about the tool
toolstring=generate_dk_la_summary()

#Get list of log files in target_directory
filelist=getfiles(args.target_directory)
#Generate dict of tests/unique reason codes.
summarydict= process_files(filelist)
#Pretty print out the summary dict to a string
logstring=generate_logs_summary(summarydict)

outputlogstring=toolstring + logstring
print outputlogstring

with open(args.output_file,'w') as file_out:
   file_out.write(outputlogstring) 





                

