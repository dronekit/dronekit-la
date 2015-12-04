.. _getting_started_top:

===============
Getting Started
===============

This topic explains how to download and install DroneKit-LA and how to perform some basic analysis operations.

.. note:: 

    This first release can only be run on Linux! These instructions have been tested on Ubuntu 14.04.3 LTS
    and Ubuntu 15.10 running in a virtual machine.


Installing 
==========

To install DroneKit-LA on your Linux computer:

#. Download :download:`dronekit-la_0.3_amd64.deb <../downloads/dronekit-la_0.3_amd64.deb>`.
#. Open the file (double-click in UI) to launch it in the *Ubuntu Software Center*. 
   Then select the **Install** button.
   
   .. tip:: 
   
       You can also install on the terminal using the command:
       
       .. code-block:: python
       
           sudo dpkg -i dronekit-la_0.3_amd64.deb


Running the tool
================

To analyze a log file simply specify it after the command:

.. code-block:: bash

    dronekit-la log_file.log

    
DroneKit-LA will identify the log's type from its file extension 
(telemetry logs - **.tlog**, dataflash binary logs **.BIN**, and dataflash text dumps - **.log**),
run *all* tests, and output the analysis in JSON format. If you wish to store the result you can just 
redirect it to a log file:

.. code-block:: bash

    dronekit-la log_file.log >result.txt
    

The tool also allows you to specify multiple files, specific analyzers to use, the output format, the model/frame
and several other features. To get help while you're using the tool, just enter ``dronekit-la -h``!


.. _guide_model_and_frame_type:

Specifying frame and model type
-------------------------------

DroneKit-LA will report a :ref:`Vehicle Definition <analyzer_listing_vehicle_definition>` 
test fail if information about the vehicle frame and model is missing from the log 
(without this information log analysis is much less effective).

.. tip::

    The quickest way to determine if the information is missing is to display the summary output 
    and look for ``NoVehicle`` error.

    .. code-block:: bash

        > dronekit-la test.tlog -s brief
        test.tlog: Score=70 NoVehicle


If the information is missing you should run the analysis again with the frame type (``-f`` - QUAD | Y6) and 
model (``-m`` - copter | rover | plane) arguments:

.. code-block:: bash

    dronekit-la a_log_file.log -f copter -m QUAD
    
    
.. tip::

    `Solo <https://3drobotics.com/solo-drone/>`_ tlogs do not include the frame and model information,
    so always specify the frame and model when analysing Solo tlogs.

.. _guide_output_formats:
    
Interpreting the output
=======================

DroneKit-LA can output three formats: *json* (the default), *plain-text*, and *brief* (a summary format).

JSON output
-----------

JSON output is the default format, and includes all information from the log analysis. 

The output is alphabetically sorted. Most of the top level information is metadata about the file itself, the flight and the quality of the telemetry link.
The most important top level field for analysis is the *severity-score*: this is the cumulative severity of all issues discovered
in the analysis. A score of 20 (as shown below) would be pretty low!

.. code-block:: bash
    :emphasize-lines: 17

    {
       "bytes-dropped" : 0,
       "duration" : 806105,
       "evilness" : 723,
       "evilness-is-deprecated" : 1,
       "format-version" : "0.1",
       "git_version" : "v0.1-122-g6f81",
       "maximum-altitude-absolute" : 230.1900024414062,
       "maximum-altitude-absolute-units" : "metres",
       "maximum-altitude-relative" : 32.97999572753906,
       "maximum-altitude-relative-units" : "metres",
       "maximum-distance-from-origin" : 71.59389640850262,
       "maximum-distance-from-origin-units" : "metres",
       "maximum-velocity" : 5.319671099254919,
       "maximum-velocity-units" : "metres/second",
       "packet-count" : 54560,
       "packet_count" : 54560,
       "severity-score" : 723,
       "tests" : {
        ...
       },
       "timestamp" : 1449206800331951,
       "total-distance-travelled-units" : "metres",
       "total-distance-travellled" : 735.6248609083336,
       "total-flight-time" : 556.1003417968750,
       "total-flight-time-units" : "seconds"
    }     

.. note::

    The default output contains some deprecated fields, which can be recognized by the accompanying field ``**fieldname**_is_deprecated``.
    For example ``evilness`` (replaced by ``severity-score``) has the accompanying field ``evilness-is-deprecated``.
    
    .. code-block:: bash
    
        "evilness" : 723,
        "evilness-is-deprecated" : 1,    
    
    Deprecated fields may be omitted in a following release, so it is important that any tools which use
    DroneKit-LA remove dependencies on these fields before upgrading to another release. You can test whether you 
    are ready to upgrade by using the :option:`-p` option to create output without deprecated fields.
    

All of the tests that were run are listed under the *tests* value. The tests have a *description*, *status*, *name*, *results*, 
and a *severity-score* for this current test. The example below shows a passing test - 
the *severity-score" is set to 0 and there are no *results*.

.. code-block:: bash
    :emphasize-lines: 5,6

      "GPS Fix" : {
         "description" : "This test will FAIL if the quality of the GPS information is poor",
         "evilness" : 0,
         "evilness-is-deprecated" : 1, 
         "name" : "GPS Fix",
         "results" : [],
         "severity-score" : 0,
         "status" : "PASS"
      },


A failing test is similar except it will have a *status* of FAIL or WARN and a non-zero severity-score (the cumulative score for
fais/warnings in the results). Within the *results* we get specific information about sub-tests that failed, including the 
*status*, *reason*, *series* (problem area). The *evidence* field explains exactly what information resulted in the fail/warning and
can be used along with the series information for further analysis of the logs (the type of "evidence" is test-dependent).

.. code-block:: bash
    :emphasize-lines: 5,6      
       
      "Good EKF" : {
         "description" : "This test will FAIL if EKF variances exceed thresholds, or if the EKF status flags indicate errors",
         "evilness" : 20,
         "name" : "Good EKF",
         "results" : [
            {
               "duration" : 32.11100006103516,
               "duration-units" : "seconds",
               "evidence" : [
                  "flags=0",
                  "attitude estimate bad",
                  "horizontal velocity estimate bad",
                  "vertical velocity estimate bad",
                  "horizontal position (relative) estimate bad",
                  "horizontal position (absolute) estimate bad",
                  "vertical position (absolute) estimate bad",
                  "vertical position (above ground) estimate bad",
                  "In constant position mode (no abs or rel position)",
                  "Predicted horizontal position (relative) bad",
                  "Predicted horizontal position (absolute) bad"
               ],
               "evilness" : 20,
               "evilness-is-deprecated" : 1, 
               "reason" : "The EKF status report indicates a problem with the EKF",
               "series" : [ "EKF_STATUS_REPORT.flags" ],
               "severity-score" : 20,
               "status" : "FAIL",
               "timestamp_start" : 1448685117411000,
               "timestamp_stop" : 1448685149522000
            },
            {
               "evilness" : 0,
               "reason" : "EKF flags were never updated",
               "series" : [ "EKF_STATUS_REPORT.flags" ],
               "severity-score" : 0,
               "status" : "WARN"
            }
         ],
         "severity-score" : 20,
         "status" : "FAIL"
      },
    
Brief
-----

The "brief" output format provides the cumulative severity of log issues issues and a brief "fact" about
the flight (for example, Flew, Crash!, NoVehicle).

A common use of this log format is to quickly check for the ``NoVehicle`` string. If this is reported
the log is missing vehicle model and frame information. The code fragment below demonstrates how this might look,
and shows how you can re-run the same test with the needed information:

.. code-block:: bash

    > dronekit-la -s brief test.tlog

    test.tlog: Score=70 NoVehicle

    > dronekit-la -s brief -m copter -f QUAD test.tlog 
    test.tlog: Score=20

.. tip::

The brief format is particularly useful when you're examining a large number of logs and want to start by
examining the most serious first. In the test below we specify a number of files (using a wildcard - but
you can also list the files of interest individually) and immediately see which logs crashed, and how 
severe the reported issues are:

.. code-block:: bash

    > ubuntu@ubuntu:~/test/LALogs$ dronekit-la s* -s brief -m copter -f QUAD

    solo10.tlog: Score=828 Crash! Flew
    solo1.tlog: Score=703 Crash! Flew
    solo2.tlog: Score=754 Crash! Flew
    solo3.tlog: Score=3787 Crash! Flew
    solo4.tlog: Score=388 Crash! Flew
    solo5.tlog: Score=940 Crash! Flew
    solo6.tlog: Score=1126 Flew
    solo7.tlog: Score=1216 Crash! Flew
    solo8.tlog: Score=434 Crash! Flew
    solo9.tlog: Score=3063 Crash! Flew


Plain text output
-----------------

The plain text output has the same indentation as the JSON output but omits the braces and other "type" markup.

This is generated using the argument ``-s plain-text``.
