.. _other_features_top:

==============
Other Features
==============


.. _input_from_stdin:

Taking input from STDIN
=======================

You can also take log information directly from STDIN by specifying a final "-" rather than a filename. Because the log has no 
file extension you will also need to confirm the log-type using the ``-i`` flag (as ``tlog`` | ``df`` | ``log``):

.. code-block:: bash

    dronekit-la -i tlog -

So for example, the following two lines are equivalent (on Linux):

.. code-block:: bash

    cat 1.BIN | dronekit-la -i df -
    
    dronekit-la 1.BIN


.. _guide_selecting_analyzers:
    
Selecting analyzers
===================

All :ref:`available analysers <analyzer_listing>` are run by default. 

You can list them using the ``-l`` flag. At time of writing (DroneKit-LA 0.3) the set is:


.. code-block:: bash

    >dronekit-la -l
    
    Any Parameters Seen
    Arming Checks
    Altitude Estimate Divergence
    Attitude Estimate Divergence
    Compass Offsets
    Compass Vector Length
    Compass Offsets
    Compass Offsets
    Ever Armed
    Ever Flew
    Good EKF
    GPS Fix
    Gyro Drift
    Attitude Control
    AutoPilot Health
    Battery
    Brownout
    Position Estimate Divergence
    Crash Test
    Sensor Health
    Vehicle Definition
    Velocity Estimate Divergence

If you just want to run a small subset of the analysers use the ``-a`` flag. 

.. code-block:: bash

    dronekit-la -a "Ever Flew, Battery" my_log.tlog
    

.. _guide_configuration_files:

Use a config file
=================

You can use a configuration file to specify threshold values past which an event is detected.  This allows you
to be more selective about the conditions which you consider serious. For example, you can use the configuration
file to set that you only care about an attitude control problem if it persists for more than 250ms.

To use a configuration file, specify it with the ``-c`` argument:

.. code-block:: bash

    dronekit-la -c "/full-path-to-config/sample.config" my_log.tlog

A sample configuration file with all current settable fields can be found in the root of the 
Github repository: `sample.config <https://github.com/dronekit/dronekit-la/blob/master/sample.config>`_.

A snapshot of **sample.config** is reproduced below:


.. include:: ../../sample.config
    :literal:   
