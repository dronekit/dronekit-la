.. _analyzer_listing:

=============
Analyzer List
=============

This topic provides an overview of the available analyzers. These are all run by default
but you can specify to only run a subset using the ``-a`` argument (see 
:ref:`guide_selecting_analyzers`).

Each section lists a description of the analyzer, along with possible fails/warnings.

.. note::

    This section is provided for reference only. The log itself will include all this information
    along with the "severity" of the issue, "series" (where the error occurred) and "evidence" 
    (what caused a fail/warning to be raised).



Any Parameters Seen
=================== 

This test will FAIL if the input does not contain parameter information.

* Fail: No parameters present
* Pass: Parameters present


.. note::

    No user action can be taken in response to this error.

    
.. internalnotes

    - defined in: 
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_any_parameters_seen.cpp
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_any_parameters_seen.h
    


        
Arming Checks
=============

This test will FAIL if the vehicle arms when any arming checks are disabled.

* Fail: Some of the arming checks were disabled when the craft was armed



.. internalnotes

    - defined in: 
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_arming_checks.cpp
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_arming_checks.h
   



Altitude Estimate Divergence
============================


This analyzer will WARN if the altitude estimate differs from the canonical craft altitude.

* Warn: This altitude estimate differs from the canonical craft altitude

    

.. internalnotes

    - defined in: 
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_altitude_estimate_divergence.cpp
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_altitude_estimate_divergence.h





Attitude Estimate Divergence
============================

This test will FAIL or WARN if the various vehicle's Altitude estimates diverge.

* Fail: This attitude estimate differs from the canonical craft attitude.
* Warn: This attitude estimate differs from the canonical craft attitude. 
    


.. internalnotes

    - defined in: 
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_attitude_estimate_divergence.cpp
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_attitude_estimate_divergence.h




Compass Offsets
===============

This test will WARN or FAIL depending on the degree that compass offset parameters exceed specified thresholds.

* Fail: Compass offsets in parameters are out of bounds
* Fail: Compass offset parameter set seen/set
* Warn: Compass offsets in parameters are out of bounds
* Warn: Compass offsets in parameters are zero
* Pass: Compass offsets in parameters look reasonable


.. internalnotes

    - defined in: 
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_compass_offsets.cpp
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_compass_offsets.h




Compass Vector Length
=====================

This test will FAIL or WARN if the compass vector length exceeds the respective threshold.  
Possible causes include flying near large metal objects.

* Fail: Compass Vector Length above threshold
* Fail: Compass Vector Length below threshold
* Fail: Compass Vector Length delta exceeds threshold
* Warn: Compass Vector Length below threshold
* Warn: Compass Vector Length above threshold



.. internalnotes

    - defined in: 
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_compass_vector_length.cpp
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_compass_vector_length.h


Ever Armed
==========

This test will FAIL if the craft did not arm.

* Fail: The vehicle never armed
* Pass: The vehicle armed



.. internalnotes

    - defined in: 
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_ever_armed.cpp
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_ever_armed.h




Ever Flew
=========

This test will FAIL if the craft did not ever seem to fly.

As evidence the test provides information about the whether the vehicle armed
and whether it reaches the servo threshold required to take off.

* Fail: The vehicle never seemed to take off
* Pass: The vehicle appeared to fly


.. internalnotes

    - defined in: 
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_ever_flew.cpp
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_ever_flew.h



Good EKF
========

This test will FAIL or WARN if EKF variances exceed the respective thresholds, or FAIL if the EKF status flags indicate errors.

For EKF status flag fails, the evidence field provides information about the specific estimates that are incorrect.

* Fail: The EKF status report indicates a problem with the EKF
* Fail: [variance] exceeds fail threshold
* Warn: [variance] exceeds warn threshold
* Warn: [variance] was never updated
* Warn: EKF flags were never updated




.. note::

    In the list above [variance] is one of:  velocity, pos_horiz_variance, 
    pos_vert_variance, compass_variance, terrain_alt_variance.    
    

.. internalnotes

    - defined in:
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_good_ekf.cpp
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_good_ekf.h



GPS Fix
=======

This test will FAIL if the quality of the GPS information is poor.

The test compares the recorded number of satellites and HDOP (horizontal degree of precision) 
to threshold values and reports both values as "evidence".

* Fail: No 3D fix was ever acquired
* Pass: First 3D GPS Fix Acquired


        

.. internalnotes

    - defined in:
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_gps_fix.cpp
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_gps_fix.h
 



Attitude Control
================

This test will FAIL or WARN if the vehicle's desired attitudes and achieved attitudes 
are not within threshold delta values for more than a threshold time.

The evidence provided includes the maximum difference between the desired/achieved roll and pitch
and the duration of the test.

* Fail: Desired attitude not achieved
* Warn: Desired attitude not achieved



.. internalnotes

    - defined in: 
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_attitude_control.cpp
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_attitude_control.h

    
.. _analyzer_autopilot:

AutoPilot Health
================

Many autopilots are capable of monitoring their own performance.  This test will FAIL if problems are detected with the autopilot

* Fail: Severe scheduler overruns



.. internalnotes

    - defined in: 
      https://github.com/peterbarker/dronekit-la/blob/peter-wip/analyzer/analyzer_autopilot.cpp
      https://github.com/peterbarker/dronekit-la/blob/peter-wip/analyzer/analyzer_autopilot.h



Battery
=======

This test will FAIL if the battery level falls below the 
`battery failsafe <http://copter.ardupilot.com/wiki/failsafe-battery/>`_ 
threshold level, or if a battery failsafe event is received.

* Fail: Battery fell below failsafe threshold
* Fail: Battery failsafe event received
* Pass: Battery never below failsafe



.. internalnotes

    - defined in:
    https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_battery.cpp
    https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_battery.h



Brownout
========

This test will pass if the log is not truncated, an issue which is often caused if the craft loses onboard power during a flight.

* Fail: Log ended while craft still flying
* Warn: Altitude never changed
* Pass: No brownout detected

.. note::

   There are several possible causes for a truncated log (including power failure due to brownout,
   running out of memory for the log file, or failure of the logging sub-system). Failing this test
   does not necessarily mean a brownout actually occurred.


.. internalnotes

    - defined in:
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_brownout.cpp
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_brownout.h



Position Estimate Divergence
============================

This test will FAIL or WARN if various position estimates diverge from given threshold levels.

* Fail: This position estimate differs from the canonical craft position
* Warn: This position estimate differs from the canonical craft position


.. internalnotes

    - defined in:
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_position_estimate_divergence.cpp
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_position_estimate_divergence.h



Crash Test
==========

This test will FAIL if the vehicle appears to crash.

* Fail: Vehicle evaluated itself as crashed
* Fail: Vehicle is past maximum allowed angle and running its motors
* Warn: Vehicle's attitude never updated
* Pass: Never crashed   


.. internalnotes

    - defined in:
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_notcrashed.cpp
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_notcrashed.h


Sensor Health
=============

This test will FAIL if any sensor is detected as failed.

* Fail: The craft's assessment of its sensors indicate a problem
* Warn: Sensor health never updated


    


.. internalnotes

    - defined in: 
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_sensor_health.cpp
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_sensor_health.h


.. _analyzer_listing_vehicle_definition:

Vehicle Definition
==================

This test will FAIL if the craft type is never defined.

* Fail: No information provided defined what type of vehicle was being analysed
* Pass: Vehicle was appropriately defined


.. note:: 
    
    Information about the vehicle type/frame allows a much deeper log analysis. This information
    is typically present in logs, but may be omitted. 
    
    If you get this error you should provide the information to the tool using the ``-m`` and ``-f`` flags as
    shown:

    .. code-block:: bash
        
        ./dronekit-la <files> -m copter -f quad
        
        
.. tip::

    `Solo <https://3drobotics.com/solo-drone/>`_ tlogs do not include the frame and model information!




.. internalnotes

    - defined in: 
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_vehicle_definition.cpp
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_vehicle_definition.h


      
.. _analyzer_velocity_estimate_divergence:  

Velocity Estimate Divergence
============================

.. warning:: 

    Currently no implementation (dronekit-la 0.3)

This test will FAIL if various craft's velocity estimates diverge.




.. internalnotes

    - defined in: 
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_velocity_estimate_divergence.cpp
      https://github.com/dronekit/dronekit-la/blob/master/analyzer/analyzer_velocity_estimate_divergence.h

