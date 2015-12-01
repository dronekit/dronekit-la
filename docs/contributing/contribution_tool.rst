.. _contributing_tool:

===========================
Contributing to DroneKit-LA
===========================

This article provides a high level overview of how to contribute changes to the DroneKit-LA source code.

.. tip::
   
    We highly recommend that changes and ideas are `discussed with the project team 
    <https://github.com/dronekit/dronekit-la/issues>`_ before starting work! 


Submitting changes
==================

Contributors should fork the main `dronekit/dronekit-la/ <https://github.com/dronekit/dronekit-la>`_ 
repository and contribute changes back to the project master branch using pull requests:

* Changes should be :ref:`tested locally <contributing-test-code>` before submission.
* Changes should be :ref:`documented <contributing-to-documentation>` (we will provide subediting support!)
* Changes should not break compatibility of the tool or tool output without prior discussion. So for example
  to rename a field you should add a new field of the same name, and arrange deprecation of the old field at
  a future time.
* Pull requests should be as small and focused as possible to make them easier to review.
* Pull requests should be rebased against the main project before submission to make integration easier.



.. _contributing-test-code:

Testing changes
===============

Tests should be used to validate new and changed functionality.

The "standard" test process is run the updated tool against the standard set of logs 
(in `dronekit-la-testdata <https://github.com/dronekit/dronekit-la-testdata>`_) and
verify that the changes *improve the output*.

Changes that are specifically designed for a particular vehicle type (e.g. Copter) or frame
should be tested against logs for that vehicle. You should also validate that platform-specific
tests are not run against the other platforms.

.. warning::

    DroneKit-LA is used by a number of other downstream tools/projects. Updates should 
    not remove tool arguments or remove/change the names of output fields without prior discussion.
