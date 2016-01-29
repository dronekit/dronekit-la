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

Contributors should create a sensibly name branch in the main 
`dronekit/dronekit-la/ <https://github.com/dronekit/dronekit-la>`_ 
repository and contribute changes back to the project master branch using pull requests:

* Changes should be :ref:`tested locally <contributing-test-code>` before submission.
* Changes should be :ref:`documented <contributing-to-documentation>` (we will provide subediting support!)
* Changes should not break compatibility of the tool or tool output without prior discussion. So for example
  to rename a field you should add a new field of the same name, and arrange deprecation of the old field at
  a future time.
* Pull requests should be as small and focused as possible to make them easier to review.
* Pull requests should be rebased against the main project before submission to make integration easier.



Building the Tool
=================

DroneKit-LA is built in the same way on all platforms; by cloning the repository and then calling ``make`` in its root directory. The only differences are in the dependencies required to build on each platform.

Building using Vagrant
----------------------

:doc:`developer_setup_vagrant` explains how to build both the tool and its documentation in a Vagrant VM. 


Building on Linux
-----------------

If you want to run natively on Linux you can manually install the dependencies listed in the 
`Vagrantfile <https://github.com/dronekit/dronekit-la/blob/master/Vagrantfile>`_. At time of writing the
commands to do so on Linux would be:

.. code-block:: bash
       
    # These are needed to build DroneKit-LA
    sudo apt-get install git build-essential libjsoncpp-dev libjsoncpp0

    # These are needed to build the documentation using Sphinx
    sudo apt-get install python-dev python-pip
    sudo easy_install -U pip
    sudo pip install sphinx sphinx-3dr-theme

To build *dronekit-la*, you can then just clone the repository and call *make* from its root:

.. code-block:: bash

    # Get the repo
    git clone https://github.com/dronekit/dronekit-la.git
    cd dronekit-la

    # Build the tool
    make


Building on Mac OSX
-------------------

First install the `XCode Developer Tools <https://developer.apple.com/xcode/download/>`_.

To build *dronekit-la*, you can then just clone the repository and call *make* from its root:

.. code-block:: bash

    # Get the repo
    git clone https://github.com/dronekit/dronekit-la.git
    cd dronekit-la

    # Build the tool
    make

    
Building on Windows
-------------------

You can build *dronekit-la* natively on Windows from within `Cygwin <https://cygwin.com/install.html>`_.

First set up the dependencies:

* `Download and install Cygwin <http://cygwin.com/setup-x86.exe>`_ (install for all users)
* Install the following packages:

  * Devel / git
  * Devel / make
  * Devel / mingw64-x86_64-gcc-g++

* Open the *Cygwin Terminal*
  
To build *dronekit-la*, you can then just clone the repository and call *make* from its root:

.. code-block:: bash

    # Get the repo
    git clone https://github.com/dronekit/dronekit-la.git
    cd dronekit-la

    # Build the tool
    make


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
