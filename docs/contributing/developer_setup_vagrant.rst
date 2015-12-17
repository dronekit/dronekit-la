.. _dronekit_la_setup_vagrant:

===============================
Building DroneKit-LA in Vagrant
===============================

This section shows how to bring up a development environment using a `Vagrant <https://www.vagrantup.com/>`_ 
virtual machine. This approach provides a reliable and reproducible build environment on any host that supports *Vagrant*
(including Windows, OS-X and Linux).

The environment can be used to build both the dronekit-la tool and the documentation.

.. tip::

    At time of writing, we only have native build support on Linux.
     
    One of the significant benefits of using *Vagrant* is that source files are shared between the host and vagrant environment.
    This allows developers to work with source files using their familiar tools and editors, and then switch to Vagrant 
    just for the final build.
    
  
.. _dronekit_la_setup_vagrant_prerequisites:
     
Prerequisites
=============

First download and set up the development environment:

#. Download and install `VirtualBox <https://www.virtualbox.org/wiki/Downloads>`_ and
   `Vagrant <https://www.vagrantup.com/downloads.html>`_ for your host computer.
   
#. Install Git

   * **Windows:** Install `Git for Windows <https://git-scm.com/download/win>`_.
   * **Linux:** Install git from the terminal using: ``sudo apt-get install git``

#. On Windows only, install SSH (this is present by default on Linux/Mac OSX). 

   * Assuming you installed *Git for Windows* in the previous step, locate SSH using the command ``C:\where ssh`` 
     (normally it is installed to **C:\Program Files (x86)\Git\bin\ssh.exe**).
   * Add the **ssh.exe** location to the *Path* (**System Properties | Advanced tab | Environment Variables | Path**)

   
#. Fork the DroneKit-LA repository. Open a command prompt and clone the repository on your host system:

   .. code:: bash

       git clone https://github.com/YOUR_REPO/dronekit-la.git
       

Building the DroneKit-LA Tool
=============================

#. Navigate to the **dronekit-la** directory (or any subdirectory) and start Vagrant in your command prompt:

   .. code:: bash

       vagrant up
       
   .. note:: 

       This may take a long time to complete the first time it is run as Vagrant needs to 
       download the virtual machine and then set up the development environment and Sphinx.
       

#. SSH into the vagrant instance and build the tool:

   .. code:: bash

       vagrant ssh
       cd /vagrant
       make
       
   The **/vagrant** directory maps to the location of the *Vagrantfile* in the host OS 
   (in this case **/dronekit-la** directory on your host computer). This is the same location 
   where the **dronekit-la** executable will be created.

#. Run dronekit-la from within Vagrant using the normal syntax:

   .. code:: bash

       ./dronekit-la -h
       



.. _dronekit_la_setup_vagrant_docs:
    
Building the Documentation in Vagrant
=====================================

Vagrant also sets up :program:`Sphinx`, which is used to build the :doc:`documentation <contribution_docs>`. 
The documentation source files are located in **/dronekit-la/docs** on the host computer, which maps to **/vagrant/docs**
within the Vagrant VM.

After you have installed the :ref:`dronekit_la_setup_vagrant_prerequisites`:

#. Start Vagrant, SSH into the VM, and navigate to **/vagrant/docs** within the VM as shown:

   .. code:: bash
   
       vagrant up
       vagrant ssh
       cd /vagrant/docs/
       
#. Build *all* the documentation as shown:

   .. code:: bash
   
       make clean
       make html
       
    To just build files that have changed just do:

   .. code:: bash
   
       make html
    
       
   The files will be built by :program:`Sphinx`, and will appear on the host system in 
   :file:`<clone-path>/dronekit-la/docs/\_build/html/`. To preview, simply open them in a Web browser.   

.. tip::

    The API Reference is built from strings in the header files which are extracted using Doxygen and then
    imported using the Sphinx "Breathe" extension. If you change the strings you will need to re-run Doxgen
    and Sphinx:    
  
   .. code-block:: bash
   
       vagrant ssh
       cd /vagrant
       doxygen doxygen/doxygen.conf
       cd /docs
       make clean
       make html


Closing and re-using the VM
===========================

When you are finished you can exit and suspend the VM. 
Next time you need to build more HTML simply restart it (this is a fast operation):

.. code-block:: bash
 
    exit              # Leave Vagrant/SSH prompt
    vagrant suspend   #Suspend the VM
    vagrant resume    #Restart the VM
    vagrant ssh       #Log back into the VM