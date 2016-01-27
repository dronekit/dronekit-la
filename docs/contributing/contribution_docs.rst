.. _contributing-to-documentation:

=================================
Contributing to the Documentation
=================================

One of the best ways that you can help is by improving this documentation.  

The documentation source files are `stored in Github <https://github.com/dronekit/dronekit-la/tree/master/docs>`_. 
The content is written in plain-text files (file-extension :file:`.rst`) using 
`reStructuredText <http://sphinx-doc.org/rest.html>`_ markup, and is compiled into HTML using the 
`Sphinx Documentation Generator <http://sphinx-doc.org/index.html>`_. 

Analyzer API reference documentation is automatically imported from doc strings in the 
source code header files (`DOxygen <http://www.stack.nl/~dimitri/doxygen/>`_ is used to 
extract the strings into XML, which are then imported by Sphinx using the 
`Breathe <https://breathe.readthedocs.org/en/latest/>`_ plugin). 

Here we explain how to use the documentation system and submit your changes.

Submitting changes
==================

The process and requirements for submitting changes to the documentation are **the same** as when 
:ref:`contributing to the source code <contributing_tool>`. 

As when submitting source code you should create a sensibly name branch in the main 
`dronekit/dronekit-la/ <https://github.com/dronekit/dronekit-la>`_ 
repository and contribute changes back to the project master branch using pull requests. The changes should be tested
locally (by :ref:`building the docs <contributing_building_docs>`) before being submitted.

See :ref:`contributing_tool` for more information. 

.. _contributing_building_docs:

Building the docs
=================

The easy way (Vagrant)
----------------------
We've made it very easy to get started by providing a "no configuration" `Vagrant <https://www.vagrantup.com/>`_ 
setup that you can use to build both the tool and documentation. 

Using :program:`Vagrant` you can work with source files on your host machine via a familiar 
:program:`git` client and text editor, and then invoke :program:`Sphinx` in the 
:program:`Vagrant` VM to compile the source to HTML.

For more information see :ref:`dronekit_la_setup_vagrant_docs`.


Building docs on native OS
--------------------------

You can also build the documentation directly on your computer.

To do this you will need:

* `Sphinx <http://sphinx-doc.org/latest/index.html>`_ - Generates the documentation from .rst source files. 

  Download for your platform from `here <http://sphinx-doc.org/latest/install.html>`_.
      
* `sphinx-3dr-theme <https://github.com/3drobotics/sphinx_3dr_theme>`_ - A custom theme for Sphinx. 

  This can be installed using PIP:

  .. code-block:: bash
      
      pip install sphinx-3dr-theme

* `Doxygen <http://www.stack.nl/~dimitri/doxygen/index.html>`_ - Generates XML files for in-source documentation strings 
  (must be run every time your source code changes). 
      
  Download for your platform from `here <http://www.stack.nl/~dimitri/doxygen/download.html>`_.
      
* `Breath <https://breathe.readthedocs.org/en/latest/>`_ - Plugin to import API definitions from XML into Sphinx documentation.
    
  This can be installed from the PyPi package repository as shown (the project is configured to use this plugin if it is present).
    
  .. code-block:: bash

    pip install breathe

After installing the toolchain, first run *Doxygen* from the root of your clone of **dronekit-la** and then 
navigate to **/docs** to build the documentation:

.. code-block:: bash
   
    cd /dronekit-la
    doxygen doxygen/doxygen.conf
    cd /docs
    make clean
    make html
    
    
Tracking todo items
===================

Generally it is best to track activities using `Github issues <https://github.com/dronekit/dronekit-la/issues>`_.
In some cases it can be useful also to include "todo" notes in the documentation source. You can do this using
the ``todo`` directive, and the output will only be rendered if you build with the ``todo_include_todos`` flag enabled.

#. Add todo notes just like a "note", "warning", "tip" etc.

   .. code-block:: bash
   
       .. todo:: 
       
           This is todo text 
           
#. Build using the following invocation to display todo messages. 

   .. code-block:: bash
   
      make html SPHINXOPTS="-D todo_include_todos=1"
        

#. Use the ``todolist`` directive to list all todo's in the build. This page has such a link below 
   (so if this is a todo-enabled build you will see them).  


        
.. todolist::
      
      
Style guide
===========

.. tip:: 

    This guide is evolving. The most important guidance we can give is 
    to *copy the existing style of reference, guide and example material*!


#. Use US English for spelling.
#. Use emphasis sparingly (italic, bold, underline). 
#. Use `Sphinx semantic markup <http://sphinx-doc.org/markup/inline.html#other-semantic-markup>`_ 
   to mark up *types* of text (key-presses, file names etc.)
#. Use double backticks (``) around ``inline code`` items.