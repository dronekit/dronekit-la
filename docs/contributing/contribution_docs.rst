.. _contributing-to-documentation:

=================================
Contributing to the Documentation
=================================

One of the best ways that you can help is by improving this documentation.  

The documentation source files are `stored in Github <https://github.com/dronekit/dronekit-la/tree/master/docs>`_. 
The content is written in plain-text files (file-extension :file:`.rst`) using 
`reStructuredText <http://sphinx-doc.org/rest.html>`_ markup, and is compiled into HTML using the 
`Sphinx Documentation Generator <http://sphinx-doc.org/index.html>`_.

Here we explain how to use the documentation system and submit your changes.

Submitting changes
==================

The process and requirements for submitting changes to the documentation are **the same** as when 
:ref:`contributing to the source code <contributing_tool>`. 

As when submitting source code you should fork the main project Github repository and 
contribute changes back to the project using pull requests. The changes should be tested
locally (by :ref:`building the docs <contributing_building_docs>`) before being submitted.

See :ref:`contributing_tool` for more information. 

.. _contributing_building_docs:

Building the docs
=================

We've made it very easy to get started by providing a `Vagrant <https://www.vagrantup.com/>`_ 
based setup for :program:`Sphinx`. 

Using :program:`Vagrant` you can work with source files on your host machine via a familiar 
:program:`git` client and text editor, and then invoke :program:`Sphinx` in the 
:program:`Vagrant` VM to compile the source to HTML.

For more information see :ref:`dronekit_la_setup_vagrant_docs`.


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