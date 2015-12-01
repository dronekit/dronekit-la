.. _dronekit-la-reference:

======================
Command Line Reference
======================

The tool outputs a brief command line summary when you enter ``dronekit-la`` or ``dronekit-la -h`` on a command prompt.

Command Syntax
==============

.. code-block:: bash

    dronekit-la [OPTIONS] [FILE LIST]
    
Multiple options can be specified. 

One or more files can be specified in the list and you can also specify a 
wildcard character (``*``) to select multiple files.


Options
=======

.. list-table::
   :widths: 8 30
   :header-rows: 1

   * - flag
     - Description
   * - ``-c`` FILEPATH
     - Specify path to a config file for specifying fail/warning trigger levels. See :ref:`guide_configuration_files`.
     
   * - ``-m`` MODELTYPE
     - Specify vehicle type/model as one of: ``copter`` | ``plane`` | ``rover``. 
     
       This overrides any existing setting in the log. 
       This argument should be set if the :ref:`analyzer_listing_vehicle_definition` test fails.
       For more information see :ref:`guide_model_and_frame_type`.
       
   * - ``-f`` FRAME
     - Set vehicle frame type as one of: ``QUAD`` | ``Y6``.
     
       This overrides any existing setting in the log. 
       This argument may be required if the :ref:`analyzer_listing_vehicle_definition` test fails.
       For more information see :ref:`guide_model_and_frame_type`.
       
   * - ``-s`` STYLE
     - Specify output format/style as one of: ``plain-text`` | ``json`` | ``brief``. The default value is ``json``.
     
       For more information see :ref:`guide_output_formats`.
       
   * - ``-h``
     - Display help/usage information.
     
   * - ``-l``
     - List analyzers. 
     
       For more information see: :ref:`guide_selecting_analyzers`. 

     
   * - ``-a``
     - Specify analyzers to run in a comma-separated list.
     
       .. code-block:: bash
       
           dronekit-la -a "Ever Flew, Battery" 1.solo.tlog
           
       For more information see: :ref:`guide_selecting_analyzers`.
           
   * - ``-i`` FORMAT
     - Specify log format as one of: ``tlog`` | ``df`` | ``log``.
     
       .. tip::
       
           This is generally not required as DroneKit-LA will infer the log
           type from the log file extension. It is needed if you are 
           :ref:`analyzing a file input from STDIN <input_from_stdin>`.
       

   * - ``-p``
     - Pure output. 
     
       The default output includes deprecated fields in order to maintain compatibility for 
       downsteam users (e.g. "evilness"). Use this flag to ensure that only the current set of
       fields are exported.
 
   * - ``-V``
     - Display version information.
     
   * - ``-``
     - Analyze input from STDIN. 
     
       When using input from STDIN you will also need to specify the log type with the ``-i`` flag:
     
       .. code-block:: bash

           dronekit-la -i tlog -
           
       For more information see: :ref:`input_from_stdin`.