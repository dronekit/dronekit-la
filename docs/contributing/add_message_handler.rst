.. _message_handler_top:

===============================
Extending the Vehicle-Log Model
===============================

The vehicle "log model" class (``AnalyzerVehicle::Base``) provides a common model for analyzers to query
log information, independent of the original log format. 

This class exposes a subset of information from the original logs. If a new analyzer needs 
information that is not present in the model (but which was present in the original logs) then the
model must be extended.

This process has three parts:

#. Extend :cpp:class:`AnalyzerVehicle::Base` with members and functions that are appropriate for capturing the 
   desired information and series of information. If you can infer useful vehicle-specific information 
   from a  message add it to the vehicle specific classes in the ``AnalyzerVehicle`` namespace (e.g. :cpp:class:`Copter <AnalyzerVehicle::Copter>`).
  
#. Add a message type handler for the desired message in the dataflash log. This will populate the vehicle
   model appropriately and add data sources that can later be referenced by analyzers for the output:

   * Declare a handler class for the desired message type in 
     `LA_MsgHandler.h <https://github.com/dronekit/dronekit-la/blob/master/LA_MsgHandler.h>`_. 
    
     This should be derived from :cpp:class:`LA_MsgHandler` and implement the pure virtual method :cpp:func:`LA_MsgHandler::xprocess()` (where you process the message and update the vehicle). 
    
     The handler should also add "data sources" to its (inherited) ``_analyze`` variable using :cpp:func:`Analyze::add_data_source()`. 
     Each source would have a name used to reference the source in analyzers, and another string indicating 
     associated the message type and field. For example:
    
     .. code-block:: cpp
    
         _analyze->add_data_source("EKF_FLAGS", "EKF4.SS");
        
   * Create an instance of the handler and pass a message to it in 
     :cpp:func:`Analyzing_DataFlash_Message_Handler::handle_format_message_received`.

#. Add a message type handler for the same message in tlogs. This also populates the vehicle model
   appropriately and adds data sources that can later be referenced by analyzers for the output (though
   the structure of code is quite different):

   * Update `MAVLink_Message_Handler <https://github.com/dronekit/dronekit-la/blob/master/mavlink_message_handler.h>`_ and 
     `Analyzing_MAVLink_Message_Handler <https://github.com/dronekit/dronekit-la/blob/master/analyzing_mavlink_message_handler.h>`_
     with virtual ``handle_decoded_message()`` methods (and implementations). As before set the data sources for the processed messages. 
  

.. tip::

    The above process may seem confusing, but there are lots of good examples in the linked source!