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

#. Extend ``AnalyzerVehicle::Base`` with members and functions that are appropriate for capturing the 
   desired information and series of information. If you can infer useful vehicle-specific information 
   from a  message add it to the vehicle specific classes in the ``AnalyzerVehicle`` namespace (e.g. ``Copter``).
  
#. Add a message type handler for the desired message in the dataflash log. This will populate the vehicle
   model appropriately and add data sources that can later be referenced by analyzers for the output:

   * Declare a handler class for the desired message type in 
     `dronekit-la/LA_MsgHandler.h <https://github.com/dronekit/dronekit-la/blob/master/LA_MsgHandler.h>`_. 
    
     This should be derived from ``LA_MsgHandler`` and implement ``virtual void xprocess(const uint8_t *msg) = 0;``
     (where you process the message and update the vehicle). 
    
     The handler should also add "data sources" to its (inherited) ``_analyze`` variable using the ``add_data_source()``. 
     Each source would have a name used to reference the source in analyzers, and another string indicating 
     associated the message type and field. For example:
    
     .. code-block:: cpp
    
         _analyze->add_data_source("EKF_FLAGS", "EKF4.SS");
        
   * Create an instance of the handler and pass a message to it in 
     `Analyzing_DataFlash_Message_Handler::handle_format_message_received <https://github.com/dronekit/dronekit-la/blob/master/analyzing_dataflash_message_handler.>`_.

#. Add a message type handler for the same message in tlogs. This also populates the vehicle model
   appropriately and adds data sources that can later be referenced by analyzers for the output (though
   the structure of code is quite different):

   * Update `MAVLink_Message_Handler <https://github.com/dronekit/dronekit-la/blob/master/mavlink_message_handler.h>`_ and 
     `Analyzing_MAVLink_Message_Handler <https://github.com/dronekit/dronekit-la/blob/master/analyzing_mavlink_message_handler.h>`_
     with virtual ``handle_decoded_message()`` methods (and implementations). As before set the data sources for the processed mesages. 
  

.. tip::

    The above process may seem confusing, but there are lots of good examples in the linked source!