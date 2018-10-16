geodesy.gen_uuid
----------------

.. module:: geodesy.gen_uuid

Generate UUIDs for Geographic Information messages.

.. deprecated:: 0.3.0

*This module was removed with version 0.4.0 in ROS Indigo.*

Instead, use the `unique_id`_ package, which has been stable and
available since ROS Fuerte:

 * ``geodesy.gen_uuid.generate(url, id)`` becomes
   ``unique_id.fromURL(url+str(id))``

 * ``geodesy.gen_uuid.makeUniqueID(url, id)`` becomes
   ``unique_id.toMsg(unique_id.fromURL(url+str(id)))``

.. _`unique_id`: http://wiki.ros.org/unique_id
