# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from controller_manager_msgs/ListControllersRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ListControllersRequest(genpy.Message):
  _md5sum = "d41d8cd98f00b204e9800998ecf8427e"
  _type = "controller_manager_msgs/ListControllersRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# The ListControllers service returns a list of controller names/states/types of the
# controllers that are loaded inside the controller_manager.

"""
  __slots__ = []
  _slot_types = []

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ListControllersRequest, self).__init__(*args, **kwds)

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      pass
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      pass
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from controller_manager_msgs/ListControllersResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import controller_manager_msgs.msg

class ListControllersResponse(genpy.Message):
  _md5sum = "943fb5d3bbc209623c0202d4e174627a"
  _type = "controller_manager_msgs/ListControllersResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """ControllerState[] controller


================================================================================
MSG: controller_manager_msgs/ControllerState
string name
string state
string type
int32 priority
controller_manager_msgs/HardwareInterfaceResources[] claimed_resources

================================================================================
MSG: controller_manager_msgs/HardwareInterfaceResources
# Type of hardware interface
string hardware_interface
# List of resources belonging to the hardware interface
string[] resources
"""
  __slots__ = ['controller']
  _slot_types = ['controller_manager_msgs/ControllerState[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       controller

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ListControllersResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.controller is None:
        self.controller = []
    else:
      self.controller = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      length = len(self.controller)
      buff.write(_struct_I.pack(length))
      for val1 in self.controller:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.state
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.type
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.priority
        buff.write(_get_struct_i().pack(_x))
        length = len(val1.claimed_resources)
        buff.write(_struct_I.pack(length))
        for val2 in val1.claimed_resources:
          _x = val2.hardware_interface
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
          length = len(val2.resources)
          buff.write(_struct_I.pack(length))
          for val3 in val2.resources:
            length = len(val3)
            if python3 or type(val3) == unicode:
              val3 = val3.encode('utf-8')
              length = len(val3)
            buff.write(struct.Struct('<I%ss'%length).pack(length, val3))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.controller is None:
        self.controller = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.controller = []
      for i in range(0, length):
        val1 = controller_manager_msgs.msg.ControllerState()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.name = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.state = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.state = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.type = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.type = str[start:end]
        start = end
        end += 4
        (val1.priority,) = _get_struct_i().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.claimed_resources = []
        for i in range(0, length):
          val2 = controller_manager_msgs.msg.HardwareInterfaceResources()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val2.hardware_interface = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val2.hardware_interface = str[start:end]
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          val2.resources = []
          for i in range(0, length):
            start = end
            end += 4
            (length,) = _struct_I.unpack(str[start:end])
            start = end
            end += length
            if python3:
              val3 = str[start:end].decode('utf-8', 'rosmsg')
            else:
              val3 = str[start:end]
            val2.resources.append(val3)
          val1.claimed_resources.append(val2)
        self.controller.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.controller)
      buff.write(_struct_I.pack(length))
      for val1 in self.controller:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.state
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.type
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.priority
        buff.write(_get_struct_i().pack(_x))
        length = len(val1.claimed_resources)
        buff.write(_struct_I.pack(length))
        for val2 in val1.claimed_resources:
          _x = val2.hardware_interface
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
          length = len(val2.resources)
          buff.write(_struct_I.pack(length))
          for val3 in val2.resources:
            length = len(val3)
            if python3 or type(val3) == unicode:
              val3 = val3.encode('utf-8')
              length = len(val3)
            buff.write(struct.Struct('<I%ss'%length).pack(length, val3))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.controller is None:
        self.controller = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.controller = []
      for i in range(0, length):
        val1 = controller_manager_msgs.msg.ControllerState()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.name = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.state = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.state = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.type = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.type = str[start:end]
        start = end
        end += 4
        (val1.priority,) = _get_struct_i().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.claimed_resources = []
        for i in range(0, length):
          val2 = controller_manager_msgs.msg.HardwareInterfaceResources()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val2.hardware_interface = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val2.hardware_interface = str[start:end]
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          val2.resources = []
          for i in range(0, length):
            start = end
            end += 4
            (length,) = _struct_I.unpack(str[start:end])
            start = end
            end += length
            if python3:
              val3 = str[start:end].decode('utf-8', 'rosmsg')
            else:
              val3 = str[start:end]
            val2.resources.append(val3)
          val1.claimed_resources.append(val2)
        self.controller.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i
class ListControllers(object):
  _type          = 'controller_manager_msgs/ListControllers'
  _md5sum = '943fb5d3bbc209623c0202d4e174627a'
  _request_class  = ListControllersRequest
  _response_class = ListControllersResponse
