# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from stoch3_msgs/QuadrupedLegState.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg
import stoch3_msgs.msg

class QuadrupedLegState(genpy.Message):
  _md5sum = "bf5e523a21942d2ff068a9be010f7df1"
  _type = "stoch3_msgs/QuadrupedLegState"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """Header header
stoch3_msgs/LegState fl
stoch3_msgs/LegState fr
stoch3_msgs/LegState bl
stoch3_msgs/LegState br

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: stoch3_msgs/LegState
string name
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 force
float64 support_probability        # [0, 1] , probability that the leg is a support leg.
                                   # Limit the value to range [0, 1] if it is outside the range.

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"""
  __slots__ = ['header','fl','fr','bl','br']
  _slot_types = ['std_msgs/Header','stoch3_msgs/LegState','stoch3_msgs/LegState','stoch3_msgs/LegState','stoch3_msgs/LegState']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,fl,fr,bl,br

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(QuadrupedLegState, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.fl is None:
        self.fl = stoch3_msgs.msg.LegState()
      if self.fr is None:
        self.fr = stoch3_msgs.msg.LegState()
      if self.bl is None:
        self.bl = stoch3_msgs.msg.LegState()
      if self.br is None:
        self.br = stoch3_msgs.msg.LegState()
    else:
      self.header = std_msgs.msg.Header()
      self.fl = stoch3_msgs.msg.LegState()
      self.fr = stoch3_msgs.msg.LegState()
      self.bl = stoch3_msgs.msg.LegState()
      self.br = stoch3_msgs.msg.LegState()

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
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.fl.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_10d().pack(_x.fl.position.x, _x.fl.position.y, _x.fl.position.z, _x.fl.velocity.x, _x.fl.velocity.y, _x.fl.velocity.z, _x.fl.force.x, _x.fl.force.y, _x.fl.force.z, _x.fl.support_probability))
      _x = self.fr.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_10d().pack(_x.fr.position.x, _x.fr.position.y, _x.fr.position.z, _x.fr.velocity.x, _x.fr.velocity.y, _x.fr.velocity.z, _x.fr.force.x, _x.fr.force.y, _x.fr.force.z, _x.fr.support_probability))
      _x = self.bl.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_10d().pack(_x.bl.position.x, _x.bl.position.y, _x.bl.position.z, _x.bl.velocity.x, _x.bl.velocity.y, _x.bl.velocity.z, _x.bl.force.x, _x.bl.force.y, _x.bl.force.z, _x.bl.support_probability))
      _x = self.br.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_10d().pack(_x.br.position.x, _x.br.position.y, _x.br.position.z, _x.br.velocity.x, _x.br.velocity.y, _x.br.velocity.z, _x.br.force.x, _x.br.force.y, _x.br.force.z, _x.br.support_probability))
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
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.fl is None:
        self.fl = stoch3_msgs.msg.LegState()
      if self.fr is None:
        self.fr = stoch3_msgs.msg.LegState()
      if self.bl is None:
        self.bl = stoch3_msgs.msg.LegState()
      if self.br is None:
        self.br = stoch3_msgs.msg.LegState()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.fl.name = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.fl.name = str[start:end]
      _x = self
      start = end
      end += 80
      (_x.fl.position.x, _x.fl.position.y, _x.fl.position.z, _x.fl.velocity.x, _x.fl.velocity.y, _x.fl.velocity.z, _x.fl.force.x, _x.fl.force.y, _x.fl.force.z, _x.fl.support_probability,) = _get_struct_10d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.fr.name = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.fr.name = str[start:end]
      _x = self
      start = end
      end += 80
      (_x.fr.position.x, _x.fr.position.y, _x.fr.position.z, _x.fr.velocity.x, _x.fr.velocity.y, _x.fr.velocity.z, _x.fr.force.x, _x.fr.force.y, _x.fr.force.z, _x.fr.support_probability,) = _get_struct_10d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.bl.name = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.bl.name = str[start:end]
      _x = self
      start = end
      end += 80
      (_x.bl.position.x, _x.bl.position.y, _x.bl.position.z, _x.bl.velocity.x, _x.bl.velocity.y, _x.bl.velocity.z, _x.bl.force.x, _x.bl.force.y, _x.bl.force.z, _x.bl.support_probability,) = _get_struct_10d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.br.name = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.br.name = str[start:end]
      _x = self
      start = end
      end += 80
      (_x.br.position.x, _x.br.position.y, _x.br.position.z, _x.br.velocity.x, _x.br.velocity.y, _x.br.velocity.z, _x.br.force.x, _x.br.force.y, _x.br.force.z, _x.br.support_probability,) = _get_struct_10d().unpack(str[start:end])
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
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.fl.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_10d().pack(_x.fl.position.x, _x.fl.position.y, _x.fl.position.z, _x.fl.velocity.x, _x.fl.velocity.y, _x.fl.velocity.z, _x.fl.force.x, _x.fl.force.y, _x.fl.force.z, _x.fl.support_probability))
      _x = self.fr.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_10d().pack(_x.fr.position.x, _x.fr.position.y, _x.fr.position.z, _x.fr.velocity.x, _x.fr.velocity.y, _x.fr.velocity.z, _x.fr.force.x, _x.fr.force.y, _x.fr.force.z, _x.fr.support_probability))
      _x = self.bl.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_10d().pack(_x.bl.position.x, _x.bl.position.y, _x.bl.position.z, _x.bl.velocity.x, _x.bl.velocity.y, _x.bl.velocity.z, _x.bl.force.x, _x.bl.force.y, _x.bl.force.z, _x.bl.support_probability))
      _x = self.br.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_10d().pack(_x.br.position.x, _x.br.position.y, _x.br.position.z, _x.br.velocity.x, _x.br.velocity.y, _x.br.velocity.z, _x.br.force.x, _x.br.force.y, _x.br.force.z, _x.br.support_probability))
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
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.fl is None:
        self.fl = stoch3_msgs.msg.LegState()
      if self.fr is None:
        self.fr = stoch3_msgs.msg.LegState()
      if self.bl is None:
        self.bl = stoch3_msgs.msg.LegState()
      if self.br is None:
        self.br = stoch3_msgs.msg.LegState()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.fl.name = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.fl.name = str[start:end]
      _x = self
      start = end
      end += 80
      (_x.fl.position.x, _x.fl.position.y, _x.fl.position.z, _x.fl.velocity.x, _x.fl.velocity.y, _x.fl.velocity.z, _x.fl.force.x, _x.fl.force.y, _x.fl.force.z, _x.fl.support_probability,) = _get_struct_10d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.fr.name = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.fr.name = str[start:end]
      _x = self
      start = end
      end += 80
      (_x.fr.position.x, _x.fr.position.y, _x.fr.position.z, _x.fr.velocity.x, _x.fr.velocity.y, _x.fr.velocity.z, _x.fr.force.x, _x.fr.force.y, _x.fr.force.z, _x.fr.support_probability,) = _get_struct_10d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.bl.name = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.bl.name = str[start:end]
      _x = self
      start = end
      end += 80
      (_x.bl.position.x, _x.bl.position.y, _x.bl.position.z, _x.bl.velocity.x, _x.bl.velocity.y, _x.bl.velocity.z, _x.bl.force.x, _x.bl.force.y, _x.bl.force.z, _x.bl.support_probability,) = _get_struct_10d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.br.name = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.br.name = str[start:end]
      _x = self
      start = end
      end += 80
      (_x.br.position.x, _x.br.position.y, _x.br.position.z, _x.br.velocity.x, _x.br.velocity.y, _x.br.velocity.z, _x.br.force.x, _x.br.force.y, _x.br.force.z, _x.br.support_probability,) = _get_struct_10d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_10d = None
def _get_struct_10d():
    global _struct_10d
    if _struct_10d is None:
        _struct_10d = struct.Struct("<10d")
    return _struct_10d
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
