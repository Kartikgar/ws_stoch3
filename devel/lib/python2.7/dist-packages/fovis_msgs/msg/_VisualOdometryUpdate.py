# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from fovis_msgs/VisualOdometryUpdate.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy
import geometry_msgs.msg
import std_msgs.msg

class VisualOdometryUpdate(genpy.Message):
  _md5sum = "86e4e9402396b9789cfeaadd317551b4"
  _type = "fovis_msgs/VisualOdometryUpdate"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """Header header
time curr_timestamp
time prev_timestamp
geometry_msgs/Transform relative_transform
float64[36] covariance
uint8 estimate_status

uint8 NO_DATA = 0
uint8 ESTIMATE_VALID = 1
uint8 ESTIMATE_INSUFFICIENT_FEATURES = 2
uint8 ESTIMATE_DEGENERATE = 3
uint8 ESTIMATE_REPROJECTION_ERROR = 4


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
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

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
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
"""
  # Pseudo-constants
  NO_DATA = 0
  ESTIMATE_VALID = 1
  ESTIMATE_INSUFFICIENT_FEATURES = 2
  ESTIMATE_DEGENERATE = 3
  ESTIMATE_REPROJECTION_ERROR = 4

  __slots__ = ['header','curr_timestamp','prev_timestamp','relative_transform','covariance','estimate_status']
  _slot_types = ['std_msgs/Header','time','time','geometry_msgs/Transform','float64[36]','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,curr_timestamp,prev_timestamp,relative_transform,covariance,estimate_status

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(VisualOdometryUpdate, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.curr_timestamp is None:
        self.curr_timestamp = genpy.Time()
      if self.prev_timestamp is None:
        self.prev_timestamp = genpy.Time()
      if self.relative_transform is None:
        self.relative_transform = geometry_msgs.msg.Transform()
      if self.covariance is None:
        self.covariance = [0.] * 36
      if self.estimate_status is None:
        self.estimate_status = 0
    else:
      self.header = std_msgs.msg.Header()
      self.curr_timestamp = genpy.Time()
      self.prev_timestamp = genpy.Time()
      self.relative_transform = geometry_msgs.msg.Transform()
      self.covariance = [0.] * 36
      self.estimate_status = 0

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
      _x = self
      buff.write(_get_struct_4I7d().pack(_x.curr_timestamp.secs, _x.curr_timestamp.nsecs, _x.prev_timestamp.secs, _x.prev_timestamp.nsecs, _x.relative_transform.translation.x, _x.relative_transform.translation.y, _x.relative_transform.translation.z, _x.relative_transform.rotation.x, _x.relative_transform.rotation.y, _x.relative_transform.rotation.z, _x.relative_transform.rotation.w))
      buff.write(_get_struct_36d().pack(*self.covariance))
      _x = self.estimate_status
      buff.write(_get_struct_B().pack(_x))
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
      if self.curr_timestamp is None:
        self.curr_timestamp = genpy.Time()
      if self.prev_timestamp is None:
        self.prev_timestamp = genpy.Time()
      if self.relative_transform is None:
        self.relative_transform = geometry_msgs.msg.Transform()
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
      _x = self
      start = end
      end += 72
      (_x.curr_timestamp.secs, _x.curr_timestamp.nsecs, _x.prev_timestamp.secs, _x.prev_timestamp.nsecs, _x.relative_transform.translation.x, _x.relative_transform.translation.y, _x.relative_transform.translation.z, _x.relative_transform.rotation.x, _x.relative_transform.rotation.y, _x.relative_transform.rotation.z, _x.relative_transform.rotation.w,) = _get_struct_4I7d().unpack(str[start:end])
      start = end
      end += 288
      self.covariance = _get_struct_36d().unpack(str[start:end])
      start = end
      end += 1
      (self.estimate_status,) = _get_struct_B().unpack(str[start:end])
      self.curr_timestamp.canon()
      self.prev_timestamp.canon()
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
      _x = self
      buff.write(_get_struct_4I7d().pack(_x.curr_timestamp.secs, _x.curr_timestamp.nsecs, _x.prev_timestamp.secs, _x.prev_timestamp.nsecs, _x.relative_transform.translation.x, _x.relative_transform.translation.y, _x.relative_transform.translation.z, _x.relative_transform.rotation.x, _x.relative_transform.rotation.y, _x.relative_transform.rotation.z, _x.relative_transform.rotation.w))
      buff.write(self.covariance.tostring())
      _x = self.estimate_status
      buff.write(_get_struct_B().pack(_x))
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
      if self.curr_timestamp is None:
        self.curr_timestamp = genpy.Time()
      if self.prev_timestamp is None:
        self.prev_timestamp = genpy.Time()
      if self.relative_transform is None:
        self.relative_transform = geometry_msgs.msg.Transform()
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
      _x = self
      start = end
      end += 72
      (_x.curr_timestamp.secs, _x.curr_timestamp.nsecs, _x.prev_timestamp.secs, _x.prev_timestamp.nsecs, _x.relative_transform.translation.x, _x.relative_transform.translation.y, _x.relative_transform.translation.z, _x.relative_transform.rotation.x, _x.relative_transform.rotation.y, _x.relative_transform.rotation.z, _x.relative_transform.rotation.w,) = _get_struct_4I7d().unpack(str[start:end])
      start = end
      end += 288
      self.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
      start = end
      end += 1
      (self.estimate_status,) = _get_struct_B().unpack(str[start:end])
      self.curr_timestamp.canon()
      self.prev_timestamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_36d = None
def _get_struct_36d():
    global _struct_36d
    if _struct_36d is None:
        _struct_36d = struct.Struct("<36d")
    return _struct_36d
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_4I7d = None
def _get_struct_4I7d():
    global _struct_4I7d
    if _struct_4I7d is None:
        _struct_4I7d = struct.Struct("<4I7d")
    return _struct_4I7d
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
