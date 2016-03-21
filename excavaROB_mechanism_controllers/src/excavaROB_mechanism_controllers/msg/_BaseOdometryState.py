"""autogenerated by genpy from excavaROB_mechanism_controllers/BaseOdometryState.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class BaseOdometryState(genpy.Message):
  _md5sum = "c0c3716bf92e25147d70d33666638b37"
  _type = "excavaROB_mechanism_controllers/BaseOdometryState"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """geometry_msgs/Twist velocity
string[] right_wheel_link_names
string[] left_wheel_link_names


================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z
"""
  __slots__ = ['velocity','right_wheel_link_names','left_wheel_link_names']
  _slot_types = ['geometry_msgs/Twist','string[]','string[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       velocity,right_wheel_link_names,left_wheel_link_names

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(BaseOdometryState, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.velocity is None:
        self.velocity = geometry_msgs.msg.Twist()
      if self.right_wheel_link_names is None:
        self.right_wheel_link_names = []
      if self.left_wheel_link_names is None:
        self.left_wheel_link_names = []
    else:
      self.velocity = geometry_msgs.msg.Twist()
      self.right_wheel_link_names = []
      self.left_wheel_link_names = []

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
      buff.write(_struct_6d.pack(_x.velocity.linear.x, _x.velocity.linear.y, _x.velocity.linear.z, _x.velocity.angular.x, _x.velocity.angular.y, _x.velocity.angular.z))
      length = len(self.right_wheel_link_names)
      buff.write(_struct_I.pack(length))
      for val1 in self.right_wheel_link_names:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      length = len(self.left_wheel_link_names)
      buff.write(_struct_I.pack(length))
      for val1 in self.left_wheel_link_names:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.velocity is None:
        self.velocity = geometry_msgs.msg.Twist()
      end = 0
      _x = self
      start = end
      end += 48
      (_x.velocity.linear.x, _x.velocity.linear.y, _x.velocity.linear.z, _x.velocity.angular.x, _x.velocity.angular.y, _x.velocity.angular.z,) = _struct_6d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.right_wheel_link_names = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.right_wheel_link_names.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.left_wheel_link_names = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.left_wheel_link_names.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_6d.pack(_x.velocity.linear.x, _x.velocity.linear.y, _x.velocity.linear.z, _x.velocity.angular.x, _x.velocity.angular.y, _x.velocity.angular.z))
      length = len(self.right_wheel_link_names)
      buff.write(_struct_I.pack(length))
      for val1 in self.right_wheel_link_names:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      length = len(self.left_wheel_link_names)
      buff.write(_struct_I.pack(length))
      for val1 in self.left_wheel_link_names:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.velocity is None:
        self.velocity = geometry_msgs.msg.Twist()
      end = 0
      _x = self
      start = end
      end += 48
      (_x.velocity.linear.x, _x.velocity.linear.y, _x.velocity.linear.z, _x.velocity.angular.x, _x.velocity.angular.y, _x.velocity.angular.z,) = _struct_6d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.right_wheel_link_names = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.right_wheel_link_names.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.left_wheel_link_names = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.left_wheel_link_names.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_6d = struct.Struct("<6d")
