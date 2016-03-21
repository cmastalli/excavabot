"""autogenerated by genpy from excavaROB_mechanism_controllers/DebugInfo.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class DebugInfo(genpy.Message):
  _md5sum = "6281356ce897e33da024b8eb319460f2"
  _type = "excavaROB_mechanism_controllers/DebugInfo"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64[] timing
int32 sequence
bool input_valid
float64 residual
"""
  __slots__ = ['timing','sequence','input_valid','residual']
  _slot_types = ['float64[]','int32','bool','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       timing,sequence,input_valid,residual

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(DebugInfo, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.timing is None:
        self.timing = []
      if self.sequence is None:
        self.sequence = 0
      if self.input_valid is None:
        self.input_valid = False
      if self.residual is None:
        self.residual = 0.
    else:
      self.timing = []
      self.sequence = 0
      self.input_valid = False
      self.residual = 0.

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
      length = len(self.timing)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.timing))
      _x = self
      buff.write(_struct_iBd.pack(_x.sequence, _x.input_valid, _x.residual))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.timing = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 13
      (_x.sequence, _x.input_valid, _x.residual,) = _struct_iBd.unpack(str[start:end])
      self.input_valid = bool(self.input_valid)
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
      length = len(self.timing)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.timing.tostring())
      _x = self
      buff.write(_struct_iBd.pack(_x.sequence, _x.input_valid, _x.residual))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.timing = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      _x = self
      start = end
      end += 13
      (_x.sequence, _x.input_valid, _x.residual,) = _struct_iBd.unpack(str[start:end])
      self.input_valid = bool(self.input_valid)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_iBd = struct.Struct("<iBd")
