# generated from rosidl_generator_py/resource/_idl.py.em
# with input from karelics_vesc_can_driver:msg/VescStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_VescStatus(type):
    """Metaclass of message 'VescStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('karelics_vesc_can_driver')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'karelics_vesc_can_driver.msg.VescStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__vesc_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__vesc_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__vesc_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__vesc_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__vesc_status

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class VescStatus(metaclass=Metaclass_VescStatus):
    """Message class 'VescStatus'."""

    __slots__ = [
        '_header',
        '_erpm',
        '_rpm',
        '_duty_cycle',
        '_current',
        '_amp_hours',
        '_amp_hours_charged',
        '_watt_hours',
        '_watt_hours_charged',
        '_temp_fet',
        '_temp_motor',
        '_current_in',
        '_pid_pos_now',
        '_tacho_value',
        '_v_in',
        '_rotations',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'erpm': 'int32',
        'rpm': 'int32',
        'duty_cycle': 'float',
        'current': 'float',
        'amp_hours': 'float',
        'amp_hours_charged': 'float',
        'watt_hours': 'float',
        'watt_hours_charged': 'float',
        'temp_fet': 'float',
        'temp_motor': 'float',
        'current_in': 'float',
        'pid_pos_now': 'float',
        'tacho_value': 'float',
        'v_in': 'float',
        'rotations': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.erpm = kwargs.get('erpm', int())
        self.rpm = kwargs.get('rpm', int())
        self.duty_cycle = kwargs.get('duty_cycle', float())
        self.current = kwargs.get('current', float())
        self.amp_hours = kwargs.get('amp_hours', float())
        self.amp_hours_charged = kwargs.get('amp_hours_charged', float())
        self.watt_hours = kwargs.get('watt_hours', float())
        self.watt_hours_charged = kwargs.get('watt_hours_charged', float())
        self.temp_fet = kwargs.get('temp_fet', float())
        self.temp_motor = kwargs.get('temp_motor', float())
        self.current_in = kwargs.get('current_in', float())
        self.pid_pos_now = kwargs.get('pid_pos_now', float())
        self.tacho_value = kwargs.get('tacho_value', float())
        self.v_in = kwargs.get('v_in', float())
        self.rotations = kwargs.get('rotations', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.erpm != other.erpm:
            return False
        if self.rpm != other.rpm:
            return False
        if self.duty_cycle != other.duty_cycle:
            return False
        if self.current != other.current:
            return False
        if self.amp_hours != other.amp_hours:
            return False
        if self.amp_hours_charged != other.amp_hours_charged:
            return False
        if self.watt_hours != other.watt_hours:
            return False
        if self.watt_hours_charged != other.watt_hours_charged:
            return False
        if self.temp_fet != other.temp_fet:
            return False
        if self.temp_motor != other.temp_motor:
            return False
        if self.current_in != other.current_in:
            return False
        if self.pid_pos_now != other.pid_pos_now:
            return False
        if self.tacho_value != other.tacho_value:
            return False
        if self.v_in != other.v_in:
            return False
        if self.rotations != other.rotations:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @property
    def erpm(self):
        """Message field 'erpm'."""
        return self._erpm

    @erpm.setter
    def erpm(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'erpm' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'erpm' field must be an integer in [-2147483648, 2147483647]"
        self._erpm = value

    @property
    def rpm(self):
        """Message field 'rpm'."""
        return self._rpm

    @rpm.setter
    def rpm(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rpm' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'rpm' field must be an integer in [-2147483648, 2147483647]"
        self._rpm = value

    @property
    def duty_cycle(self):
        """Message field 'duty_cycle'."""
        return self._duty_cycle

    @duty_cycle.setter
    def duty_cycle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'duty_cycle' field must be of type 'float'"
        self._duty_cycle = value

    @property
    def current(self):
        """Message field 'current'."""
        return self._current

    @current.setter
    def current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current' field must be of type 'float'"
        self._current = value

    @property
    def amp_hours(self):
        """Message field 'amp_hours'."""
        return self._amp_hours

    @amp_hours.setter
    def amp_hours(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'amp_hours' field must be of type 'float'"
        self._amp_hours = value

    @property
    def amp_hours_charged(self):
        """Message field 'amp_hours_charged'."""
        return self._amp_hours_charged

    @amp_hours_charged.setter
    def amp_hours_charged(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'amp_hours_charged' field must be of type 'float'"
        self._amp_hours_charged = value

    @property
    def watt_hours(self):
        """Message field 'watt_hours'."""
        return self._watt_hours

    @watt_hours.setter
    def watt_hours(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'watt_hours' field must be of type 'float'"
        self._watt_hours = value

    @property
    def watt_hours_charged(self):
        """Message field 'watt_hours_charged'."""
        return self._watt_hours_charged

    @watt_hours_charged.setter
    def watt_hours_charged(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'watt_hours_charged' field must be of type 'float'"
        self._watt_hours_charged = value

    @property
    def temp_fet(self):
        """Message field 'temp_fet'."""
        return self._temp_fet

    @temp_fet.setter
    def temp_fet(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'temp_fet' field must be of type 'float'"
        self._temp_fet = value

    @property
    def temp_motor(self):
        """Message field 'temp_motor'."""
        return self._temp_motor

    @temp_motor.setter
    def temp_motor(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'temp_motor' field must be of type 'float'"
        self._temp_motor = value

    @property
    def current_in(self):
        """Message field 'current_in'."""
        return self._current_in

    @current_in.setter
    def current_in(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_in' field must be of type 'float'"
        self._current_in = value

    @property
    def pid_pos_now(self):
        """Message field 'pid_pos_now'."""
        return self._pid_pos_now

    @pid_pos_now.setter
    def pid_pos_now(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pid_pos_now' field must be of type 'float'"
        self._pid_pos_now = value

    @property
    def tacho_value(self):
        """Message field 'tacho_value'."""
        return self._tacho_value

    @tacho_value.setter
    def tacho_value(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tacho_value' field must be of type 'float'"
        self._tacho_value = value

    @property
    def v_in(self):
        """Message field 'v_in'."""
        return self._v_in

    @v_in.setter
    def v_in(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'v_in' field must be of type 'float'"
        self._v_in = value

    @property
    def rotations(self):
        """Message field 'rotations'."""
        return self._rotations

    @rotations.setter
    def rotations(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rotations' field must be of type 'float'"
        self._rotations = value
