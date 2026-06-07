# service_interfaces

This package only defines ROS 2 service types.

The current `inspire_hand_modbus_ros2` driver implements:

- `Setpos`
- `Setforce`
- `Setspeed`
- `Getposact`

The other service definitions are kept as reserved/vendor API declarations for
compatibility and future driver support. A service type existing in this package
does not mean the current driver creates a matching ROS service endpoint.
