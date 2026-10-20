"""go2-ble-census — passive BLE/BT advertisement census for the Go2.

The only thing in here that can be wrong *quietly* is the advertisement
decoder, so it lives in `ble_scan` as pure functions with no radio, no ROS and
no clock, and is pinned by tests before any hardware exists.
"""
