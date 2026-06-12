"""robot_radio.calibration — shared calibration helpers and interactive logic.

Package structure:

    helpers.py   — Pure math helpers: scale_to_int8, int8_to_scale, mean_stdev,
                   deep_merge, save_config, resolve_save_path.  No hardware
                   dependencies; fully unit-testable.

    push.py      — push_calibration(conn_or_proto, config) → dict.
                   Resolves the interface duality between NezhaProtocol (MCP path)
                   and SerialConnection (CLI path): delegates to proto.push_calibration
                   when a NezhaProtocol is available, otherwise builds SET commands
                   directly on the connection.

    angular.py   — calibrate_turns(conn, config, ...) — interactive turns
                   calibration logic.  No CLI arg-parsing; the caller handles
                   argparse and calls this function.

    linear.py    — calibrate_distance(conn, config, ...) — interactive distance
                   calibration logic.  Same separation as angular.py.
"""
