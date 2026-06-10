#!/usr/bin/env python3
"""sim_rt_verify.py — verify the firmware RT (relative encoder-arc turn) in sim.

Sends `RT <cdeg>` and measures the simulator's TRUE body yaw (ExactPoseTracker),
accumulating incremental yaw so it is unwrap-safe past 180°. Also waits for the
`EVT done RT` so we confirm the command actually terminates on its own.

  uv run python tests/dev/sim_rt_verify.py
"""
import math
import sys


def main() -> int:
    from robot_radio.io.sim_conn import SimConnection

    conn = SimConnection()
    res = conn.connect()
    if "error" in res:
        print(res["error"])
        return 1
    conn.send("SET sTimeout=60000")

    print(f"  {'cmd°':>6} {'TRUE°':>9} {'fw_odom°':>9} {'err°':>7} {'evt':>6} {'creep°':>8}")
    for cmd_deg in (45.0, 90.0, 135.0, 180.0, -90.0, 360.0):
        cmd_cdeg = int(round(cmd_deg * 100))
        prev = conn.get_exact_pose()["h"]
        accum = 0.0
        conn.send(f"RT {cmd_cdeg}", read_ms=50, stop_token="OK")
        done = False
        t = 0
        while t < 9000:
            lines = conn.tick(50)
            t += 50
            h = conn.get_exact_pose()["h"]
            d = math.atan2(math.sin(h - prev), math.cos(h - prev))
            accum += d
            prev = h
            if any("done RT" in ln for ln in lines):
                done = True
                break
        # NO masking X — let RT stop itself. Settle, then keep watching to
        # confirm the robot is actually stopped (yaw flat = motors off).
        for _ in range(6):
            conn.tick(50)
            h = conn.get_exact_pose()["h"]
            accum += math.atan2(math.sin(h - prev), math.cos(h - prev))
            prev = h
        true_deg = math.degrees(accum)
        # Stopped-check: 1 s more with no commands; how much more does it turn?
        yaw_at_done = prev
        for _ in range(20):
            conn.tick(50)
            h = conn.get_exact_pose()["h"]
            prev = h
        creep_deg = math.degrees(math.atan2(math.sin(prev - yaw_at_done),
                                            math.cos(prev - yaw_at_done)))
        fw_deg = math.degrees(conn.get_state()["pose_h"])
        err = true_deg - cmd_deg
        flag = "" if abs(creep_deg) < 1.0 else "  <-- STILL MOVING"
        print(f"  {cmd_deg:6.0f} {true_deg:9.1f} {fw_deg:9.1f} {err:7.1f} "
              f"{'yes' if done else 'NO':>6} {creep_deg:7.1f}{flag}")

    conn.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
