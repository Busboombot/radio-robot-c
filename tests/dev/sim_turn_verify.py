#!/usr/bin/env python3
"""sim_turn_verify.py — verify turn geometry against the simulator's TRUTH.

Drives a pure in-place spin in the sim, watches the encoders, and at the
target per-wheel arc compares:
  - encoder-derived angle  (arc / mm_per_deg, using trackwidth TW)
  - firmware odometry yaw   (pose_h)
  - GROUND-TRUTH body yaw   (ExactPoseTracker, exact_pose_h)

If "encoder angle" == "true yaw", the trackwidth used to turn the wheel arc
into degrees matches the simulator's physical geometry. A constant ratio
between them is exactly the geometry error to correct.

Noise/slip are OFF — this is a pure kinematics check.

  uv run python tests/dev/sim_turn_verify.py
"""
import math
import sys


def main() -> int:
    from robot_radio.io.sim_conn import SimConnection

    TW = 126.0                                   # trackwidth used by the arc formula
    mm_per_deg = (TW / 2.0) * (math.pi / 180.0)
    SPEED = 60                                   # wheel mm/s (left -, right +  => CCW)

    conn = SimConnection()
    res = conn.connect()
    if "error" in res:
        print(res["error"])
        return 1
    conn.send("SET sTimeout=60000")              # no watchdog stop during the spin

    def exact_h():
        return conn.get_exact_pose()["h"]

    print(f"  TW={TW}  mm_per_deg={mm_per_deg:.4f}  speed={SPEED}mm/s   (noise/slip OFF)")
    print(f"  {'cmd°':>5} {'enc°':>8} {'fw_pose°':>9} {'TRUE°':>8} {'true/cmd':>9}")
    for cmd_deg in (45.0, 90.0, 135.0, 180.0):
        conn.set_enc(0.0, 0.0)
        conn.tick(50)
        e0l = conn.get_state()["enc_l"]
        e0r = conn.get_state()["enc_r"]
        h0 = exact_h()
        target_arc = cmd_deg * mm_per_deg

        conn.send("S -%d %d" % (SPEED, SPEED))   # spin CCW (left back, right fwd)
        t = 0
        while t < 30000:
            conn.tick(50)
            t += 50
            st = conn.get_state()
            prog = (abs(st["enc_l"] - e0l) + abs(st["enc_r"] - e0r)) / 2.0
            if prog >= target_arc:
                break
        conn.send("X")
        conn.tick(300)

        st = conn.get_state()
        prog = (abs(st["enc_l"] - e0l) + abs(st["enc_r"] - e0r)) / 2.0
        enc_deg = prog / mm_per_deg
        fw_deg = math.degrees(st["pose_h"])
        true_deg = math.degrees(exact_h() - h0)
        ratio = true_deg / cmd_deg if cmd_deg else 0.0
        print(f"  {cmd_deg:5.0f} {enc_deg:8.1f} {fw_deg:9.1f} {true_deg:8.1f} {ratio:9.3f}")

    conn.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
