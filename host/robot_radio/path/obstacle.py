"""Obstacle-avoidance path planning using visibility graphs and Catmull-Rom splines.

Extracted from ``test/old/track_tag.py`` (sprint 006, ticket 004).
"""

from __future__ import annotations

import heapq
import math

from robot_radio.path.catmull_rom import catmull_rom


def segment_clears(a, b, obstacles, clearance_cm):
    """True if segment a→b stays at least clearance_cm from every obstacle circle."""
    ax, ay = a; bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx*dx + dy*dy
    for ox, oy, r in obstacles:
        if L2 < 1e-9:
            if math.hypot(ax - ox, ay - oy) < r + clearance_cm:
                return False
            continue
        t = max(0.0, min(1.0, ((ox - ax)*dx + (oy - ay)*dy) / L2))
        if math.hypot(ax + t*dx - ox, ay + t*dy - oy) < r + clearance_cm:
            return False
    return True


# Keep the private name as an alias for internal use within this module
_segment_clears = segment_clears


def plan_path(start, goal, obstacles, clearance_cm=5.0):
    """Visibility-graph path from start to goal avoiding circular obstacles.

    Each obstacle is (cx, cy, radius_cm).  Returns a waypoint list including
    start and goal.  Falls back to a direct segment if no clear path is found.
    """
    if _segment_clears(start, goal, obstacles, clearance_cm):
        return [start, goal]

    # Candidate nodes: start, goal, + tangent bypass points around each obstacle
    nodes = [start, goal]
    bypass_r_extra = 2.0  # small extra margin beyond clearance
    for ox, oy, r in obstacles:
        bypass_r = r + clearance_cm + bypass_r_extra
        # Bypass perpendicular to start→goal axis
        dx, dy = goal[0] - start[0], goal[1] - start[1]
        L = math.hypot(dx, dy)
        if L > 1e-6:
            px, py = -dy / L, dx / L
            nodes.append((ox + bypass_r * px, oy + bypass_r * py))
            nodes.append((ox - bypass_r * px, oy - bypass_r * py))
        # Bypass perpendicular to obstacle→goal axis
        dx2, dy2 = goal[0] - ox, goal[1] - oy
        L2 = math.hypot(dx2, dy2)
        if L2 > 1e-6:
            px2, py2 = -dy2 / L2, dx2 / L2
            nodes.append((ox + bypass_r * px2, oy + bypass_r * py2))
            nodes.append((ox - bypass_r * px2, oy - bypass_r * py2))

    # Dijkstra on visibility graph
    n = len(nodes)
    INF = float('inf')
    best = [INF] * n
    best[0] = 0.0
    prev = [-1] * n
    heap = [(0.0, 0)]

    while heap:
        d, u = heapq.heappop(heap)
        if d > best[u]:
            continue
        if u == 1:
            break
        for v in range(n):
            if v == u:
                continue
            if _segment_clears(nodes[u], nodes[v], obstacles, clearance_cm):
                nd = d + math.hypot(nodes[u][0] - nodes[v][0],
                                    nodes[u][1] - nodes[v][1])
                if nd < best[v]:
                    best[v] = nd
                    prev[v] = u
                    heapq.heappush(heap, (nd, v))

    # Reconstruct
    path = []
    cur = 1
    while cur != -1:
        path.append(nodes[cur])
        cur = prev[cur]
    path.reverse()

    if len(path) < 2 or math.hypot(path[0][0] - start[0],
                                   path[0][1] - start[1]) > 0.01:
        return [start, goal]
    return path


def bypass_for_segment(a, b, obstacles, clearance_cm):
    """Return a bypass waypoint that routes segment a→b around the first blocking obstacle."""
    ax, ay = a; bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx*dx + dy*dy
    for ox, oy, r in obstacles:
        if L2 < 1e-9:
            continue
        t = max(0.0, min(1.0, ((ox-ax)*dx + (oy-ay)*dy) / L2))
        closest_x = ax + t*dx; closest_y = ay + t*dy
        if math.hypot(closest_x - ox, closest_y - oy) >= r + clearance_cm:
            continue
        # Obstacle is blocking — generate bypass point perpendicular to segment
        bypass_r = r + clearance_cm + 2.0
        L = math.sqrt(L2)
        px, py = -dy / L, dx / L
        for sign in (1.0, -1.0):
            bp = (ox + sign * bypass_r * px, oy + sign * bypass_r * py)
            if (_segment_clears(a, bp, obstacles, clearance_cm) and
                    _segment_clears(bp, b, obstacles, clearance_cm)):
                return bp
        # Neither side is fully clear — return the one with more clearance
        return (ox + bypass_r * px, oy + bypass_r * py)
    return None


# Keep the private name as an alias for internal use within this module
_bypass_for_segment = bypass_for_segment


def build_safe_spline(start, goal, obstacles, clearance_cm, samples_per_segment,
                      max_fix_iter=4):
    """Build a Catmull-Rom spline that keeps the curve itself clear of obstacles.

    Catmull-Rom shortcuts corners, so we iteratively check the spline segments
    and insert fix-up waypoints wherever the curve dips into an obstacle zone,
    then rebuild the spline.  Converges in 1-3 passes for typical field layouts.
    """
    wps = plan_path(start, goal, obstacles, clearance_cm)

    for _ in range(max_fix_iter):
        spline = catmull_rom(wps, samples_per_segment=samples_per_segment)

        # Find first spline segment that violates obstacle clearance
        bad_pt = None
        for i in range(len(spline) - 1):
            if not _segment_clears(spline[i], spline[i+1], obstacles, clearance_cm):
                bad_pt = spline[i]
                break

        if bad_pt is None:
            return spline, wps  # All segments clear

        # Find which consecutive waypoint pair contains this spline point
        best_dist = float('inf')
        insert_after = 0
        for j in range(len(wps) - 1):
            mx = (wps[j][0] + wps[j+1][0]) / 2
            my = (wps[j][1] + wps[j+1][1]) / 2
            d = math.hypot(bad_pt[0] - mx, bad_pt[1] - my)
            if d < best_dist:
                best_dist = d
                insert_after = j

        bp = _bypass_for_segment(wps[insert_after], wps[insert_after + 1],
                                  obstacles, clearance_cm)
        if bp is None:
            break
        wps.insert(insert_after + 1, bp)

    return catmull_rom(wps, samples_per_segment=samples_per_segment), wps
