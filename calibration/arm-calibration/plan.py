"""Plan a safe single-step joint sweep for arm calibration from the vendor model (x forward, y up, metres)."""
import math, json, itertools, collections, sys

SH = (0.0475, 0.14); L1, L2 = 0.1775, 0.205
LIM = {"m3": (90, 270), "m4": (35, 170), "m5": (0, 155)}
START = (150, 45, 0)


def fk(m3, m4):
    a1 = math.radians(270 - m3); a2 = math.radians(m4 - m3 + 90)
    elbow = (SH[0] + L1 * math.cos(a1), SH[1] + L1 * math.sin(a1))
    wrist = (elbow[0] + L2 * math.cos(a2), elbow[1] + L2 * math.sin(a2))
    return elbow, wrist


def safe(m3, m4, m5, floor=0.06):
    if not (LIM["m3"][0] <= m3 <= LIM["m3"][1] and LIM["m4"][0] <= m4 <= LIM["m4"][1] and LIM["m5"][0] <= m5 <= LIM["m5"][1]):
        return False
    elbow, wrist = fk(m3, m4)
    for p in (elbow, wrist):
        if p[1] < floor: return False                     # keep wrist/elbow well above the floor (fingers hang below)
        if abs(p[0]) < 0.24 and p[1] < 0.14: return False  # inside the body
    return True


def neighbours(p):
    for j in range(3):
        for s in (-10, 10):
            q = list(p); q[j] += s; yield tuple(q), [s // 10 if k == j else 0 for k in range(3)]


def reachable(floor):
    seen = {START: None}; dq = collections.deque([START])
    while dq:
        p = dq.popleft()
        for q, step in neighbours(p):
            if q not in seen and safe(*q, floor=floor):
                seen[q] = (p, step); dq.append(q)
    return seen


if __name__ == "__main__":
    floor = float(sys.argv[1]) if len(sys.argv) > 1 else 0.06
    seen = reachable(floor)
    pts = sorted({(p[0], p[1]) for p in seen})
    print(f"{len(seen)} reachable poses, {len(pts)} distinct (m3,m4)")
    for m3, m4 in pts:
        e, w = fk(m3, m4)
        print(f"m3={m3:3d} m4={m4:3d} wrist=({w[0]:.3f},{w[1]:.3f}) elbow=({e[0]:.3f},{e[1]:.3f})")


def path(seen_fn, a, b, floor):
    """Shortest single-step path a->b inside the safe set."""
    prev = {a: None}; dq = collections.deque([a])
    while dq:
        p = dq.popleft()
        if p == b: break
        for q, step in neighbours(p):
            if q not in prev and safe(*q, floor=floor):
                prev[q] = (p, step); dq.append(q)
    steps = []; p = b
    while prev[p] is not None:
        p0, s = prev[p]; steps.append(s); p = p0
    return steps[::-1]


def sweep(floor=0.06):
    targets = [(150, 45, 0)]
    for m3, m4 in [(170, 65), (190, 75), (200, 95), (210, 85), (220, 105), (230, 105), (230, 115), (240, 105), (240, 115),
                   (240, 125), (250, 125), (250, 135), (260, 145), (260, 155), (250, 155), (240, 135), (220, 95), (200, 75), (180, 65)]:
        if safe(m3, m4, 0, floor):
            targets.append((m3, m4, 0))
    # wrist-angle variations at two low poses (gripper camera tilts toward the floor)
    for m5 in (30, 60, 90):
        targets.insert(9, (240, 115, m5))
    for m5 in (40, 80):
        targets.insert(14, (250, 135, m5))
    targets.append((150, 45, 0))
    plan, cur = [], START
    for t in targets[1:]:
        if not safe(*t, floor=floor): continue
        for s in path(None, cur, t, floor):
            plan.append({"move": s})
        plan.append({"capture": list(t), "wrist": fk(t[0], t[1])[1]})
        cur = t
    return plan


def write_plan(fn="sweep.json", floor=0.06):
    plan = sweep(floor)
    json.dump(plan, open(fn, "w"), indent=1)
    return plan
