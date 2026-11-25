"""Offline check of the new swept-capsule CCD vs the old one.

Mirrors the C++ geometry:
  * thigh capsule hip->knee, 0.45 m long, r = 0.11
  * one frame: thigh swings 45 deg about the hip (running stride)
  * cloth particle sits just in front of / beside the thigh and moves with the
    body (as a skirt particle does)

Old implementation:
  * caller skips the sweep when |(pos-prev) - (aWorld-aPrevWorld)| < r*0.5
  * sampling uses the END-pose capsule for every sample
New implementation:
  * sub-steps sized by max(capsule motion, particle motion)
  * interpolated capsule per sub-step, particle slides on the surface
"""

import math


def sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def add(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def mul(a, s): return (a[0] * s, a[1] * s, a[2] * s)
def dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def length(a): return math.sqrt(dot(a, a))


def closest_seg(p, a, b):
    ab = sub(b, a)
    ll = dot(ab, ab)
    if ll < 1e-10:
        return a, length(sub(p, a))
    t = max(0.0, min(1.0, dot(sub(p, a), ab) / ll))
    c = add(a, mul(ab, t))
    return c, length(sub(p, c))


def resolve(p, a, b, r):
    c, d = closest_seg(p, a, b)
    if d >= r:
        return p, 0.0
    n = mul(sub(p, c), 1.0 / d) if d > 1e-5 else (1.0, 0.0, 0.0)
    return add(c, mul(n, r)), r - d


def old_ccd(pos, prev, a_prev, b_prev, a_now, b_now, r):
    cap_move = sub(a_now, a_prev)
    rel = sub(sub(pos, prev), cap_move)
    if dot(rel, rel) < r * r * 0.25:
        return pos, 0, "skipped by relMove threshold"

    delta = sub(pos, prev)
    ln = length(delta)
    steps = max(1, min(8, 1 + int(ln / max(r * 0.5, 0.01))))
    cur = pos
    hits = 0
    for s in range(1, steps + 1):
        k = s / steps
        sample = add(prev, mul(delta, k))
        c, d = closest_seg(sample, a_now, b_now)   # end pose only
        if d >= r:
            continue
        n = mul(sub(sample, c), 1.0 / d)
        cur = add(c, mul(n, r))
        hits += 1
        break
    return cur, hits, ""


def new_ccd(pos, prev, a_prev, b_prev, a_now, b_now, r):
    cap_move = max(length(sub(a_now, a_prev)), length(sub(b_now, b_prev)))
    if cap_move > 1.2:
        return pos, 0, "teleport"
    p_move = sub(pos, prev)
    travel = length(p_move)

    _, d0 = closest_seg(prev, a_prev, b_prev)
    _, d1 = closest_seg(pos, a_now, b_now)
    if min(d0, d1) > r + cap_move + travel + 0.01:
        return pos, 0, "culled"

    step_len = max(r * 0.4, 0.008)
    steps = max(1, min(16, 1 + int(max(cap_move, travel) / step_len)))

    q, _ = resolve(prev, a_prev, b_prev, r)
    prev_target = prev
    hits = 0
    for i in range(1, steps + 1):
        k = i / steps
        target = add(prev, mul(p_move, k))
        q = add(q, sub(target, prev_target))
        prev_target = target
        A = add(a_prev, mul(sub(a_now, a_prev), k))
        B = add(b_prev, mul(sub(b_now, b_prev), k))
        q, pen = resolve(q, A, B, r)
        if pen > 0:
            hits += 1
    return q, hits, ""


def dist_report(p, a, b, r):
    c, d = closest_seg(p, a, b)
    return f"d={d:.3f} r={r:.3f} {'INSIDE' if d < r else 'outside'}"


def main():
    R = 0.11
    L = 0.45
    hip = (0.0, 0.0, 0.0)

    def knee(deg):
        t = math.radians(deg)
        return (L * math.sin(t), 0.0, -L * math.cos(t))

    def dense_crossed(prev, pos, a0, b0, a1, b1, r, n=240):
        """Brute-force reference: does the particle path ever enter the capsule?"""
        worst = 0.0
        for i in range(1, n + 1):
            k = i / n
            sample = add(prev, mul(sub(pos, prev), k))
            A = add(a0, mul(sub(a1, a0), k))
            B = add(b0, mul(sub(b1, b0), k))
            _, d = closest_seg(sample, A, B)
            worst = max(worst, r - d)
        return worst

    print("=== case 1: thigh swings 45 deg while the whole body moves forward ===")
    print("    (this is the real running situation: hip translates, knee arcs)")
    for deg in (15, 30, 45, 60):
        body = (0.0, 0.06, 0.0)          # pelvis moved 6 cm this frame
        a0, b0 = hip, knee(0)
        a1, b1 = add(hip, body), add(knee(deg), body)
        prev = (0.14, 0.0, -0.22)        # cloth in front of mid-thigh
        pos = add(prev, body)            # cloth rides with the body

        o, oh, omsg = old_ccd(pos, prev, a0, b0, a1, b1, R)
        n, nh, nmsg = new_ccd(pos, prev, a0, b0, a1, b1, R)
        print(f"  swing {deg:2d} deg")
        print(f"    old: {omsg or 'ran'} hits={oh} moved={length(sub(o, pos)):.4f} m")
        print(f"    new: {nmsg or 'ran'} hits={nh} moved={length(sub(n, pos)):.4f} m")

    print("\n=== case 2: leg sweeps PAST a particle (start and end both outside) ===")
    print("    particle at 22.5 deg, 0.50 m from the hip; leg goes 0 -> 45 deg")
    body = (0.0, 0.02, 0.0)
    a0, b0 = hip, knee(0)
    a1, b1 = add(hip, body), add(knee(45), body)
    prev = (0.191, 0.0, -0.462)
    pos = add(prev, body)

    pen = dense_crossed(prev, pos, a0, b0, a1, b1, R)
    print(f"    reference: deepest penetration during the frame = {pen:.4f} m")

    o, oh, omsg = old_ccd(pos, prev, a0, b0, a1, b1, R)
    n, nh, nmsg = new_ccd(pos, prev, a0, b0, a1, b1, R)
    print(f"    old: {omsg or 'ran'} hits={oh} moved={length(sub(o, pos)):.4f} m <- tunnelled")
    print(f"    new: {nmsg or 'ran'} hits={nh} moved={length(sub(n, pos)):.4f} m <- pushed out")
    print(f"    new end {dist_report(n, a1, b1, R)}")

    print("\n=== case 2: pure translation of the whole body (no swing) ===")
    prev = (0.14, 0.0, -0.22)
    pos = add(prev, (0.30, 0.0, 0.0))
    # hip/knee both translate 0.30 in x, so relative motion == 0
    a0, b0 = hip, knee(0)
    a1 = add(a0, (0.30, 0.0, 0.0))
    b1 = add(b0, (0.30, 0.0, 0.0))
    o, oh, _ = old_ccd(pos, prev, a0, b0, a1, b1, R)
    n, nh, nmsg = new_ccd(pos, prev, a0, b0, a1, b1, R)
    print(f"  old hits={oh} new hits={nh} ({nmsg or 'ran'}) — both should be 0")
    print(f"  new displacement = {length(sub(n, pos)):.4f} m")

    print("\n=== case 3: fast particle far from the body must not be touched ===")
    prev = (2.0, 0.0, -0.3)
    pos = add(prev, (0.0, 0.25, 0.0))
    n, nh, nmsg = new_ccd(pos, prev, hip, knee(0), hip, knee(60), R)
    print(f"  hits={nh} cull='{nmsg}' displacement={length(sub(n, pos)):.6f} m")

    print("\n=== case 4: particle already inside at frame start ===")
    prev = (0.03, 0.0, -0.20)   # only 0.03 from the axis
    pos = add(prev, (0.0, 0.02, 0.0))
    n, nh, nmsg = new_ccd(pos, prev, hip, knee(0), hip, knee(20), R)
    print(f"  hits={nh} end {dist_report(n, hip, knee(20), R)}")

    print("\n=== case 5: degenerate — particle exactly on the axis ===")
    prev = (0.0, 0.0, -0.20)
    n, nh, nmsg = new_ccd(prev, prev, hip, knee(0), hip, knee(0), R)
    print(f"  hits={nh} end {dist_report(n, hip, knee(0), R)}")


if __name__ == "__main__":
    main()
