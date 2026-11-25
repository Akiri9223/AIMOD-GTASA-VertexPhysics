"""Show the inertia curve: hair lag over time for a 0 -> 6 m/s ramp.

Current solver: the lag grows with speed and stays (permanent, speed
proportional).  With frame transport: a transient bump during the
acceleration, then it settles back to the rest droop.

Usage: python tools/hair_inertia_curve.py <dff>
"""

import importlib.util
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    "hair_sim_verify", os.path.join(HERE, "hair_sim_verify.py"))
sim = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sim)


def main(path):
    rig = sim.build(path)

    for label, kwargs in (
        ("current (capsules, consensus)", dict(use_capsules=True,
                                              capsule_fix=True,
                                              pin_consensus=True)),
        ("+ frame transport", dict(use_capsules=True, capsule_fix=True,
                                   pin_consensus=True, fix_transport=True)),
    ):
        pos, anc, hist = sim.simulate(rig, 240, speed=6.0, record=True, **kwargs)
        peak = hist.max()
        peak_at = hist.argmax() / 60.0
        print(f"\n{label}")
        print(f"  lag mean at 1s={hist[60]*1000:6.1f}mm  "
              f"2s={hist[120]*1000:6.1f}mm  4s={hist[-1]*1000:6.1f}mm")
        print(f"  peak lag = {peak*1000:6.1f}mm at t={peak_at:.2f}s")
        print(f"  first 1.2s (mm): " +
              " ".join(f"{v*1000:.0f}" for v in hist[:72:4]))


if __name__ == "__main__":
    main(sys.argv[1])
