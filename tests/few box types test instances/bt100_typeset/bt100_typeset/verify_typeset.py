"""
verify_typeset.py -- consistency check for the few-box-type set.

Checks, for every instance:
  * all copies of a box type are byte-identical in every attribute
    (dimensions, weight, group, family, zone, priority, seq, load bearing,
     fragility, orientation restriction, volume, volume category),
  * the number of types is 5..10 and each type is used at least once,
  * the box count is 10..100 and matches meta.n and the multiplicities,
  * types differ from each other in their grid dimensions,
  * the recorded lower bounds match a fresh recomputation,
  * the stored reference packing is feasible (containment, no overlap,
    >= 75 % support, payload, height, no chemical above food).

Usage:  python verify_typeset.py [dir]
"""

import glob
import json
import os
import sys

from pack_heuristic import bounds, verify

ATTRS = ("w_mm", "d_mm", "h_mm", "w_g", "d_g", "h_g", "weight_kg", "group",
         "family", "zone", "priority", "seq", "load_bearing_kpa", "fragile",
         "upright_only", "volume_dm3", "volume_category")


def main(root="."):
    packs = json.load(open(os.path.join(root, "reference_packings.json"),
                          encoding="utf-8"))
    files = sorted(glob.glob(os.path.join(root, "instances", "*.json")))
    bad = 0
    for f in files:
        inst = json.load(open(f, encoding="utf-8"))
        name, meta = inst["meta"]["name"], inst["meta"]
        p = []
        sig = {}
        for b in inst["boxes"]:
            sig.setdefault(b["type"], set()).add(tuple(b[k] for k in ATTRS))
        if any(len(v) != 1 for v in sig.values()):
            p.append("copies of a type are not identical")
        if not (5 <= meta["n_types"] <= 10) or len(sig) != meta["n_types"]:
            p.append("type count wrong")
        if not (10 <= len(inst["boxes"]) <= 100) or len(inst["boxes"]) != meta["n"]:
            p.append("box count wrong")
        if sum(meta["type_multiplicities"]) != meta["n"] \
                or min(meta["type_multiplicities"]) < 1:
            p.append("multiplicities wrong")
        dims = {(t["w_g"], t["d_g"], t["h_g"]) for t in inst["types"]}
        if len(dims) != len(inst["types"]):
            p.append("two types share the same grid dimensions")
        bd = bounds(inst)
        if bd["lb"] != meta["bounds"]["lb_pallets"]:
            p.append("LB mismatch")
        if name in packs:
            sol = packs[name]
            sol = {**sol,
                   "placements": {int(k): v for k, v in sol["placements"].items()}}
            p += verify(inst, sol)
            if sol["pallets_used"] != meta["bounds"]["ub_pallets_heuristic"]:
                p.append("UB mismatch")
        if p:
            bad += 1
            print(f"FAIL {name}: {p[:4]}")
    print(f"{len(files)} instances checked, {bad} failed")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1] if len(sys.argv) > 1 else "."))
