"""
make_typeset.py -- 100 test instances with FEW BOX TYPES for the retail pallet
loading problem.

Difference to the PL-100 set:
  * 5 to 10 box types per instance; all copies of a type are identical in
    dimensions, weight, product group, family, zone, retrieval priority,
    load bearing capacity, fragility and orientation restriction,
  * 10 to 100 boxes in total, distributed over the types,
  * NO filtering on the number of pallets - the pallet is always the full
    Euro pallet 1200 x 800 x 1800 mm with 1000 kg payload, and whatever
    pallet count the instance implies is simply reported.

This is the "few types, many copies" structure of the classical distributor's
pallet packing benchmarks (Bischoff, Janetz & Ratcliff 1995; OR-Library
thpack9 with 2-5 box types), while the type geometry and attributes still come
from the industrial distributions of Elhedhli/Gzara et al. via
mcpp_generator.py.

Output: instances/*.json (pallet_mip.py-ready), index.csv, types.csv,
        reference_packings.json
"""

from __future__ import annotations

import csv
import json
import math
import os
import random

import mcpp_generator as G
from pack_heuristic import bounds, pack_best, verify, solution_stats

GRID = 50
EURO = (1200, 800, 1800)          # full Euro pallet, no reduced heights here
PAYLOAD = 1000.0
P_MAX_HEURISTIC = 15              # only to report an upper bound, not a filter

SIZES = [10, 12, 14, 16, 18, 20, 22, 25, 28, 30,
         34, 38, 42, 46, 50, 55, 60, 65, 70, 75,
         80, 85, 90, 95, 100]

# four (number of types, volume mix) settings per size; mixes 1-4 are the
# instance classes of Elhedhli et al. (2019), Table 3
SETTINGS = [(5, 1), (7, 2), (8, 3), (10, 4),
            (6, 2), (9, 1), (5, 4), (10, 3),
            (7, 4), (6, 3), (8, 1), (9, 2)]


def sample_types(rng, n_types, mix, pallet, Lg, Wg, Hg):
    """Draw n_types distinct, pallet-feasible box types with all attributes."""
    types = []
    guard = 0
    while len(types) < n_types and guard < 100000:
        guard += 1
        w, d, h, v, cat = G.sample_box(rng, mix, pallet)
        gw, gd, gh = (int(math.ceil(w / GRID)), int(math.ceil(d / GRID)),
                      int(math.ceil(h / GRID)))
        if gh > Hg or not ((gw <= Lg and gd <= Wg) or (gd <= Lg and gw <= Wg)):
            continue
        if any((t["w_g"], t["d_g"], t["h_g"]) == (gw, gd, gh) for t in types):
            continue                      # keep the types visibly different
        fam = G.sample_family(rng)
        rep = G.Item(id=0, sku=len(types) + 1, width=w, depth=d, height=h,
                     volume_dm3=v, volume_category=cat)
        G.attach_extensions(rng, rep, fam)
        G.assign_retrieval_priorities([rep], rng)
        types.append({
            "type": len(types) + 1,
            "w_mm": gw * GRID, "d_mm": gd * GRID, "h_mm": gh * GRID,
            "w_g": gw, "d_g": gd, "h_g": gh,
            "raw_w_mm": round(w, 1), "raw_d_mm": round(d, 1),
            "raw_h_mm": round(h, 1),
            "weight_kg": rep.weight_kg,
            "group": ("chemical" if rep.is_chemical
                      else "food" if rep.is_food else "other"),
            "family": rep.family, "zone": rep.zone,
            "fragile": rep.fragile, "upright_only": rep.upright_only,
            "load_bearing_kpa": rep.load_bearing_kpa,
            "priority": rep.retrieval_priority,
            "volume_dm3": v, "volume_category": cat,
        })
    return types if len(types) == n_types else None


def multiplicities(rng, n, n_types):
    """Split n boxes over n_types types, every type used at least once.
    Type weights follow the frequency-of-occurrence distribution of
    Elhedhli et al. (lognormal(0.544, 0.658))."""
    counts = [1] * n_types
    if n <= n_types:
        return counts
    wts = [rng.lognormvariate(*(0.544, 0.658)) for _ in range(n_types)]
    tot = sum(wts)
    for _ in range(n - n_types):
        r, acc = rng.random() * tot, 0.0
        for t, wt in enumerate(wts):
            acc += wt
            if r <= acc:
                counts[t] += 1
                break
        else:
            counts[-1] += 1
    return counts


def make_instance(n, n_types, mix, seed, name):
    L, W, H = EURO
    Lg, Wg, Hg = L // GRID, W // GRID, H // GRID
    pallet = G.Pallet(float(L), float(W), float(H), "euro-180", PAYLOAD)
    rng = random.Random(seed)
    types = sample_types(rng, n_types, mix, pallet, Lg, Wg, Hg)
    if types is None:
        return None
    counts = multiplicities(rng, n, n_types)

    # planogram rank of the types: order in which the zones are served
    order = sorted(range(len(types)), key=lambda t: (types[t]["priority"], t))
    for rank, t in enumerate(order, 1):
        types[t]["seq"] = rank            # type level, so copies stay identical

    boxes = []
    for t, c in zip(types, counts):
        t["count"] = c
        for _ in range(c):
            b = {"id": len(boxes) + 1, "type": t["type"], "sku": t["type"]}
            b.update({k: t[k] for k in
                      ("w_mm", "d_mm", "h_mm", "w_g", "d_g", "h_g",
                       "raw_w_mm", "raw_d_mm", "raw_h_mm", "weight_kg",
                       "group", "family", "zone", "fragile", "upright_only",
                       "load_bearing_kpa", "priority", "seq",
                       "volume_dm3", "volume_category")})
            boxes.append(b)
    assert len(boxes) == n

    return {
        "meta": {
            "name": name,
            "generator": "mcpp_generator.py (Elhedhli/Gzara distributions) "
                         "+ make_typeset.py",
            "sources": ["Elhedhli, Gzara & Yildiz (2019), INFORMS J. Opt. 1(4)",
                        "Gzara, Elhedhli & Yildiz (2020), EJOR 287"],
            "n": n, "n_types": n_types, "volume_mix": mix, "seed": seed,
            "grid_mm": GRID, "reduced_height": False,
            "identical_within_type": True,
            "type_multiplicities": counts,
        },
        "pallet": {
            "L_mm": L, "W_mm": W, "H_mm": H,
            "L_g": Lg, "W_g": Wg, "H_g": Hg, "payload_kg": PAYLOAD,
        },
        "types": types,
        "boxes": boxes,
    }


def build(outdir="typeset"):
    os.makedirs(os.path.join(outdir, "instances"), exist_ok=True)
    index, typerows, packings = [], [], {}
    idx = 0
    for si, n in enumerate(SIZES):
        for slot in range(4):
            n_types, mix = SETTINGS[(4 * si + slot) % len(SETTINGS)]
            n_types = min(n_types, max(5, min(10, n)))    # n >= 10 anyway
            inst = None
            for seed in range(1, 200):
                cand = make_instance(n, n_types, mix, seed, "tmp")
                if cand is not None:
                    inst, used_seed = cand, seed
                    break
            if inst is None:
                print(f"  !! failed for n={n}, T={n_types}, mix={mix}")
                continue

            idx += 1
            name = f"bt{idx:03d}_n{n:03d}_T{n_types}_c{mix}_s{used_seed}"
            inst["meta"]["name"] = name
            bd = bounds(inst)
            sol = pack_best(inst, p_max=P_MAX_HEURISTIC)
            if sol is not None:
                errs = verify(inst, sol)
                if errs:
                    raise AssertionError(f"{name}: {errs[:3]}")
                stats = solution_stats(inst, sol)
                ub = sol["pallets_used"]
                packings[name] = sol
            else:
                stats, ub = {}, None
            inst["meta"]["bounds"] = {
                "lb_volume": bd["lb_vol"], "lb_weight": bd["lb_wt"],
                "lb_tall_footprint": bd["lb_area"], "lb_pallets": bd["lb"],
                "ub_pallets_heuristic": ub,
            }
            inst["meta"]["reference_solution_stats"] = stats
            with open(os.path.join(outdir, "instances", name + ".json"), "w",
                      encoding="utf-8") as fh:
                json.dump(inst, fh, indent=1)

            B = inst["boxes"]
            tv = sum(b["w_g"] * b["d_g"] * b["h_g"] for b in B) * GRID ** 3 / 1e9
            index.append({
                "name": name, "file": f"instances/{name}.json",
                "n_boxes": n, "n_types": n_types, "volume_mix": mix,
                "seed": used_seed, "grid_mm": GRID,
                "max_type_count": max(inst["meta"]["type_multiplicities"]),
                "lb_volume": bd["lb_vol"], "lb_weight": bd["lb_wt"],
                "lb_tall": bd["lb_area"], "LB": bd["lb"],
                "UB_heuristic": ub,
                "total_volume_m3": round(tv, 3),
                "total_weight_kg": round(sum(b["weight_kg"] for b in B), 1),
                "volume_use_pct": stats.get("volume_use_pct"),
                "pack_density_pct": stats.get("pack_density_pct"),
                "max_stack_mm": stats.get("max_stack_mm"),
                "n_groups": len({b["group"] for b in B}),
                "c17_active": int({"food", "chemical"} <= {b["group"] for b in B}),
                "pct_food": round(100 * sum(b["group"] == "food" for b in B) / n, 1),
                "pct_chemical": round(100 * sum(b["group"] == "chemical" for b in B) / n, 1),
                "n_priorities": len({b["priority"] for b in B}),
            })
            for t in inst["types"]:
                typerows.append({"instance": name, **{k: t[k] for k in
                                 ("type", "count", "w_mm", "d_mm", "h_mm",
                                  "weight_kg", "group", "family", "zone",
                                  "priority", "seq", "load_bearing_kpa",
                                  "fragile", "upright_only", "volume_dm3",
                                  "volume_category")}})
        print(f"n={n:>3}: {sum(1 for r in index if r['n_boxes'] == n)} instances")

    for fn, rows in (("index.csv", index), ("types.csv", typerows)):
        with open(os.path.join(outdir, fn), "w", newline="",
                  encoding="utf-8") as fh:
            wr = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            wr.writeheader()
            wr.writerows(rows)
    with open(os.path.join(outdir, "reference_packings.json"), "w",
              encoding="utf-8") as fh:
        json.dump(packings, fh, indent=1)
    print(f"\n{len(index)} instances written to {outdir}/")
    return index


if __name__ == "__main__":
    build()
