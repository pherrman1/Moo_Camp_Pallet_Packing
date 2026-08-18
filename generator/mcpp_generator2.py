"""
mcpp_generator.py
=================
Realistic instance generator for the Mixed-Case Palletization Problem (MCPP),
reimplemented from the specifications in

  [1] Elhedhli, Gzara, Yildiz (2019), "Three-Dimensional Bin Packing and
      Mixed-Case Palletization", INFORMS J. on Optimization 1(4), Sec. 7.1
      -> item dimensions from volume + shape ratios, volume categories,
         frequency of occurrence.
  [2] Gzara, Elhedhli, Yildiz (2020), "The pallet loading problem: Three-
      dimensional bin packing with practical constraints", EJOR 287, Sec. 4
      -> density (hence weight), load bearing capacity, planogram sequence.

Extensions for the retail pallet-loading task (not in [1,2]):
  * product group (food / chemical / other), needed for the vertical
    ordering requirement,
  * retrieval priority derived from the planogram sequence.

Distributions used
------------------
depth/width ratio      ~ Normal(0.695, 0.118)                       [1, Tab. 2]
height/width ratio     ~ Lognormal(-0.654, 0.453)                   [1, Tab. 2]
repetition (frequency) ~ Lognormal(0.544, 0.658)                    [1, Tab. 2]
volume (dm^3)          ~ Uniform within one of 5 k-means categories [1, Sec 7.1]
     cat1 [ 2.72, 12.04]  p=0.4329     cat2 [12.05, 20.23]  p=0.3331
     cat3 [20.28, 32.42]  p=0.1578     cat4 [32.44, 54.08]  p=0.0559
     cat5 [54.31,100.21]  p=0.0203     (p = share of the 166,406 items in [1])
density (g/dm^3)       ~ Gamma(3.211, scale 58.824)  w.p. 0.3132    [2, Tab. 2]
                         Lognormal(6.502, 0.208)     w.p. 0.6868
weight                 = volume * density                           [2, Step 3]

Usage
-----
    python mcpp_generator.py --n 10 --seed 7 --volume-mix 0,0,60,40,0 \
        --grid 5 --out instance_tiny.json
"""

import argparse
import json
import math
import random

# ----------------------------------------------------------------------------
# distribution parameters (papers [1], [2])
# ----------------------------------------------------------------------------
DW_RATIO = (0.695, 0.118)            # normal(mu, sigma)
HW_RATIO = (-0.654, 0.453)           # lognormal(meanlog, sdlog)
REPETITION = (0.544, 0.658)          # lognormal(meanlog, sdlog)

VOLUME_CATEGORIES = [                # (lo, hi) in dm^3, share of items
    (2.72, 12.04, 72037),
    (12.05, 20.23, 55436),
    (20.28, 32.42, 26254),
    (32.44, 54.08, 9304),
    (54.31, 100.21, 3376),
]

DENSITY_CURVES = [                   # (kind, params, probability)
    ("gamma", (3.211, 58.824), 0.3132),
    ("lognorm", (6.502, 0.208), 0.6868),
]

# load bearing capacity (kg/m^2) by density section, [2, Sec. 4 / Tab. 3].
# The paper fits several trend lines per section; we use the reported strong
# density<->capacity correlation (rho = 0.75) as a linear surrogate plus noise.
DENSITY_SECTIONS = [(31.76, 434.64), (434.64, 617.56), (617.56, 1771.11)]

# retail product groups (task extension)
PRODUCT_GROUPS = [("food", 0.55), ("chemical", 0.20), ("other", 0.25)]


def _sample_volume(rng, weights):
    lo, hi, _ = rng.choices(VOLUME_CATEGORIES, weights=weights, k=1)[0]
    return rng.uniform(lo, hi)


def _sample_density(rng):
    kind, params, _ = rng.choices(
        DENSITY_CURVES, weights=[c[2] for c in DENSITY_CURVES], k=1
    )[0]
    if kind == "gamma":
        shape, scale = params
        return rng.gammavariate(shape, scale)
    meanlog, sdlog = params
    return math.exp(rng.gauss(meanlog, sdlog))


def _load_capacity(rng, density):
    """kg/m^2, monotone in density (correlation 0.75 in [2, Tab. 1])."""
    for k, (lo, hi) in enumerate(DENSITY_SECTIONS):
        if density < hi:
            section = k
            break
    else:
        section = 2
    base = [180.0, 520.0, 900.0][section]
    return max(0.0, base * rng.lognormvariate(0.0, 0.35))


def _dimensions(rng, volume_dm3):
    """Volume + two shape ratios -> (w, d, h) in mm.  [1, Sec. 7.1]"""
    dw = max(0.35, rng.gauss(*DW_RATIO))
    hw = math.exp(rng.gauss(*HW_RATIO))
    # volume = w * (dw*w) * (hw*w) = dw*hw*w^3
    w = (volume_dm3 / (dw * hw)) ** (1.0 / 3.0)         # dm
    return (100.0 * w, 100.0 * dw * w, 100.0 * hw * w)  # mm


def generate(n, seed=0, volume_mix=None, grid_mm=50,
             pallet=(1200, 800, 1800), payload_kg=1000.0,
             use_repetition=False):
    """Return an instance dict with n boxes, dimensions snapped up to `grid_mm`."""
    rng = random.Random(seed)
    if volume_mix is None:
        weights = [c[2] for c in VOLUME_CATEGORIES]
    else:
        weights = list(volume_mix)

    boxes = []
    while len(boxes) < n:
        vol = _sample_volume(rng, weights)
        w, d, h = _dimensions(rng, vol)
        # how many identical copies of this SKU arrive (frequency of occurrence)
        reps = 1
        if use_repetition:
            reps = max(1, int(round(rng.lognormvariate(*REPETITION))))
        density = _sample_density(rng)
        cap = _load_capacity(rng, density)
        group = rng.choices([g for g, _ in PRODUCT_GROUPS],
                            weights=[p for _, p in PRODUCT_GROUPS], k=1)[0]
        for _ in range(reps):
            if len(boxes) >= n:
                break
            # snap UP to the modelling grid (conservative, never claims a fit
            # that does not exist)
            gw = int(math.ceil(w / grid_mm))
            gd = int(math.ceil(d / grid_mm))
            gh = int(math.ceil(h / grid_mm))
            L, W, H = pallet
            if gw * grid_mm > L or gd * grid_mm > W or gh * grid_mm > H:
                continue
            boxes.append({
                "id": len(boxes),
                "w_mm": gw * grid_mm,
                "d_mm": gd * grid_mm,
                "h_mm": gh * grid_mm,
                "w_g": gw, "d_g": gd, "h_g": gh,        # in grid units
                "weight_kg": round(vol * density / 1000.0, 2),
                "load_cap_kg_m2": round(cap, 1),
                "group": group,
                "volume_dm3": round(vol, 2),
                "density_g_dm3": round(density, 1),
            })

    # planogram sequence -> retrieval priority (1 = picked first)
    order = list(range(len(boxes)))
    rng.shuffle(order)
    for rank, i in enumerate(order):
        boxes[i]["seq"] = rank + 1
        boxes[i]["priority"] = rank + 1

    return {
        "meta": {
            "generator": "mcpp_generator (Elhedhli 2019 / Gzara 2020 dists)",
            "n": len(boxes), "seed": seed, "grid_mm": grid_mm,
            "volume_mix": weights,
        },
        "pallet": {
            "L_mm": pallet[0], "W_mm": pallet[1], "H_mm": pallet[2],
            "L_g": pallet[0] // grid_mm, "W_g": pallet[1] // grid_mm,
            "H_g": pallet[2] // grid_mm,
            "payload_kg": payload_kg,
        },
        "boxes": boxes,
    }


def summarise(inst):
    P, B = inst["pallet"], inst["boxes"]
    tv = sum(b["w_mm"] * b["d_mm"] * b["h_mm"] for b in B) / 1e9
    pv = P["L_mm"] * P["W_mm"] * P["H_mm"] / 1e9
    fa = sum(b["w_mm"] * b["d_mm"] for b in B) / 1e6
    pa = P["L_mm"] * P["W_mm"] / 1e6
    lines = [
        f"n = {len(B)} boxes, grid = {inst['meta']['grid_mm']} mm",
        f"pallet {P['L_mm']}x{P['W_mm']}x{P['H_mm']} mm  "
        f"= {P['L_g']}x{P['W_g']}x{P['H_g']} grid units",
        f"total box volume   {tv:6.3f} m^3   (pallet {pv:.3f} m^3"
        f"  -> volume LB = {math.ceil(tv / pv)} pallet(s))",
        f"total footprint    {fa:6.3f} m^2   (pallet {pa:.3f} m^2"
        f"  -> at least {math.ceil(fa / pa)} tier(s))",
        f"total weight       {sum(b['weight_kg'] for b in B):6.1f} kg"
        f"   (limit {P['payload_kg']:.0f} kg)",
        "",
        f"{'id':>2} {'w':>4} {'d':>4} {'h':>4} {'kg':>6} {'group':>9} {'prio':>4}",
    ]
    for b in B:
        lines.append(f"{b['id']:>2} {b['w_mm']:>4} {b['d_mm']:>4} {b['h_mm']:>4} "
                     f"{b['weight_kg']:>6.1f} {b['group']:>9} {b['priority']:>4}")
    return "\n".join(lines)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--grid", type=int, default=50, help="grid resolution in mm")
    ap.add_argument("--volume-mix", type=str, default=None,
                    help="5 comma-separated weights for the volume categories")
    ap.add_argument("--payload", type=float, default=1000.0)
    ap.add_argument("--out", type=str, default="instance.json")
    a = ap.parse_args()
    mix = [float(x) for x in a.volume_mix.split(",")] if a.volume_mix else None
    inst = generate(a.n, seed=a.seed, volume_mix=mix, grid_mm=a.grid,
                    payload_kg=a.payload)
    with open(a.out, "w") as f:
        json.dump(inst, f, indent=1)
    print(summarise(inst))
    print(f"\nwritten to {a.out}")
