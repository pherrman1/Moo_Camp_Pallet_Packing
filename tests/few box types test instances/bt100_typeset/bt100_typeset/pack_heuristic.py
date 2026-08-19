"""
pack_heuristic.py -- feasibility oracle for the retail pallet loading problem.

Two jobs:

  bounds(inst)   -> the same three lower bounds that pallet_mip.py computes
                    (volume, weight, tall-box footprint), in grid units.
  pack(inst)     -> a *feasible* packing produced by a deepest-bottom-left
                    height-map heuristic that respects every hard constraint
                    of the model:
                      - containment, no overlap (height map + footprint update)
                      - >= 75 % of the base area supported (exact on the grid)
                      - pallet payload limit
                      - stack height limit
                      - no chemical vertically above food (C17)
                      - rotation only about the vertical axis (A2)
  verify(...)    -> independent re-check of a packing, so the reported upper
                    bound is not an artefact of the placement code.

LB and UB together certify the optimal pallet count: LB <= p* <= UB.
All lengths in GRID UNITS, exactly as in pallet_mip.py.
"""

from __future__ import annotations

import math
import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

ALPHA = 0.75
INF = 10 ** 9


# --------------------------------------------------------------------------- #
# lower bounds (identical to pallet_mip.build)
# --------------------------------------------------------------------------- #

def bounds(inst):
    P, B = inst["pallet"], inst["boxes"]
    Lg, Wg, Hg = P["L_g"], P["W_g"], P["H_g"]
    Q = P["payload_kg"]
    vol = sum(b["w_g"] * b["d_g"] * b["h_g"] for b in B)
    wt = sum(b["weight_kg"] for b in B)
    tall = sum(b["w_g"] * b["d_g"] for b in B if 2 * b["h_g"] > Hg)
    lb_vol = math.ceil(vol / (Lg * Wg * Hg))
    lb_wt = math.ceil(wt / Q)
    lb_area = math.ceil(tall / (Lg * Wg)) if tall else 0
    return {"lb_vol": lb_vol, "lb_wt": lb_wt, "lb_area": lb_area,
            "lb": max(1, lb_vol, lb_wt, lb_area)}


# --------------------------------------------------------------------------- #
# heuristic
# --------------------------------------------------------------------------- #

class _PalletState:
    def __init__(self, Lg, Wg, Hg, payload):
        self.Lg, self.Wg, self.Hg = Lg, Wg, Hg
        self.payload = payload
        self.height = np.zeros((Lg, Wg), dtype=np.int64)      # skyline
        self.food_low = np.full((Lg, Wg), INF, dtype=np.int64)  # lowest food top
        self.weight = 0.0
        self.placed = []                                       # placement dicts


def _candidate(state, bw, bd, bh, is_chem):
    """Best (z, y, x) for one orientation, or None."""
    Lg, Wg, Hg = state.Lg, state.Wg, state.Hg
    if bw > Lg or bd > Wg or bh > Hg:
        return None
    win = sliding_window_view(state.height, (bw, bd))          # (nx, ny, bw, bd)
    zmax = win.max(axis=(2, 3))
    ok = zmax + bh <= Hg
    if not ok.any():
        return None
    eq = (win == zmax[:, :, None, None]).sum(axis=(2, 3))
    sup = eq / float(bw * bd)
    ok &= (zmax == 0) | (sup >= ALPHA - 1e-9)
    if is_chem:
        fwin = sliding_window_view(state.food_low, (bw, bd)).min(axis=(2, 3))
        ok &= fwin > zmax                    # no food strictly underneath
    if not ok.any():
        return None
    nx, ny = zmax.shape
    xs = np.arange(nx)[:, None].repeat(ny, 1)
    ys = np.arange(ny)[None, :].repeat(nx, 0)
    key = np.where(ok, zmax * 10 ** 6 + ys * 10 ** 3 + xs, INF)  # low, back, left
    idx = np.unravel_index(np.argmin(key), key.shape)
    return int(zmax[idx]), int(idx[1]), int(idx[0])             # z, y, x


def _place(state, box, bw, bd, bh, x, y, z, is_food):
    state.height[x:x + bw, y:y + bd] = z + bh
    if is_food:
        reg = state.food_low[x:x + bw, y:y + bd]
        np.minimum(reg, z + bh, out=reg)
    state.weight += box["weight_kg"]
    state.placed.append({"id": box["id"], "x": x, "y": y, "z": z,
                         "w": bw, "d": bd, "h": bh,
                         "rot": int(bw != box["w_g"])})


def _group_rank(b):
    return {"chemical": 0, "other": 1, "food": 2}.get(b["group"], 1)


ORDERS = {
    "group_area": lambda b: (_group_rank(b), -b["w_g"] * b["d_g"], -b["h_g"], b["id"]),
    "area":       lambda b: (-b["w_g"] * b["d_g"], -b["h_g"], b["id"]),
    "height":     lambda b: (-b["h_g"], -b["w_g"] * b["d_g"], b["id"]),
    "volume":     lambda b: (-b["w_g"] * b["d_g"] * b["h_g"], b["id"]),
    "group_vol":  lambda b: (_group_rank(b), -b["w_g"] * b["d_g"] * b["h_g"], b["id"]),
}


def pack(inst, p_max=3, order="group_area"):
    """First-fit over pallets, deepest-bottom-left inside a pallet."""
    P, B = inst["pallet"], inst["boxes"]
    Lg, Wg, Hg, Q = P["L_g"], P["W_g"], P["H_g"], P["payload_kg"]

    order = sorted(B, key=ORDERS[order] if isinstance(order, str) else order)
    states = [_PalletState(Lg, Wg, Hg, Q) for _ in range(p_max)]

    for b in order:
        is_chem = b["group"] == "chemical"
        is_food = b["group"] == "food"
        upright = bool(b.get("upright_only", False))
        oris = {(b["w_g"], b["d_g"])}
        oris.add((b["d_g"], b["w_g"]))          # A2: 90 deg about z only
        done = False
        for st in states:
            if st.weight + b["weight_kg"] > Q + 1e-9:
                continue
            best = None
            for (bw, bd) in sorted(oris):
                c = _candidate(st, bw, bd, b["h_g"], is_chem)
                if c is None:
                    continue
                cand = (c[0], c[1], c[2], bw, bd)
                if best is None or cand[:3] < best[:3]:
                    best = cand
            if best is None:
                continue
            z, y, x, bw, bd = best
            _place(st, b, bw, bd, b["h_g"], x, y, z, is_food)
            done = True
            break
        if not done:
            return None                        # heuristic needs > p_max pallets
        _ = upright                            # kept in data, not used here

    used = [i for i, st in enumerate(states) if st.placed]
    sol = {"pallets_used": len(used),
           "placements": {i + 1: states[i].placed for i in used},
           "stack_height_g": {i + 1: int(states[i].height.max()) for i in used},
           "weight_kg": {i + 1: round(states[i].weight, 2) for i in used}}
    return sol


def pack_best(inst, p_max=3):
    """Best of all item orderings: fewest pallets, then lowest total height."""
    best = None
    for name in ORDERS:
        sol = pack(inst, p_max=p_max, order=name)
        if sol is None:
            continue
        key = (sol["pallets_used"], sum(sol["stack_height_g"].values()))
        if best is None or key < best[0]:
            best = (key, name, sol)
    if best is None:
        return None
    best[2]["order"] = best[1]
    return best[2]


# --------------------------------------------------------------------------- #
# independent verifier
# --------------------------------------------------------------------------- #

def verify(inst, sol):
    """Re-check a packing from scratch. Returns list of violations (empty = ok)."""
    P, B = inst["pallet"], inst["boxes"]
    Lg, Wg, Hg, Q = P["L_g"], P["W_g"], P["H_g"], P["payload_kg"]
    by_id = {b["id"]: b for b in B}
    err = []
    seen = set()

    for p, pl in sol["placements"].items():
        occ = np.zeros((Lg, Wg, Hg), dtype=np.int8)
        wsum = 0.0
        for q in pl:
            b = by_id[q["id"]]
            if q["id"] in seen:
                err.append(f"box {q['id']} placed twice")
            seen.add(q["id"])
            if sorted((q["w"], q["d"])) != sorted((b["w_g"], b["d_g"])) \
                    or q["h"] != b["h_g"]:
                err.append(f"box {q['id']}: illegal orientation")
            if q["x"] < 0 or q["y"] < 0 or q["z"] < 0 \
                    or q["x"] + q["w"] > Lg or q["y"] + q["d"] > Wg \
                    or q["z"] + q["h"] > Hg:
                err.append(f"box {q['id']}: outside pallet {p}")
                continue
            cell = occ[q["x"]:q["x"] + q["w"], q["y"]:q["y"] + q["d"],
                       q["z"]:q["z"] + q["h"]]
            if cell.any():
                err.append(f"box {q['id']}: overlap on pallet {p}")
            cell[:] = 1
            wsum += b["weight_kg"]
        if wsum > Q + 1e-6:
            err.append(f"pallet {p}: payload {wsum:.1f} > {Q}")

        # support: >= 75 % of the base area on the pallet deck or on box tops
        for q in pl:
            if q["z"] == 0:
                continue
            below = occ[q["x"]:q["x"] + q["w"], q["y"]:q["y"] + q["d"], q["z"] - 1]
            frac = below.sum() / float(q["w"] * q["d"])
            if frac < ALPHA - 1e-9:
                err.append(f"box {q['id']}: support {frac:.2f} < {ALPHA}")

        # C17: no chemical vertically above food (only if footprints overlap)
        for qi in pl:
            if by_id[qi["id"]]["group"] != "chemical":
                continue
            for qj in pl:
                if by_id[qj["id"]]["group"] != "food":
                    continue
                ox = min(qi["x"] + qi["w"], qj["x"] + qj["w"]) - max(qi["x"], qj["x"])
                oy = min(qi["y"] + qi["d"], qj["y"] + qj["d"]) - max(qi["y"], qj["y"])
                if ox > 0 and oy > 0 and qj["z"] + qj["h"] <= qi["z"]:
                    err.append(f"chemical {qi['id']} above food {qj['id']}")

    if len(seen) != len(B):
        err.append(f"only {len(seen)} of {len(B)} boxes placed")
    return err


def solution_stats(inst, sol):
    P, B = inst["pallet"], inst["boxes"]
    Lg, Wg, Hg = P["L_g"], P["W_g"], P["H_g"]
    vol = sum(b["w_g"] * b["d_g"] * b["h_g"] for b in B)
    used = sol["pallets_used"]
    hsum = sum(sol["stack_height_g"].values())
    return {"pallets_used": used,
            "volume_use_pct": round(100.0 * vol / (used * Lg * Wg * Hg), 1),
            "pack_density_pct": round(100.0 * vol / max(1, Lg * Wg * hsum), 1),
            "max_stack_mm": max(sol["stack_height_g"].values()) * inst["meta"]["grid_mm"]}
