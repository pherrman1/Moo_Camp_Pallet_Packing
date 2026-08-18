"""
pallet_mip.py -- MIP for the retail pallet loading problem.

Implements the formulation in pallet_loading_mip.md:
  C1-C5   assignment / orientation / capacity
  C6-C8   containment + disjunctive non-overlap
  C9-C16  75 % support (exact linearisation of the contact area on the grid)
  C17     no chemical vertically above food (hard)
  C19/f4  retrieval priority (soft)
  C20-C23 symmetry breaking + pallet lower bound
  C24     stack height, objectives f1..f4

All lengths are in GRID UNITS (1 unit = grid_mm).
"""

import json
import math
import sys
import pulp


ALPHA = 0.75          # required supported fraction of the base area
GAMMA_HEAVY = None    # set to e.g. 2.0 to activate the strict rule (C18)


def z_normal_patterns(heights, cap):
    """Achievable bottom coordinates: 0 and every subset sum of box heights
    that is <= cap (Herz/Christofides normal patterns in the z direction).
    A box either stands on the deck or on top of a stack of other boxes, so
    no other z value can ever occur in a feasible packing."""
    reach = {0}
    for h in heights:
        reach |= {v + h for v in reach if v + h <= cap}
    return sorted(reach)


def build(inst, p_max=2, hard_priority=False, alpha=ALPHA, h_cap=None,
          support_mode="area"):
    """support_mode = "area"   -> exact 75 % contact-area constraint (C12-C16)
                     "corner" -> all four bottom corner CELLS must be covered
                                 by a directly supporting box (cheap surrogate)"""
    P = inst["pallet"]
    B = inst["boxes"]
    n = len(B)
    Lg, Wg, Hg = P["L_g"], P["W_g"], P["H_g"]
    if h_cap:                    # preprocessing bound on the stack height
        Hg = min(Hg, h_cap)
    Q = P["payload_kg"]

    N = range(n)
    PAL = range(1, p_max + 1)

    # ---- box data in grid units -------------------------------------------
    # A2: only 90 deg rotations about the vertical axis -> 2 orientations
    ori = {i: [(B[i]["w_g"], B[i]["d_g"]), (B[i]["d_g"], B[i]["w_g"])] for i in N}
    ori = {i: sorted(set(o)) for i, o in ori.items()}       # square base -> 1
    hg = {i: B[i]["h_g"] for i in N}
    beta = {i: B[i]["w_g"] * B[i]["d_g"] for i in N}        # base area, constant
    mass = {i: B[i]["weight_kg"] for i in N}
    prio = {i: B[i]["priority"] for i in N}
    grp = {i: B[i]["group"] for i in N}
    vol = {i: B[i]["w_g"] * B[i]["d_g"] * B[i]["h_g"] for i in N}
    maxlen = {i: max(max(a, b) for a, b in ori[i]) for i in N}
    hmin = min(hg.values())

    m = pulp.LpProblem("pallet_loading", pulp.LpMinimize)

    # ---- variables --------------------------------------------------------
    v = pulp.LpVariable.dicts("v", PAL, cat="Binary")
    u = pulp.LpVariable.dicts("u", (N, PAL), cat="Binary")
    rho = {(i, r): pulp.LpVariable(f"rho_{i}_{r}", cat="Binary")
           for i in N for r in range(len(ori[i]))}
    x = pulp.LpVariable.dicts("x", N, 0, Lg, cat="Integer")
    y = pulp.LpVariable.dicts("y", N, 0, Wg, cat="Integer")
    z = pulp.LpVariable.dicts("z", N, 0, Hg, cat="Integer")
    g = pulp.LpVariable.dicts("g", N, cat="Binary")

    pairs = [(i, j) for i in N for j in N if i < j]
    opairs = [(i, j) for i in N for j in N if i != j]

    a = pulp.LpVariable.dicts("a", pairs, cat="Binary")   # i left of j
    b = pulp.LpVariable.dicts("b", pairs, cat="Binary")   # j left of i
    c = pulp.LpVariable.dicts("c", pairs, cat="Binary")   # i in front of j
    d = pulp.LpVariable.dicts("d", pairs, cat="Binary")   # j in front of i
    e = pulp.LpVariable.dicts("e", pairs, cat="Binary")   # i below j
    f = pulp.LpVariable.dicts("f", pairs, cat="Binary")   # j below i

    theta = pulp.LpVariable.dicts("th", (pairs, PAL), 0, 1)    # u_ip AND u_jp
    tau = pulp.LpVariable.dicts("tau", pairs, 0, 1)            # same pallet
    s = pulp.LpVariable.dicts("s", opairs, cat="Binary")       # j supports i

    ox = oy = Tset = psi = mu = prod = A = chi = None
    CORNERS = [(0, 0), (1, 0), (0, 1), (1, 1)]
    if support_mode == "area":
        ox = {pr: pulp.LpVariable(f"ox_{pr[0]}_{pr[1]}", 0,
                                  min(maxlen[pr[0]], maxlen[pr[1]]), cat="Integer")
              for pr in pairs}
        oy = {pr: pulp.LpVariable(f"oy_{pr[0]}_{pr[1]}", 0,
                                  min(maxlen[pr[0]], maxlen[pr[1]]), cat="Integer")
              for pr in pairs}
        Tset = {pr: range(0, min(maxlen[pr[0]], maxlen[pr[1]]) + 1) for pr in pairs}
        psi = {(pr, t): pulp.LpVariable(f"psi_{pr[0]}_{pr[1]}_{t}", cat="Binary")
               for pr in pairs for t in Tset[pr]}
        mu = {(pr, t): pulp.LpVariable(f"mu_{pr[0]}_{pr[1]}_{t}", 0,
                                       min(maxlen[pr[0]], maxlen[pr[1]]))
              for pr in pairs for t in Tset[pr]}
        prod = {pr: pulp.LpVariable(f"prod_{pr[0]}_{pr[1]}", 0,
                                    min(beta[pr[0]], beta[pr[1]])) for pr in pairs}
        A = {pr: pulp.LpVariable(f"A_{pr[0]}_{pr[1]}", 0, beta[pr[0]])
             for pr in opairs}
    else:
        # chi[i,j,k] = 1  <->  corner cell k of box i is covered by box j
        chi = {(i, j, k): pulp.LpVariable(f"chi_{i}_{j}_{k}", cat="Binary")
               for (i, j) in opairs for k in range(4)}
    Theta = pulp.LpVariable.dicts("Theta", PAL, 0, Hg)

    # effective dimensions (linear expressions)
    lam = {i: pulp.lpSum(ori[i][r][0] * rho[i, r] for r in range(len(ori[i])))
           for i in N}
    om = {i: pulp.lpSum(ori[i][r][1] * rho[i, r] for r in range(len(ori[i])))
          for i in N}

    # ---- C1-C5 -----------------------------------------------------------
    for i in N:
        m += pulp.lpSum(u[i][p] for p in PAL) == 1, f"C1_{i}"
        m += pulp.lpSum(rho[i, r] for r in range(len(ori[i]))) == 1, f"C2_{i}"
        for p in PAL:
            m += u[i][p] <= v[p]
    for p in PAL:
        m += pulp.lpSum(vol[i] * u[i][p] for i in N) <= Lg * Wg * Hg * v[p]
        m += pulp.lpSum(mass[i] * u[i][p] for i in N) <= Q * v[p]
    for (i, j) in pairs:
        for p in PAL:
            m += theta[(i, j)][p] <= u[i][p]
            m += theta[(i, j)][p] <= u[j][p]
            m += theta[(i, j)][p] >= u[i][p] + u[j][p] - 1
        m += tau[(i, j)] == pulp.lpSum(theta[(i, j)][p] for p in PAL)

    # ---- C6 containment ---------------------------------------------------
    for i in N:
        m += x[i] + lam[i] <= Lg
        m += y[i] + om[i] <= Wg
        m += z[i] + hg[i] <= Hg

    # ---- C7/C8 non-overlap ------------------------------------------------
    for (i, j) in pairs:
        m += a[(i, j)] + b[(i, j)] + c[(i, j)] + d[(i, j)] \
             + e[(i, j)] + f[(i, j)] >= tau[(i, j)]
        m += x[i] + lam[i] <= x[j] + Lg * (1 - a[(i, j)])
        m += x[j] + lam[j] <= x[i] + Lg * (1 - b[(i, j)])
        m += y[i] + om[i] <= y[j] + Wg * (1 - c[(i, j)])
        m += y[j] + om[j] <= y[i] + Wg * (1 - d[(i, j)])
        m += z[i] + hg[i] <= z[j] + Hg * (1 - e[(i, j)])
        m += z[j] + hg[j] <= z[i] + Hg * (1 - f[(i, j)])
        m += a[(i, j)] + b[(i, j)] <= 1
        m += c[(i, j)] + d[(i, j)] <= 1
        m += e[(i, j)] + f[(i, j)] <= 1

    # ---- C9 floor indicator ----------------------------------------------
    for i in N:
        m += z[i] <= Hg * (1 - g[i])
        m += z[i] >= hmin * (1 - g[i])

    # ---- normal patterns in z (valid inequality, no loss of optimality) ---
    Zset = z_normal_patterns([hg[i] for i in N], Hg)
    zeta = {(i, q): pulp.LpVariable(f"zeta_{i}_{q}", cat="Binary")
            for i in N for q in Zset if q + hg[i] <= Hg}
    for i in N:
        qs = [q for q in Zset if q + hg[i] <= Hg]
        m += pulp.lpSum(zeta[i, q] for q in qs) == 1
        m += z[i] == pulp.lpSum(q * zeta[i, q] for q in qs)
        m += g[i] == zeta[i, 0]

    # ---- C10/C11 direct support relation ---------------------------------
    for (i, j) in opairs:
        pr = (min(i, j), max(i, j))
        m += z[i] - z[j] - hg[j] <= Hg * (1 - s[(i, j)])
        m += z[j] + hg[j] - z[i] <= Hg * (1 - s[(i, j)])
        m += s[(i, j)] <= tau[pr]
    for (i, j) in pairs:
        m += s[(i, j)] + s[(j, i)] <= 1

    if support_mode == "area":
        # ---- C12 footprint overlap lengths -------------------------------
        for (i, j) in pairs:
            m += ox[(i, j)] <= x[i] + lam[i] - x[j]
            m += ox[(i, j)] <= x[j] + lam[j] - x[i]
            m += oy[(i, j)] <= y[i] + om[i] - y[j]
            m += oy[(i, j)] <= y[j] + om[j] - y[i]

        # ---- C13-C15 exact linearisation of ox*oy ------------------------
        for pr in pairs:
            UBy = min(maxlen[pr[0]], maxlen[pr[1]])
            m += pulp.lpSum(psi[pr, t] for t in Tset[pr]) == 1
            m += ox[pr] == pulp.lpSum(t * psi[pr, t] for t in Tset[pr])
            for t in Tset[pr]:
                m += mu[pr, t] <= UBy * psi[pr, t]
                m += mu[pr, t] <= oy[pr]
            m += prod[pr] <= pulp.lpSum(t * mu[pr, t] for t in Tset[pr])
        for (i, j) in opairs:
            pr = (min(i, j), max(i, j))
            m += A[(i, j)] <= prod[pr]
            m += A[(i, j)] <= beta[i] * s[(i, j)]

        # ---- C16 the alpha-support requirement ---------------------------
        for i in N:
            m += pulp.lpSum(A[(i, j)] for j in N if j != i) \
                 >= alpha * beta[i] * (1 - g[i]), f"C16_{i}"
    else:
        # ---- C16' corner support -----------------------------------------
        # The four corner CELLS of box i (not the mathematical corner points)
        # must each be covered by the top face of a directly supporting box.
        # Using cells rather than points rules out the degenerate case of a
        # corner touching only the edge of a box with zero contact area.
        for (i, j) in opairs:
            for k, (ax_, ay_) in enumerate(CORNERS):
                cx = x[i] + ax_ * (lam[i] - 1)
                cy = y[i] + ay_ * (om[i] - 1)
                M1 = 1 - chi[i, j, k]
                m += cx >= x[j] - Lg * M1
                m += cx <= x[j] + lam[j] - 1 + Lg * M1
                m += cy >= y[j] - Wg * M1
                m += cy <= y[j] + om[j] - 1 + Wg * M1
                m += chi[i, j, k] <= s[(i, j)]
        for i in N:
            for k in range(4):
                m += pulp.lpSum(chi[i, j, k] for j in N if j != i) >= 1 - g[i], \
                     f"C16c_{i}_{k}"

    # ---- C17 no chemical vertically above food ---------------------------
    for (i, j) in pairs:
        if grp[i] == "food" and grp[j] == "chemical":
            m += e[(i, j)] == 0            # forbid "food below chemical"
        if grp[i] == "chemical" and grp[j] == "food":
            m += f[(i, j)] == 0            # forbid "food (j) below chemical (i)"

    # ---- C18 optional strict heavy-below-light ---------------------------
    if GAMMA_HEAVY:
        for (i, j) in pairs:
            if mass[i] >= GAMMA_HEAVY * mass[j]:
                m += f[(i, j)] == 0        # j (light) may not be below i (heavy)
            if mass[j] >= GAMMA_HEAVY * mass[i]:
                m += e[(i, j)] == 0

    # ---- C19 optional hard accessibility ---------------------------------
    if hard_priority:
        for (i, j) in opairs:
            if prio[i] > prio[j]:
                m += s[(i, j)] == 0

    # ---- C20-C23 symmetry breaking ---------------------------------------
    for p in PAL:
        if p + 1 in PAL:
            m += v[p] >= v[p + 1]
    for i in N:
        for p in PAL:
            if p > i + 1:
                m += u[i][p] == 0
            elif p >= 2:
                m += u[i][p] <= pulp.lpSum(u[j][p - 1] for j in N if j < i)
    # (S7) tall-box area bound: a box higher than H/2 cannot carry another such
    # box nor stand on one, so the footprints of all tall boxes on one pallet
    # must be pairwise disjoint.  Valid because under A2 the height is fixed.
    tall_area = sum(beta[i] for i in N if hg[i] * 2 > Hg)
    lb_area = math.ceil(tall_area / (Lg * Wg)) if tall_area else 0
    lb_vol = math.ceil(sum(vol.values()) / (Lg * Wg * Hg))
    lb_wt = math.ceil(sum(mass.values()) / Q)
    lb_pallets = max(lb_vol, lb_wt, lb_area)
    m += pulp.lpSum(v[p] for p in PAL) >= lb_pallets

    # ---- C24 stack heights ------------------------------------------------
    for i in N:
        for p in PAL:
            m += Theta[p] >= z[i] + hg[i] - Hg * (1 - u[i][p])

    # ---- objective expressions -------------------------------------------
    f1 = pulp.lpSum(v[p] for p in PAL)
    f2 = pulp.lpSum(Theta[p] for p in PAL)
    f3 = pulp.lpSum(mass[i] * z[i] + 0.5 * mass[i] * hg[i] for i in N)
    if support_mode == "area":
        # area-weighted blocking
        f4 = pulp.lpSum(max(0, prio[i] - prio[j]) * A[(i, j)] for (i, j) in opairs)
    else:
        # no contact area available -> weight the blocking relation by the
        # base area of the blocking box, which is its upper bound anyway
        f4 = pulp.lpSum(max(0, prio[i] - prio[j]) * beta[i] * s[(i, j)]
                        for (i, j) in opairs)

    handles = dict(model=m, v=v, u=u, rho=rho, x=x, y=y, z=z, g=g, s=s, A=A,
                   Theta=Theta, ox=ox, oy=oy, f1=f1, f2=f2, f3=f3, f4=f4,
                   ori=ori, hg=hg, beta=beta, mass=mass, prio=prio, grp=grp,
                   n=n, PAL=PAL, Lg=Lg, Wg=Wg, Hg=Hg, lb_pallets=lb_pallets,
                   pairs=pairs, opairs=opairs, Zset=Zset,
                   lb_vol=lb_vol, lb_wt=lb_wt, lb_area=lb_area,
                   support_mode=support_mode, chi=chi)
    return handles


def solve(h, obj, time_limit=120, gap=0.0, msg=True):
    h["model"].setObjective(obj)
    solver = pulp.HiGHS(timeLimit=time_limit, msg=msg, gapRel=gap)
    h["model"].solve(solver)
    return pulp.LpStatus[h["model"].status]


if __name__ == "__main__":
    inst = json.load(open(sys.argv[1] if len(sys.argv) > 1
                          else "instance_tiny.json"))
    h = build(inst)
    print("variables:", len(h["model"].variables()),
          " constraints:", len(h["model"].constraints))
    print("pallet lower bound:", h["lb_pallets"])
