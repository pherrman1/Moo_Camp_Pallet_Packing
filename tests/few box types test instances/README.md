# BT-100: 100 instances with few box types (5–10) for the retail pallet loading problem

100 instances, **10–100 boxes**, **5–10 box types** each. All copies of a type
are fully identical. **No filtering on the required number of pallets** — the
pallet is always the full Euro pallet and whatever pallet count follows from the
data is simply reported.

```
index.csv                 one row per instance
types.csv                 the type table of every instance (one row per type)
instances/*.json          the 100 instances, pallet_mip.py-ready
reference_packings.json   a verified feasible packing per instance
verify_typeset.py         re-checks type identity, counts, bounds, packings
make_typeset.py           rebuilds the set (needs mcpp_generator.py)
pack_heuristic.py         bounds + packing heuristic + verifier
```

`python verify_typeset.py .` prints `100 instances checked, 0 failed`.

## What "one type" means

Every box carries a `type` field, and all boxes of a type agree in **every**
attribute: `w_mm/d_mm/h_mm`, the grid dimensions `w_g/d_g/h_g`, `weight_kg`,
`group` (food / chemical / other), `family`, `zone`, `priority`, `seq`,
`load_bearing_kpa`, `fragile`, `upright_only`, `volume_dm3`,
`volume_category`. Only `id` differs. The retrieval priority is therefore also a
type property — the planogram rank `seq` is given per type, not per box, so
copies stay indistinguishable.

Two types within an instance never share the same grid dimensions, so the types
are visibly different and not accidental duplicates.

Each instance also carries a top-level `types` array with the type table and
`count` per type, plus `meta.type_multiplicities`. Useful because identical
copies create massive symmetry in the MIP: you can index decisions by type
instead of by box, or add symmetry-breaking constraints between copies (e.g.
lexicographic ordering of positions within a type), which is exactly the
structure the classical distributor's benchmarks have.

## Composition

* 25 sizes: 10, 12, 14, 16, 18, 20, 22, 25, 28, 30, 34, 38, 42, 46, 50, 55, 60,
  65, 70, 75, 80, 85, 90, 95, 100 boxes × 4 instances each.
* Type counts 5–10, roughly 16–17 instances each; mean 6.6 boxes per type.
* Multiplicities are uneven (a few frequent types, several rare ones), drawn
  from the frequency-of-occurrence distribution of Elhedhli et al.
  (lognormal(0.544, 0.658)), with every type used at least once.
* Volume mixes: the four instance classes of Elhedhli et al. (2019), Table 3,
  25 instances each. No artificial bulky mixes are used in this set.
* Pallet: Euro **1200 × 800 × 1800 mm**, payload **1000 kg**, grid **50 mm**
  (24 × 16 × 36 cells). Box dimensions are snapped **up** to the grid, so any
  packing found on the grid is physically realisable; `raw_*_mm` keeps the
  un-snapped values.
* 75 of 100 instances contain both food and chemical boxes, so the vertical
  ordering constraint C17 actually bites (`c17_active` in `index.csv`); in the
  remaining 25 it is vacuous.
* 2–7 distinct retrieval priority levels per instance.

## Data source

Type geometry and attributes come from the attached `mcpp_generator.py`, i.e.
from the distributions fitted to 166,406 industrial cases by Elhedhli, Gzara &
Yildiz (2019) — depth/width ~ N(0.695, 0.118), height/width ~ LN(−0.654,
0.453), five k-means volume categories — with weight, load bearing and
planogram attributes following Gzara, Elhedhli & Yildiz (2020).

The "few types, many identical copies" structure matches the classical
distributor's pallet packing benchmarks (Bischoff, Janetz & Ratcliff 1995;
OR-Library `thpack9`, which uses 2–5 box types), while the individual type
dimensions stay realistic rather than uniformly random.

## Pallet counts (reported, not enforced)

| | 1 pallet | 2 pallets | 3 pallets |
|---|---|---|---|
| lower bound LB | 88 | 12 | – |
| heuristic UB | 70 | 27 | 3 |

Since no filter was applied, most instances fit on a single pallet: 10–100
average retail cases are 0.13–3.01 m³ against a 1.73 m³ pallet. In 21 instances
LB < UB, i.e. the heuristic needs a pallet more than the bound allows — those
are the ones where your model can show something. `index.csv` carries all three
bounds separately (`lb_volume`, `lb_weight`, `lb_tall`), so you can filter for
whatever subset you need:

* volume-driven: `lb_volume = 2` in 12 instances,
* weight-driven: `lb_weight = 2` in 3 instances (up to 1279 kg total against the
  1000 kg payload),
* the tall-box footprint bound is never binding here, because the full 1800 mm
  height makes almost no case "tall" (h > H/2 = 900 mm).

If you want a set where 2–3 pallets are *certified*, use the PL-100 set from
before; this one is deliberately unfiltered.

## Reference packings

`reference_packings.json` holds a feasible packing per instance from a
deepest-bottom-left height-map heuristic, re-checked by an independent verifier
for containment, pairwise non-overlap, ≥ 75 % base-area support (exact on the
grid), payload, height, no chemical vertically above food, and 90°-only
rotations about the vertical axis. Placements are
`{id, x, y, z, w, d, h, rot}` in grid units. Mean pack density is 69.8 %, close
to the ~69 % Gzara et al. report on industry instances — a fair baseline for
your objectives, which the heuristic does not optimise at all (it minimises
pallet count only).

## Reproducing

```bash
python make_typeset.py        # deterministic, ~1 minute
python verify_typeset.py .
```

Every instance is fixed by (n, number of types, volume mix, seed), all recorded
in the file name and in `index.csv`, e.g. `bt042_n046_T7_c4_s1`.
