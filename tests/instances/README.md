# PL-100: 100 small test instances for the retail pallet loading problem

100 instances with **10–100 boxes** each whose optimal pallet count is
**certified to lie in {2, 3}**. Format is exactly what `pallet_mip.py` reads,
so `python pallet_mip.py instances/pl001_n010_H900_B2_LB2UB2.json` works
out of the box.

```
index.csv                    one row per instance: bounds, mix, utilisation, ...
instances/*.json             the 100 instances
reference_packings.json      a verified feasible packing for every instance
variants_weight/             22 weight-critical twins (see below)
pack_heuristic.py            bounds + packing heuristic + verifier
verify_testset.py            re-checks the whole set from scratch
weight_variants.py           builds the weight-critical twins
make_testset.py              rebuilds everything (needs mcpp_generator.py)
```

`python verify_testset.py .` prints `100 instances checked, 0 failed`.

## Where the data comes from

Item geometry and attributes are drawn from the attached generator
`mcpp_generator.py`, i.e. from the distributions fitted to 166,406 industrial
cases in

* Elhedhli, Gzara & Yildiz (2019), *Three-Dimensional Bin Packing and
  Mixed-Case Palletization*, INFORMS J. on Optimization 1(4) — depth/width
  ~ N(0.695, 0.118), height/width ~ LN(−0.654, 0.453), repetition
  ~ LN(0.544, 0.658), five k-means volume categories;
* Gzara, Elhedhli & Yildiz (2020), *The Pallet Loading Problem*, EJOR 287 —
  density (hence weight), load bearing capacity, planogram sequence.

Product group, family, fragility, orientation restriction and retrieval
priority are the generator's task extensions, not fitted to industry data.

**Why not OR-Library `thpack9`?** Those 47 literature instances (Ivancic,
Mathur & Mohanty 1989) have 70–150 boxes but lower bounds of 3–19 pallets, so
essentially none of them falls in the 2–3 pallet window; truncating them to fit
would make them no longer literature instances. They remain the right reference
set for the multi-pallet experiments in the talk — this set complements them at
MIP-solvable scale.

## The certificate

Every instance satisfies

```
2  <=  LB  <=  p*  <=  UB  <=  3
```

* **LB** = max(volume bound, weight bound, tall-box footprint bound), computed
  in grid units by exactly the same formulas as `pallet_mip.build`, so the LB in
  `index.csv` is the LB your model will print.
* **UB** = number of pallets used by a deepest-bottom-left height-map packing
  that is then re-checked by an independent verifier: containment, pairwise
  non-overlap, ≥ 75 % base-area support (exact on the 50 mm grid), payload
  limit, height limit, no chemical vertically above food, only 90° rotations
  about the vertical axis, every box placed exactly once.

So "needs 2 to 3 pallets" is proved, not assumed. The distribution over the
set:

| LB | UB | instances | character |
|----|----|-----------|-----------|
| 2 | 2 | 37 | tight — a 2-pallet solution exists and 2 is optimal |
| 2 | 3 | 38 | open — the interesting ones, is 2 achievable under all constraints? |
| 3 | 3 | 25 | tight — 3 is optimal |

The 38 open instances are the useful ones for benchmarking: if your model finds
a 2-pallet solution it beats the heuristic; if it proves 3, it closes the gap.

## Instance parameters

25 sizes (10, 12, 14, 16, 18, 20, 22, 25, 28, 30, 34, 38, 42, 46, 50, 55, 60,
65, 70, 75, 80, 85, 90, 95, 100 boxes) × 4 instances per size, with different
(height, volume mix, seed) configurations within each size.

* Footprint is always the Euro pallet **1200 × 800 mm**, grid 50 mm
  (24 × 16 cells), all box dimensions snapped **up** to the grid, so a packing
  found on the grid is always realisable.
* Stacking height: **1800 mm in 65 of the 100 instances**; the rest use a
  reduced usable height (1500/1200/900/750/450 mm) with the payload scaled as
  `1000 kg × H / 1800`.
* Volume mixes: literature classes 1–3 of Elhedhli et al. Table 3 (38
  instances) plus four bulky mixes `M1`, `B1`, `B2`, `B3` shifted towards volume
  categories 3–5 (62 instances).
* Total volume 0.73–3.62 m³, total weight 148–1166 kg, mean 19 distinct SKUs
  per instance, 2–7 distinct retrieval priority levels.

### Why heights and mixes had to be varied

A full Euro pallet holds 1.73 m³. An average retail case is ~13 dm³, so 10–50
average cases cannot fill even one pallet and any such instance would be
trivially single-pallet — the 2-pallet requirement and the small box count pull
in opposite directions. Two knobs resolve it, both recorded in every instance
file (`meta.volume_mix`, `meta.reduced_height`) rather than hidden:

1. **bulky volume mixes** — real large-format cases, not invented dimensions;
2. **reduced stacking height** — a low-bay storage position or half-height
   stack, with the payload scaled consistently.

Both are deviations from the plain task statement and should be stated as such
if the set is used in the presentation. Instances with `H_mm = 1800` and mix 1,
2 or 3 (`meta.reduced_height = false`) are the ones with no deviation at all;
`index.csv` lets you filter them.

### Which bound binds

The volume bound is binding in 94 of 100 instances, the tall-box footprint
bound in 8 (overlapping cases included) — e.g. `pl003_n010_H450_2_LB3UB3`, a
10-box instance where volume alone gives LB 2 but tall boxes that cannot be
stacked force 3. Those 8 are worth keeping: they show that the `lb_area` term in
the model earns its place.

The weight bound never binds in the base set (retail cases are capped at 25 kg,
so 100 of them stay under 1 tonne). Since a pallet count driven by weight rather
than by space is a distinct phenomenon, `variants_weight/` holds **22
weight-critical twins**: identical geometry, payload lowered so that the weight
bound alone forces 3 pallets (e.g. 300 kg for `pl097`). Read them as a
weight-limited handling device, not as a modified Euro pallet. LB and UB are
certified there too.

## Instance format

```json
{
  "meta": { "name", "n", "seed", "volume_mix", "grid_mm", "reduced_height",
            "bounds": { "lb_volume", "lb_weight", "lb_tall_footprint",
                        "lb_pallets", "ub_pallets_heuristic",
                        "certified_optimum_in": [LB, UB] },
            "reference_solution_stats": { ... } },
  "pallet": { "L_mm", "W_mm", "H_mm", "L_g", "W_g", "H_g", "payload_kg" },
  "boxes": [ { "id", "sku",
               "w_mm", "d_mm", "h_mm",        // snapped up to the grid
               "w_g", "d_g", "h_g",           // grid units, used by the model
               "raw_w_mm", "raw_d_mm", "raw_h_mm",   // before snapping
               "weight_kg", "group",          // food | chemical | other
               "priority", "seq",             // 1 = retrieved first
               "family", "zone", "fragile", "upright_only",
               "load_bearing_kpa", "volume_dm3", "volume_category" } ]
}
```

`pallet_mip.py` uses `w_g, d_g, h_g, weight_kg, group, priority` and the pallet
block; the remaining fields support the extensions (load bearing,
orientation-restricted cases such as liquids, fragile cases, order-picking
across pallets).

## Reference packings

`reference_packings.json` gives, per instance, the placement list
(`id, x, y, z, w, d, h, rot` in grid units), pallets used, stack height and
weight per pallet. Two uses:

* **feasibility baseline** — a known-feasible point for the hard constraints;
* **objective comparison** — the heuristic optimises nothing but pallet count,
  so its stack height, centre of gravity and priority blocking are what your
  multi-objective model should improve on. Mean volume use is 50 % and mean pack
  density 61 % across the set (Gzara et al. report ~69 % pack density on
  industry instances with a column-generation approach).

## Reproducing

```bash
python make_testset.py                      # all 25 sizes, ~15 min
python make_testset.py 10,12,14 0 a         # sizes subset, index offset, tag
python verify_testset.py .
python weight_variants.py . 3
```

Deterministic: every instance is fixed by (n, height, mix, seed) recorded in
`index.csv` and in the file name, e.g. `pl042_n034_H1800_B1_LB2UB3`.
