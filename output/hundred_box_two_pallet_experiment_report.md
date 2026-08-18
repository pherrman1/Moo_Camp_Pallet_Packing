# 100-box, 20-size, two-Euro-pallet experiment

## Instance design

This deterministic baseline contains exactly 100 boxes and 20 distinct `(length, width, height)` combinations. Fifty boxes are assigned to each `120 x 80 x 180 cm` Euro pallet. Each type forms one complete layer using one of three footprint tilings:

- four `60 x 40 cm` boxes in a `2 x 2` grid;
- five `24 x 80 cm` boxes in five vertical strips;
- six `40 x 40 cm` boxes in a `3 x 2` grid.

Each pallet has ten layers. Layer heights vary from 11 to 20 cm, making all 20 three-dimensional box sizes distinct. Both pallets have a packed height of 155 cm, leaving 25 cm below the legal maximum.

## Operational assumptions

Within each pallet, weight decreases from the bottom layer toward the top. The lowest three layers represent chemical products, the middle four represent general goods, and the highest three represent food. Retrieval priority increases toward the top, with the top layer retrieved first. Loading and retrieval are assumed to occur from above, and all boxes remain upright.

Because every layer completely tiles the pallet footprint, every raised box has 100% support. This exceeds the assignment's 75% minimum support rule. It also avoids unsupported bridges and overhangs.

## Results

- Boxes packed: **100**.
- Distinct box sizes: **20**.
- Boxes per pallet: **50 and 50**.
- Pallets used: **2**.
- Total box volume: **2,976,000 cm³**.
- One-pallet legal capacity: **1,728,000 cm³**.
- Volume lower bound: `ceil(2,976,000 / 1,728,000) = 2 pallets`.
- Two-pallet maximum-volume utilization: **86.11%**.
- Packed height of each pallet: **155 cm**.
- Maximum permitted packed height: **180 cm**.
- Free height on each pallet: **25 cm**.
- Raised-box support: **100%**.
- Boundary violations: **none**.
- Box overlaps: **none**.
- Full inherited CQM feasibility check: **passed**.

The occupied-envelope utilization is 100% because the boxes completely fill both `120 x 80 x 155 cm` occupied envelopes. The lower 86.11% utilization measures the same load against two pallets at the full legal height of 180 cm. This distinction separates compactness from maximum-capacity utilization.

## Model-size observation

With 100 boxes, the inherited pairwise CQM creates 4,950 box pairs and six relative-position selectors for every pair. The experiment records the resulting variable and constraint counts at runtime. This illustrates the assignment's scaling concern: even a deterministic feasibility check becomes substantially larger as the number of boxes grows.

## Limitation

This is a constructed, CQM-feasible baseline rather than a D-Wave-optimized solution because no Leap token is configured. The CQM validates geometry, orientation, pallet assignment, boundaries, and non-overlap. The support, product, weight, and retrieval properties are guaranteed by the layer construction but are not yet encoded as general optimization constraints.
