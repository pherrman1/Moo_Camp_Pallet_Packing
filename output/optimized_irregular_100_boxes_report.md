# Locally optimized irregular 100-box experiment

## Method

This experiment uses 100 boxes from 20 deliberately awkward size classes. Each class has five boxes. A randomized height-map heuristic performs 24 search trials and chooses the pallet, one of the six axis-aligned orientations, and integer-centimetre coordinates for every box.

The heuristic enforces pallet boundaries, non-overlap, a maximum height of 180 cm, and at least 75% support for every raised box. Its objective first reduces maximum and total packed height and then penalizes chemical boxes above food, heavy boxes above lighter boxes, and later-priority boxes above earlier-priority boxes.

## Results

- Search trials: **24**.
- Feasible trials: **2**.
- Boxes packed: **100 of 100**.
- Distinct original sizes: **20**.
- Boxes on pallet 1: **50**.
- Boxes on pallet 2: **50**.
- Packed height of pallet 1: **179 cm**.
- Packed height of pallet 2: **172 cm**.
- Maximum permitted height: **180 cm**.
- Boxes rotated away from their input orientation: **84**.
- Raised boxes: **81**.
- Minimum support: **75.00%**.
- Average support over all boxes: **93.72%**.
- Raised boxes exactly at the 75% threshold: **3**.
- Chemical-above-food penalty: **0**.
- Remaining weight-order penalty: **88**.
- Remaining retrieval-order penalty: **63**.
- Product composition: **30 chemical, 35 food, and 35 general-goods boxes**.
- Total box volume: **2,217,215 cm³** (calculated from the input instance).
- Utilization of two maximum-height pallets: **64.16%**.
- Volume-based minimum pallet count: **2**.
- Inherited geometric CQM variables: **30,800**.
- Inherited geometric CQM constraints: **65,149**.
- Final inherited CQM feasibility check: **passed**.

## Interpretation

Unlike the perfect-layer baseline, this result contains gaps, many rotations, mixed support percentages, and packed heights close to the legal limit. The 75% support constraint is active rather than automatically satisfied. The search completely avoids placing chemicals above food, but it accepts some weight and retrieval-order inversions to fit all boxes on two pallets. These remaining penalties are the kind of trade-off the assignment asks to analyze.

The result is heuristic, not a proof of global optimality. Only two of the 24 randomized trials succeeded, which indicates that this instance is challenging for the local greedy method. A longer search, a stronger metaheuristic, or D-Wave's hybrid CQM solver after adding the missing operational constraints could potentially improve the height and ordering penalties.

## Reproduction

Run:

```powershell
.\.venv\Scripts\python.exe optimize_irregular_100_boxes.py
```

The random seed is fixed, so the experiment is reproducible.
