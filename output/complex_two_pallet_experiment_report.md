# Complex two-Euro-pallet experiment

## Purpose

This experiment extends the small single-pallet baseline to 20 boxes of five different sizes. The box volume is deliberately greater than the entire legal volume of one `120 x 80 x 180 cm` Euro pallet, so at least two pallets are required independently of the chosen packing method.

## Box data and operational assumptions

| Case ID | Quantity | Dimensions (cm) | Weight | Product group | Retrieval priority |
|---:|---:|---:|---:|---|---:|
| 0 | 4 | `60 x 40 x 50` | 32 kg | Chemicals | 4 |
| 1 | 2 | `60 x 80 x 40` | 17 kg | Food | 2 |
| 2 | 2 | `120 x 40 x 60` | 38 kg | Chemicals | 5 |
| 3 | 8 | `30 x 40 x 30` | 14 kg | General goods | 3 |
| 4 | 4 | `60 x 40 x 20` | 6 kg | Food | 1 |

Priority 1 means the boxes should be retrieved earliest. Loading and retrieval are assumed to occur from above. All boxes remain upright in this baseline even though the inherited geometric model permits six axis-aligned orientations.

## Layout

Pallet 1 contains four case-0 boxes as a complete base layer and two case-1 boxes as a complete upper layer. Its packed height is 90 cm.

Pallet 2 contains two case-2 boxes as a complete base layer, eight case-3 boxes as a complete middle layer, and four case-4 boxes as a complete top layer. Its packed height is 110 cm.

Every layer completely tiles the `120 x 80 cm` footprint. Consequently, each raised box has 100% base support, exceeding the required 75%. Heavy chemical boxes are on the bottom, lighter food boxes are above them, and the earliest-priority food boxes are on the top layer.

## Results

- Boxes packed: **20**.
- Different box sizes: **5**.
- Pallets used: **2**.
- Total box volume: **1,920,000 cm³**.
- One-pallet legal capacity: **1,728,000 cm³**.
- Volume-based lower bound: `ceil(1,920,000 / 1,728,000) = 2 pallets`.
- Two-pallet full-volume utilization: **55.56%**.
- Pallet 1 packed height: **90 cm**.
- Pallet 2 packed height: **110 cm**.
- Maximum allowed height: **180 cm**.
- Raised-box support: **100%**.
- Boundary violations: **none**.
- Box overlaps: **none**.
- Full inherited CQM feasibility check: **passed**.

The occupied-envelope utilization is 100% on each pallet because all layers cover the complete footprint: pallet 1 occupies `120 x 80 x 90 cm`, and pallet 2 occupies `120 x 80 x 110 cm`. The lower 55.56% value instead measures utilization against two pallets loaded to their maximum legal height of 180 cm, illustrating why utilization and compactness should be reported separately.

## Limitation

This is a deterministic feasible baseline rather than an optimized result because no D-Wave Leap token is configured. Geometry, orientation, assignment, boundary, and non-overlap are validated by the inherited CQM. Support, weight, product-group, and retrieval rules are verified explicitly for this experiment but are not yet general constraints in the model.
