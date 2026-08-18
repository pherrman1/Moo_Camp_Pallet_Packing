# Euro-pallet example experiment

## Instance and assumptions

The experiment follows the assignment's recommendation to begin with a small, self-constructed, single-pallet instance. The pallet measures `120 x 80 x 180 cm`. All boxes remain upright, and loading or retrieval is assumed to occur from above.

| Case ID | Quantity | Dimensions (cm) | Weight per box | Product group | Retrieval priority |
|---:|---:|---:|---:|---|---:|
| 0 | 4 | `60 x 40 x 30` | 25 kg | Chemicals | 2 (later) |
| 1 | 4 | `60 x 40 x 20` | 8 kg | Food | 1 (earlier) |

The four heavy chemical boxes tile the complete pallet footprint at `z = 0`. The four lighter food boxes tile the same footprint at `z = 30 cm`.

## Results

- Pallets used: **1**.
- Packed height: **50 cm** out of the permitted **180 cm**.
- Maximum boundary violation: **0 cm**.
- Box overlap: **none**; the full arrangement passes the inherited CQM feasibility check.
- Support for every raised box: **100%**, exceeding the required **75%**.
- Vertical product rule: all food boxes are above all chemical boxes.
- Weight rule: 25 kg boxes are below 8 kg boxes.
- Accessibility assumption: priority-1 boxes are on the top layer and can be removed before priority-2 boxes.
- Used volume: `480,000 cm^3`.
- Full-pallet volume utilization: `480,000 / (120 x 80 x 180) = 27.78%`.
- Occupied-envelope utilization: `480,000 / (120 x 80 x 50) = 100%`.

The last two measures illustrate the assignment's distinction between overall pallet utilization and space-saving compactness: the load fills only 27.78% of the maximum legal pallet volume, but it is perfectly compact within its actual 50 cm packed height.

## Limitation

The geometric CQM verifies orientation, boundary, and non-overlap constraints. The support, weight, product-group, and retrieval checks are evaluated explicitly for this constructed experiment but are not yet encoded as general CQM constraints for arbitrary input data.
