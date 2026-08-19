"""Create an exactly 50-item, 10-SKU stress instance from generated MCPP data."""

from __future__ import annotations

import argparse
import json
from copy import deepcopy
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("target", type=Path)
    args = parser.parse_args()

    payload = json.loads(args.source.read_text(encoding="utf-8"))
    prototypes = {}
    for item in payload["items"]:
        prototypes.setdefault(int(item["sku"]), item)
    selected_skus = sorted(prototypes)[:10]
    if len(selected_skus) != 10:
        raise ValueError(f"source contains only {len(selected_skus)} distinct SKUs")

    items = []
    for new_sku, source_sku in enumerate(selected_skus, start=1):
        for copy_index in range(5):
            item = deepcopy(prototypes[source_sku])
            item["id"] = len(items) + 1
            item["sku"] = new_sku
            items.append(item)

    result = deepcopy(payload)
    result["name"] = "mcpp_50_boxes_10_types"
    result["source"] = "10 generated MCPP prototypes replicated five times"
    result["seed"] = 50
    result["items"] = items
    result["statistics"] = {}
    args.target.parent.mkdir(parents=True, exist_ok=True)
    args.target.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"wrote {args.target} with {len(items)} boxes and {len(selected_skus)} SKUs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
