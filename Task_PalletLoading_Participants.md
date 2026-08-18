# Intelligent Pallet Loading in Retail

*Modeling assignment for groups of 3–4 people · Duration: approx. 4 days · Conclusion: presentation and joint discussion*

## Context

A large retailer receives daily deliveries containing a large variety of different goods. A delivery typically comprises about 1000 boxes. The boxes are cuboid and differ in their length, width, and height. In addition, for each box, among other things, its weight, the product group it contains, and a desired retrieval position or retrieval priority are known.

The boxes must be distributed and stacked on Euro pallets for further storage and internal transport. A Euro pallet has a footprint of 120 cm × 80 cm. The maximum height of a loaded pallet is 180 cm.

Several practical requirements must be taken into account when loading:

- The boxes should be arranged as **space-savingly** as possible.
- A good **utilization** of the available pallet space should be achieved.
- The **number of pallets required** should be kept as low as possible.

The **stability** of the load is also important. A box may not be placed arbitrarily on top of boxes below it. For a stable load, at least 75 % of a box's base area must be supported by the boxes underneath or by the pallet.

Furthermore, there are requirements regarding the **vertical arrangement** of the goods. Food items should preferably be stored above chemical products such as cleaning agents or detergents. Likewise, heavy boxes should be arranged further down and light boxes further up. Depending on the specific modeling, such requirements can be interpreted with varying degrees of strictness.

Finally, it is already known before delivery in which order the goods are expected to be needed and retrieved from the pallets. This order results, among other things, from which areas of the supermarket the respective goods are needed in. The loading should therefore enable good **accessibility** of the boxes according to their retrieval priorities. For the core task, first consider accessibility *within a single pallet*.

## Your Task

Develop a mathematical optimization model for the problem described, which can be used to determine a suitable loading of the Euro pallets.

1. Make the assumptions necessary for your modeling and identify the essential decisions of the problem – whether and how boxes may be rotated is also part of your modeling.
2. Translate the requirements of the real problem into a suitable mathematical formulation.
3. In doing so, pay particular attention to the geometric arrangement and stability of the boxes, the number and utilization of the pallets, the properties of the goods contained, and the desired retrieval priority.
4. Since several requirements can compete with one another, develop a multi-objective formulation of the problem. Justify which requirements you treat as binding conditions and which you treat as optimization criteria. Note that "space-saving," "good space utilization," and "few pallets" sound similar at first glance but do not necessarily measure the same thing – work out how these quantities can be meaningfully distinguished.
5. Define suitable measures wherever terms such as "good space utilization," "space-saving," "stable," or "good accessibility" need to be mathematically operationalized.
6. Develop a suitable solution approach for your model. We recommend first working with a substantially smaller, self-constructed instance (e.g., few box types, a single pallet) before considering transferability to realistic order-of-magnitude sizes (about 1000 boxes).
7. Analyze and discuss the solutions obtained. Compare different modeling decisions and examine how they affect the resulting pallet loadings and the trade-offs involved.

## Extension – choose one option

**(a) Order-picking logic across multiple pallets.** Extend the accessibility consideration from a single pallet to the entire pallet stock.

**(b) Dynamic stability.** Additionally take into account how the load behaves during acceleration and braking during internal transport.

**(c) Allocation of the entire delivery.** Treat the assignment of all roughly 1000 boxes to multiple pallets as a separate resource-allocation problem.

**(d) Full heterogeneity and scaling.** Work with a realistically large, heterogeneous instance and investigate which solution approaches are still practicable for it.

## Goal of the Task

The goal is not to develop a single prescribed model formulation. What matters is the traceable and well-justified translation of the real problem into a mathematical multi-objective optimization problem, as well as the development of a suitable approach to solving it.

## Conclusion

Prepare a presentation (approx. 20 minutes) in which you present your model, your central modeling decisions, and your results. Explicitly address alternatives that you considered but discarded, and justify your choice.
