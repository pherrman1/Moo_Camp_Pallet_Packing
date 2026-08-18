# Literature review and recommended modelling route

## Executive conclusion

The assignment is best classified as a **distributor’s pallet-loading problem** or **mixed-case palletization problem**: a single-bin-size, heterogeneous three-dimensional bin-packing problem with orthogonal placement, partial base support, product compatibility, weight-ordering preferences, and retrieval priorities. It is substantially different from the classical **manufacturer’s pallet-loading problem**, which usually concerns identical boxes arranged in two-dimensional pallet layers.

The brief asks for roughly 1,000 heterogeneous boxes on (120\times80) cm pallets with maximum height (180) cm, at least (75%) base support per box, product- and weight-dependent vertical preferences, accessibility by retrieval priority, and multiple competing objectives. 

For a four-day modelling project, I recommend presenting a hierarchy of models:

1. **A compact coordinate-based MILP** as the classical theoretical baseline.
2. **A position-indexed 0–1 ILP** as your principal exact model for small instances. This is the cleanest way to model the stated (75%) support condition exactly.
3. **A layer- or pallet-pattern ILP** as the scalable model for realistic orders, solved using restricted pattern generation, column generation, or decomposition.

Do not promise to solve a completely heterogeneous 1,000-box instance to proven optimality with a monolithic MILP. The palletization literature explicitly treats medium and large practical instances using layers, column generation, matheuristics, and other decompositions because direct exact models become intractable. ([PubsOnline][1])

The strongest modelling-week story is:

> “We first formulate the physical problem exactly on a discretized space, then show why the exact model does not scale, and derive a layer/pattern decomposition for realistic instances.”

---

# 1. Literature map

## 1.1 Foundational MILP formulations

| Reference                                                                                                                                    | Main formulation and relevance                                                                                                                                                                                                                                                               | Limitation for this assignment                                                                                                                            |
| -------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Chen, Lee and Shen (1995), “An analytical model for the container loading problem”**                                                       | Foundational coordinate-based zero-one MILP. It models heterogeneous boxes, multiple containers, box orientation, boundary constraints, and pairwise non-overlap through relative-position binaries. This is the standard starting point for a compact formulation. ([discovery.fiu.edu][2]) | It does not naturally express support by the union of several lower boxes. Exact supported area introduces products of overlap lengths.                   |
| **Junqueira, Morabito and Yamashita (2012), “Three-dimensional container loading models with cargo stability and load bearing constraints”** | Extends exact loading models to vertical and horizontal stability and load-bearing constraints. The models can also be used for pallet loading without requiring strict horizontal layers. This is the closest classical paper to the assignment’s support requirement. ([ScienceDirect][3]) | Placement-indexed models become large and were demonstrated only for moderate instances.                                                                  |
| **Paquay, Schyns and Limbourg (2016), “A mixed integer programming formulation … deriving from an air cargo application”**                   | A comprehensive coordinate MILP including rotations, stability, fragility and weight distribution. It is useful as an example of how practical constraints are appended to a compact geometric formulation. ([Wiley Online Library][4])                                                      | The reported exact testing concerns small instances, and its air-cargo geometry is more general than required here.                                       |
| **Kurpel et al. (2020), “The exact solutions of several types of container loading problems”**                                               | Studies exact formulations using several discretizations of feasible box positions. Candidate-position reduction considerably decreases the number of variables relative to a full unit grid. ([ScienceDirect][5])                                                                           | Still an exact-placement approach and therefore chiefly suitable for small or moderately heterogeneous instances.                                         |
| **Nascimento, de Queiroz and Junqueira (2021), “Practical constraints in the container loading problem”**                                    | A particularly useful formulation catalogue. It combines ILP and constraint programming and covers twelve constraints, including priorities, stability, load bearing, multi-drop order, load balance, grouping, separation, and orientations. ([ScienceDirect][6])                           | It is a single-container output-maximization setting rather than exactly the minimum-pallet problem, but most constraint constructions transfer directly. |

## 1.2 Palletization-specific scalable models

| Reference                                                                                                                                | Main formulation and relevance                                                                                                                                                                                                                                                                                                                   | Limitation or adaptation needed                                                                                                                                                                                                    |
| ---------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Elhedhli, Gzara and Yildiz (2019), “Three-Dimensional Bin Packing and Mixed-Case Palletization”**                                      | Introduces a layer-based formulation and column-generation approach. The pricing problem is a two-dimensional layer-generation problem. Layers allow support, product-family grouping, aisle friendliness and load-bearing requirements to be incorporated. ([PubsOnline][1])                                                                    | The resulting problem is not a single small MILP: it is a branch-price or column-generation framework.                                                                                                                             |
| **Gzara, Elhedhli and Yildiz (2020), “The Pallet Loading Problem: Three-dimensional bin packing with practical constraints”**            | Directly addresses distributor pallet loading with support, load bearing, sequence information and weight limits. Its support procedure uses layer placement and second-order conic optimization, while load propagation is checked using a graph. ([UWSpace][7])                                                                                | Its strongest support model is not a pure ILP. This is useful evidence that exact continuous supported-area geometry is difficult to retain in a compact linear formulation.                                                       |
| **Dell’Amico and Magnani (2021), and Dell’Amico et al. (2026)**                                                                          | Formulate an exponential-size layer ILP with binaries (x_{\ell pk}) indicating that layer (\ell) is placed on pallet (p) at level (k). The objective is the number of pallets, with height, weight, stackability, stability and compression constraints. The practical algorithm creates only a restricted subset of layers. ([Iris Unimore][8]) | Their layer stability condition compares aggregate areas of consecutive layers. That is not equivalent to requiring every individual upper box to have (75%) support. Your compatibility calculation should therefore be stricter. |
| **Calzavara et al. (2021/2025), “Mathematical models and heuristic algorithms for pallet building problems with practical constraints”** | Decomposes pallet building into layer creation and layer-to-pallet assignment. It models product families, contiguous groups and visibility from a layer boundary, which are useful interpretations of accessibility. ([Springer][9])                                                                                                            | Its loading rules are motivated by robotized palletizing and impose a specific layered structure.                                                                                                                                  |

## 1.3 Accessibility, balance and stability literature

| Reference                                     | Contribution                                                                                                                                                                                                                                                                                |
| --------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Bonet Filella, Trivella and Corman (2023)** | Argues for treating unloading restrictions as soft constraints. Their MILP penalizes relocations rather than requiring a perfect no-blocking packing; this avoids destroying utilization merely to satisfy a rigid unloading order. ([UT Research Info][10])                                |
| **Trivella and Pisinger (2016)**              | Formulates load-balanced multidimensional bin packing, minimizing the number of bins while keeping the centre of mass near a target point. It supplies a defensible alternative to a vague “heavy boxes below” rule. ([Welcome to DTU Research Database][11])                               |
| **Mazur et al. (2025)**                       | Distinguishes full base support, partial base support, static mechanical equilibrium and physical simulation. The assignment’s (75%) rule is precisely a partial-base-support criterion. Values around (0.55)–(0.8), including (0.75), occur in the literature. ([Universität zu Köln][12]) |
| **Bortfeldt and Wäscher (2013)**              | The principal review of practical container-loading constraints. It is useful for your literature-review section and for classifying constraints as container-, item-, cargo- and load-related. ([ScienceDirect][13])                                                                       |
| **Zhao et al. (2016)**                        | Comparative review focused on container-loading algorithms and solution methodology. Useful for explaining why practical methods generally use blocks, layers, extreme points and decomposition rather than a direct monolithic MILP. ([Eprints Soton][14])                                 |

---

# 2. Modelling assumptions to state explicitly

A good mathematical model here depends more on transparent assumptions than on adding every possible constraint.

I would use the following core assumptions:

1. **Axis-aligned boxes.** Every edge is parallel to a pallet edge or the vertical axis.
2. **No pallet overhang.** Every box lies completely within (120\times80\times180).
3. **Default rotation rule:** boxes may be rotated (90^\circ) around the vertical axis, but may not be tipped onto another face. This preserves the designated top face and is common in palletization. Run a sensitivity experiment allowing all six orthogonal orientations.
4. **Integer dimensions and positions.** For the exact model, use a grid such as 5 or 10 cm, or use dimension-derived candidate coordinates.
5. **Direct support.** A box at height (z>0) receives support only from boxes whose upper face is exactly at height (z), or from the pallet when (z=0).
6. **Support is geometric.** The supported fraction is the area of the union of the contacts below divided by the box’s base area.
7. **Top retrieval.** Retrieval occurs from above. A later-priority box located vertically above an earlier-priority box can block it. This must be stated because “accessibility” is otherwise undefined.
8. **Product ordering is local.** “Food above chemicals” matters where their horizontal projections overlap or where one directly supports the other. Requiring every food box on a pallet to have a greater (z)-coordinate than every chemical box would be unnecessarily strong.
9. **No quantitative compression model unless data are supplied.** The task gives weights but no box-specific load-bearing capacities. Do not invent such capacities.
10. **The 180 cm convention must be fixed.** State whether it includes the wooden pallet itself; the brief does not specify this.

The first five are natural hard assumptions. Rotation, access direction and the interpretation of food/chemical separation should be discussed as modelling decisions.

---

# 3. Formulation A: compact coordinate-based MILP

This is the clearest literature baseline, following the Chen/Paquay family.

## 3.1 Sets and data

Let:

* (I): boxes;
* (P={1,\ldots,\bar P}): candidate pallets;
* (O_i): allowed orientations of box (i);
* (L=120,\ W=80,\ H=180);
* ((l_{io},w_{io},h_{io})): dimensions of box (i) in orientation (o).

## 3.2 Variables

[
a_{ip}=
\begin{cases}
1,&\text{if box }i\text{ is assigned to pallet }p,\
0,&\text{otherwise,}
\end{cases}
]

[
y_p=
\begin{cases}
1,&\text{if pallet }p\text{ is used,}\
0,&\text{otherwise,}
\end{cases}
]

[
r_{io}=1 \quad\Longleftrightarrow\quad
\text{orientation }o\text{ is used for box }i.
]

Let (X_i,Y_i,Z_i\ge0) be the lower-left-bottom coordinates of box (i) in its pallet.

For each pair (i<j), pallet (p), and direction

[
d\in D={x^+,x^-,y^+,y^-,z^+,z^-},
]

let (\delta_{ijp}^{d}) indicate the relative position. For example, (\delta_{ijp}^{x^+}=1) means that (i) lies completely to the left of (j).

The oriented dimensions are

[
\ell_i=\sum_{o\in O_i}l_{io}r_{io},\qquad
w_i=\sum_{o\in O_i}w_{io}r_{io},\qquad
h_i=\sum_{o\in O_i}h_{io}r_{io}.
]

## 3.3 Assignment, orientation and boundary constraints

[
\sum_{p\in P}a_{ip}=1
\qquad i\in I,
]

[
a_{ip}\le y_p
\qquad i\in I,\ p\in P,
]

[
\sum_{o\in O_i}r_{io}=1
\qquad i\in I,
]

[
0\le X_i\le L-\ell_i,
]

[
0\le Y_i\le W-w_i,
]

[
0\le Z_i\le H-h_i.
]

## 3.4 Non-overlap

For every (i<j) and (p),

[
X_i+\ell_i
\le X_j+L(1-\delta_{ijp}^{x^+}),
]

[
X_j+\ell_j
\le X_i+L(1-\delta_{ijp}^{x^-}),
]

and analogously for (y) and (z).

If both boxes are on the same pallet, at least one separating relation must hold:

[
\sum_{d\in D}\delta_{ijp}^{d}
\ge a_{ip}+a_{jp}-1.
]

Also impose

[
\delta_{ijp}^{d}\le a_{ip},\qquad
\delta_{ijp}^{d}\le a_{jp}.
]

The basic objective is

[
\min \sum_{p\in P}y_p.
]

Symmetry can be reduced through

[
y_p\ge y_{p+1}.
]

## 3.5 Why this should not be your final support model

Suppose box (i) overlaps box (j) below it by lengths

[
s^x_{ij}
========

\max\left{0,,
\min(X_i+\ell_i,X_j+\ell_j)-\max(X_i,X_j)
\right},
]

and analogously (s^y_{ij}).

The contact area is

[
s^x_{ij}s^y_{ij}.
]

This is bilinear. Furthermore, if several boxes support (i), the relevant quantity is the area of a **union** of rectangles. Consequently, exact (75%) support is awkward in a compact coordinate MILP. One can use approximations, piecewise linearization, or additional discretization, but the resulting model loses the simplicity that justified the coordinate formulation.

Therefore:

> Use the coordinate model as the classical baseline and explain that support motivates switching to a position-indexed model.

---

# 4. Formulation B: recommended position-indexed 0–1 ILP

This should be the main exact formulation in the project.

It is exact **relative to the chosen candidate-position grid**. For a self-constructed instance whose dimensions are multiples of 10 cm, a 10 cm grid gives an exact representation of that discretized instance.

## 4.1 Candidate placements

For each box (i), enumerate all feasible placements

[
q=(i,p,o,x,y,z)
]

such that the oriented box fits inside pallet (p).

Let (Q_i) be all placements of item (i), and let (Q_p) be the placements on pallet (p).

For each placement (q), precompute:

* item (i(q));
* pallet (p(q));
* coordinates (x_q,y_q,z_q);
* dimensions (\ell_q,w_q,h_q);
* base rectangle (F_q);
* base area (A_q=\ell_qw_q);
* top height (t_q=z_q+h_q);
* centre coordinates;
* item weight, product type and priority.

## 4.2 Variables

[
x_q=
\begin{cases}
1,&\text{if placement }q\text{ is selected,}\
0,&\text{otherwise,}
\end{cases}
]

and (y_p\in{0,1}) as before.

## 4.3 Exact placement constraints

Each box is placed once:

[
\sum_{q\in Q_i}x_q=1
\qquad i\in I.
]

A placement activates its pallet:

[
x_q\le y_{p(q)}
\qquad q\in Q.
]

## 4.4 Non-overlap

Precompute the conflict set

[
\mathcal C=
\bigl{
{q,r}:
p(q)=p(r),\
\operatorname{int}(B_q)\cap\operatorname{int}(B_r)\ne\varnothing
\bigr},
]

where (B_q) is the three-dimensional occupied cuboid.

Then impose

[
x_q+x_r\le1
\qquad {q,r}\in\mathcal C.
]

This can be strengthened using clique or grid-cell constraints. For every elementary spatial cell (c),

[
\sum_{q:,c\subseteq B_q}x_q\le1.
]

Cell inequalities can represent many pairwise conflicts in one constraint and often give a stronger relaxation.

## 4.5 Exact (75%) support

For a placement (q) with (z_q>0), define

[
S(q)=
\left{
r:
p(r)=p(q),
z_r+h_r=z_q,
i(r)\ne i(q),
\operatorname{area}(F_q\cap F_r)>0
\right}.
]

Precompute

[
a_{qr}=\operatorname{area}(F_q\cap F_r).
]

The support constraint is

[
\boxed{
\sum_{r\in S(q)}a_{qr}x_r
\ge
0.75,A_q,x_q
}
\qquad q:\ z_q>0.
]

No support constraint is required when (z_q=0), because the pallet supplies full support.

This is linear because (a_{qr}) is a constant. The selected lower boxes cannot overlap in their interiors, so their positive-area contacts do not double-count the same part of (F_q).

This formulation implements exactly the partial-base-support interpretation in the task. Partial-base support is a recognized but simplified stability criterion; it should not be presented as a full mechanical-equilibrium model. ([Universität zu Köln][12])

## 4.6 Used height

Introduce (H_p\ge0):

[
H_p\ge (z_q+h_q)x_q
\qquad q\in Q_p,
]

[
H_p\le H y_p.
]

For the maximum pallet height, introduce (H^{\max}) with

[
H^{\max}\ge H_p
\qquad p\in P.
]

---

# 5. Product, weight and accessibility constraints

## 5.1 Food above chemicals

Let (\mathcal B^{FC}) contain pairs ((q,r)) such that:

* (q) contains food;
* (r) contains chemicals;
* (q) is below (r);
* their horizontal projections overlap.

Such a pair corresponds to the undesirable event “chemical box above food box.”

Introduce

[
v_{qr}^{FC}\ge x_q+x_r-1,
\qquad
v_{qr}^{FC}\ge0.
]

Then

[
f_{FC}=\sum_{(q,r)\in\mathcal B^{FC}}v_{qr}^{FC}
]

counts these violations when minimized.

For a strict rule, replace this with

[
x_q+x_r\le1
\qquad (q,r)\in\mathcal B^{FC}.
]

The soft version better matches the word “preferably” in the brief.

A useful sensitivity comparison is:

* **global strict:** no food may be lower than any chemical on the same pallet;
* **overlap strict:** only vertically aligned pairs are forbidden;
* **soft overlap:** aligned inversions are penalized.

The global version will probably waste substantial space and gives you an instructive modelling comparison.

## 5.2 Heavy boxes lower down

A clean linear proxy is the total vertical mass moment:

[
f_{\mathrm{vert}}
=================

\sum_{q\in Q}
m_{i(q)}
\left(z_q+\frac{h_q}{2}\right)x_q.
]

After the number of pallets is fixed, minimizing (f_{\mathrm{vert}}) moves mass downward.

This is preferable to a hard statement such as

> every heavier box must be lower than every lighter box,

which would be both overly restrictive and physically unjustified.

A second, more local measure penalizes heavy-on-light contacts. Let (\mathcal B^W) consist of pairs where:

* (q) is directly above (r);
* they have positive contact area;
* (m_{i(q)}>m_{i(r)}).

Use

[
v_{qr}^{W}\ge x_q+x_r-1,
]

and minimize

[
f_W=\sum_{(q,r)\in\mathcal B^W}
\omega_{qr}v_{qr}^{W},
]

where, for example,

[
\omega_{qr}
===========

\max{0,m_{i(q)}-m_{i(r)}}.
]

The global vertical moment and the local inversion count measure different things and can be compared experimentally.

## 5.3 Horizontal load balance

For pallet (p), define the mass moment around the horizontal midpoint:

[
M_p^x=
\sum_{q\in Q_p}
m_{i(q)}
\left(x_q+\frac{\ell_q}{2}-\frac L2\right)x_q,
]

[
M_p^y=
\sum_{q\in Q_p}
m_{i(q)}
\left(y_q+\frac{w_q}{2}-\frac W2\right)x_q.
]

Introduce

[
T_p^x\ge M_p^x,\qquad T_p^x\ge-M_p^x,
]

and similarly for (T_p^y). Then

[
f_{\mathrm{balance}}
====================

\sum_p(T_p^x+T_p^y)
]

pulls the centre of mass toward the centre of the pallet. Centre-of-mass balancing has been formulated directly as a MILP objective in multidimensional bin packing. ([Welcome to DTU Research Database][11])

## 5.4 Retrieval accessibility

Let priority (\pi_i) be smaller for earlier retrieval.

Under top retrieval, a later box (j) blocks an earlier box (i) if:

* (\pi_i<\pi_j);
* (j) is above (i);
* their horizontal footprints overlap.

Precompute all blocking placement pairs:

[
\mathcal B^A=
\left{
(q,r):
\pi_{i(q)}<\pi_{i(r)},
z_r\ge z_q+h_q,
\operatorname{area}(F_q\cap F_r)>0
\right}.
]

Introduce

[
v_{qr}^{A}\ge x_q+x_r-1.
]

Possible accessibility objectives are:

### Unweighted inversion count

[
f_A^{(1)}
=========

\sum_{(q,r)\in\mathcal B^A}v_{qr}^{A}.
]

### Priority-distance-weighted inversions

[
f_A^{(2)}
=========

\sum_{(q,r)\in\mathcal B^A}
|\pi_{i(r)}-\pi_{i(q)}|,v_{qr}^{A}.
]

### Expected relocation effort

If (c_i) estimates retrieval frequency and (m_j) or (V_j) estimates relocation effort,

[
f_A^{(3)}
=========

\sum_{(q,r)\in\mathcal B^A}
c_{i(q)}
\left(\lambda_m m_{i(r)}+\lambda_V V_{i(r)}\right)
v_{qr}^{A}.
]

The third is the most operational, but the first is easier to explain.

Literature on multi-drop loading finds that strict no-relocation constraints may substantially restrict packing, while soft penalties permit a direct trade-off between unloading work and utilization. ([UT Research Info][10])

For the modelling week, use (f_A^{(1)}) as the principal measure and mention the others as alternatives.

---

# 6. The multi-objective issue

The task specifically asks you to distinguish “few pallets,” “good utilization,” and “space-saving.” This is an important mathematical point.

Let

[
V_{\mathrm{tot}}=\sum_{i\in I}l_iw_ih_i
]

be fixed. If all pallets are identical and all boxes must be packed, global utilization based on the full (180) cm height is

[
U_{\mathrm{full}}
=================

\frac{V_{\mathrm{tot}}}
{LWH\sum_p y_p}.
]

Therefore,

[
\max U_{\mathrm{full}}
\quad\Longleftrightarrow\quad
\min\sum_p y_p.
]

So **full-pallet utilization and pallet count are not independent objectives**.

That is one of the central observations your presentation should contain.

## 6.1 A meaningful distinction

Use:

### Number of pallets

[
f_1=\sum_p y_p.
]

This measures transport/storage units.

### Vertical compactness or “space-saving”

[
f_2=\sum_p H_p
]

or

[
f_2^{\max}=H^{\max}.
]

Minimizing total occupied height minimizes the aggregate envelope

[
LW\sum_p H_p.
]

An associated effective utilization is

[
U_{\mathrm{envelope}}
=====================

\frac{V_{\mathrm{tot}}}
{LW\sum_p H_p}.
]

### Pallet fill balance

At a fixed number of pallets, define

[
V_p=\sum_{q\in Q_p}V_{i(q)}x_q.
]

You may maximize the minimum pallet fill, or report

[
\min_{p:y_p=1}\frac{V_p}{LWH}
]

as a post-processing measure. This distinguishes a solution with one nearly empty pallet from one with evenly loaded pallets, even though aggregate utilization is identical.

### Operational quality

[
f_3=f_A,\qquad
f_4=f_{FC},\qquad
f_5=f_{\mathrm{vert}},\qquad
f_6=f_{\mathrm{balance}}.
]

## 6.2 Recommended solution method: lexicographic optimization

Avoid putting all raw objectives directly into one arbitrary weighted sum.

A defensible sequence is:

### Stage 1: minimum pallet count

[
P^\star
=======

\min\sum_p y_p.
]

### Stage 2: compactness

Add

[
\sum_p y_p=P^\star
]

and minimize

[
\sum_p H_p
]

or (H^{\max}).

### Stage 3: operational quality

Allow a small compactness tolerance

[
\sum_p H_p
\le (1+\varepsilon_H)H^\star_{\mathrm{sum}},
]

and minimize

[
\lambda_A f_A+
\lambda_{FC}f_{FC}+
\lambda_W f_{\mathrm{vert}}+
\lambda_B f_{\mathrm{balance}}.
]

A stronger experimental design is an (\varepsilon)-constraint analysis:

* fix pallet count;
* impose several bounds on accessibility violations;
* minimize compactness under each bound;
* display the resulting Pareto curve.

That gives a much better modelling-week result than reporting one arbitrarily weighted solution.

---

# 7. Formulation C: scalable layer ILP

The position model will grow rapidly because the number of possible placements is approximately

[
|I|\times|P|\times|\text{orientations}|
\times|\text{candidate coordinates}|.
]

For the realistic problem, use a layer model.

## 7.1 Candidate layers

Let (\mathcal L) be a set of feasible two-dimensional layers.

For each layer (\ell), precompute:

* (a_{t\ell}): number of boxes of type (t);
* (h_\ell): layer height;
* (m_\ell): layer weight;
* the complete two-dimensional arrangement;
* its priority and product composition;
* the exact top support surfaces.

For clean support geometry, it is useful to construct layers from boxes with a common oriented height. Otherwise, shorter boxes in a layer do not touch the layer above and cannot contribute support.

Let

[
c_{\ell m}=
\begin{cases}
1,&\text{if upper layer }m\text{ may be placed on lower layer }\ell,\
0,&\text{otherwise.}
\end{cases}
]

Do **not** define this solely through aggregate layer area.

Instead, set (c_{\ell m}=1) only if every upper box (i) satisfies

[
\operatorname{area}
\left(
F_i^m
\cap
\bigcup_{j\in\ell:,t_j=h_\ell}F_j^\ell
\right)
\ge0.75,\operatorname{area}(F_i^m).
]

Because the arrangements are known, this is a preprocessing calculation.

## 7.2 Variables

Let

[
x_{\ell pk}=1
]

when layer (\ell) is loaded at level (k) of pallet (p), and (y_p=1) when pallet (p) is used.

## 7.3 Layer ILP

Minimize

[
\min \sum_p y_p.
]

Pack the required number of each type:

[
\sum_{p,k,\ell}
a_{t\ell}x_{\ell pk}
====================

n_t
\qquad t\in T.
]

Respect pallet height:

[
\sum_{k,\ell}h_\ell x_{\ell pk}
\le H y_p
\qquad p\in P.
]

At most one layer per level:

[
\sum_{\ell}x_{\ell pk}\le1.
]

No skipped levels:

[
\sum_{\ell}x_{\ell,p,k+1}
\le
\sum_{\ell}x_{\ell pk}.
]

Support compatibility:

[
x_{m,p,k+1}
\le
\sum_{\ell:c_{\ell m}=1}x_{\ell pk}
\qquad m,p,k.
]

Pallet symmetry:

[
y_p\ge y_{p+1}.
]

The direct recent layer ILP has essentially this (x_{\ell pk}) structure, with additional weight, stackability, stability and compression conditions. Its main computational issue is the exponential layer set. ([Iris Unimore][8])

## 7.4 Generating layers

A practical four-day implementation does not need full column generation. Use a **restricted master problem**:

1. Sort boxes by height class, footprint, weight, product group and priority.
2. Generate layers using a two-dimensional first-fit or extreme-point heuristic.
3. Generate several randomized variants.
4. Retain nondominated layers according to:

   * number/volume of boxes;
   * density;
   * product compatibility;
   * priority homogeneity;
   * load-bearing quality.
5. Solve the layer ILP on this restricted pool.
6. Generate additional layers for types with high dual prices or large residual counts.

This is a matheuristic. Unless the layer-generation problem is solved exhaustively or through column generation, it does not guarantee global optimality. State that explicitly.

Layer-based column generation is the principal scalable direction in the mixed-case palletization literature, precisely because the pricing problem can be reduced to a two-dimensional layer-generation problem. ([PubsOnline][1])

---

# 8. An even cleaner allocation model: pallet patterns

For extension **(c), allocation of the entire delivery**, define a **pallet pattern** as a complete feasible pallet, including geometry and all hard constraints.

Let (\mathcal S) be the set of feasible patterns and let (a_{ts}) be the number of type-(t) boxes in pattern (s).

Use

[
\lambda_s\in\mathbb Z_+
]

for the number of times pattern (s) is selected:

[
\min\sum_{s\in\mathcal S}\lambda_s
]

subject to

[
\sum_{s\in\mathcal S}a_{ts}\lambda_s=n_t
\qquad t\in T.
]

This is a set-covering or set-partitioning master problem.

For distinct individual boxes rather than types, use binary (\lambda_s) and require each box to appear in exactly one chosen pattern.

Advantages:

* no explicit pallet-index symmetry;
* allocation and physical packing are separated cleanly;
* each pattern can carry precomputed accessibility, compactness and product penalties;
* the master can handle 1,000 boxes or aggregated item types even when producing a feasible pattern is difficult.

The subproblem is to generate a new feasible pallet pattern with good reduced cost. For the modelling week, pattern generation can use your layer heuristic rather than a complete branch-and-price implementation.

This is the most natural extension to choose from the assignment.

---

# 9. What not to over-model

## Exact compression

Weight alone is not sufficient to infer whether a box can support another box. You would need:

* a box-specific maximum top load;
* possibly strength as a function of contact area;
* a rule for distributing the weight of an upper box among several supporting boxes.

Without this data, use heavy-low preferences rather than claiming to enforce structural load-bearing.

## Dynamic stability

Acceleration and braking require assumptions about friction, centres of mass, rigid-body mechanics and possibly the loading sequence. The stability literature distinguishes partial base support from static mechanical equilibrium and physical simulation; they are not interchangeable. ([Universität zu Köln][12])

This is too large an extension for four days unless another group member has a mechanics focus.

## Global food-above-chemical ordering

A requirement such as

[
z_i\ge z_j+h_j
]

for every food box (i) and every chemical box (j) on the same pallet would force all chemicals into a global lower region. This may be an interesting strict benchmark, but it should not be the default interpretation.

## Exact 1,000-box monolithic MILP

Even direct practical pallet models usually use layer generation, restricted layer pools, column generation, heuristics or matheuristics. Exact coordinate and position formulations are mainly used to establish models, solve small instances and produce bounds. ([PubsOnline][1])

---

# 10. Computational improvements

For either exact formulation:

### Aggregate identical items

If many boxes share dimensions, weight, product group and priority band, model counts rather than individual labels where possible.

### Candidate-coordinate reduction

Instead of every centimetre, use coordinates derived from subset sums of oriented box dimensions:

[
X=
\left{
\sum_i n_i\ell_i:
0\le \sum_i n_i\ell_i\le L
\right},
]

and analogous sets for (Y) and (Z).

Placement-discretization methods are specifically used in exact container-loading formulations to reduce the variable count. ([ScienceDirect][5])

### Heuristic upper bound

Construct a feasible solution before solving the MILP. Its pallet count gives (\bar P) and eliminates unnecessary candidate pallets.

### Volume lower bound

[
P_{\mathrm{vol}}
================

\left\lceil
\frac{\sum_iV_i}{LWH}
\right\rceil.
]

This is weak because it ignores geometry and support, but it gives a simple baseline.

### Symmetry breaking

Use:

[
y_p\ge y_{p+1},
]

and optionally impose an ordering on pallet loads:

[
\sum_iV_i a_{ip}
\ge
\sum_iV_i a_{i,p+1}.
]

For identical boxes, order their selected placement indices.

### Warm starts and staged solving

First solve without accessibility and product penalties. Feed that packing as a warm start to the full model.

### Solve assignment and packing separately

For 1,000 boxes:

1. assign boxes to provisional pallets using volume, weight, priority and product groups;
2. solve each pallet-packing subproblem;
3. if a pallet is infeasible, generate a no-good cut or alter the assignment.

This provides a simple Benders-like decomposition without requiring a full formal Benders implementation.

---

# 11. Experimental programme

A small but scientifically useful experiment matrix would be:

| Experiment                                             | Question                                                                                        |
| ------------------------------------------------------ | ----------------------------------------------------------------------------------------------- |
| Support (\alpha=0.50,0.75,1.00)                        | How much packing efficiency is lost by stricter support? Keep (0.75) as the required main case. |
| Yaw-only versus six orientations                       | How valuable is tipping boxes, and is the gain operationally acceptable?                        |
| Hard versus soft food/chemical rule                    | How costly is strict vertical separation?                                                       |
| No priority versus soft priority versus hard priority  | How much accessibility conflicts with compactness?                                              |
| Vertical moment versus direct heavy-on-light penalties | Which interpretation of “heavy below light” changes solutions most?                             |
| Coordinate MILP versus grid ILP                        | Which model is smaller, and which handles support correctly?                                    |
| Exact placement ILP versus restricted layer ILP        | What is lost or gained by restricting solutions to layers?                                      |

Report at least:

* pallets used;
* total and maximum occupied height;
* full-volume and envelope utilization;
* number of priority inversions;
* food/chemical violations;
* vertical centre-of-mass height;
* mean support fraction and minimum support fraction;
* solve time;
* optimality gap;
* model size: binaries and constraints.

A useful graph is a Pareto plot with accessibility violations on one axis and total occupied height or pallet count on the other.

---

# 12. Four-day plan

## Monday, 17 August: settle formulation and assumptions

Deliverables by the end of Monday:

* one-page problem classification;
* explicit assumptions on rotations, retrieval and pallet height;
* exact position-indexed ILP written mathematically;
* self-constructed instance of approximately 10–15 boxes;
* implementation skeleton and placement generator.

Read first:

1. Junqueira, Morabito and Yamashita;
2. Elhedhli, Gzara and Yildiz;
3. Gzara, Elhedhli and Yildiz;
4. Nascimento, de Queiroz and Junqueira;
5. the support sections of Mazur et al.

## Tuesday: get the exact geometric model working

Implement:

* placement generation;
* exactly-one-placement constraints;
* non-overlap;
* pallet activation;
* (75%) support;
* pallet-count objective;
* 3D visualization or coordinate output.

Validate using hand-checkable examples:

* one box fully supported;
* one box exactly (75%) supported;
* one box supported by two lower boxes;
* one placement with (74%) support, which must be rejected;
* a bridge placement supported by two separated boxes.

Do not add all preference objectives before these tests pass.

## Wednesday: add the operational objectives

Implement:

* occupied height;
* priority inversions;
* food/chemical violations;
* vertical mass moment;
* optional horizontal load balance.

Run lexicographic and (\varepsilon)-constraint experiments on approximately 15–30 boxes, depending on the grid and solver performance.

Prepare the first result tables and visualizations.

## Thursday: scaling model

Implement a basic layer generator and restricted layer ILP.

A sufficient result would be:

* exact ILP solved on small instances;
* restricted layer model solved on substantially larger instances;
* comparison of solution quality, runtime and model restrictions;
* one experiment using perhaps 100–300 boxes or aggregated box types;
* conceptual set-partitioning extension for the entire 1,000-box delivery.

A complete branch-and-price implementation is not necessary for a convincing modelling-week submission.

## Friday: presentation

A strong 20-minute structure is:

1. **2 minutes:** real problem and assumptions;
2. **3 minutes:** relation to 3D bin packing and palletization literature;
3. **5 minutes:** exact position-indexed ILP, especially support;
4. **3 minutes:** multi-objective definitions and why utilization duplicates pallet count;
5. **3 minutes:** computational results and Pareto trade-offs;
6. **2 minutes:** layer/pattern scaling approach;
7. **2 minutes:** limitations, discarded alternatives and extension.

---

# 13. Recommended final model

For the written formulation, I would use:

### Hard constraints

* every box placed exactly once;
* allowed rotations only;
* pallet boundaries;
* non-overlap;
* maximum height;
* (75%) support;
* product incompatibilities only where truly prohibited.

### Primary objective

[
\min\sum_p y_p.
]

### Secondary objective

[
\min\sum_p H_p.
]

### Tertiary objectives

[
\min
\left(
\lambda_A f_A+
\lambda_{FC}f_{FC}+
\lambda_W f_{\mathrm{vert}}+
\lambda_B f_{\mathrm{balance}}
\right).
]

### Scaling method

Restricted layer generation followed by the layer ILP, with the entire-delivery allocation represented as a pallet-pattern set-partitioning problem.

That gives you:

* a mathematically precise exact model;
* an explicit treatment of every requirement in the brief;
* a meaningful multi-objective distinction;
* a defensible explanation of scalability;
* a natural extension for the full 1,000-box delivery;
* several modelling choices that can be compared experimentally rather than asserted without evidence.

[1]: https://pubsonline.informs.org/doi/10.1287/ijoo.2019.0013 "https://pubsonline.informs.org/doi/10.1287/ijoo.2019.0013"
[2]: https://discovery.fiu.edu/display/pub64482 "https://discovery.fiu.edu/display/pub64482"
[3]: https://www.sciencedirect.com/science/article/abs/pii/S0305054810001486 "https://www.sciencedirect.com/science/article/abs/pii/S0305054810001486"
[4]: https://onlinelibrary.wiley.com/doi/abs/10.1111/itor.12111 "https://onlinelibrary.wiley.com/doi/abs/10.1111/itor.12111"
[5]: https://www.sciencedirect.com/science/article/abs/pii/S0377221719310136 "https://www.sciencedirect.com/science/article/abs/pii/S0377221719310136"
[6]: https://www.sciencedirect.com/science/article/pii/S0305054820303038 "https://www.sciencedirect.com/science/article/pii/S0305054820303038"
[7]: https://uwspace.uwaterloo.ca/bitstreams/136f5f5f-19a8-418e-be0e-309219d79683/download "https://uwspace.uwaterloo.ca/bitstreams/136f5f5f-19a8-418e-be0e-309219d79683/download"
[8]: https://iris.unimore.it/retrieve/aad7d261-b410-4f8d-aed3-f1a0fba2ef20/s10732-026-09586-5.pdf "https://iris.unimore.it/retrieve/aad7d261-b410-4f8d-aed3-f1a0fba2ef20/s10732-026-09586-5.pdf"
[9]: https://link.springer.com/article/10.1007/s10479-021-04349-w "https://link.springer.com/article/10.1007/s10479-021-04349-w"
[10]: https://research.utwente.nl/en/publications/modeling-soft-unloading-constraints-in-the-multi-drop-container-l/ "https://research.utwente.nl/en/publications/modeling-soft-unloading-constraints-in-the-multi-drop-container-l/"
[11]: https://orbit.dtu.dk/en/publications/the-load-balanced-multi-dimensional-bin-packing-problem/ "https://orbit.dtu.dk/en/publications/the-load-balanced-multi-dimensional-bin-packing-problem/"
[12]: https://kups.ub.uni-koeln.de/80554/1/Int%20Trans%20Operational%20Res%20-%202025%20-%20Mazur%20-%20Standing%20on%20a%20common%20ground%20a%20comparison%20of%20static%20stability%20approaches%20for.pdf "https://kups.ub.uni-koeln.de/80554/1/Int%20Trans%20Operational%20Res%20-%202025%20-%20Mazur%20-%20Standing%20on%20a%20common%20ground%20a%20comparison%20of%20static%20stability%20approaches%20for.pdf"
[13]: https://www.sciencedirect.com/science/article/abs/pii/S037722171200937X "https://www.sciencedirect.com/science/article/abs/pii/S037722171200937X"
[14]: https://eprints.soton.ac.uk/364226/ "https://eprints.soton.ac.uk/364226/"
