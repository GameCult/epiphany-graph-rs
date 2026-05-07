# Tuning Doctrine

This file records tuning hypotheses, sweep results, and working defaults for
the realtime 3D layout solver.

The goal is not "fast" in the abstract. The goal is to preserve the visible
large-scale structure of the graph while spending as little frame time as we can
get away with. Fast nonsense is still nonsense, just with better posture.

## Current Measurement Harness

Run:

```bash
cargo run --release --example tuning_sweep
```

The sweep compares approximate repulsion against exact all-pairs repulsion on
fixed positions. It reports:

- elapsed time
- mean absolute error
- max absolute error
- mean relative error
- RMS relative error
- max relative error

The exact solver is the ground truth for repulsion force at the same positions.
This does not yet evaluate full layout trajectory divergence, structural-force
error, edge readability, or user-facing visual quality.

## Force Scale Doctrine

Different force categories should not share one approximation strategy.

- **Near-field forces** should be local and precise:
  collision, node spacing, short springs, local cluster cohesion.

- **Far-field forces** should preserve large-scale structure:
  broad repulsion, weak component separation, community/body relationships,
  core/periphery pressure.

- **Hierarchy forces** should remain explicit:
  rank pressure, ordering pressure, edge direction constraints.

- **Fold/body forces** should operate on analyzed structure:
  SCCs, communities, biconnected components, core shells, and fold groups.

The practical split:

```text
local precision     -> exact/local neighborhoods/grid
global field        -> Barnes-Hut/coarse bodies
semantic structure  -> analysis-derived fold/body constraints
hierarchy           -> Sugiyama-derived constraints
```

Do not tune this as one force soup. That is how the machine becomes fast and
wrong in a way that looks expensive.

## Preliminary Sweep

Date: 2026-05-07

Command:

```bash
cargo run --release --example tuning_sweep
```

Datasets:

- `layered_dag_256`: shallow hierarchy with rank/order-like spread.
- `clustered_fold_512`: dense local folded bodies.
- `uniform_cloud_1024`: broad 3D distribution.
- `uniform_cloud_4096`: larger broad 3D distribution.

Candidate set:

- exact
- Barnes-Hut theta: `0.35`, `0.50`, `0.65`, `0.80`, `1.00`, `1.20`
- spatial grid: `(cell=120, radius=1)`, `(cell=180, radius=1)`,
  `(cell=240, radius=1)`, `(cell=180, radius=2)`

Representative results:

| Dataset | Candidate | Time | Mean Rel Error | RMS Rel Error | Note |
|---|---:|---:|---:|---:|---|
| layered_dag_256 | exact | 275 us | 0.000 | 0.000 | Exact is cheap here. |
| layered_dag_256 | grid 180/r2 | 557 us | 0.419 | 0.492 | Best approximate error, slower than exact. |
| layered_dag_256 | Barnes-Hut theta 1.0 | 533 us | 0.579 | 0.699 | No win at this size. |
| clustered_fold_512 | exact | 897 us | 0.000 | 0.000 | Exact still cheap. |
| clustered_fold_512 | grid 180/r2 | 5814 us | 0.062 | 0.081 | Accurate but slow from dense local neighborhoods. |
| clustered_fold_512 | grid 240/r1 | 691 us | 0.087 | 0.110 | Best practical approximate candidate. |
| clustered_fold_512 | Barnes-Hut theta 1.2 | 1546 us | 0.856 | 1.012 | Bad fit for dense folded local bodies. |
| uniform_cloud_1024 | exact | 5079 us | 0.000 | 0.000 | Exact still competitive. |
| uniform_cloud_1024 | Barnes-Hut theta 1.2 | 3235 us | 0.444 | 0.632 | First speed win, high error. |
| uniform_cloud_1024 | grid 180/r2 | 4010 us | 0.770 | 0.848 | Faster than exact, worse global error. |
| uniform_cloud_4096 | exact | 60155 us | 0.000 | 0.000 | Too slow for per-frame use. |
| uniform_cloud_4096 | Barnes-Hut theta 1.2 | 18804 us | 0.309 | 0.451 | Best large-scale candidate in this sweep. |
| uniform_cloud_4096 | Barnes-Hut theta 0.8 | 31793 us | 0.328 | 0.460 | More cost, little accuracy gain. |
| uniform_cloud_4096 | grid 180/r2 | 27519 us | 0.782 | 0.828 | Faster than exact, poor far-field preservation. |

## Hypotheses Tested

### Hypothesis 1: exact repulsion remains viable for small graphs

Result: confirmed.

On this machine, exact all-pairs repulsion is still hard to beat at 256-512
nodes and remains competitive at 1024. The implementation is simple, cache
friendly, and has no tree/grid build overhead.

Doctrine:

- Use exact repulsion for small graphs and quality baselines.
- Do not automatically pay approximation overhead below roughly 1000 nodes
  without measuring first.

### Hypothesis 2: Barnes-Hut preserves far-field structure better than grid

Result: directionally confirmed for broad large graphs, not for dense local
folds.

At `uniform_cloud_4096`, Barnes-Hut theta `1.2` beat exact runtime by about
3.2x while keeping mean relative error around `0.309`. Spatial grid was faster
than exact but had much worse mean relative error around `0.782`.

Doctrine:

- Use Barnes-Hut for large broad layouts where global repulsion matters.
- Keep grid as a local/near-field approximation, not the primary large-scale
  structure force.

### Hypothesis 3: lower Barnes-Hut theta is always better

Result: rejected for this preliminary harness.

Lower theta increased runtime sharply but did not buy enough error reduction in
the tested cases. For `uniform_cloud_4096`, theta `1.2` was fastest and had the
best mean relative error among tested Barnes-Hut candidates. That is suspicious
enough to keep watching, but clear enough not to default to tiny theta.

Doctrine:

- Keep default `barnes_hut_theta = 0.65` conservative for now.
- Benchmark theta `0.8`, `1.0`, and `1.2` on real Epiphany graphs before
  lowering theta out of superstition.

### Hypothesis 4: dense folded clusters need local treatment

Result: confirmed.

Barnes-Hut performed poorly on `clustered_fold_512`, while spatial grid with
larger cells handled local dense structure much better. Dense local bodies need
near-field precision or body-aware forces; far-field aggregation alone does not
understand folds.

Doctrine:

- For folded bodies, use local exact/grid interactions inside the body.
- Use Barnes-Hut between bodies or for broad background repulsion.
- Multilevel layout should promote fold groups into coarse particles, then
  solve internals locally.

## Current Recommended Defaults

Current crate defaults remain:

```rust
repulsion_mode = RepulsionMode::BarnesHut
barnes_hut_theta = 0.65
barnes_hut_max_depth = 24
grid_cell_size = 180.0
grid_radius = 1
```

Supervised tuning recommendation:

- `< 1000 nodes`: try `Exact` first.
- `1000-4000 broad nodes`: sweep Barnes-Hut theta `0.8`, `1.0`, `1.2`.
- dense folded bodies: use grid/local exact internally.
- mixed graphs: split force categories instead of picking one global repulsion
  approximation.

## Next Tuning Work

1. Add force-category sweeps:
   - local exact radius
   - Barnes-Hut far-field theta
   - component/body repulsion scale
   - community cohesion scale
   - hierarchy strength

2. Add multilevel measurement:
   - compare raw node Barnes-Hut against fold-group coarse repulsion plus local
     body solves.

3. Add trajectory metrics:
   - compare final coordinates after N ticks, not only one-step force error.
   - measure rank preservation, component separation, edge length variance, and
     fold group compactness.

4. Add real graph corpora:
   - small DAG
   - cyclic concept cluster
   - mixed hierarchy/community graph
   - large Epiphany memory graph

The doctrine for now is simple: exact is the truth, Barnes-Hut is the large
scale field, grid is local blunt force, and fold groups are the path to making
large graphs readable without making every node audition for camera time.

## Organ Battleground

Date: 2026-05-07

Harness:

```bash
.\scripts\fetch_tuning_datasets.ps1
cargo run --release --example organ_battleground
```

Actual datasets:

- SNAP `email-Eu-core`: directed email network from a European research
  institution, 1,005 nodes and 25,571 edges.
- SNAP `p2p-Gnutella08`: directed Gnutella peer-to-peer network snapshot from
  August 8 2002, 6,301 nodes and 20,777 edges. The battleground currently
  limits it to the first 3,000 remapped nodes for runtime.

Synthetic fixtures:

- `synthetic_layered_512`: hierarchy-heavy layered graph.
- `synthetic_clustered_1024`: clustered ring/fold graph.

Candidate families:

- raw exact node repulsion
- raw Barnes-Hut node repulsion
- raw spatial-grid node repulsion
- structured body candidates:
  intra-body exact/grid plus inter-body coarse repulsion

Representative results:

| Dataset | Candidate | Time | Mean Rel Error | RMS Rel Error | Note |
|---|---:|---:|---:|---:|---|
| synthetic_layered_512 | exact | 1557 us | 0.000 | 0.000 | Baseline. |
| synthetic_layered_512 | Barnes-Hut theta 1.0 + body exact | 3774 us | 8.151 | 12.443 | Body forces are not raw-force approximations here. |
| synthetic_layered_512 | structured theta 0.65, body exact 0.20, body far 0.50 | 2852 us | 5.075 | 7.702 | Less bad, still semantic not physical. |
| synthetic_clustered_1024 | exact | 5651 us | 0.000 | 0.000 | Baseline. |
| synthetic_clustered_1024 | spatial grid | 2765 us | 0.491 | 0.498 | Best raw approximation among tested. |
| synthetic_clustered_1024 | structured grid body | 13920 us | 0.962 | 0.973 | Too much body overhead for this fixture. |
| snap_email_eu_core | exact | 3405 us | 0.000 | 0.000 | Baseline still cheap around 1k nodes. |
| snap_email_eu_core | spatial grid | 920 us | 0.489 | 0.567 | Fastest useful approximation. |
| snap_email_eu_core | Barnes-Hut theta 1.2 | 5978 us | 0.571 | 0.756 | Slower than exact here. |
| snap_p2p_gnutella08 sampled | exact | 29018 us | 0.000 | 0.000 | Baseline too expensive per frame. |
| snap_p2p_gnutella08 sampled | Barnes-Hut theta 1.0 | 17847 us | 0.669 | 0.828 | Speed win, high error. |
| snap_p2p_gnutella08 sampled | spatial grid | 2737 us | 0.784 | 0.941 | Huge speed win, poor far field. |

### Hypothesis 8: body organs approximate exact node repulsion cheaply

Result: rejected.

The first body-force organs do not approximate raw exact node repulsion. They
add semantic pressure, so comparing them only against exact physical force makes
them look bad, because they are intentionally changing the field.

Doctrine:

- Do not treat body forces as a direct approximation of exact all-node
  repulsion.
- Measure body forces with structural metrics: component separation, fold group
  compactness, rank preservation, edge length variance, and camera-readable
  body spacing.

### Hypothesis 9: actual graph fixtures change the policy

Result: confirmed.

On real SNAP fixtures, exact remains viable around 1k nodes, spatial grid is a
very strong local approximation, and Barnes-Hut begins to matter on the larger
Gnutella sample. This matches the synthetic sweep direction, but the real graphs
make the crossover less theoretical and more irritatingly specific.

Doctrine:

- Keep exact for small real graphs.
- Use grid when local structure dominates and frame budget is tight.
- Use Barnes-Hut for larger broad graphs where far-field structure matters.
- Do not enable body organs by default as an "accuracy" optimization. They need
  structure-quality metrics and likely multilevel coarse solving.

### Next Organ Work

The next battleground must score structure, not only force error:

- fold group compactness before/after tick
- distance between community centers
- rank error versus Sugiyama target Y
- edge length variance
- bridge/articulation readability
- node displacement from exact trajectory after N ticks

The organs are real, but they are not proven useful yet. Good. Better a failed
organ in a jar than one quietly installed in the patient.

## Second Sweep: More Knobs

Date: 2026-05-07

New knobs:

- `barnes_hut_near_radius`: prevents Barnes-Hut aggregation inside a local
  radius, forcing recursion toward exact local behavior.
- `near_repulsion_scale`: scales exact/local pairwise repulsion.
- `far_repulsion_scale`: scales aggregate Barnes-Hut far-field repulsion.

New candidates added:

- Barnes-Hut theta `0.8`, `1.0`, `1.2`
- near radius `120` or `240`
- far scale `0.85` or `1.0`

Representative second-pass results:

| Dataset | Candidate | Time | Mean Rel Error | RMS Rel Error | Note |
|---|---:|---:|---:|---:|---|
| layered_dag_256 | exact | 178 us | 0.000 | 0.000 | Exact still wins small. |
| layered_dag_256 | Barnes-Hut theta 1.0, near 180, far 1.0 | 556 us | 0.560 | 0.676 | Best plain Barnes-Hut balance here. |
| layered_dag_256 | Barnes-Hut theta 1.0, near 240, far 1.0 | 644 us | 0.555 | 0.673 | Tiny accuracy gain, more cost. |
| layered_dag_256 | grid 180/r2 | 452 us | 0.419 | 0.492 | Still better local approximation than Barnes-Hut. |
| clustered_fold_512 | exact | 821 us | 0.000 | 0.000 | Exact still cheap. |
| clustered_fold_512 | Barnes-Hut theta 1.2, near 120, far 0.85 | 1910 us | 0.916 | 0.946 | Damping far field helps a little, still poor. |
| clustered_fold_512 | grid 240/r1 | 568 us | 0.087 | 0.110 | Best practical local folded-body candidate. |
| uniform_cloud_1024 | exact | 3433 us | 0.000 | 0.000 | Exact remains viable. |
| uniform_cloud_1024 | Barnes-Hut theta 1.2, near 240, far 1.0 | 2633 us | 0.476 | 0.658 | Speed win, high error. |
| uniform_cloud_1024 | Barnes-Hut theta 1.2, near 120, far 0.85 | 3270 us | 0.475 | 0.686 | Far damping did not clearly help. |
| uniform_cloud_4096 | exact | 70970 us | 0.000 | 0.000 | Too slow per frame. |
| uniform_cloud_4096 | Barnes-Hut theta 1.2, near 180, far 1.0 | 18528 us | 0.343 | 0.486 | Best broad large graph speed/accuracy point here. |
| uniform_cloud_4096 | Barnes-Hut theta 1.0, near 180, far 1.0 | 28078 us | 0.336 | 0.472 | More time, modest accuracy gain. |
| uniform_cloud_4096 | Barnes-Hut theta 1.0, near 120, far 0.85 | 20274 us | 0.366 | 0.517 | Far damping worsened force accuracy. |
| uniform_cloud_4096 | Barnes-Hut theta 1.0, near 240, far 1.0 | 36039 us | 0.350 | 0.483 | Larger near radius cost more without enough gain. |

### Hypothesis 5: larger local exact radius improves Barnes-Hut enough to pay

Result: mostly rejected.

Increasing `barnes_hut_near_radius` from `180` to `240` often increased runtime
with only tiny accuracy improvement, and sometimes worsened mean relative error.
The current implementation already recurses for target-containing octants, so a
large near radius is not a free lunch. It is lunch with a service charge and
slightly cold fries.

Doctrine:

- Keep `barnes_hut_near_radius = 180.0` as the default.
- Sweep near radius on real dense graphs, but do not raise it globally.

### Hypothesis 6: reducing far-field scale preserves local accuracy

Result: rejected for force-ground-truth accuracy.

`far_repulsion_scale = 0.85` sometimes reduced absolute force magnitude, but it
did not improve relative error against exact all-pairs ground truth. On broad
4096-node clouds it worsened mean/RMS relative error compared to far scale `1.0`.

Doctrine:

- Keep `far_repulsion_scale = 1.0` for physical force approximation.
- Use far-field scale as an artistic/readability control later, not as an
  accuracy knob.

### Hypothesis 7: exact/grid/Barnes-Hut should be selected per force category

Result: strengthened.

The second sweep made the split more obvious:

- exact wins small graphs.
- grid wins dense local folded bodies.
- Barnes-Hut wins large broad far-field layouts.
- far-field damping and larger near radius do not replace category separation.

Doctrine:

- Add a hybrid repulsion architecture next:
  local exact/grid inside fold groups and near neighborhoods;
  Barnes-Hut between distant bodies;
  coarse fold-group repulsion for global structure.
