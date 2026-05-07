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
