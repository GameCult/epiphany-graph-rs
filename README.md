# epiphany-graph-rs

Hybrid graph layout for DAGs that still want to breathe.

This crate treats Sugiyama as a constraint generator instead of a final layout.
The Sugiyama pass assigns ranks and same-rank order; the force pass relaxes
those constraints with springs, repulsion, collision, and edge direction
pressure.

The result is a spectrum:

- force-dominant for organic structure
- Sugiyama-dominant for hierarchical DAGs
- hybrid for the usual graph reality, where purity gets mugged in the hallway

## Current Pipeline

1. Analyze graph anatomy: weak components, SCCs, bridges, articulation points,
   edge roles, degree metrics, and cyclic nodes.
2. Assign ranks from directed edges using a longest-path DAG pass.
3. Preserve partial hierarchy for cyclic leftovers with local edge pressure.
4. Order nodes inside each rank with repeated barycenter sweeps.
5. Generate rank, order, and edge-direction constraints.
6. Relax the graph with deterministic force iterations.
7. Emit fold groups for renderer-side collapse, expansion, highlighting, or
   protein-ish folding.

## Example

```rust
use epiphany_graph_rs::{layout, Graph, LayoutConfig};

let mut graph = Graph::new();
let a = graph.add_node(1.0);
let b = graph.add_node(1.0);
let c = graph.add_node(1.0);

graph.add_edge(a, b);
graph.add_edge(b, c);

let result = layout(&graph, &LayoutConfig::default());

for node in result.nodes {
    println!("{:?}: ({}, {}) rank={}", node.id, node.x, node.y, node.rank);
}
```

## 3D Layout

Use `layout_3d` when the renderer has a real spatial scene, such as Bevy.
Hierarchy stays on `Y`; organic spread happens across `X/Z`; grounding and
depth forces keep the graph shallow instead of letting it become decorative fog.

```rust
use epiphany_graph_rs::{layout_3d, Graph, Layout3dConfig};

let mut graph = Graph::new();
let root = graph.add_node(1.0);
let child = graph.add_node(1.0);

graph.add_edge(root, child);

let result = layout_3d(&graph, &Layout3dConfig::default());

for node in result.nodes {
    let [x, y, z] = node.as_xyz();
    println!("{:?}: ({x}, {y}, {z}) rank={}", node.id, node.rank);
}
```

For Bevy, map `NodeLayout3d::as_xyz()` directly into `Vec3::new(x, y, z)`.
The crate does not depend on Bevy just to borrow its vector type; the layout
core stays small and renderer-agnostic.

## Realtime Solver

Use `Layout3dSolver` when Bevy needs to iterate layout over frames:

```rust
use epiphany_graph_rs::{Graph, Layout3dConfig, Layout3dSolver};

let graph = Graph::new();
let mut solver = Layout3dSolver::new(graph, Layout3dConfig::default());

// In a Bevy system, run a small number of iterations per frame.
solver.tick(2);

for position in solver.positions() {
    let (x, y, z) = *position;
    // Transform::from_translation(Vec3::new(x, y, z))
}
```

The solver caches graph analysis, Sugiyama constraints, ranks, order, adjacency,
positions, and force buffers. Use `with_initial_positions` to warm-start after a
graph edit. That keeps the layout from starting over like it suffered a small
bureaucratic head injury.

`Layout3dConfig::repulsion_mode` controls the expensive part:

- `RepulsionMode::BarnesHut` is the default realtime path.
- `RepulsionMode::SpatialGrid` keeps a simpler local approximation available.
- `RepulsionMode::Exact` keeps all-pairs repulsion for small graphs or quality
  comparisons.

Barnes-Hut uses an octree and `barnes_hut_theta` as the opening-angle knob.
Lower theta values are more accurate; higher values are faster. Large graphs
still want multilevel coarsening, because even good approximation does not make
every visual decision worth rendering individually.

Additional scale knobs split local and global behavior:

- `barnes_hut_near_radius` forces Barnes-Hut to recurse near the target instead
  of aggregating local neighborhoods.
- `near_repulsion_scale` controls exact pairwise/local repulsion.
- `far_repulsion_scale` controls Barnes-Hut aggregate far-field repulsion.

## Accuracy Sweeps

Use the repulsion accuracy sweep to compare approximation settings against exact
all-pairs repulsion on the same positions:

```rust
use epiphany_graph_rs::{
    repulsion_accuracy_sweep, Layout3dConfig, RepulsionAccuracyCandidate,
};

let positions = [
    [0.0, 0.0, 0.0],
    [120.0, 20.0, 40.0],
    [-80.0, 10.0, -60.0],
];

let reports = repulsion_accuracy_sweep(
    &positions,
    &Layout3dConfig::default(),
    &[
        RepulsionAccuracyCandidate::barnes_hut(0.4),
        RepulsionAccuracyCandidate::barnes_hut(0.7),
        RepulsionAccuracyCandidate::barnes_hut(1.0),
        RepulsionAccuracyCandidate::barnes_hut_tuned(1.0, 240.0, 0.85),
        RepulsionAccuracyCandidate::spatial_grid(180.0, 1),
    ],
);

for report in reports {
    println!(
        "{:?}: mean_rel={} rms_rel={} max_rel={} elapsed={:?}",
        report.candidate.repulsion_mode,
        report.mean_relative_error,
        report.rms_relative_error,
        report.max_relative_error,
        report.elapsed
    );
}
```

For a live Bevy-style solver, call `solver_repulsion_accuracy_sweep(&solver,
&candidates)` to evaluate the current positions without advancing the layout.
This is the tuning harness: sweep theta, grid radius, and later force-category
scale knobs until the error/time curve stops paying rent.

Use `organ_battleground` for structure-aware force experiments:

```bash
.\scripts\fetch_tuning_datasets.ps1
cargo run --release --example organ_battleground
```

It compares raw node approximations and body-force candidates on synthetic
fixtures plus SNAP email/Gnutella datasets when available.

## Graph Analysis And Folding

The crate exposes `analyze(&graph)` for layout-independent structure reads:

- weak components
- strongly connected components
- biconnected components
- label-propagation communities
- k-core shells
- articulation points
- bridge edges
- per-node degree metrics
- local clustering coefficients
- edge roles
- cyclic node flags

`layout_3d` uses that same analysis to add structural pressure:

- SCC cohesion folds cycles and tight feedback regions into local bodies.
- weak component cohesion keeps broad regions visually legible.
- community cohesion gives dense modules local territory.
- k-core anchoring lets dense nuclei read differently from peripheral leaves.
- local clustering cohesion tightens triangle-rich neighborhoods.
- bridge hinges keep connector edges readable between bodies.
- centrality anchoring keeps important nodes from drifting into decorative exile.
- cycle folding uses depth so feedback does not have to lose every argument
  against hierarchy.

The result includes `fold_groups`, which are renderer-facing bodies:

```rust
for group in result.fold_groups {
    println!(
        "{:?}: center={:?} radius={} nodes={}",
        group.kind,
        group.center,
        group.radius,
        group.nodes.len()
    );
}
```

Fold groups currently cover weak components, strongly connected components,
biconnected components, communities, core shells, and cycle bodies. They are
meant to be useful handles for Bevy interactions: collapse a component, pulse a
cycle, dim a subtree, or unfold a dense region without asking the raw node soup
for permission.

See [RESEARCH.md](RESEARCH.md) for the paper trail and the next deeper cuts:
multilevel coarsening, constrained stress, edge bundling, adaptive solvers, and
layout memory. See [TUNING_DOCTRINE.md](TUNING_DOCTRINE.md) for measured
performance/accuracy tradeoffs and current tuning policy.

## Notes

The first release favors a small deterministic core over a sprawling layout
cathedral. The public API exposes generated constraints so callers can inspect,
reuse, tune, or replace the force relaxation phase later.

The realtime solver defaults to Barnes-Hut repulsion. Exact pairwise repulsion
is still available for small graphs and quality comparisons.
