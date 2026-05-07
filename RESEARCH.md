# Research Notes

This crate is moving toward a staged graph anatomy pipeline:

```text
graph -> analysis -> coarse bodies -> constraints -> 3D relaxation -> fold groups
```

The useful lesson from the literature is blunt: do not ask one force soup to
discover every structure at once. Extract graph structure first, then make the
solver express it.

## Landed

These features are implemented in the crate now:

- Persistent `Layout3dSolver` for realtime frame-by-frame iteration.
- Warm-start positions for graph edits and interaction.
- Barnes-Hut octree repulsion for the 3D solver.
- Spatial-grid approximate repulsion as a simpler fallback.
- SCC detection for cyclic bodies.
- weak components for broad spatial islands.
- articulation points and bridges for hinge-like connectors.
- biconnected components for blocks that should fold as more durable bodies.
- k-core numbers and shells for dense nucleus/periphery structure.
- local clustering coefficients for triangle-rich neighborhoods.
- deterministic label-propagation communities.
- fold groups for weak components, SCCs, biconnected blocks, communities,
  core shells, and cycles.

## Paper-Derived Design Moves

### Constrained Stress

Dwyer, Koren, and Marriott show that stress majorization can be augmented with
linear ordering/separation constraints. That gives us a more principled future
solver than hand-added forces when rank, spacing, or no-overlap constraints need
to become harder.

Implementation direction:

- Add a `StressConfig`.
- Convert Sugiyama ranks, ordering, and component separation into weighted
  constraints.
- Use force relaxation as the interactive fast path and constrained stress as
  the higher-quality settle pass.

Source:
https://doi.org/10.1016/j.disc.2007.12.103

### Multilevel Layout

Walshaw's multilevel force-directed method repeatedly coarsens the graph, lays
out the coarse graph, then expands and refines. This is exactly the missing
piece for large graphs and foldable bodies: layout the anatomy before the cells.

Implementation direction:

- Coarsen by SCC, community, biconnected block, or k-core shell.
- Lay out coarse bodies first.
- Expand each body into local coordinates.
- Relax with cross-body hinge and boundary forces.

Source:
https://doi.org/10.7155/jgaa.00070

### Continuous Layout Ergonomics

ForceAtlas2 is not magic; it is a practical assembly of force choices, adaptive
speed, Barnes-Hut approximation, and usability knobs. The useful part for this
crate is adaptive step control and Barnes-Hut for large interactive layouts.

Implementation direction:

- Replace fixed cooling with adaptive speed.
- Tune Barnes-Hut and add benchmarks against exact/grid modes.
- Expose live-tick APIs for Bevy interaction instead of only batch layout.

Current state:

- `Layout3dSolver` exists.
- `RepulsionMode::BarnesHut` is the default realtime path.
- `RepulsionMode::SpatialGrid` remains available as a simpler local approximation.
- `RepulsionMode::Exact` remains available for small graphs and comparisons.

Source:
https://doi.org/10.1371/journal.pone.0098679

### Communities

Louvain and Leiden are the serious modularity route. Leiden improves Louvain by
guaranteeing better-connected communities. The crate currently uses deterministic
label propagation because it is small, fast, dependency-free, and good enough as
a first structural signal. It is not the final boss.

Implementation direction:

- Keep label propagation as the cheap default.
- Add optional Louvain/Leiden-style modularity optimization later.
- Use community stability across multiple passes as a visual confidence score.

Sources:
https://arxiv.org/abs/0803.0476
https://www.nature.com/articles/s41598-019-41695-z
https://doi.org/10.1103/PhysRevE.76.036106

### Core/Periphery

k-core decomposition is cheap and useful for visualization. Dense cores should
read as nuclei; low-core leaves should read as peripheral atmosphere, not equal
citizens in the same spatial argument.

Implementation direction:

- Use core number to scale anchoring, node mass, and opacity hints.
- Add `FoldGroupKind::CoreShell` interactions in Bevy.
- Give high-core regions their own collision shell.

Sources:
https://arxiv.org/abs/cs/0504107
https://www.nature.com/articles/30918

### Edge Bundling

Edge bundling is powerful but dangerous. It can reduce clutter while also making
the underlying graph harder to recover. The right version here is structural and
inspectable: bundles should be generated from known bodies, edge roles, and
hierarchy paths, not vague visual mush.

Implementation direction:

- Add edge control points derived from fold groups.
- Bundle bridge/cross/back edges differently.
- Preserve unbundled edge identity for picking and inspection.
- Treat bundling as renderer geometry, not graph truth.

Sources:
https://www.microsoft.com/en-us/research/publication/improving-layered-graph-layouts-with-edgebundling/
https://arxiv.org/abs/2108.05467

### Dynamic Mental Map

Mental-map preservation is more task-dependent than the usual folklore admits.
The useful move is not "never move nodes"; it is preserving stable landmarks
when the user needs orientation, while allowing meaningful rearrangement when
new structure appears.

Implementation direction:

- Add stable node anchors from previous layouts.
- Preserve fold group centers more strongly than individual leaf nodes.
- Expose a `LayoutMemory` object for incremental Bevy updates.

Source:
https://doi.org/10.1016/j.ijhcs.2013.08.004

## Next High-Value Crate Work

1. Add multilevel coarsening and expansion.
2. Add a live iterative solver API for Bevy.
3. Add Barnes-Hut/grid repulsion.
4. Add edge routing and structural bundling output.
5. Add optional constrained-stress settling.

That is the machine with teeth. Everything else is decoration wearing a badge.
