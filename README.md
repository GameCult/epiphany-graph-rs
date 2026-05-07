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

1. Assign ranks from directed edges using a longest-path DAG pass.
2. Preserve partial hierarchy for cyclic leftovers with local edge pressure.
3. Order nodes inside each rank with repeated barycenter sweeps.
4. Generate rank, order, and edge-direction constraints.
5. Relax the graph with deterministic force iterations.

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

## Notes

The first release favors a small deterministic core over a sprawling layout
cathedral. The public API exposes generated constraints so callers can inspect,
reuse, tune, or replace the force relaxation phase later.

The current repulsion pass is exact pairwise repulsion. That is simple and
stable for modest graphs; larger graphs should move toward a Barnes-Hut or grid
approximation before pretending heroism is a scalability plan.
