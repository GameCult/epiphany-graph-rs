//! Hybrid Sugiyama/force-directed graph layout.
//!
//! Sugiyama is used here as a constraint generator: it supplies ranks, local
//! ordering, and edge direction pressure. A deterministic force solver then
//! relaxes those constraints alongside springs, repulsion, and collision.

use std::collections::VecDeque;

/// Index of a node in a [`Graph`].
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct NodeId(pub usize);

/// Index of an edge in a [`Graph`].
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct EdgeId(pub usize);

/// A directed graph with optional per-node weight.
#[derive(Clone, Debug, Default)]
pub struct Graph {
    node_weights: Vec<f32>,
    edges: Vec<(NodeId, NodeId)>,
}

impl Graph {
    /// Create an empty graph.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create an empty graph with capacity for nodes and edges.
    pub fn with_capacity(nodes: usize, edges: usize) -> Self {
        Self {
            node_weights: Vec::with_capacity(nodes),
            edges: Vec::with_capacity(edges),
        }
    }

    /// Add a node and return its stable index.
    pub fn add_node(&mut self, weight: f32) -> NodeId {
        let id = NodeId(self.node_weights.len());
        self.node_weights.push(weight.max(0.001));
        id
    }

    /// Add a directed edge.
    pub fn add_edge(&mut self, source: NodeId, target: NodeId) -> EdgeId {
        assert!(source.0 < self.node_count(), "source node is out of bounds");
        assert!(target.0 < self.node_count(), "target node is out of bounds");
        let id = EdgeId(self.edges.len());
        self.edges.push((source, target));
        id
    }

    /// Number of nodes.
    pub fn node_count(&self) -> usize {
        self.node_weights.len()
    }

    /// Number of edges.
    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }

    /// Directed edges as `(source, target)` pairs.
    pub fn edges(&self) -> &[(NodeId, NodeId)] {
        &self.edges
    }
}

/// Tunable parameters for hybrid layout.
#[derive(Clone, Debug)]
pub struct LayoutConfig {
    /// Number of force relaxation iterations.
    pub iterations: usize,
    /// Distance between Sugiyama ranks.
    pub rank_gap: f32,
    /// Minimum same-rank spacing.
    pub node_gap: f32,
    /// Preferred spring length for one-rank edges.
    pub edge_length: f32,
    /// Force pulling nodes toward their Sugiyama rank coordinate.
    pub rank_strength: f32,
    /// Force preserving Sugiyama order inside each rank.
    pub order_strength: f32,
    /// Force encouraging edges to travel forward through ranks.
    pub edge_direction_strength: f32,
    /// Edge spring force.
    pub spring_strength: f32,
    /// Pairwise repulsion force.
    pub repulsion_strength: f32,
    /// Collision separation force.
    pub collision_strength: f32,
    /// Initial and maximum per-iteration step size.
    pub step_size: f32,
}

impl Default for LayoutConfig {
    fn default() -> Self {
        Self {
            iterations: 350,
            rank_gap: 140.0,
            node_gap: 80.0,
            edge_length: 120.0,
            rank_strength: 0.12,
            order_strength: 0.08,
            edge_direction_strength: 0.04,
            spring_strength: 0.035,
            repulsion_strength: 4800.0,
            collision_strength: 0.35,
            step_size: 0.75,
        }
    }
}

/// Tunable parameters for hybrid 3D layout.
#[derive(Clone, Debug)]
pub struct Layout3dConfig {
    /// Shared 2D-like force parameters.
    pub base: LayoutConfig,
    /// Initial same-rank depth spacing on the Z axis.
    pub depth_gap: f32,
    /// Force pulling nodes toward the ground plane.
    ///
    /// Rank pressure still owns hierarchy on Y. Ground strength simply keeps
    /// the whole structure from drifting into needless altitude.
    pub ground_strength: f32,
    /// Force pulling nodes back toward a shallow Z envelope.
    pub depth_strength: f32,
    /// Extra multiplier for horizontal-plane repulsion.
    pub horizontal_repulsion: f32,
    /// Extra multiplier for vertical repulsion.
    pub vertical_repulsion: f32,
}

impl Default for Layout3dConfig {
    fn default() -> Self {
        Self {
            base: LayoutConfig::default(),
            depth_gap: 56.0,
            ground_strength: 0.015,
            depth_strength: 0.025,
            horizontal_repulsion: 1.0,
            vertical_repulsion: 0.35,
        }
    }
}

/// Output for one laid-out node.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NodeLayout {
    pub id: NodeId,
    pub x: f32,
    pub y: f32,
    pub rank: usize,
    pub order: usize,
}

/// Full layout result plus generated Sugiyama-derived constraints.
#[derive(Clone, Debug, PartialEq)]
pub struct Layout {
    pub nodes: Vec<NodeLayout>,
    pub constraints: ConstraintSet,
}

/// Output for one laid-out 3D node.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NodeLayout3d {
    pub id: NodeId,
    pub x: f32,
    /// Hierarchy axis. Higher values are deeper Sugiyama ranks.
    pub y: f32,
    pub z: f32,
    pub rank: usize,
    pub order: usize,
}

impl NodeLayout3d {
    /// Return coordinates in the same order Bevy's `Vec3::new` expects.
    pub fn as_xyz(self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }
}

/// Full 3D layout result plus generated Sugiyama-derived constraints.
#[derive(Clone, Debug, PartialEq)]
pub struct Layout3d {
    pub nodes: Vec<NodeLayout3d>,
    pub constraints: ConstraintSet,
}

/// Constraints generated by the Sugiyama pass.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ConstraintSet {
    pub ranks: Vec<RankConstraint>,
    pub orders: Vec<OrderConstraint>,
    pub edge_directions: Vec<EdgeDirectionConstraint>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RankConstraint {
    pub node: NodeId,
    pub rank: usize,
    pub target_y: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OrderConstraint {
    pub left: NodeId,
    pub right: NodeId,
    pub min_gap: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EdgeDirectionConstraint {
    pub source: NodeId,
    pub target: NodeId,
    pub min_delta_y: f32,
}

/// Compute a hybrid layout.
pub fn layout(graph: &Graph, config: &LayoutConfig) -> Layout {
    let n = graph.node_count();
    if n == 0 {
        return Layout {
            nodes: Vec::new(),
            constraints: ConstraintSet::default(),
        };
    }

    let ranks = assign_ranks(graph);
    let orders = order_ranks(graph, &ranks);
    let constraints = build_constraints(graph, &ranks, &orders, config);
    let mut positions = initial_positions(&ranks, &orders, config);

    relax(graph, &constraints, config, &mut positions);

    let mut nodes = (0..n)
        .map(|idx| NodeLayout {
            id: NodeId(idx),
            x: positions[idx].0,
            y: positions[idx].1,
            rank: ranks[idx],
            order: orders[idx],
        })
        .collect::<Vec<_>>();
    nodes.sort_by_key(|node| node.id.0);

    Layout { nodes, constraints }
}

/// Compute a hybrid 3D layout.
///
/// Sugiyama rank pressure acts on Y. Organic force relaxation primarily spreads
/// nodes across X/Z, while ground and depth forces keep the structure shallow
/// enough to read in a 3D scene.
pub fn layout_3d(graph: &Graph, config: &Layout3dConfig) -> Layout3d {
    let n = graph.node_count();
    if n == 0 {
        return Layout3d {
            nodes: Vec::new(),
            constraints: ConstraintSet::default(),
        };
    }

    let ranks = assign_ranks(graph);
    let orders = order_ranks(graph, &ranks);
    let constraints = build_constraints(graph, &ranks, &orders, &config.base);
    let mut positions = initial_positions_3d(&ranks, &orders, config);

    relax_3d(graph, &constraints, config, &mut positions);

    let mut nodes = (0..n)
        .map(|idx| NodeLayout3d {
            id: NodeId(idx),
            x: positions[idx].0,
            y: positions[idx].1,
            z: positions[idx].2,
            rank: ranks[idx],
            order: orders[idx],
        })
        .collect::<Vec<_>>();
    nodes.sort_by_key(|node| node.id.0);

    Layout3d { nodes, constraints }
}

fn assign_ranks(graph: &Graph) -> Vec<usize> {
    let n = graph.node_count();
    let mut indegree = vec![0usize; n];
    let mut outgoing = vec![Vec::new(); n];

    for &(source, target) in graph.edges() {
        outgoing[source.0].push(target.0);
        indegree[target.0] += 1;
    }

    let mut ranks = vec![0usize; n];
    let mut queue = VecDeque::new();
    for (idx, degree) in indegree.iter().enumerate() {
        if *degree == 0 {
            queue.push_back(idx);
        }
    }

    let mut visited = 0usize;
    while let Some(node) = queue.pop_front() {
        visited += 1;
        let next_rank = ranks[node] + 1;
        for &target in &outgoing[node] {
            ranks[target] = ranks[target].max(next_rank);
            indegree[target] -= 1;
            if indegree[target] == 0 {
                queue.push_back(target);
            }
        }
    }

    if visited == n {
        return ranks;
    }

    // Cyclic leftovers keep their best discovered rank and get nudged by
    // local edge pressure. This preserves partial DAG structure without
    // pretending cycles can be layered perfectly.
    for _ in 0..n.min(32) {
        let mut changed = false;
        for &(source, target) in graph.edges() {
            if ranks[target.0] <= ranks[source.0] {
                ranks[target.0] = ranks[source.0] + 1;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }

    normalize_ranks(&mut ranks);
    ranks
}

fn normalize_ranks(ranks: &mut [usize]) {
    let mut used = ranks.to_vec();
    used.sort_unstable();
    used.dedup();

    for rank in ranks {
        *rank = used.binary_search(rank).unwrap_or(0);
    }
}

fn order_ranks(graph: &Graph, ranks: &[usize]) -> Vec<usize> {
    let n = graph.node_count();
    let max_rank = ranks.iter().copied().max().unwrap_or(0);
    let mut by_rank = vec![Vec::new(); max_rank + 1];
    for (node, &rank) in ranks.iter().enumerate() {
        by_rank[rank].push(node);
    }

    let mut orders = vec![0usize; n];
    for rank_nodes in &by_rank {
        for (order, &node) in rank_nodes.iter().enumerate() {
            orders[node] = order;
        }
    }

    let mut neighbors = vec![Vec::<usize>::new(); n];
    for &(source, target) in graph.edges() {
        neighbors[source.0].push(target.0);
        neighbors[target.0].push(source.0);
    }

    for _ in 0..8 {
        for rank_nodes in &mut by_rank {
            rank_nodes.sort_by(|&a, &b| {
                let ba = barycenter(a, &neighbors, &orders);
                let bb = barycenter(b, &neighbors, &orders);
                ba.total_cmp(&bb).then_with(|| a.cmp(&b))
            });
            for (order, &node) in rank_nodes.iter().enumerate() {
                orders[node] = order;
            }
        }

        for rank_nodes in by_rank.iter_mut().rev() {
            rank_nodes.sort_by(|&a, &b| {
                let ba = barycenter(a, &neighbors, &orders);
                let bb = barycenter(b, &neighbors, &orders);
                ba.total_cmp(&bb).then_with(|| a.cmp(&b))
            });
            for (order, &node) in rank_nodes.iter().enumerate() {
                orders[node] = order;
            }
        }
    }

    orders
}

fn barycenter(node: usize, neighbors: &[Vec<usize>], orders: &[usize]) -> f32 {
    let adjacent = &neighbors[node];
    if adjacent.is_empty() {
        return orders[node] as f32;
    }
    let sum = adjacent
        .iter()
        .map(|&other| orders[other] as f32)
        .sum::<f32>();
    sum / adjacent.len() as f32
}

fn build_constraints(
    graph: &Graph,
    ranks: &[usize],
    orders: &[usize],
    config: &LayoutConfig,
) -> ConstraintSet {
    let mut constraints = ConstraintSet::default();

    for (idx, &rank) in ranks.iter().enumerate() {
        constraints.ranks.push(RankConstraint {
            node: NodeId(idx),
            rank,
            target_y: rank as f32 * config.rank_gap,
        });
    }

    let max_rank = ranks.iter().copied().max().unwrap_or(0);
    for rank in 0..=max_rank {
        let mut nodes = ranks
            .iter()
            .enumerate()
            .filter_map(|(idx, &node_rank)| (node_rank == rank).then_some(idx))
            .collect::<Vec<_>>();
        nodes.sort_by_key(|&node| orders[node]);
        for pair in nodes.windows(2) {
            constraints.orders.push(OrderConstraint {
                left: NodeId(pair[0]),
                right: NodeId(pair[1]),
                min_gap: config.node_gap,
            });
        }
    }

    for &(source, target) in graph.edges() {
        let span = ranks[target.0].saturating_sub(ranks[source.0]).max(1);
        constraints.edge_directions.push(EdgeDirectionConstraint {
            source,
            target,
            min_delta_y: span as f32 * config.rank_gap * 0.65,
        });
    }

    constraints
}

fn initial_positions(ranks: &[usize], orders: &[usize], config: &LayoutConfig) -> Vec<(f32, f32)> {
    ranks
        .iter()
        .zip(orders.iter())
        .map(|(&rank, &order)| {
            (
                order as f32 * config.node_gap,
                rank as f32 * config.rank_gap,
            )
        })
        .collect()
}

fn initial_positions_3d(
    ranks: &[usize],
    orders: &[usize],
    config: &Layout3dConfig,
) -> Vec<(f32, f32, f32)> {
    ranks
        .iter()
        .zip(orders.iter())
        .map(|(&rank, &order)| {
            let parity = if order % 2 == 0 { 1.0 } else { -1.0 };
            let depth_ring = (order / 2) as f32 + 1.0;
            (
                order as f32 * config.base.node_gap,
                rank as f32 * config.base.rank_gap,
                parity * depth_ring * config.depth_gap,
            )
        })
        .collect()
}

fn relax(
    graph: &Graph,
    constraints: &ConstraintSet,
    config: &LayoutConfig,
    positions: &mut [(f32, f32)],
) {
    let n = graph.node_count();
    let mut forces = vec![(0.0f32, 0.0f32); n];

    for iter in 0..config.iterations {
        forces.fill((0.0, 0.0));

        apply_repulsion(positions, &mut forces, config.repulsion_strength);
        apply_springs(graph, positions, &mut forces, config);
        apply_constraints(constraints, positions, &mut forces, config);

        let cooling = 1.0 - (iter as f32 / config.iterations.max(1) as f32);
        let step = config.step_size * cooling.max(0.08);
        for (position, force) in positions.iter_mut().zip(forces.iter()) {
            position.0 += clamp(force.0, -24.0, 24.0) * step;
            position.1 += clamp(force.1, -24.0, 24.0) * step;
        }
    }
}

fn relax_3d(
    graph: &Graph,
    constraints: &ConstraintSet,
    config: &Layout3dConfig,
    positions: &mut [(f32, f32, f32)],
) {
    let n = graph.node_count();
    let mut forces = vec![(0.0f32, 0.0f32, 0.0f32); n];

    for iter in 0..config.base.iterations {
        forces.fill((0.0, 0.0, 0.0));

        apply_repulsion_3d(positions, &mut forces, config);
        apply_springs_3d(graph, positions, &mut forces, config);
        apply_constraints_3d(constraints, positions, &mut forces, config);
        apply_grounding_3d(positions, &mut forces, config);

        let cooling = 1.0 - (iter as f32 / config.base.iterations.max(1) as f32);
        let step = config.base.step_size * cooling.max(0.08);
        for (position, force) in positions.iter_mut().zip(forces.iter()) {
            position.0 += clamp(force.0, -24.0, 24.0) * step;
            position.1 += clamp(force.1, -24.0, 24.0) * step;
            position.2 += clamp(force.2, -24.0, 24.0) * step;
        }
    }
}

fn apply_repulsion(positions: &[(f32, f32)], forces: &mut [(f32, f32)], strength: f32) {
    for a in 0..positions.len() {
        for b in (a + 1)..positions.len() {
            let dx = positions[a].0 - positions[b].0;
            let dy = positions[a].1 - positions[b].1;
            let dist2 = (dx * dx + dy * dy).max(0.01);
            let dist = dist2.sqrt();
            let magnitude = strength / dist2;
            let fx = dx / dist * magnitude;
            let fy = dy / dist * magnitude;
            forces[a].0 += fx;
            forces[a].1 += fy;
            forces[b].0 -= fx;
            forces[b].1 -= fy;
        }
    }
}

fn apply_repulsion_3d(
    positions: &[(f32, f32, f32)],
    forces: &mut [(f32, f32, f32)],
    config: &Layout3dConfig,
) {
    for a in 0..positions.len() {
        for b in (a + 1)..positions.len() {
            let dx = positions[a].0 - positions[b].0;
            let dy = positions[a].1 - positions[b].1;
            let dz = positions[a].2 - positions[b].2;
            let dist2 = (dx * dx + dy * dy + dz * dz).max(0.01);
            let dist = dist2.sqrt();
            let magnitude = config.base.repulsion_strength / dist2;
            let fx = dx / dist * magnitude * config.horizontal_repulsion;
            let fy = dy / dist * magnitude * config.vertical_repulsion;
            let fz = dz / dist * magnitude * config.horizontal_repulsion;
            forces[a].0 += fx;
            forces[a].1 += fy;
            forces[a].2 += fz;
            forces[b].0 -= fx;
            forces[b].1 -= fy;
            forces[b].2 -= fz;
        }
    }
}

fn apply_springs(
    graph: &Graph,
    positions: &[(f32, f32)],
    forces: &mut [(f32, f32)],
    config: &LayoutConfig,
) {
    for &(source, target) in graph.edges() {
        let a = source.0;
        let b = target.0;
        let dx = positions[b].0 - positions[a].0;
        let dy = positions[b].1 - positions[a].1;
        let dist = (dx * dx + dy * dy).sqrt().max(0.01);
        let delta = dist - config.edge_length;
        let magnitude = delta * config.spring_strength;
        let fx = dx / dist * magnitude;
        let fy = dy / dist * magnitude;
        forces[a].0 += fx;
        forces[a].1 += fy;
        forces[b].0 -= fx;
        forces[b].1 -= fy;
    }
}

fn apply_springs_3d(
    graph: &Graph,
    positions: &[(f32, f32, f32)],
    forces: &mut [(f32, f32, f32)],
    config: &Layout3dConfig,
) {
    for &(source, target) in graph.edges() {
        let a = source.0;
        let b = target.0;
        let dx = positions[b].0 - positions[a].0;
        let dy = positions[b].1 - positions[a].1;
        let dz = positions[b].2 - positions[a].2;
        let dist = (dx * dx + dy * dy + dz * dz).sqrt().max(0.01);
        let delta = dist - config.base.edge_length;
        let magnitude = delta * config.base.spring_strength;
        let fx = dx / dist * magnitude;
        let fy = dy / dist * magnitude;
        let fz = dz / dist * magnitude;
        forces[a].0 += fx;
        forces[a].1 += fy;
        forces[a].2 += fz;
        forces[b].0 -= fx;
        forces[b].1 -= fy;
        forces[b].2 -= fz;
    }
}

fn apply_constraints(
    constraints: &ConstraintSet,
    positions: &[(f32, f32)],
    forces: &mut [(f32, f32)],
    config: &LayoutConfig,
) {
    for constraint in &constraints.ranks {
        let node = constraint.node.0;
        forces[node].1 += (constraint.target_y - positions[node].1) * config.rank_strength;
    }

    for constraint in &constraints.orders {
        let left = constraint.left.0;
        let right = constraint.right.0;
        let gap = positions[right].0 - positions[left].0;
        if gap < constraint.min_gap {
            let push = (constraint.min_gap - gap) * 0.5 * config.order_strength;
            forces[left].0 -= push;
            forces[right].0 += push;
        }
    }

    for constraint in &constraints.edge_directions {
        let source = constraint.source.0;
        let target = constraint.target.0;
        let delta = positions[target].1 - positions[source].1;
        if delta < constraint.min_delta_y {
            let push = (constraint.min_delta_y - delta) * 0.5 * config.edge_direction_strength;
            forces[source].1 -= push;
            forces[target].1 += push;
        }
    }

    for constraint in &constraints.orders {
        let left = constraint.left.0;
        let right = constraint.right.0;
        let dx = positions[right].0 - positions[left].0;
        let dy = positions[right].1 - positions[left].1;
        let min_dist = constraint.min_gap * 0.75;
        let dist = (dx * dx + dy * dy).sqrt().max(0.01);
        if dist < min_dist {
            let push = (min_dist - dist) * config.collision_strength;
            let fx = dx / dist * push;
            let fy = dy / dist * push;
            forces[left].0 -= fx;
            forces[left].1 -= fy;
            forces[right].0 += fx;
            forces[right].1 += fy;
        }
    }
}

fn apply_constraints_3d(
    constraints: &ConstraintSet,
    positions: &[(f32, f32, f32)],
    forces: &mut [(f32, f32, f32)],
    config: &Layout3dConfig,
) {
    for constraint in &constraints.ranks {
        let node = constraint.node.0;
        forces[node].1 += (constraint.target_y - positions[node].1) * config.base.rank_strength;
    }

    for constraint in &constraints.orders {
        let left = constraint.left.0;
        let right = constraint.right.0;
        let gap = positions[right].0 - positions[left].0;
        if gap < constraint.min_gap {
            let push = (constraint.min_gap - gap) * 0.5 * config.base.order_strength;
            forces[left].0 -= push;
            forces[right].0 += push;
        }
    }

    for constraint in &constraints.edge_directions {
        let source = constraint.source.0;
        let target = constraint.target.0;
        let delta = positions[target].1 - positions[source].1;
        if delta < constraint.min_delta_y {
            let push = (constraint.min_delta_y - delta) * 0.5 * config.base.edge_direction_strength;
            forces[source].1 -= push;
            forces[target].1 += push;
        }
    }

    for constraint in &constraints.orders {
        let left = constraint.left.0;
        let right = constraint.right.0;
        let dx = positions[right].0 - positions[left].0;
        let dy = positions[right].1 - positions[left].1;
        let dz = positions[right].2 - positions[left].2;
        let min_dist = constraint.min_gap * 0.75;
        let dist = (dx * dx + dy * dy + dz * dz).sqrt().max(0.01);
        if dist < min_dist {
            let push = (min_dist - dist) * config.base.collision_strength;
            let fx = dx / dist * push;
            let fy = dy / dist * push;
            let fz = dz / dist * push;
            forces[left].0 -= fx;
            forces[left].1 -= fy;
            forces[left].2 -= fz;
            forces[right].0 += fx;
            forces[right].1 += fy;
            forces[right].2 += fz;
        }
    }
}

fn apply_grounding_3d(
    positions: &[(f32, f32, f32)],
    forces: &mut [(f32, f32, f32)],
    config: &Layout3dConfig,
) {
    for (position, force) in positions.iter().zip(forces.iter_mut()) {
        force.1 -= position.1 * config.ground_strength;
        force.2 -= position.2 * config.depth_strength;
    }
}

fn clamp(value: f32, min: f32, max: f32) -> f32 {
    value.max(min).min(max)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dag_edges_receive_forward_ranks() {
        let mut graph = Graph::new();
        let a = graph.add_node(1.0);
        let b = graph.add_node(1.0);
        let c = graph.add_node(1.0);
        graph.add_edge(a, b);
        graph.add_edge(b, c);

        let result = layout(&graph, &LayoutConfig::default());

        assert_eq!(result.nodes[a.0].rank, 0);
        assert_eq!(result.nodes[b.0].rank, 1);
        assert_eq!(result.nodes[c.0].rank, 2);
        assert!(result.nodes[c.0].y > result.nodes[a.0].y);
    }

    #[test]
    fn same_rank_order_constraints_are_generated() {
        let mut graph = Graph::new();
        let a = graph.add_node(1.0);
        let b = graph.add_node(1.0);
        let c = graph.add_node(1.0);
        graph.add_edge(a, c);
        graph.add_edge(b, c);

        let result = layout(&graph, &LayoutConfig::default());

        assert_eq!(result.constraints.orders.len(), 1);
        assert_eq!(result.nodes[a.0].rank, result.nodes[b.0].rank);
    }

    #[test]
    fn empty_graph_layout_is_empty() {
        let graph = Graph::new();
        let result = layout(&graph, &LayoutConfig::default());

        assert!(result.nodes.is_empty());
        assert!(result.constraints.ranks.is_empty());
    }

    #[test]
    fn layout_3d_uses_y_for_hierarchy_and_z_for_depth() {
        let mut graph = Graph::new();
        let a = graph.add_node(1.0);
        let b = graph.add_node(1.0);
        let c = graph.add_node(1.0);
        graph.add_edge(a, b);
        graph.add_edge(a, c);

        let result = layout_3d(&graph, &Layout3dConfig::default());

        assert_eq!(result.nodes[a.0].rank, 0);
        assert_eq!(result.nodes[b.0].rank, 1);
        assert_eq!(result.nodes[c.0].rank, 1);
        assert!(result.nodes[b.0].y > result.nodes[a.0].y);
        assert!(result.nodes.iter().any(|node| node.z.abs() > 0.0));
    }

    #[test]
    fn empty_3d_graph_layout_is_empty() {
        let graph = Graph::new();
        let result = layout_3d(&graph, &Layout3dConfig::default());

        assert!(result.nodes.is_empty());
        assert!(result.constraints.ranks.is_empty());
    }
}
