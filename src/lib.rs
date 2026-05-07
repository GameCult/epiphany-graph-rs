//! Hybrid Sugiyama/force-directed graph layout.
//!
//! Sugiyama is used here as a constraint generator: it supplies ranks, local
//! ordering, and edge direction pressure. A deterministic force solver then
//! relaxes those constraints alongside springs, repulsion, and collision.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, VecDeque};

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
    /// Force pulling strongly connected components into foldable bodies.
    pub component_cohesion_strength: f32,
    /// Force pulling weakly connected components into broad local regions.
    pub weak_component_cohesion_strength: f32,
    /// Force letting bridge edges behave like hinges between folded bodies.
    pub bridge_hinge_strength: f32,
    /// Force making cyclic nodes fold into depth instead of fighting hierarchy.
    pub cycle_fold_strength: f32,
    /// Force anchoring high-centrality nodes closer to their component center.
    pub centrality_anchor_strength: f32,
    /// Force pulling label-propagation communities into local regions.
    pub community_cohesion_strength: f32,
    /// Force giving dense core nodes stronger horizontal anchoring.
    pub core_anchor_strength: f32,
    /// Force keeping high-clustering neighborhoods compact.
    pub clustering_cohesion_strength: f32,
    /// Repulsion strategy used by the 3D solver.
    pub repulsion_mode: RepulsionMode,
    /// Spatial-grid cell size for approximate repulsion.
    pub grid_cell_size: f32,
    /// Number of neighboring cells searched around each node.
    pub grid_radius: i32,
    /// Barnes-Hut opening angle. Lower is more accurate; higher is faster.
    pub barnes_hut_theta: f32,
    /// Maximum Barnes-Hut octree depth.
    pub barnes_hut_max_depth: usize,
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
            component_cohesion_strength: 0.045,
            weak_component_cohesion_strength: 0.012,
            bridge_hinge_strength: 0.025,
            cycle_fold_strength: 0.035,
            centrality_anchor_strength: 0.02,
            community_cohesion_strength: 0.025,
            core_anchor_strength: 0.018,
            clustering_cohesion_strength: 0.02,
            repulsion_mode: RepulsionMode::BarnesHut,
            grid_cell_size: 180.0,
            grid_radius: 1,
            barnes_hut_theta: 0.65,
            barnes_hut_max_depth: 24,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum RepulsionMode {
    /// Exact all-pairs repulsion. Best for quality on small graphs.
    Exact,
    /// Barnes-Hut octree approximation. Best default for realtime 3D layouts.
    #[default]
    BarnesHut,
    /// Approximate local repulsion through a uniform spatial grid.
    SpatialGrid,
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
    pub analysis: GraphAnalysis,
    pub fold_groups: Vec<FoldGroup>,
}

/// Cached, incremental 3D layout solver for realtime render loops.
#[derive(Clone, Debug)]
pub struct Layout3dSolver {
    graph: Graph,
    config: Layout3dConfig,
    analysis: GraphAnalysis,
    ranks: Vec<usize>,
    orders: Vec<usize>,
    constraints: ConstraintSet,
    undirected: Vec<Vec<(usize, EdgeId)>>,
    positions: Vec<(f32, f32, f32)>,
    forces: Vec<(f32, f32, f32)>,
    tick_count: usize,
}

impl Layout3dSolver {
    /// Build a solver with fresh deterministic initial positions.
    pub fn new(graph: Graph, config: Layout3dConfig) -> Self {
        Self::with_initial_positions(graph, config, &[])
    }

    /// Build a solver that reuses any supplied coordinates by node index.
    ///
    /// This is the warm-start path for graph edits and Bevy interactions.
    pub fn with_initial_positions(
        graph: Graph,
        config: Layout3dConfig,
        initial_positions: &[[f32; 3]],
    ) -> Self {
        let analysis = analyze(&graph);
        let ranks = assign_ranks(&graph);
        let orders = order_ranks(&graph, &ranks);
        let constraints = build_constraints(&graph, &ranks, &orders, &config.base);
        let undirected = undirected_adjacency(&graph);
        let mut positions = initial_positions_3d(&ranks, &orders, &config);

        for (position, initial) in positions.iter_mut().zip(initial_positions.iter()) {
            *position = (initial[0], initial[1], initial[2]);
        }

        let forces = vec![(0.0, 0.0, 0.0); graph.node_count()];

        Self {
            graph,
            config,
            analysis,
            ranks,
            orders,
            constraints,
            undirected,
            positions,
            forces,
            tick_count: 0,
        }
    }

    /// Advance the solver by a small number of iterations.
    pub fn tick(&mut self, iterations: usize) {
        for _ in 0..iterations {
            self.step();
        }
    }

    /// Current node positions.
    pub fn positions(&self) -> &[(f32, f32, f32)] {
        &self.positions
    }

    /// Cached graph analysis.
    pub fn analysis(&self) -> &GraphAnalysis {
        &self.analysis
    }

    /// Cached Sugiyama-derived constraints.
    pub fn constraints(&self) -> &ConstraintSet {
        &self.constraints
    }

    /// Convert the current solver state into a layout snapshot.
    pub fn snapshot(&self) -> Layout3d {
        let mut nodes = (0..self.graph.node_count())
            .map(|idx| NodeLayout3d {
                id: NodeId(idx),
                x: self.positions[idx].0,
                y: self.positions[idx].1,
                z: self.positions[idx].2,
                rank: self.ranks[idx],
                order: self.orders[idx],
            })
            .collect::<Vec<_>>();
        nodes.sort_by_key(|node| node.id.0);

        Layout3d {
            nodes,
            constraints: self.constraints.clone(),
            analysis: self.analysis.clone(),
            fold_groups: build_fold_groups(&self.analysis, &self.positions),
        }
    }

    fn step(&mut self) {
        self.forces.fill((0.0, 0.0, 0.0));

        match self.config.repulsion_mode {
            RepulsionMode::Exact => {
                apply_repulsion_3d_exact(&self.positions, &mut self.forces, &self.config)
            }
            RepulsionMode::BarnesHut => {
                apply_repulsion_3d_barnes_hut(&self.positions, &mut self.forces, &self.config)
            }
            RepulsionMode::SpatialGrid => {
                apply_repulsion_3d_grid(&self.positions, &mut self.forces, &self.config)
            }
        }
        apply_springs_3d(&self.graph, &self.positions, &mut self.forces, &self.config);
        apply_constraints_3d(
            &self.constraints,
            &self.positions,
            &mut self.forces,
            &self.config,
        );
        apply_structural_forces_3d(
            &self.graph,
            &self.undirected,
            &self.analysis,
            &self.positions,
            &mut self.forces,
            &self.config,
        );
        apply_grounding_3d(&self.positions, &mut self.forces, &self.config);

        let cooling =
            1.0 - (self.tick_count as f32 / self.config.base.iterations.max(1) as f32).min(1.0);
        let step = self.config.base.step_size * cooling.max(0.08);
        for (position, force) in self.positions.iter_mut().zip(self.forces.iter()) {
            position.0 += clamp(force.0, -24.0, 24.0) * step;
            position.1 += clamp(force.1, -24.0, 24.0) * step;
            position.2 += clamp(force.2, -24.0, 24.0) * step;
        }

        self.tick_count += 1;
    }
}

/// Structural analysis used to turn graph anatomy into layout pressure.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct GraphAnalysis {
    pub weak_components: Vec<Component>,
    pub strongly_connected_components: Vec<Component>,
    pub biconnected_components: Vec<Component>,
    pub communities: Vec<Component>,
    pub core_shells: Vec<Component>,
    pub articulation_points: Vec<NodeId>,
    pub bridges: Vec<EdgeId>,
    pub node_metrics: Vec<NodeMetrics>,
    pub edge_roles: Vec<EdgeRole>,
    pub node_to_weak_component: Vec<usize>,
    pub node_to_strong_component: Vec<usize>,
    pub node_to_community: Vec<usize>,
    pub max_core: usize,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Component {
    pub id: usize,
    pub nodes: Vec<NodeId>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NodeMetrics {
    pub in_degree: usize,
    pub out_degree: usize,
    pub undirected_degree: usize,
    pub degree_centrality: f32,
    pub core_number: usize,
    pub local_clustering_coefficient: f32,
    pub community_id: usize,
    pub biconnected_component_count: usize,
    pub is_articulation: bool,
    pub is_cyclic: bool,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum EdgeRole {
    Tree,
    Back,
    #[default]
    Cross,
    Bridge,
    IntraComponent,
}

/// A renderer-facing body that can be folded, collapsed, or highlighted.
#[derive(Clone, Debug, PartialEq)]
pub struct FoldGroup {
    pub id: usize,
    pub kind: FoldGroupKind,
    pub nodes: Vec<NodeId>,
    pub center: [f32; 3],
    pub radius: f32,
    pub parent: Option<usize>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FoldGroupKind {
    WeakComponent,
    StrongComponent,
    BiconnectedComponent,
    Community,
    CoreShell,
    Cycle,
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
    let mut solver = Layout3dSolver::new(graph.clone(), config.clone());
    solver.tick(config.base.iterations);
    solver.snapshot()
}

/// Analyze graph structure without running layout.
pub fn analyze(graph: &Graph) -> GraphAnalysis {
    let n = graph.node_count();
    if n == 0 {
        return GraphAnalysis::default();
    }

    let directed = directed_adjacency(graph);
    let undirected = undirected_adjacency(graph);
    let weak_components = weak_components(&undirected);
    let (sccs, node_to_scc) = strongly_connected_components(&directed);
    let (articulation_points, bridges, biconnected_components) =
        articulation_points_bridges_and_blocks(&undirected);
    let core_numbers = core_numbers(&undirected);
    let max_core = core_numbers.iter().copied().max().unwrap_or(0);
    let core_shells = core_shells(&core_numbers);
    let local_clustering = local_clustering_coefficients(&undirected);
    let (communities, node_to_community) = label_propagation_communities(&undirected, 16);
    let biconnected_counts = biconnected_component_counts(n, &biconnected_components);
    let edge_roles = classify_edges(graph, &node_to_scc, &bridges);
    let node_metrics = node_metrics(MetricsInput {
        graph,
        undirected: &undirected,
        articulation_points: &articulation_points,
        sccs: &sccs,
        node_to_scc: &node_to_scc,
        core_numbers: &core_numbers,
        local_clustering: &local_clustering,
        node_to_community: &node_to_community,
        biconnected_counts: &biconnected_counts,
    });
    let mut node_to_weak_component = vec![0usize; n];
    for component in &weak_components {
        for node in &component.nodes {
            node_to_weak_component[node.0] = component.id;
        }
    }

    GraphAnalysis {
        weak_components,
        strongly_connected_components: sccs,
        biconnected_components,
        communities,
        core_shells,
        articulation_points,
        bridges,
        node_metrics,
        edge_roles,
        node_to_weak_component,
        node_to_strong_component: node_to_scc,
        node_to_community,
        max_core,
    }
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

fn directed_adjacency(graph: &Graph) -> Vec<Vec<usize>> {
    let mut adjacency = vec![Vec::new(); graph.node_count()];
    for &(source, target) in graph.edges() {
        adjacency[source.0].push(target.0);
    }
    adjacency
}

fn undirected_adjacency(graph: &Graph) -> Vec<Vec<(usize, EdgeId)>> {
    let mut adjacency = vec![Vec::new(); graph.node_count()];
    for (idx, &(source, target)) in graph.edges().iter().enumerate() {
        let edge = EdgeId(idx);
        adjacency[source.0].push((target.0, edge));
        adjacency[target.0].push((source.0, edge));
    }
    adjacency
}

fn weak_components(adjacency: &[Vec<(usize, EdgeId)>]) -> Vec<Component> {
    let mut components = Vec::new();
    let mut seen = vec![false; adjacency.len()];

    for start in 0..adjacency.len() {
        if seen[start] {
            continue;
        }

        let mut queue = VecDeque::from([start]);
        let mut nodes = Vec::new();
        seen[start] = true;

        while let Some(node) = queue.pop_front() {
            nodes.push(NodeId(node));
            for &(next, _) in &adjacency[node] {
                if !seen[next] {
                    seen[next] = true;
                    queue.push_back(next);
                }
            }
        }

        components.push(Component {
            id: components.len(),
            nodes,
        });
    }

    components
}

fn strongly_connected_components(adjacency: &[Vec<usize>]) -> (Vec<Component>, Vec<usize>) {
    struct Tarjan<'a> {
        adjacency: &'a [Vec<usize>],
        index: usize,
        stack: Vec<usize>,
        on_stack: Vec<bool>,
        indices: Vec<Option<usize>>,
        lowlink: Vec<usize>,
        components: Vec<Component>,
        node_to_component: Vec<usize>,
    }

    impl Tarjan<'_> {
        fn visit(&mut self, node: usize) {
            self.indices[node] = Some(self.index);
            self.lowlink[node] = self.index;
            self.index += 1;
            self.stack.push(node);
            self.on_stack[node] = true;

            for &next in &self.adjacency[node] {
                if self.indices[next].is_none() {
                    self.visit(next);
                    self.lowlink[node] = self.lowlink[node].min(self.lowlink[next]);
                } else if self.on_stack[next] {
                    self.lowlink[node] = self.lowlink[node].min(self.indices[next].unwrap_or(0));
                }
            }

            if self.lowlink[node] == self.indices[node].unwrap_or(0) {
                let id = self.components.len();
                let mut nodes = Vec::new();
                while let Some(member) = self.stack.pop() {
                    self.on_stack[member] = false;
                    self.node_to_component[member] = id;
                    nodes.push(NodeId(member));
                    if member == node {
                        break;
                    }
                }
                nodes.sort_by_key(|node| node.0);
                self.components.push(Component { id, nodes });
            }
        }
    }

    let n = adjacency.len();
    let mut tarjan = Tarjan {
        adjacency,
        index: 0,
        stack: Vec::new(),
        on_stack: vec![false; n],
        indices: vec![None; n],
        lowlink: vec![0; n],
        components: Vec::new(),
        node_to_component: vec![0; n],
    };

    for node in 0..n {
        if tarjan.indices[node].is_none() {
            tarjan.visit(node);
        }
    }

    (tarjan.components, tarjan.node_to_component)
}

fn articulation_points_bridges_and_blocks(
    adjacency: &[Vec<(usize, EdgeId)>],
) -> (Vec<NodeId>, Vec<EdgeId>, Vec<Component>) {
    struct Dfs<'a> {
        adjacency: &'a [Vec<(usize, EdgeId)>],
        time: usize,
        visited: Vec<bool>,
        discovery: Vec<usize>,
        low: Vec<usize>,
        articulation: Vec<bool>,
        bridges: Vec<EdgeId>,
        edge_stack: Vec<EdgeId>,
        blocks: Vec<Component>,
    }

    impl Dfs<'_> {
        fn visit(&mut self, node: usize, parent_edge: Option<EdgeId>) {
            self.visited[node] = true;
            self.discovery[node] = self.time;
            self.low[node] = self.time;
            self.time += 1;

            let mut child_count = 0usize;
            for &(next, edge) in &self.adjacency[node] {
                if Some(edge) == parent_edge {
                    continue;
                }
                if self.visited[next] {
                    if self.discovery[next] < self.discovery[node] {
                        self.edge_stack.push(edge);
                    }
                    self.low[node] = self.low[node].min(self.discovery[next]);
                    continue;
                }

                child_count += 1;
                self.edge_stack.push(edge);
                self.visit(next, Some(edge));
                self.low[node] = self.low[node].min(self.low[next]);

                if parent_edge.is_some() && self.low[next] >= self.discovery[node] {
                    self.articulation[node] = true;
                }
                if self.low[next] >= self.discovery[node] {
                    self.pop_block(edge);
                }
                if self.low[next] > self.discovery[node] {
                    self.bridges.push(edge);
                }
            }

            if parent_edge.is_none() && child_count > 1 {
                self.articulation[node] = true;
            }
        }

        fn pop_block(&mut self, stop_edge: EdgeId) {
            let mut nodes = Vec::new();
            while let Some(edge) = self.edge_stack.pop() {
                let (source, target) = endpoints_for_edge(self.adjacency, edge);
                nodes.push(NodeId(source));
                nodes.push(NodeId(target));
                if edge == stop_edge {
                    break;
                }
            }
            nodes.sort_by_key(|node| node.0);
            nodes.dedup();
            if !nodes.is_empty() {
                self.blocks.push(Component {
                    id: self.blocks.len(),
                    nodes,
                });
            }
        }
    }

    let n = adjacency.len();
    let mut dfs = Dfs {
        adjacency,
        time: 0,
        visited: vec![false; n],
        discovery: vec![0; n],
        low: vec![0; n],
        articulation: vec![false; n],
        bridges: Vec::new(),
        edge_stack: Vec::new(),
        blocks: Vec::new(),
    };

    for node in 0..n {
        if !dfs.visited[node] {
            dfs.visit(node, None);
        }
    }

    let articulation_points = dfs
        .articulation
        .iter()
        .enumerate()
        .filter_map(|(idx, &is_articulation)| is_articulation.then_some(NodeId(idx)))
        .collect();
    dfs.bridges.sort_by_key(|edge| edge.0);
    (articulation_points, dfs.bridges, dfs.blocks)
}

fn endpoints_for_edge(adjacency: &[Vec<(usize, EdgeId)>], edge: EdgeId) -> (usize, usize) {
    for (source, edges) in adjacency.iter().enumerate() {
        for &(target, candidate) in edges {
            if candidate == edge && source <= target {
                return (source, target);
            }
        }
    }

    for (source, edges) in adjacency.iter().enumerate() {
        for &(target, candidate) in edges {
            if candidate == edge {
                return (source, target);
            }
        }
    }

    (0, 0)
}

fn core_numbers(adjacency: &[Vec<(usize, EdgeId)>]) -> Vec<usize> {
    let n = adjacency.len();
    let mut degree = adjacency.iter().map(Vec::len).collect::<Vec<_>>();
    let mut core = vec![0usize; n];
    let mut removed = vec![false; n];
    let mut heap = BinaryHeap::new();

    for (node, &node_degree) in degree.iter().enumerate() {
        heap.push(Reverse((node_degree, node)));
    }

    while let Some(Reverse((candidate_degree, node))) = heap.pop() {
        if removed[node] || candidate_degree != degree[node] {
            continue;
        }

        removed[node] = true;
        core[node] = candidate_degree;

        for &(next, _) in &adjacency[node] {
            if !removed[next] && degree[next] > candidate_degree {
                degree[next] -= 1;
                heap.push(Reverse((degree[next], next)));
            }
        }
    }

    core
}

fn core_shells(core_numbers: &[usize]) -> Vec<Component> {
    let max_core = core_numbers.iter().copied().max().unwrap_or(0);
    let mut shells = vec![Vec::new(); max_core + 1];
    for (idx, &core) in core_numbers.iter().enumerate() {
        shells[core].push(NodeId(idx));
    }

    shells
        .into_iter()
        .enumerate()
        .filter_map(|(id, nodes)| (!nodes.is_empty()).then_some(Component { id, nodes }))
        .collect()
}

fn local_clustering_coefficients(adjacency: &[Vec<(usize, EdgeId)>]) -> Vec<f32> {
    let n = adjacency.len();
    let mut marks = vec![false; n];
    let mut coefficients = vec![0.0f32; n];

    for node in 0..n {
        let neighbors = adjacency[node]
            .iter()
            .map(|&(neighbor, _)| neighbor)
            .collect::<Vec<_>>();
        let degree = neighbors.len();
        if degree < 2 {
            continue;
        }

        for &neighbor in &neighbors {
            marks[neighbor] = true;
        }

        let mut links = 0usize;
        for &neighbor in &neighbors {
            for &(candidate, _) in &adjacency[neighbor] {
                if marks[candidate] {
                    links += 1;
                }
            }
        }

        for &neighbor in &neighbors {
            marks[neighbor] = false;
        }

        coefficients[node] = links as f32 / (degree * (degree - 1)) as f32;
    }

    coefficients
}

fn label_propagation_communities(
    adjacency: &[Vec<(usize, EdgeId)>],
    max_iterations: usize,
) -> (Vec<Component>, Vec<usize>) {
    let n = adjacency.len();
    let mut labels = (0..n).collect::<Vec<_>>();
    let mut counts = Vec::<(usize, usize)>::new();

    for _ in 0..max_iterations {
        let mut changed = false;
        for node in 0..n {
            counts.clear();
            for &(neighbor, _) in &adjacency[node] {
                let label = labels[neighbor];
                if let Some((_, count)) = counts.iter_mut().find(|(seen, _)| *seen == label) {
                    *count += 1;
                } else {
                    counts.push((label, 1));
                }
            }

            if counts.is_empty() {
                continue;
            }

            counts.sort_by_key(|&(label, count)| (Reverse(count), label));
            let best_label = counts[0].0;
            if labels[node] != best_label {
                labels[node] = best_label;
                changed = true;
            }
        }

        if !changed {
            break;
        }
    }

    let mut unique = labels.clone();
    unique.sort_unstable();
    unique.dedup();

    let mut node_to_community = vec![0usize; n];
    let mut communities = unique
        .iter()
        .enumerate()
        .map(|(id, _)| Component {
            id,
            nodes: Vec::new(),
        })
        .collect::<Vec<_>>();

    for (node, label) in labels.iter().enumerate() {
        let community = unique.binary_search(label).unwrap_or(0);
        node_to_community[node] = community;
        communities[community].nodes.push(NodeId(node));
    }

    (communities, node_to_community)
}

fn biconnected_component_counts(n: usize, components: &[Component]) -> Vec<usize> {
    let mut counts = vec![0usize; n];
    for component in components {
        for node in &component.nodes {
            counts[node.0] += 1;
        }
    }
    counts
}

fn classify_edges(graph: &Graph, node_to_scc: &[usize], bridges: &[EdgeId]) -> Vec<EdgeRole> {
    let mut roles = vec![EdgeRole::Cross; graph.edge_count()];
    let mut bridge_marks = vec![false; graph.edge_count()];
    for edge in bridges {
        bridge_marks[edge.0] = true;
    }

    for (idx, &(source, target)) in graph.edges().iter().enumerate() {
        roles[idx] = if bridge_marks[idx] {
            EdgeRole::Bridge
        } else if node_to_scc[source.0] == node_to_scc[target.0] {
            EdgeRole::IntraComponent
        } else if source.0 < target.0 {
            EdgeRole::Tree
        } else {
            EdgeRole::Back
        };
    }

    roles
}

struct MetricsInput<'a> {
    graph: &'a Graph,
    undirected: &'a [Vec<(usize, EdgeId)>],
    articulation_points: &'a [NodeId],
    sccs: &'a [Component],
    node_to_scc: &'a [usize],
    core_numbers: &'a [usize],
    local_clustering: &'a [f32],
    node_to_community: &'a [usize],
    biconnected_counts: &'a [usize],
}

fn node_metrics(input: MetricsInput<'_>) -> Vec<NodeMetrics> {
    let n = input.graph.node_count();
    let mut in_degree = vec![0usize; n];
    let mut out_degree = vec![0usize; n];
    for &(source, target) in input.graph.edges() {
        out_degree[source.0] += 1;
        in_degree[target.0] += 1;
    }

    let mut articulation = vec![false; n];
    for node in input.articulation_points {
        articulation[node.0] = true;
    }

    let mut cyclic_scc = vec![false; input.sccs.len()];
    for component in input.sccs {
        if component.nodes.len() > 1 {
            cyclic_scc[component.id] = true;
        }
    }
    for &(source, target) in input.graph.edges() {
        if source == target {
            cyclic_scc[input.node_to_scc[source.0]] = true;
        }
    }

    let centrality_denominator = n.saturating_sub(1).max(1) as f32;
    (0..n)
        .map(|idx| NodeMetrics {
            in_degree: in_degree[idx],
            out_degree: out_degree[idx],
            undirected_degree: input.undirected[idx].len(),
            degree_centrality: input.undirected[idx].len() as f32 / centrality_denominator,
            core_number: input.core_numbers[idx],
            local_clustering_coefficient: input.local_clustering[idx],
            community_id: input.node_to_community[idx],
            biconnected_component_count: input.biconnected_counts[idx],
            is_articulation: articulation[idx],
            is_cyclic: cyclic_scc[input.node_to_scc[idx]],
        })
        .collect()
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

fn apply_structural_forces_3d(
    graph: &Graph,
    undirected: &[Vec<(usize, EdgeId)>],
    analysis: &GraphAnalysis,
    positions: &[(f32, f32, f32)],
    forces: &mut [(f32, f32, f32)],
    config: &Layout3dConfig,
) {
    apply_component_cohesion(
        &analysis.strongly_connected_components,
        positions,
        forces,
        config.component_cohesion_strength,
    );
    apply_component_cohesion(
        &analysis.weak_components,
        positions,
        forces,
        config.weak_component_cohesion_strength,
    );
    apply_component_cohesion(
        &analysis.communities,
        positions,
        forces,
        config.community_cohesion_strength,
    );
    apply_centrality_anchors(analysis, positions, forces, config);
    apply_core_anchors(analysis, positions, forces, config);
    apply_clustering_cohesion(graph, undirected, analysis, positions, forces, config);
    apply_cycle_folds(analysis, positions, forces, config);
    apply_bridge_hinges(graph, analysis, positions, forces, config);
}

fn apply_component_cohesion(
    components: &[Component],
    positions: &[(f32, f32, f32)],
    forces: &mut [(f32, f32, f32)],
    strength: f32,
) {
    if strength <= 0.0 {
        return;
    }

    for component in components {
        if component.nodes.len() < 2 {
            continue;
        }

        let center = component_center(&component.nodes, positions);
        for node in &component.nodes {
            let idx = node.0;
            forces[idx].0 += (center[0] - positions[idx].0) * strength;
            forces[idx].1 += (center[1] - positions[idx].1) * strength * 0.25;
            forces[idx].2 += (center[2] - positions[idx].2) * strength;
        }
    }
}

fn apply_centrality_anchors(
    analysis: &GraphAnalysis,
    positions: &[(f32, f32, f32)],
    forces: &mut [(f32, f32, f32)],
    config: &Layout3dConfig,
) {
    if config.centrality_anchor_strength <= 0.0 {
        return;
    }

    for component in &analysis.weak_components {
        if component.nodes.len() < 3 {
            continue;
        }

        let center = component_center(&component.nodes, positions);
        for node in &component.nodes {
            let idx = node.0;
            let metric = analysis.node_metrics[idx];
            let strength = config.centrality_anchor_strength * metric.degree_centrality;
            forces[idx].0 += (center[0] - positions[idx].0) * strength;
            forces[idx].2 += (center[2] - positions[idx].2) * strength;
        }
    }
}

fn apply_core_anchors(
    analysis: &GraphAnalysis,
    positions: &[(f32, f32, f32)],
    forces: &mut [(f32, f32, f32)],
    config: &Layout3dConfig,
) {
    if config.core_anchor_strength <= 0.0 || analysis.max_core == 0 {
        return;
    }

    for component in &analysis.weak_components {
        let center = component_center(&component.nodes, positions);
        for node in &component.nodes {
            let idx = node.0;
            let core_ratio =
                analysis.node_metrics[idx].core_number as f32 / analysis.max_core as f32;
            let strength = config.core_anchor_strength * core_ratio * core_ratio;
            forces[idx].0 += (center[0] - positions[idx].0) * strength;
            forces[idx].2 += (center[2] - positions[idx].2) * strength;
        }
    }
}

fn apply_clustering_cohesion(
    graph: &Graph,
    adjacency: &[Vec<(usize, EdgeId)>],
    analysis: &GraphAnalysis,
    positions: &[(f32, f32, f32)],
    forces: &mut [(f32, f32, f32)],
    config: &Layout3dConfig,
) {
    if config.clustering_cohesion_strength <= 0.0 {
        return;
    }

    for node in 0..graph.node_count() {
        let coefficient = analysis.node_metrics[node].local_clustering_coefficient;
        if coefficient <= 0.0 || adjacency[node].len() < 2 {
            continue;
        }

        let mut nodes = adjacency[node]
            .iter()
            .map(|&(neighbor, _)| NodeId(neighbor))
            .collect::<Vec<_>>();
        nodes.push(NodeId(node));
        let center = component_center(&nodes, positions);
        let strength = config.clustering_cohesion_strength * coefficient;
        forces[node].0 += (center[0] - positions[node].0) * strength;
        forces[node].2 += (center[2] - positions[node].2) * strength;
    }
}

fn apply_cycle_folds(
    analysis: &GraphAnalysis,
    positions: &[(f32, f32, f32)],
    forces: &mut [(f32, f32, f32)],
    config: &Layout3dConfig,
) {
    if config.cycle_fold_strength <= 0.0 {
        return;
    }

    for component in &analysis.strongly_connected_components {
        if component.nodes.len() < 2 {
            continue;
        }

        let center = component_center(&component.nodes, positions);
        for (offset, node) in component.nodes.iter().enumerate() {
            let idx = node.0;
            let side = if offset % 2 == 0 { 1.0 } else { -1.0 };
            let target_z =
                center[2] + side * config.depth_gap * component.nodes.len() as f32 * 0.35;
            forces[idx].2 += (target_z - positions[idx].2) * config.cycle_fold_strength;
            forces[idx].1 -= positions[idx].1 * config.cycle_fold_strength * 0.08;
        }
    }
}

fn apply_bridge_hinges(
    graph: &Graph,
    analysis: &GraphAnalysis,
    positions: &[(f32, f32, f32)],
    forces: &mut [(f32, f32, f32)],
    config: &Layout3dConfig,
) {
    if config.bridge_hinge_strength <= 0.0 {
        return;
    }

    for bridge in &analysis.bridges {
        let (source, target) = graph.edges()[bridge.0];
        let source_component = analysis.node_to_weak_component[source.0];
        let target_component = analysis.node_to_weak_component[target.0];
        if source_component == target_component {
            let midpoint = midpoint_3d(positions[source.0], positions[target.0]);
            for node in [source, target] {
                let idx = node.0;
                let multiplier = if analysis.node_metrics[idx].is_articulation {
                    1.5
                } else {
                    1.0
                };
                forces[idx].0 +=
                    (midpoint.0 - positions[idx].0) * config.bridge_hinge_strength * multiplier;
                forces[idx].2 +=
                    (midpoint.2 - positions[idx].2) * config.bridge_hinge_strength * multiplier;
            }
        }
    }
}

fn component_center(nodes: &[NodeId], positions: &[(f32, f32, f32)]) -> [f32; 3] {
    let mut center = [0.0f32; 3];
    for node in nodes {
        center[0] += positions[node.0].0;
        center[1] += positions[node.0].1;
        center[2] += positions[node.0].2;
    }
    let len = nodes.len().max(1) as f32;
    center[0] /= len;
    center[1] /= len;
    center[2] /= len;
    center
}

fn midpoint_3d(a: (f32, f32, f32), b: (f32, f32, f32)) -> (f32, f32, f32) {
    ((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5, (a.2 + b.2) * 0.5)
}

fn build_fold_groups(analysis: &GraphAnalysis, positions: &[(f32, f32, f32)]) -> Vec<FoldGroup> {
    let mut groups = Vec::new();

    for component in &analysis.weak_components {
        if component.nodes.is_empty() {
            continue;
        }
        groups.push(fold_group(
            groups.len(),
            FoldGroupKind::WeakComponent,
            component.nodes.clone(),
            positions,
            None,
        ));
    }

    for component in &analysis.strongly_connected_components {
        if component.nodes.len() < 2 {
            continue;
        }
        let parent = component
            .nodes
            .first()
            .map(|node| analysis.node_to_weak_component[node.0]);
        groups.push(fold_group(
            groups.len(),
            FoldGroupKind::StrongComponent,
            component.nodes.clone(),
            positions,
            parent,
        ));
    }

    for component in &analysis.biconnected_components {
        if component.nodes.len() < 3 {
            continue;
        }
        let parent = component
            .nodes
            .first()
            .map(|node| analysis.node_to_weak_component[node.0]);
        groups.push(fold_group(
            groups.len(),
            FoldGroupKind::BiconnectedComponent,
            component.nodes.clone(),
            positions,
            parent,
        ));
    }

    for component in &analysis.communities {
        if component.nodes.len() < 2 {
            continue;
        }
        let parent = component
            .nodes
            .first()
            .map(|node| analysis.node_to_weak_component[node.0]);
        groups.push(fold_group(
            groups.len(),
            FoldGroupKind::Community,
            component.nodes.clone(),
            positions,
            parent,
        ));
    }

    for component in &analysis.core_shells {
        if component.nodes.len() < 2 {
            continue;
        }
        groups.push(fold_group(
            groups.len(),
            FoldGroupKind::CoreShell,
            component.nodes.clone(),
            positions,
            None,
        ));
    }

    for component in &analysis.strongly_connected_components {
        let is_cycle = component
            .nodes
            .iter()
            .any(|node| analysis.node_metrics[node.0].is_cyclic);
        if !is_cycle || component.nodes.len() < 2 {
            continue;
        }
        let parent = component
            .nodes
            .first()
            .map(|node| analysis.node_to_weak_component[node.0]);
        groups.push(fold_group(
            groups.len(),
            FoldGroupKind::Cycle,
            component.nodes.clone(),
            positions,
            parent,
        ));
    }

    groups
}

fn fold_group(
    id: usize,
    kind: FoldGroupKind,
    nodes: Vec<NodeId>,
    positions: &[(f32, f32, f32)],
    parent: Option<usize>,
) -> FoldGroup {
    let center = component_center(&nodes, positions);
    let mut radius = 0.0f32;
    for node in &nodes {
        let dx = positions[node.0].0 - center[0];
        let dy = positions[node.0].1 - center[1];
        let dz = positions[node.0].2 - center[2];
        radius = radius.max((dx * dx + dy * dy + dz * dz).sqrt());
    }

    FoldGroup {
        id,
        kind,
        nodes,
        center,
        radius,
        parent,
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

fn apply_repulsion_3d_exact(
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

fn apply_repulsion_3d_barnes_hut(
    positions: &[(f32, f32, f32)],
    forces: &mut [(f32, f32, f32)],
    config: &Layout3dConfig,
) {
    if positions.len() < 2 {
        return;
    }

    let tree = BarnesHutTree::new(positions, config.barnes_hut_max_depth);
    let theta = config.barnes_hut_theta.max(0.01);
    for target in 0..positions.len() {
        tree.apply_force(target, theta, positions, forces, config);
    }
}

#[derive(Clone, Debug)]
struct BarnesHutTree {
    nodes: Vec<BarnesHutNode>,
}

impl BarnesHutTree {
    fn new(positions: &[(f32, f32, f32)], max_depth: usize) -> Self {
        let (center, half_size) = barnes_hut_bounds(positions);
        let mut tree = Self {
            nodes: vec![BarnesHutNode::new(center, half_size)],
        };

        for index in 0..positions.len() {
            tree.insert(0, index, positions, 0, max_depth);
        }

        tree
    }

    fn insert(
        &mut self,
        node_index: usize,
        body: usize,
        positions: &[(f32, f32, f32)],
        depth: usize,
        max_depth: usize,
    ) {
        self.nodes[node_index].add_mass(positions[body]);

        if self.nodes[node_index].is_leaf()
            && (self.nodes[node_index].bodies.is_empty() || depth >= max_depth)
        {
            self.nodes[node_index].bodies.push(body);
            return;
        }

        if self.nodes[node_index].is_leaf() {
            let existing = std::mem::take(&mut self.nodes[node_index].bodies);
            self.subdivide(node_index);
            for old_body in existing {
                let child = self.child_for_body(node_index, positions[old_body]);
                self.insert(child, old_body, positions, depth + 1, max_depth);
            }
        }

        let child = self.child_for_body(node_index, positions[body]);
        self.insert(child, body, positions, depth + 1, max_depth);
    }

    fn subdivide(&mut self, node_index: usize) {
        let center = self.nodes[node_index].center;
        let child_half = self.nodes[node_index].half_size * 0.5;
        let mut children = [None; 8];

        for (slot, child) in children.iter_mut().enumerate() {
            let offset = (
                if slot & 1 == 0 {
                    -child_half
                } else {
                    child_half
                },
                if slot & 2 == 0 {
                    -child_half
                } else {
                    child_half
                },
                if slot & 4 == 0 {
                    -child_half
                } else {
                    child_half
                },
            );
            let child_center = (
                center.0 + offset.0,
                center.1 + offset.1,
                center.2 + offset.2,
            );
            let child_index = self.nodes.len();
            self.nodes
                .push(BarnesHutNode::new(child_center, child_half));
            *child = Some(child_index);
        }

        self.nodes[node_index].children = children;
    }

    fn child_for_body(&self, node_index: usize, position: (f32, f32, f32)) -> usize {
        let node = &self.nodes[node_index];
        let mut slot = 0usize;
        if position.0 >= node.center.0 {
            slot |= 1;
        }
        if position.1 >= node.center.1 {
            slot |= 2;
        }
        if position.2 >= node.center.2 {
            slot |= 4;
        }
        node.children[slot].unwrap_or(node_index)
    }

    fn apply_force(
        &self,
        target: usize,
        theta: f32,
        positions: &[(f32, f32, f32)],
        forces: &mut [(f32, f32, f32)],
        config: &Layout3dConfig,
    ) {
        self.apply_node_force(0, target, theta, positions, forces, config);
    }

    fn apply_node_force(
        &self,
        node_index: usize,
        target: usize,
        theta: f32,
        positions: &[(f32, f32, f32)],
        forces: &mut [(f32, f32, f32)],
        config: &Layout3dConfig,
    ) {
        let node = &self.nodes[node_index];
        if node.mass == 0 {
            return;
        }

        if node.is_leaf() {
            for &body in &node.bodies {
                if body != target {
                    apply_pair_repulsion_3d(target, body, positions, forces, config);
                }
            }
            return;
        }

        let target_position = positions[target];
        let dx = target_position.0 - node.center_of_mass.0;
        let dy = target_position.1 - node.center_of_mass.1;
        let dz = target_position.2 - node.center_of_mass.2;
        let distance = (dx * dx + dy * dy + dz * dz).sqrt().max(0.01);
        let width = node.half_size * 2.0;

        if !node.contains(target_position) && width / distance < theta {
            apply_aggregate_repulsion_3d(target, node, positions, forces, config);
            return;
        }

        for child in node.children.iter().flatten() {
            self.apply_node_force(*child, target, theta, positions, forces, config);
        }
    }
}

#[derive(Clone, Debug)]
struct BarnesHutNode {
    center: (f32, f32, f32),
    half_size: f32,
    mass: usize,
    center_of_mass: (f32, f32, f32),
    children: [Option<usize>; 8],
    bodies: Vec<usize>,
}

impl BarnesHutNode {
    fn new(center: (f32, f32, f32), half_size: f32) -> Self {
        Self {
            center,
            half_size,
            mass: 0,
            center_of_mass: (0.0, 0.0, 0.0),
            children: [None; 8],
            bodies: Vec::new(),
        }
    }

    fn add_mass(&mut self, position: (f32, f32, f32)) {
        let old_mass = self.mass as f32;
        let new_mass = old_mass + 1.0;
        self.center_of_mass.0 = (self.center_of_mass.0 * old_mass + position.0) / new_mass;
        self.center_of_mass.1 = (self.center_of_mass.1 * old_mass + position.1) / new_mass;
        self.center_of_mass.2 = (self.center_of_mass.2 * old_mass + position.2) / new_mass;
        self.mass += 1;
    }

    fn is_leaf(&self) -> bool {
        self.children.iter().all(Option::is_none)
    }

    fn contains(&self, position: (f32, f32, f32)) -> bool {
        position.0 >= self.center.0 - self.half_size
            && position.0 <= self.center.0 + self.half_size
            && position.1 >= self.center.1 - self.half_size
            && position.1 <= self.center.1 + self.half_size
            && position.2 >= self.center.2 - self.half_size
            && position.2 <= self.center.2 + self.half_size
    }
}

fn barnes_hut_bounds(positions: &[(f32, f32, f32)]) -> ((f32, f32, f32), f32) {
    let mut min = positions[0];
    let mut max = positions[0];

    for &position in positions.iter().skip(1) {
        min.0 = min.0.min(position.0);
        min.1 = min.1.min(position.1);
        min.2 = min.2.min(position.2);
        max.0 = max.0.max(position.0);
        max.1 = max.1.max(position.1);
        max.2 = max.2.max(position.2);
    }

    let center = (
        (min.0 + max.0) * 0.5,
        (min.1 + max.1) * 0.5,
        (min.2 + max.2) * 0.5,
    );
    let span = (max.0 - min.0).max(max.1 - min.1).max(max.2 - min.2);
    (center, (span * 0.5).max(1.0) + 0.001)
}

fn apply_repulsion_3d_grid(
    positions: &[(f32, f32, f32)],
    forces: &mut [(f32, f32, f32)],
    config: &Layout3dConfig,
) {
    if positions.len() < 2 {
        return;
    }

    let cell_size = config.grid_cell_size.max(1.0);
    let radius = config.grid_radius.max(0);
    let mut grid = HashMap::<(i32, i32, i32), Vec<usize>>::new();

    for (idx, &position) in positions.iter().enumerate() {
        grid.entry(grid_cell(position, cell_size))
            .or_default()
            .push(idx);
    }

    for a in 0..positions.len() {
        let (cx, cy, cz) = grid_cell(positions[a], cell_size);
        for x in (cx - radius)..=(cx + radius) {
            for y in (cy - radius)..=(cy + radius) {
                for z in (cz - radius)..=(cz + radius) {
                    if let Some(candidates) = grid.get(&(x, y, z)) {
                        for &b in candidates {
                            if b <= a {
                                continue;
                            }
                            apply_pair_repulsion_3d(a, b, positions, forces, config);
                        }
                    }
                }
            }
        }
    }
}

fn grid_cell(position: (f32, f32, f32), cell_size: f32) -> (i32, i32, i32) {
    (
        (position.0 / cell_size).floor() as i32,
        (position.1 / cell_size).floor() as i32,
        (position.2 / cell_size).floor() as i32,
    )
}

fn apply_aggregate_repulsion_3d(
    target: usize,
    node: &BarnesHutNode,
    positions: &[(f32, f32, f32)],
    forces: &mut [(f32, f32, f32)],
    config: &Layout3dConfig,
) {
    let dx = positions[target].0 - node.center_of_mass.0;
    let dy = positions[target].1 - node.center_of_mass.1;
    let dz = positions[target].2 - node.center_of_mass.2;
    let dist2 = (dx * dx + dy * dy + dz * dz).max(0.01);
    let dist = dist2.sqrt();
    let magnitude = config.base.repulsion_strength * node.mass as f32 / dist2;
    forces[target].0 += dx / dist * magnitude * config.horizontal_repulsion;
    forces[target].1 += dy / dist * magnitude * config.vertical_repulsion;
    forces[target].2 += dz / dist * magnitude * config.horizontal_repulsion;
}

fn apply_pair_repulsion_3d(
    a: usize,
    b: usize,
    positions: &[(f32, f32, f32)],
    forces: &mut [(f32, f32, f32)],
    config: &Layout3dConfig,
) {
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

    #[test]
    fn analysis_finds_cycles_bridges_and_articulation_points() {
        let mut graph = Graph::new();
        let a = graph.add_node(1.0);
        let b = graph.add_node(1.0);
        let c = graph.add_node(1.0);
        let d = graph.add_node(1.0);
        graph.add_edge(a, b);
        graph.add_edge(b, c);
        graph.add_edge(c, a);
        let bridge = graph.add_edge(c, d);

        let analysis = analyze(&graph);

        assert_eq!(analysis.weak_components.len(), 1);
        assert_eq!(analysis.strongly_connected_components.len(), 2);
        assert!(analysis.node_metrics[a.0].is_cyclic);
        assert!(analysis.node_metrics[b.0].is_cyclic);
        assert!(analysis.node_metrics[c.0].is_cyclic);
        assert!(!analysis.node_metrics[d.0].is_cyclic);
        assert!(analysis.articulation_points.contains(&c));
        assert!(analysis.bridges.contains(&bridge));
        assert_eq!(analysis.edge_roles[bridge.0], EdgeRole::Bridge);
        assert!(analysis.biconnected_components.len() >= 2);
        assert!(analysis.max_core >= 1);
        assert!(analysis.node_metrics[a.0].core_number >= 2);
        assert!(analysis.node_metrics[a.0].local_clustering_coefficient > 0.0);
        assert_eq!(
            analysis.node_metrics[c.0].biconnected_component_count,
            analysis.biconnected_components.len()
        );
    }

    #[test]
    fn layout_3d_exposes_fold_groups() {
        let mut graph = Graph::new();
        let a = graph.add_node(1.0);
        let b = graph.add_node(1.0);
        let c = graph.add_node(1.0);
        graph.add_edge(a, b);
        graph.add_edge(b, c);
        graph.add_edge(c, a);

        let result = layout_3d(&graph, &Layout3dConfig::default());

        assert!(
            result
                .fold_groups
                .iter()
                .any(|group| group.kind == FoldGroupKind::WeakComponent)
        );
        assert!(
            result
                .fold_groups
                .iter()
                .any(|group| group.kind == FoldGroupKind::StrongComponent)
        );
        assert!(
            result
                .fold_groups
                .iter()
                .any(|group| group.kind == FoldGroupKind::Cycle)
        );
        assert!(
            result
                .fold_groups
                .iter()
                .any(|group| group.kind == FoldGroupKind::Community)
        );
        assert!(
            result
                .fold_groups
                .iter()
                .any(|group| group.kind == FoldGroupKind::CoreShell)
        );
    }

    #[test]
    fn solver_ticks_incrementally_and_snapshots_layout() {
        let mut graph = Graph::new();
        let a = graph.add_node(1.0);
        let b = graph.add_node(1.0);
        graph.add_edge(a, b);

        let mut solver = Layout3dSolver::new(graph, Layout3dConfig::default());
        let before = solver.positions().to_vec();
        solver.tick(4);
        let after = solver.positions().to_vec();
        let snapshot = solver.snapshot();

        assert_ne!(before, after);
        assert_eq!(snapshot.nodes.len(), 2);
        assert_eq!(snapshot.constraints.ranks.len(), 2);
    }

    #[test]
    fn solver_accepts_warm_start_positions() {
        let mut graph = Graph::new();
        graph.add_node(1.0);
        graph.add_node(1.0);
        let initial = [[10.0, 20.0, 30.0], [40.0, 50.0, 60.0]];

        let solver =
            Layout3dSolver::with_initial_positions(graph, Layout3dConfig::default(), &initial);

        assert_eq!(solver.positions()[0], (10.0, 20.0, 30.0));
        assert_eq!(solver.positions()[1], (40.0, 50.0, 60.0));
    }

    #[test]
    fn exact_repulsion_mode_still_runs() {
        let mut graph = Graph::new();
        let a = graph.add_node(1.0);
        let b = graph.add_node(1.0);
        graph.add_edge(a, b);
        let config = Layout3dConfig {
            repulsion_mode: RepulsionMode::Exact,
            ..Layout3dConfig::default()
        };

        let mut solver = Layout3dSolver::new(graph, config);
        solver.tick(2);

        assert_eq!(solver.snapshot().nodes.len(), 2);
    }

    #[test]
    fn barnes_hut_repulsion_mode_runs_as_default() {
        let mut graph = Graph::new();
        let a = graph.add_node(1.0);
        let b = graph.add_node(1.0);
        let c = graph.add_node(1.0);
        graph.add_edge(a, b);
        graph.add_edge(b, c);

        let mut solver = Layout3dSolver::new(graph, Layout3dConfig::default());
        solver.tick(3);
        let snapshot = solver.snapshot();

        assert_eq!(snapshot.nodes.len(), 3);
        assert_eq!(snapshot.constraints.ranks.len(), 3);
    }

    #[test]
    fn spatial_grid_repulsion_mode_still_runs() {
        let mut graph = Graph::new();
        let a = graph.add_node(1.0);
        let b = graph.add_node(1.0);
        graph.add_edge(a, b);
        let config = Layout3dConfig {
            repulsion_mode: RepulsionMode::SpatialGrid,
            ..Layout3dConfig::default()
        };

        let mut solver = Layout3dSolver::new(graph, config);
        solver.tick(2);

        assert_eq!(solver.snapshot().nodes.len(), 2);
    }
}
