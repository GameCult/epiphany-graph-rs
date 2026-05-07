use std::collections::HashMap;
use std::fs;
use std::path::Path;

use epiphany_graph_rs::{
    Graph, Layout3dConfig, Layout3dSolver, RepulsionAccuracyCandidate, RepulsionMode,
    solver_structural_accuracy_sweep,
};

fn main() {
    let candidates = [
        RepulsionAccuracyCandidate::exact(),
        RepulsionAccuracyCandidate::barnes_hut(1.0),
        RepulsionAccuracyCandidate::barnes_hut(1.2),
        RepulsionAccuracyCandidate::spatial_grid(240.0, 1),
        RepulsionAccuracyCandidate::structured(RepulsionMode::Exact, 0.20, 0.50, 256),
        RepulsionAccuracyCandidate::structured(RepulsionMode::SpatialGrid, 0.25, 0.75, 512),
    ];

    println!(
        "dataset,nodes,edges,communities,cores,mode,theta,body_mode,body_scale,body_far,elapsed_us,mean_abs,max_abs,mean_rel,rms_rel,max_rel"
    );

    for fixture in fixtures() {
        let mut solver = Layout3dSolver::new(fixture.graph, Layout3dConfig::default());
        solver.tick(8);
        let reports = solver_structural_accuracy_sweep(&solver, &candidates);
        for report in reports {
            println!(
                "{},{},{},{},{},{},{:.2},{},{:.2},{:.2},{},{:.6},{:.6},{:.6},{:.6},{:.6}",
                fixture.name,
                solver.positions().len(),
                fixture.edge_count,
                solver.analysis().communities.len(),
                solver.analysis().core_shells.len(),
                mode_name(report.candidate.repulsion_mode),
                report.candidate.barnes_hut_theta,
                mode_name(report.candidate.body_repulsion_mode),
                report.candidate.body_repulsion_scale,
                report.candidate.body_far_repulsion_scale,
                report.elapsed.as_micros(),
                report.mean_absolute_error,
                report.max_absolute_error,
                report.mean_relative_error,
                report.rms_relative_error,
                report.max_relative_error
            );
        }
    }
}

struct Fixture {
    name: &'static str,
    graph: Graph,
    edge_count: usize,
}

fn fixtures() -> Vec<Fixture> {
    let mut fixtures = vec![
        Fixture {
            name: "synthetic_layered_512",
            graph: synthetic_layered(512),
            edge_count: 0,
        },
        Fixture {
            name: "synthetic_clustered_1024",
            graph: synthetic_clustered(1024),
            edge_count: 0,
        },
    ];

    for (name, path, limit) in [
        (
            "snap_email_eu_core",
            "data/tuning/email-Eu-core.txt",
            1500usize,
        ),
        (
            "snap_p2p_gnutella08",
            "data/tuning/p2p-Gnutella08.txt",
            3000usize,
        ),
    ] {
        if let Some((graph, edge_count)) = load_edge_list(path, limit) {
            fixtures.push(Fixture {
                name,
                graph,
                edge_count,
            });
        }
    }

    fixtures
}

fn load_edge_list(path: &str, node_limit: usize) -> Option<(Graph, usize)> {
    let path = Path::new(path);
    let contents = fs::read_to_string(path).ok()?;
    let mut ids = HashMap::<usize, usize>::new();
    let mut edges = Vec::<(usize, usize)>::new();

    for line in contents.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        let mut parts = line.split_whitespace();
        let Some(source) = parts.next().and_then(|part| part.parse::<usize>().ok()) else {
            continue;
        };
        let Some(target) = parts.next().and_then(|part| part.parse::<usize>().ok()) else {
            continue;
        };

        if !ids.contains_key(&source) {
            if ids.len() >= node_limit {
                continue;
            }
            ids.insert(source, ids.len());
        }
        if !ids.contains_key(&target) {
            if ids.len() >= node_limit {
                continue;
            }
            ids.insert(target, ids.len());
        }

        edges.push((ids[&source], ids[&target]));
    }

    let mut graph = Graph::with_capacity(ids.len(), edges.len());
    let nodes = (0..ids.len())
        .map(|_| graph.add_node(1.0))
        .collect::<Vec<_>>();
    for (source, target) in &edges {
        graph.add_edge(nodes[*source], nodes[*target]);
    }

    Some((graph, edges.len()))
}

fn synthetic_layered(count: usize) -> Graph {
    let mut graph = Graph::with_capacity(count, count * 3);
    let nodes = (0..count).map(|_| graph.add_node(1.0)).collect::<Vec<_>>();
    let width = 32usize;

    for idx in 0..count {
        let rank = idx / width;
        let next_rank = rank + 1;
        for offset in [0usize, 3, 11] {
            let target = next_rank * width + ((idx + offset) % width);
            if target < count {
                graph.add_edge(nodes[idx], nodes[target]);
            }
        }
    }

    graph
}

fn synthetic_clustered(count: usize) -> Graph {
    let mut graph = Graph::with_capacity(count, count * 4);
    let nodes = (0..count).map(|_| graph.add_node(1.0)).collect::<Vec<_>>();
    let clusters = 8usize;
    let cluster_size = count / clusters;

    for cluster in 0..clusters {
        let start = cluster * cluster_size;
        let end = if cluster == clusters - 1 {
            count
        } else {
            start + cluster_size
        };
        for idx in start..end {
            for step in [1usize, 2, 5] {
                let target = start + ((idx - start + step) % (end - start));
                graph.add_edge(nodes[idx], nodes[target]);
            }
        }
        if cluster + 1 < clusters {
            graph.add_edge(nodes[start], nodes[(cluster + 1) * cluster_size]);
        }
    }

    graph
}

fn mode_name(mode: RepulsionMode) -> &'static str {
    match mode {
        RepulsionMode::Exact => "exact",
        RepulsionMode::BarnesHut => "barnes_hut",
        RepulsionMode::SpatialGrid => "spatial_grid",
    }
}
