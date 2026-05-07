use epiphany_graph_rs::{
    Layout3dConfig, RepulsionAccuracyCandidate, RepulsionMode, repulsion_accuracy_sweep,
};

fn main() {
    let mut config = Layout3dConfig::default();
    config.base.repulsion_strength = 4800.0;
    config.horizontal_repulsion = 1.0;
    config.vertical_repulsion = 0.35;

    let candidates = [
        RepulsionAccuracyCandidate::exact(),
        RepulsionAccuracyCandidate::barnes_hut(0.35),
        RepulsionAccuracyCandidate::barnes_hut(0.50),
        RepulsionAccuracyCandidate::barnes_hut(0.65),
        RepulsionAccuracyCandidate::barnes_hut(0.80),
        RepulsionAccuracyCandidate::barnes_hut(1.00),
        RepulsionAccuracyCandidate::barnes_hut(1.20),
        RepulsionAccuracyCandidate::barnes_hut_tuned(0.80, 120.0, 0.85),
        RepulsionAccuracyCandidate::barnes_hut_tuned(0.80, 240.0, 1.00),
        RepulsionAccuracyCandidate::barnes_hut_tuned(1.00, 120.0, 0.85),
        RepulsionAccuracyCandidate::barnes_hut_tuned(1.00, 240.0, 1.00),
        RepulsionAccuracyCandidate::barnes_hut_tuned(1.20, 120.0, 0.85),
        RepulsionAccuracyCandidate::barnes_hut_tuned(1.20, 240.0, 1.00),
        RepulsionAccuracyCandidate::spatial_grid(120.0, 1),
        RepulsionAccuracyCandidate::spatial_grid(180.0, 1),
        RepulsionAccuracyCandidate::spatial_grid(240.0, 1),
        RepulsionAccuracyCandidate::spatial_grid(180.0, 2),
    ];

    println!(
        "dataset,n,mode,theta,near_radius,far_scale,cell,radius,elapsed_us,mean_abs,max_abs,mean_rel,rms_rel,max_rel"
    );

    for (name, positions) in [
        ("layered_dag_256", layered_dag(256)),
        ("clustered_fold_512", clustered_fold(512)),
        ("uniform_cloud_1024", uniform_cloud(1024)),
        ("uniform_cloud_4096", uniform_cloud(4096)),
    ] {
        let reports = repulsion_accuracy_sweep(&positions, &config, &candidates);
        for report in reports {
            println!(
                "{},{},{},{:.2},{:.1},{:.2},{:.1},{},{},{:.6},{:.6},{:.6},{:.6},{:.6}",
                name,
                report.node_count,
                mode_name(report.candidate.repulsion_mode),
                report.candidate.barnes_hut_theta,
                report.candidate.barnes_hut_near_radius,
                report.candidate.far_repulsion_scale,
                report.candidate.grid_cell_size,
                report.candidate.grid_radius,
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

fn mode_name(mode: RepulsionMode) -> &'static str {
    match mode {
        RepulsionMode::Exact => "exact",
        RepulsionMode::BarnesHut => "barnes_hut",
        RepulsionMode::SpatialGrid => "spatial_grid",
    }
}

fn layered_dag(count: usize) -> Vec<[f32; 3]> {
    let mut positions = Vec::with_capacity(count);
    let per_rank = 32usize;
    for idx in 0..count {
        let rank = idx / per_rank;
        let order = idx % per_rank;
        let x = order as f32 * 72.0 + wave(idx, 23.0);
        let y = rank as f32 * 125.0 + wave(idx * 3, 9.0);
        let z = ((order % 6) as f32 - 2.5) * 54.0 + wave(idx * 7, 17.0);
        positions.push([x, y, z]);
    }
    positions
}

fn clustered_fold(count: usize) -> Vec<[f32; 3]> {
    let mut positions = Vec::with_capacity(count);
    let clusters = 8usize;
    for idx in 0..count {
        let cluster = idx % clusters;
        let local = idx / clusters;
        let angle = local as f32 * 0.39 + cluster as f32 * 0.73;
        let radius = 42.0 + (local % 19) as f32 * 3.0;
        let center_x = (cluster % 4) as f32 * 360.0;
        let center_y = (cluster / 4) as f32 * 180.0;
        let center_z = ((cluster % 2) as f32 - 0.5) * 260.0;
        positions.push([
            center_x + angle.cos() * radius,
            center_y + wave(local + cluster, 35.0),
            center_z + angle.sin() * radius,
        ]);
    }
    positions
}

fn uniform_cloud(count: usize) -> Vec<[f32; 3]> {
    let mut seed = 0x9E37_79B9_7F4A_7C15u64;
    (0..count)
        .map(|_| {
            let x = lcg_unit(&mut seed) * 2400.0 - 1200.0;
            let y = lcg_unit(&mut seed) * 900.0;
            let z = lcg_unit(&mut seed) * 1800.0 - 900.0;
            [x, y, z]
        })
        .collect()
}

fn wave(seed: usize, amplitude: f32) -> f32 {
    ((seed as f32 * 12.9898).sin() * 43758.547).fract() * amplitude - amplitude * 0.5
}

fn lcg_unit(seed: &mut u64) -> f32 {
    *seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
    ((*seed >> 32) as u32) as f32 / u32::MAX as f32
}
