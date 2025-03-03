#![allow(unknown_lints)]
#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(clippy::suboptimal_flops)]
#![allow(clippy::similar_names)]
#![allow(clippy::many_single_char_names)]
#![allow(clippy::multiple_crate_versions)]

#[cfg(not(target_env = "msvc"))]
use tikv_jemallocator::Jemalloc;

#[cfg(not(target_env = "msvc"))]
#[global_allocator]
static GLOBAL: Jemalloc = Jemalloc;

use crate::collisions::{CollisionCheckerStyle, load_collision_checker};
use crate::nav::{Nav, get_visibility_cache};
use crate::spread::{
    Spawns, SpreadStyle, generate_spreads, get_distances_from_spawns, save_spreads_to_json,
};
use clap::{Args, Parser, Subcommand};
use nav::regularize_nav_areas;
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use sha2::{Digest, Sha256};
use std::{
    collections::HashSet,
    fs::{self, File},
    io::{Read, Write},
    path::Path,
};

mod collisions;
mod constants;
mod nav;
mod position;
mod spread;
mod utils;

/// Expected file structure for a given `map_name`
fn expected_files(map_name: &str) -> Vec<String> {
    vec![
        format!("maps/{}.png", map_name),
        format!("tris/{}.tris", map_name),
        format!("tris/{}-clippings.tris", map_name),
        format!("nav/{}.json", map_name),
        format!("spawns/{}.json", map_name),
    ]
}

/// Get all unique `{map_name}` that have the required four files
fn collect_valid_maps() -> HashSet<String> {
    let mut valid_maps = HashSet::new();

    if let Ok(entries) = fs::read_dir("maps") {
        for entry in entries.flatten() {
            if let Some(file_name) = entry.file_name().to_str() {
                if let Some((map_name, _)) = file_name.rsplit_once('.') {
                    // Check if all required files exist
                    let all_exist = expected_files(map_name)
                        .iter()
                        .all(|path| Path::new(path).exists());

                    if all_exist {
                        valid_maps.insert(map_name.to_string());
                    }
                }
            }
        }
    }

    valid_maps
}

/// Compute a SHA-256 hash for the combined contents of a map's files
fn compute_hash(map_name: &str) -> Option<String> {
    let mut hasher = Sha256::new();

    for file in expected_files(map_name) {
        let path = Path::new(&file);
        if let Ok(mut file) = File::open(path) {
            let mut contents = Vec::new();
            if file.read_to_end(&mut contents).is_ok() {
                hasher.update(&contents);
            } else {
                return None;
            }
        } else {
            return None;
        }
    }

    let result = hasher.finalize();
    Some(format!("{result:x}"))
}

/// Load the existing hash from `hashes/{map_name}.txt`
fn load_existing_hash(map_name: &str) -> Option<String> {
    let path = format!("hashes/{map_name}.txt");
    fs::read_to_string(path).ok()
}

/// Save the new hash to `hashes/{map_name}.txt`
fn save_hash(map_name: &str, hash: &str) {
    let path = format!("hashes/{map_name}.txt");
    if let Ok(mut file) = File::create(path) {
        let _ = file.write_all(hash.as_bytes());
    }
}

/// Process maps: Compute and compare hashes, return those that need updates
fn process_maps(maps: HashSet<String>) -> Vec<String> {
    maps.into_par_iter()
        .filter_map(|map_name| {
            let new_hash = compute_hash(&map_name)?;
            let old_hash = load_existing_hash(&map_name);

            if old_hash.as_deref() == Some(&new_hash) {
                None
            } else {
                save_hash(&map_name, &new_hash);
                Some(map_name)
            }
        })
        .collect()
}

#[derive(Parser)]
#[command(name = "Map Processor")]
#[command(about = "Processes map hashes or performs navigation analysis")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Process map hashes and update them if necessary
    ProcessMaps,

    /// Perform navigation analysis for a specific map
    NavAnalysis(NavAnalysisArgs),
}

#[derive(Args)]
struct NavAnalysisArgs {
    /// The name of the map to analyze
    map_name: String,

    /// The granularity level (default: 200)
    #[arg(short, long, default_value_t = 200)]
    granularity: usize,
}

fn main() {
    let cli = Cli::parse();

    match cli.command {
        Commands::ProcessMaps => {
            let valid_maps = collect_valid_maps();
            let maps_to_update = process_maps(valid_maps);

            println!("Maps with updated or new hashes: {maps_to_update:?}");
        }
        Commands::NavAnalysis(args) => {
            let (map_name, granularity) = (&args.map_name, args.granularity);
            println!("At config: map_name: {map_name}, granularity: {granularity}");

            let old_nav = Nav::from_json(Path::new(&format!("./nav/{map_name}.json")));
            let map_areas = regularize_nav_areas(&old_nav.areas, granularity, map_name);
            let nav = Nav::new(0, 0, map_areas, true);

            let spawns_path = format!("./spawns/{map_name}.json");
            let spawns = Spawns::from_json(Path::new(&spawns_path));
            let spawn_distances = get_distances_from_spawns(&nav, &spawns);

            let vis_checker = load_collision_checker(map_name, CollisionCheckerStyle::Visibility);

            let visibility_cache =
                get_visibility_cache(map_name, granularity, &nav, &vis_checker, false);

            let rough_spreads = generate_spreads(
                &spawn_distances.CT,
                &spawn_distances.T,
                SpreadStyle::Rough,
                &visibility_cache,
            );
            let rough_spreads_path_str =
                format!("./results/{map_name}_rough_spreads_{granularity}.json");
            save_spreads_to_json(&rough_spreads, Path::new(&rough_spreads_path_str));

            let fine_spreads = generate_spreads(
                &spawn_distances.CT,
                &spawn_distances.T,
                SpreadStyle::Fine,
                &visibility_cache,
            );
            let fine_spreads_path_str =
                format!("./results/{map_name}_fine_spreads_{granularity}.json");
            save_spreads_to_json(&fine_spreads, Path::new(&fine_spreads_path_str));
        }
    }
}
