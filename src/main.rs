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
    SpawnDistances, Spawns, SpreadStyle, generate_spreads, get_distances_from_spawns,
    save_spreads_to_json,
};
use itertools::iproduct;
use nav::regularize_nav_areas;
use std::path::Path;

mod collisions;
mod constants;
mod nav;
mod position;
mod spread;
mod utils;

fn main() {
    let map_names = [
        "de_ancient",
        "de_anubis",
        "de_dust2",
        "de_inferno",
        "de_mirage",
        "de_nuke",
        "de_overpass",
        "de_train",
    ];
    let granularities = [200];
    for (map_name, granularity) in iproduct!(map_names, granularities) {
        println!("At config: map_name: {map_name}, granularity: {granularity}");

        let json_path_str = format!("./data/nav_{granularity}/{map_name}.json");
        let json_path = Path::new(&json_path_str);
        let nav = if json_path.exists() {
            println!("Loading nav from json.");
            Nav::from_json(json_path)
        } else {
            println!("Building nav from scratch.");
            let old_nav = Nav::from_json(Path::new(&format!("./data/nav/{map_name}.json")));
            let map_areas = regularize_nav_areas(&old_nav.areas, granularity, map_name);
            let new_nav = Nav::new(0, 0, map_areas, true);
            new_nav.clone().save_to_json(json_path);
            new_nav
        };

        let spawn_distances_path_str =
            format!("./data/{map_name}_spawn_distances_{granularity}.json");
        let spawn_distances_path = Path::new(&spawn_distances_path_str);

        let spawn_distances = if Path::new(&spawn_distances_path).exists() {
            println!("Loading spawn distances from json.");
            SpawnDistances::from_json(spawn_distances_path)
        } else {
            println!("Building spawn distances from scratch.");
            let spawns_path = format!("./data/spawns/{map_name}.json");
            let spawns = Spawns::from_json(Path::new(&spawns_path));
            let spawn_distances = get_distances_from_spawns(&nav, &spawns);
            spawn_distances
                .clone()
                .save_to_json(Path::new(&spawn_distances_path));
            spawn_distances
        };

        let vis_checker = load_collision_checker(map_name, CollisionCheckerStyle::Visibility);

        let visibility_cache = get_visibility_cache(map_name, granularity, &nav, &vis_checker);

        let rough_spreads = generate_spreads(
            &spawn_distances.CT,
            &spawn_distances.T,
            &vis_checker,
            SpreadStyle::Rough,
            Some(&visibility_cache),
        );
        let rough_spreads_path_str = format!("./data/{map_name}_rough_spreads_{granularity}.json");
        save_spreads_to_json(&rough_spreads, Path::new(&rough_spreads_path_str));

        let fine_spreads = generate_spreads(
            &spawn_distances.CT,
            &spawn_distances.T,
            &vis_checker,
            SpreadStyle::Fine,
            Some(&visibility_cache),
        );
        let fine_spreads_path_str = format!("./data/{map_name}_fine_spreads_{granularity}.json");
        save_spreads_to_json(&fine_spreads, Path::new(&fine_spreads_path_str));
    }
}
