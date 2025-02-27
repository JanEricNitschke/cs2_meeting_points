#![allow(unknown_lints)]
#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(clippy::suboptimal_flops)]
#![allow(clippy::similar_names)]
#![allow(clippy::many_single_char_names)]
#![allow(clippy::multiple_crate_versions)]

use crate::nav::Nav;
use itertools::iproduct;
use nav::regularize_nav_areas;
use spread::{Spawns, get_distances_from_spawns};
use std::path::Path;
mod constants;
mod nav;
mod position;
mod spread;
mod utils;
mod visibility;

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
    let granularities = [100, 200];
    for (map_name, granularity) in iproduct!(map_names, granularities) {
        let json_path = format!("./nav_{granularity}/{map_name}.json");
        let nav = if Path::new(&json_path).exists() {
            Nav::from_json(Path::new(&json_path))
        } else {
            let old_nav = Nav::from_json(Path::new(&format!("./nav/{map_name}.json")));
            let map_areas = regularize_nav_areas(&old_nav.areas, granularity, map_name);
            let new_nav = Nav::new(0, 0, map_areas, true);
            new_nav.clone().save_to_json(Path::new(&json_path));
            new_nav
        };

        let spawns_path = format!("./spawns/{map_name}.json");
        let spawns = Spawns::from_json(Path::new(&spawns_path));
        let spawn_distances = get_distances_from_spawns(&nav, &spawns);
        let spawn_distances_path = format!("./{map_name}_spawn_distances.json");
        spawn_distances.save_to_json(Path::new(&spawn_distances_path));
    }
}
