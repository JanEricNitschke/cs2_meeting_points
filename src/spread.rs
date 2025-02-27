use crate::nav::{AreaIdent, Nav, NavArea, PathResult};
use crate::position::Position;
use crate::utils::create_file_with_parents;
use core::f64;
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::path::Path;

#[derive(Debug, Clone, Deserialize, Serialize)]
#[allow(non_snake_case)]
pub struct Spawns {
    CT: Vec<Position>,
    T: Vec<Position>,
}

impl Spawns {
    pub fn from_json(filename: &Path) -> Self {
        let mut file = File::open(filename).unwrap();
        serde_json::from_reader(&mut file).unwrap()
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct SpawnDistance {
    area: NavArea,
    distance: f64,
    path: Vec<u32>,
}

#[derive(Debug, Serialize, Deserialize)]
#[allow(non_snake_case)]
pub struct SpawnDistances {
    CT: Vec<SpawnDistance>,
    T: Vec<SpawnDistance>,
}

impl SpawnDistances {
    pub fn from_json(filename: &Path) -> Self {
        let mut file = File::open(filename).unwrap();
        serde_json::from_reader(&mut file).unwrap()
    }

    pub fn save_to_json(self, filename: &Path) {
        let mut file = create_file_with_parents(filename);
        serde_json::to_writer(&mut file, &self).unwrap();
    }
}

pub fn get_distances_from_spawns(map_areas: &Nav, spawns: &Spawns) -> SpawnDistances {
    let mut ct_distances: Vec<SpawnDistance> = Vec::new();
    let mut t_distances: Vec<SpawnDistance> = Vec::new();

    for area in map_areas.areas.values() {
        let ct_path = spawns
            .CT
            .iter()
            .map(|&spawn_point| {
                map_areas.find_path(AreaIdent::Pos(spawn_point), AreaIdent::Id(area.area_id))
            })
            .min_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap())
            .unwrap_or(PathResult {
                distance: f64::INFINITY,
                path: Vec::new(),
            });

        let t_path = spawns
            .T
            .iter()
            .map(|&spawn_point| {
                map_areas.find_path(AreaIdent::Pos(spawn_point), AreaIdent::Id(area.area_id))
            })
            .min_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap())
            .unwrap_or(PathResult {
                distance: f64::INFINITY,
                path: Vec::new(),
            });

        ct_distances.push(SpawnDistance {
            area: area.clone(),
            distance: ct_path.distance,
            path: ct_path.path.iter().map(|a| a.area_id).collect(),
        });

        t_distances.push(SpawnDistance {
            area: area.clone(),
            distance: t_path.distance,
            path: t_path.path.iter().map(|a| a.area_id).collect(),
        });
    }

    ct_distances.sort_by(|a, b| {
        a.distance
            .partial_cmp(&b.distance)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    t_distances.sort_by(|a, b| {
        a.distance
            .partial_cmp(&b.distance)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    SpawnDistances {
        CT: ct_distances,
        T: t_distances,
    }
}
