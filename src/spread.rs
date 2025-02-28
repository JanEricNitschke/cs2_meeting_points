use crate::collisions::CollisionChecker;
use crate::nav::{AreaIdent, Nav, NavArea, PathResult, areas_visible};
use crate::position::Position;
use crate::utils::create_file_with_parents;
use core::f64;
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use serde::{Deserialize, Serialize};
use simple_tqdm::{Config, ParTqdm};
use std::collections::HashSet;
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpawnDistance {
    area: NavArea,
    distance: f64,
    path: Vec<u32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[allow(non_snake_case)]
pub struct SpawnDistances {
    pub CT: Vec<SpawnDistance>,
    pub T: Vec<SpawnDistance>,
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
    let tqdm_config = Config::new().with_leave(true);

    let distances: Vec<(SpawnDistance, SpawnDistance)> = map_areas
        .areas
        .values()
        .collect::<Vec<_>>()
        .par_iter()
        .tqdm_config(tqdm_config.with_desc("Getting distances per spawn."))
        .map(|area| {
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

            (
                SpawnDistance {
                    area: (*area).clone(),
                    distance: ct_path.distance,
                    path: ct_path.path.iter().map(|a| a.area_id).collect(),
                },
                SpawnDistance {
                    area: (*area).clone(),
                    distance: t_path.distance,
                    path: t_path.path.iter().map(|a| a.area_id).collect(),
                },
            )
        })
        .collect();
    println!(); // Newline after tqdm so bars dont override each other.

    let mut ct_distances: Vec<SpawnDistance> = Vec::new();
    let mut t_distances: Vec<SpawnDistance> = Vec::new();

    for (ct, t) in distances {
        ct_distances.push(ct);
        t_distances.push(t);
    }

    ct_distances.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());
    t_distances.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());

    SpawnDistances {
        CT: ct_distances,
        T: t_distances,
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SpreadResult {
    new_marked_areas_ct: HashSet<u32>,
    new_marked_areas_t: HashSet<u32>,

    old_marked_areas_ct: HashSet<u32>,
    old_marked_areas_t: HashSet<u32>,

    visibility_connections: Vec<(SpawnDistance, SpawnDistance)>,

    contains_new_connections: bool,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum SpreadStyle {
    Fine,
    Rough,
}

pub fn generate_spreads(
    spawn_distances_ct: &[SpawnDistance],
    spawn_distances_t: &[SpawnDistance],
    vis_checker: &CollisionChecker,
    style: SpreadStyle,
) -> Vec<SpreadResult> {
    let mut ct_index = 0;
    let mut t_index = 0;

    let mut marked_areas_ct: HashSet<u32> = HashSet::new();
    let mut marked_areas_t: HashSet<u32> = HashSet::new();
    let mut new_marked_areas_ct: HashSet<u32> = HashSet::new();
    let mut new_marked_areas_t: HashSet<u32> = HashSet::new();

    let mut spotted_areas_t: HashSet<u32> = HashSet::new();
    let mut spotted_areas_ct: HashSet<u32> = HashSet::new();
    let mut visibility_connections: Vec<(SpawnDistance, SpawnDistance)> = Vec::new();

    let mut last_plotted: f64 = f64::NEG_INFINITY;

    let mut result = Vec::new();

    let n_iterations = spawn_distances_ct
        .iter()
        .chain(spawn_distances_t.iter())
        .filter(|a| a.distance < f64::INFINITY)
        .count();

    loop {
        let (current_area, opposing_spotted_areas, own_spotted_areas, opposing_previous_areas) =
            if spawn_distances_ct[ct_index].distance < spawn_distances_t[t_index].distance {
                let current = &spawn_distances_ct[ct_index];
                new_marked_areas_ct.insert(current.area.area_id);
                let opposing_prev: Vec<_> = spawn_distances_t
                    .iter()
                    .filter(|a| {
                        marked_areas_t.contains(&a.area.area_id)
                            || new_marked_areas_t.contains(&a.area.area_id)
                    })
                    .collect();

                ct_index += 1;
                (
                    current,
                    &mut spotted_areas_t,
                    &mut spotted_areas_ct,
                    opposing_prev,
                )
            } else {
                let current = &spawn_distances_t[t_index];
                new_marked_areas_t.insert(current.area.area_id);
                let opposing_prev: Vec<_> = spawn_distances_ct
                    .iter()
                    .filter(|a| {
                        marked_areas_ct.contains(&a.area.area_id)
                            || new_marked_areas_ct.contains(&a.area.area_id)
                    })
                    .collect();

                t_index += 1;
                (
                    current,
                    &mut spotted_areas_ct,
                    &mut spotted_areas_t,
                    opposing_prev,
                )
            };

        if current_area.distance == f64::INFINITY {
            result.push(SpreadResult {
                new_marked_areas_ct: new_marked_areas_ct.clone(),
                new_marked_areas_t: new_marked_areas_t.clone(),
                old_marked_areas_ct: marked_areas_ct.clone(),
                old_marked_areas_t: marked_areas_t.clone(),
                visibility_connections: visibility_connections.clone(),
                contains_new_connections: false,
            });
            break;
        }

        if current_area.path.len() >= 2
            && own_spotted_areas.contains(&current_area.path[current_area.path.len() - 2])
        {
            own_spotted_areas.insert(current_area.area.area_id);
        }

        let visible_areas = newly_visible(
            current_area,
            &opposing_previous_areas,
            vis_checker,
            own_spotted_areas,
            opposing_spotted_areas,
            style,
        );

        if !visible_areas.is_empty() {
            own_spotted_areas.insert(current_area.area.area_id);
            for spotted_by_area in &visible_areas {
                opposing_spotted_areas.insert(spotted_by_area.area.area_id);
                visibility_connections.push((current_area.clone(), spotted_by_area.clone()));
            }
        }

        if visible_areas.is_empty() && current_area.distance <= last_plotted + 100.0 {
            continue;
        }

        result.push(SpreadResult {
            new_marked_areas_ct: new_marked_areas_ct.clone(),
            new_marked_areas_t: new_marked_areas_t.clone(),
            old_marked_areas_ct: marked_areas_ct.clone(),
            old_marked_areas_t: marked_areas_t.clone(),
            visibility_connections: visibility_connections.clone(),
            contains_new_connections: !visible_areas.is_empty(),
        });

        last_plotted = round_up_to_next_100(current_area.distance);

        marked_areas_ct.extend(&new_marked_areas_ct);
        marked_areas_t.extend(&new_marked_areas_t);
        new_marked_areas_ct.clear();
        new_marked_areas_t.clear();

        // Currently we only want the new connections of each frame to be shown.
        // At least for the fine style right now because otherwise there are
        // ALOT of connections.
        // We will see how it will be with rough style.
        if style == SpreadStyle::Fine {
            visibility_connections.clear();
        }
    }
    result
}

fn newly_visible(
    current_area: &SpawnDistance,
    previous_opposing_areas: &[&SpawnDistance],
    vis_checker: &CollisionChecker,
    own_spotted_areas: &mut HashSet<u32>,
    opposing_spotted_areas: &mut HashSet<u32>,
    style: SpreadStyle,
) -> Vec<SpawnDistance> {
    match style {
        SpreadStyle::Fine => newly_visible_fine(
            current_area,
            previous_opposing_areas,
            vis_checker,
            own_spotted_areas,
            opposing_spotted_areas,
        ),
        SpreadStyle::Rough => newly_visible_rough(
            current_area,
            previous_opposing_areas,
            vis_checker,
            own_spotted_areas,
            opposing_spotted_areas,
        ),
    }
}

fn newly_visible_rough(
    current_area: &SpawnDistance,
    previous_opposing_areas: &[&SpawnDistance],
    vis_checker: &CollisionChecker,
    own_spotted_areas: &mut HashSet<u32>,
    opposing_spotted_areas: &mut HashSet<u32>,
) -> Vec<SpawnDistance> {
    if current_area
        .path
        .iter()
        .any(|path_id| own_spotted_areas.contains(path_id))
    {
        return Vec::new();
    }

    let mut results = Vec::new();
    let mut sorted_opposing_areas = previous_opposing_areas.to_vec();
    sorted_opposing_areas.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());

    for opposing_area in sorted_opposing_areas {
        if areas_visible(&current_area.area, &opposing_area.area, vis_checker) {
            own_spotted_areas.insert(current_area.area.area_id);
            opposing_spotted_areas.insert(opposing_area.area.area_id);
            results.push(opposing_area.clone());
        }
    }
    results
}

fn newly_visible_fine(
    current_area: &SpawnDistance,
    previous_opposing_areas: &[&SpawnDistance],
    vis_checker: &CollisionChecker,
    own_spotted_areas: &HashSet<u32>,
    opposing_spotted_areas: &HashSet<u32>,
) -> Vec<SpawnDistance> {
    previous_opposing_areas
        .iter()
        .filter(|opposing_area| {
            !(own_spotted_areas.contains(&current_area.area.area_id)
                && opposing_spotted_areas.contains(&opposing_area.area.area_id))
                && areas_visible(&current_area.area, &opposing_area.area, vis_checker)
        })
        .map(|&s| s.clone())
        .collect()
}

fn round_up_to_next_100(value: f64) -> f64 {
    (value / 100.0).ceil() * 100.0
}
