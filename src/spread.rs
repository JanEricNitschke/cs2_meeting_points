use crate::nav::{AreaIdent, AreaLike, Nav, NavArea, PathResult};
use crate::position::Position;
use crate::utils::create_file_with_parents;
use core::f64;
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use rayon::prelude::ParallelSliceMut;
use rustc_hash::FxHashMap as HashMap;
use rustc_hash::FxHashSet as HashSet;
use serde::{Deserialize, Serialize};
use simple_tqdm::{Config, ParTqdm, Tqdm};
use std::fs::File;
use std::iter;
use std::mem;
use std::path::Path;

#[derive(Debug, Clone, Deserialize, Serialize)]
#[allow(non_snake_case)]
pub struct Spawns {
    CT: Vec<Position>,
    T: Vec<Position>,
}

impl Spawns {
    pub fn from_json(filename: &Path) -> Self {
        let file = File::open(filename).unwrap();
        serde_json::from_reader(&file).unwrap()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpawnDistance {
    area: NavArea,
    distance: f64,
    path: Vec<u32>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ReducedSpawnDistance {
    area: u32,
    path: Vec<u32>,
}

impl From<&SpawnDistance> for ReducedSpawnDistance {
    fn from(spawn_distance: &SpawnDistance) -> Self {
        Self {
            area: spawn_distance.area.area_id,
            path: spawn_distance.path.clone(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[allow(non_snake_case)]
pub struct SpawnDistances {
    pub CT: Vec<SpawnDistance>,
    pub T: Vec<SpawnDistance>,
}

impl SpawnDistances {
    pub fn from_json(filename: &Path) -> Self {
        let file = File::open(filename).unwrap();
        serde_json::from_reader(&file).unwrap()
    }

    pub fn save_to_json(self, filename: &Path) {
        let mut file = create_file_with_parents(filename);
        serde_json::to_writer(&mut file, &self).unwrap();
    }
}

pub fn get_distances_from_spawns(map_areas: &Nav, spawns: &Spawns) -> SpawnDistances {
    println!("Getting distances from spawns.");
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
                    distance: f64::MAX,
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
                    distance: f64::MAX,
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

    ct_distances.par_sort_unstable_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());
    t_distances.par_sort_unstable_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());

    SpawnDistances {
        CT: ct_distances,
        T: t_distances,
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SpreadResult {
    new_marked_areas_ct: HashSet<u32>,
    new_marked_areas_t: HashSet<u32>,

    visibility_connections: Vec<(ReducedSpawnDistance, ReducedSpawnDistance)>,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum SpreadStyle {
    Fine,
    Rough,
}

pub fn save_spreads_to_json(spreads: &[SpreadResult], filename: &Path) {
    let mut file = create_file_with_parents(filename);
    serde_json::to_writer(&mut file, &spreads).unwrap();
}

fn assert_sorted(spawn_distances: &[SpawnDistance]) {
    assert!(
        spawn_distances
            .windows(2)
            .all(|w| w[0].distance <= w[1].distance)
    );
}

#[allow(clippy::too_many_lines)]
pub fn generate_spreads(
    spawn_distances_ct: &[SpawnDistance],
    spawn_distances_t: &[SpawnDistance],
    style: SpreadStyle,
    visibility_cache: &HashMap<(u32, u32), bool>,
) -> Vec<SpreadResult> {
    assert_sorted(spawn_distances_ct);
    assert_sorted(spawn_distances_t);
    println!("Generating spreads");

    let mut ct_index = 0;
    let mut t_index = 0;

    let mut new_marked_areas_ct: HashSet<u32> = HashSet::default();
    let mut new_marked_areas_t: HashSet<u32> = HashSet::default();

    let mut previous_areas_ct: Vec<&SpawnDistance> = Vec::with_capacity(spawn_distances_ct.len());
    let mut previous_areas_t: Vec<&SpawnDistance> = Vec::with_capacity(spawn_distances_t.len());

    let mut spotted_areas_ct: HashSet<u32> = HashSet::default();
    let mut spotted_areas_t: HashSet<u32> = HashSet::default();
    let mut visibility_connections: Vec<(ReducedSpawnDistance, ReducedSpawnDistance)> = Vec::new();

    let mut last_plotted: f64 = 0.0;

    let mut result = Vec::with_capacity(spawn_distances_ct.len() + spawn_distances_t.len());

    let n_iterations = spawn_distances_ct
        .iter()
        .chain(spawn_distances_t.iter())
        .filter(|a| a.distance < f64::MAX)
        .count();

    let tqdm_config = Config::new()
        .with_leave(true)
        .with_desc(format!("Generating spreads with style: {style:?}"));
    let mut p_bar = iter::repeat(()).take(n_iterations).tqdm_config(tqdm_config);

    loop {
        p_bar.next();
        let (current_area, opposing_spotted_areas, own_spotted_areas, opposing_previous_areas) =
            if ct_index < spawn_distances_ct.len()
                && (t_index >= spawn_distances_t.len()
                    || spawn_distances_ct[ct_index].distance < spawn_distances_t[t_index].distance)
            {
                let current = &spawn_distances_ct[ct_index];
                new_marked_areas_ct.insert(current.area.area_id);
                previous_areas_ct.push(current);

                ct_index += 1;
                (
                    current,
                    &mut spotted_areas_t,
                    &mut spotted_areas_ct,
                    &mut previous_areas_t,
                )
            } else if t_index < spawn_distances_t.len() {
                let current = &spawn_distances_t[t_index];
                new_marked_areas_t.insert(current.area.area_id);
                previous_areas_t.push(current);

                t_index += 1;
                (
                    current,
                    &mut spotted_areas_ct,
                    &mut spotted_areas_t,
                    &mut previous_areas_ct,
                )
            } else {
                result.push(SpreadResult {
                    new_marked_areas_ct: mem::take(&mut new_marked_areas_ct),
                    new_marked_areas_t: mem::take(&mut new_marked_areas_t),
                    visibility_connections: mem::take(&mut visibility_connections),
                });
                break;
            };

        if current_area.distance == f64::MAX {
            result.push(SpreadResult {
                new_marked_areas_ct: mem::take(&mut new_marked_areas_ct),
                new_marked_areas_t: mem::take(&mut new_marked_areas_t),
                visibility_connections: mem::take(&mut visibility_connections),
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
            opposing_previous_areas,
            own_spotted_areas,
            opposing_spotted_areas,
            style,
            visibility_cache,
        );

        if !visible_areas.is_empty() {
            own_spotted_areas.insert(current_area.area.area_id);
            for spotted_by_area in &visible_areas {
                opposing_spotted_areas.insert(spotted_by_area.area.area_id);
                visibility_connections.push((
                    Into::<ReducedSpawnDistance>::into(current_area),
                    Into::<ReducedSpawnDistance>::into(*spotted_by_area),
                ));
            }
        }

        if visible_areas.is_empty() && current_area.distance <= last_plotted + 100.0 {
            continue;
        }

        result.push(SpreadResult {
            new_marked_areas_ct: mem::take(&mut new_marked_areas_ct),
            new_marked_areas_t: mem::take(&mut new_marked_areas_t),
            visibility_connections: mem::take(&mut visibility_connections),
        });

        last_plotted = round_up_to_next_100(current_area.distance);
    }
    p_bar.for_each(|()| {});
    println!(); // Newline after tqdm so bars dont override each other.
    result
}

fn newly_visible<'a>(
    current_area: &SpawnDistance,
    previous_opposing_areas: &'a [&'a SpawnDistance],
    own_spotted_areas: &mut HashSet<u32>,
    opposing_spotted_areas: &mut HashSet<u32>,
    style: SpreadStyle,
    visibility_cache: &HashMap<(u32, u32), bool>,
) -> Vec<&'a SpawnDistance> {
    match style {
        SpreadStyle::Fine => newly_visible_fine(
            current_area,
            previous_opposing_areas,
            own_spotted_areas,
            opposing_spotted_areas,
            visibility_cache,
        ),
        SpreadStyle::Rough => newly_visible_rough(
            current_area,
            previous_opposing_areas,
            own_spotted_areas,
            opposing_spotted_areas,
            visibility_cache,
        ),
    }
}

fn newly_visible_rough<'a>(
    current_area: &SpawnDistance,
    previous_opposing_areas: &'a [&'a SpawnDistance],
    own_spotted_areas: &mut HashSet<u32>,
    opposing_spotted_areas: &mut HashSet<u32>,
    visibility_cache: &HashMap<(u32, u32), bool>,
) -> Vec<&'a SpawnDistance> {
    if current_area
        .path
        .iter()
        .any(|path_id| own_spotted_areas.contains(path_id))
    {
        return Vec::new();
    }

    let mut results = Vec::new();
    // Previous opposing areas should already be sorted by distance.
    for &opposing_area in previous_opposing_areas {
        if visibility_cache[&(current_area.area.area_id(), opposing_area.area.area_id())] {
            own_spotted_areas.insert(current_area.area.area_id);
            opposing_spotted_areas.insert(opposing_area.area.area_id);
            results.push(opposing_area);
        }
    }
    results
}

fn newly_visible_fine<'a>(
    current_area: &SpawnDistance,
    previous_opposing_areas: &'a [&'a SpawnDistance],
    own_spotted_areas: &HashSet<u32>,
    opposing_spotted_areas: &HashSet<u32>,
    visibility_cache: &HashMap<(u32, u32), bool>,
) -> Vec<&'a SpawnDistance> {
    previous_opposing_areas
        .par_iter()
        .filter(|opposing_area| {
            !(own_spotted_areas.contains(&current_area.area.area_id)
                && opposing_spotted_areas.contains(&opposing_area.area.area_id))
                && visibility_cache[&(current_area.area.area_id(), opposing_area.area.area_id())]
        })
        .copied()
        .collect()
}

fn round_up_to_next_100(value: f64) -> f64 {
    (value / 100.0).ceil() * 100.0
}
