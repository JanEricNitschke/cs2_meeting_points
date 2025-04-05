/// Module for modelling the spread of players after the round starts and when they can see each other.
use crate::nav::{AreaIdent, AreaLike, GroupId, Nav, NavArea, PathResult, areas_audible};
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

/// Struct for the positions of CT and T spawn points.
#[derive(Debug, Clone, Deserialize, Serialize)]
#[allow(non_snake_case)]
pub struct Spawns {
    CT: Vec<Position>,
    T: Vec<Position>,
}

pub enum Perceivability {
    Visibility(HashMap<(u32, u32), bool>),
    Audibility,
}

impl Spawns {
    /// Read the spawn points from a JSON file.
    ///
    /// # Panics
    ///
    /// Will panic if the file cannot be opened or the JSON cannot be deserialized.
    #[must_use]
    pub fn from_json(filename: &Path) -> Self {
        let file = File::open(filename).unwrap();
        serde_json::from_reader(&file).unwrap()
    }
}

/// Struct for the distance of an area from a collection of spawn points.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpawnDistance {
    /// Nav area for which this represents the distance.
    area: NavArea,
    /// Distance to the closest spawn point in the considered collection.
    distance: f64,
    /// Path to the closest spawn point in the considered collection.
    path: Vec<u32>,
}

/// Only the parts of the `SpawnDistance` struct that are needed for the spread plotting.
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

/// Struct for the distances of all areas from CT and T spawn points.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[allow(non_snake_case)]
pub struct SpawnDistances {
    pub CT: Vec<SpawnDistance>,
    pub T: Vec<SpawnDistance>,
}

impl SpawnDistances {
    /// Read the spawn distances from a JSON file.
    ///
    /// # Panics
    ///
    /// Will panic if a centroid comparison returns `None`. Basically if there is a NaN somewhere.
    #[must_use]
    pub fn from_json(filename: &Path) -> Self {
        let file = File::open(filename).unwrap();
        serde_json::from_reader(&file).unwrap()
    }

    /// Save the spawn distances to a JSON file.
    ///
    /// # Panics
    ///
    /// Will panic if the file cannot be created or the JSON cannot be serialized.
    pub fn save_to_json(self, filename: &Path) {
        let mut file = create_file_with_parents(filename);
        serde_json::to_writer(&mut file, &self).unwrap();
    }
}

/// For each area in `map_areas`, find the distances and paths to CT and T spawns.
///
/// The contents of each vector are sorted by distance.
///
/// # Panics
///
/// Will panic if any pathfinding yields a NaN distance.
#[must_use]
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

/// Result of one spread step.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SpreadResult {
    /// New areas that are reachable by CTs.
    new_marked_areas_ct: HashSet<u32>,
    /// New areas that are reachable by Ts.
    new_marked_areas_t: HashSet<u32>,

    /// New connections between areas that are visible to each other.
    visibility_connections: Vec<(ReducedSpawnDistance, ReducedSpawnDistance)>,
}

/// Save the spread results to a JSON file.
///
/// # Panics
///
/// Will panic if the file cannot be created or the JSON cannot be serialized.
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

/// Generate spread steps from the distances of areas to CT and T spawns.
///
/// The slices of `spawn_distances_ct` and `spawn_distances_t` need to be pre-sorted by distance.
/// Also requires a mapping of areas to close groups to ensure a reduced number of spread points.
/// In the extreme case this can just be `AreaID` -> `AreaID` to indicate no grouping at all.
/// Usually a 4x4 to 10x10 grid structure is sensible. See `group_nav_areas`.
///
/// Logic wise we iterate over the T and CT distances in parallel and always take the closest distance to the spawn points.
/// We keep track of all of the processed (reachable) areas for both sides.
///
/// Then we try to check which reachable areas of the other side can be seen from the new area.
/// We only consider that have not already been spotted. We determine this "spottednes" by whether it or its last path step
/// are in a group that has been spotted. We only consider the parent and not the full path to not mark areas
/// that have separated from that group before it was spotted.
/// For example if you could have a look at the spawn of the other side after 2 seconds, then you would not have seen
/// player that moved away from the spawn and behind walls within that time.
///
/// There is some significant difficulty here with getting this perfect because paths do not go from one area to the next
/// but skip over them or take one every so slightly at an angle. This means that we do not mark areas as spotted that
/// sensibly already have.
///
/// Meanwhile the grouping can cause areas to be declared spotted that were separated by small walls or other obstacles
/// from the actually spotted one. Without any grouping we get >1000 spread points for each map which is excssive.
/// With grouping 5x5 or 10x10 we get around 200-300 spread points which is much more manageable.
#[allow(clippy::too_many_lines)]
#[must_use]
pub fn generate_spreads(
    spawn_distances_ct: &[SpawnDistance],
    spawn_distances_t: &[SpawnDistance],
    area_to_group: &HashMap<u32, GroupId>,
    perceiption: &Perceivability,
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

    let mut spotted_groups_ct: HashSet<GroupId> = HashSet::default();
    let mut spotted_groups_t: HashSet<GroupId> = HashSet::default();
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
        .with_desc("Generating spreads".to_string());
    let mut p_bar = iter::repeat_n((), n_iterations).tqdm_config(tqdm_config);

    loop {
        p_bar.next();
        // Get the next T or CT area based on the distance from the spawns.
        let (current_area, opposing_spotted_groups, own_spotted_groups, opposing_previous_areas) =
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
                    &mut spotted_groups_t,
                    &mut spotted_groups_ct,
                    &mut previous_areas_t,
                )
            } else if t_index < spawn_distances_t.len() {
                let current = &spawn_distances_t[t_index];
                new_marked_areas_t.insert(current.area.area_id);
                previous_areas_t.push(current);

                t_index += 1;
                (
                    current,
                    &mut spotted_groups_ct,
                    &mut spotted_groups_t,
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

        // Spot when only unreachable areas are left.
        if current_area.distance == f64::MAX {
            result.push(SpreadResult {
                new_marked_areas_ct: mem::take(&mut new_marked_areas_ct),
                new_marked_areas_t: mem::take(&mut new_marked_areas_t),
                visibility_connections: mem::take(&mut visibility_connections),
            });
            break;
        }

        // Set spottednes based on the spottedness of the group of the last path element.
        if current_area.path.len() >= 2
            && own_spotted_groups
                .contains(&area_to_group[&current_area.path[current_area.path.len() - 2]])
        {
            own_spotted_groups.insert(area_to_group[&current_area.area.area_id]);
        }

        // Get new areas that are visible from the current area.
        let visible_areas = newly_perceivable(
            current_area,
            opposing_previous_areas,
            own_spotted_groups,
            opposing_spotted_groups,
            area_to_group,
            perceiption,
        );

        // If there are any newly visible areas that declare this one and the opposing one as spotted.
        if !visible_areas.is_empty() {
            own_spotted_groups.insert(area_to_group[&current_area.area.area_id]);
            for spotted_by_area in &visible_areas {
                opposing_spotted_groups.insert(area_to_group[&spotted_by_area.area.area_id]);
                visibility_connections.push((
                    Into::<ReducedSpawnDistance>::into(current_area),
                    Into::<ReducedSpawnDistance>::into(*spotted_by_area),
                ));
            }
        }

        // Save a spread point either after a fixed distance or whenever a new area has been spotted.
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

/// Get the new areas that can be seen from `current_area`.
///
/// This is the fine version where we only skip areas that are marked as spotted explicitly.
/// This has previously been set based on the spottedness of the group of the last path element
/// for the `current_area` itself.
fn newly_perceivable<'a>(
    current_area: &SpawnDistance,
    previous_opposing_areas: &'a [&'a SpawnDistance],
    own_spotted_groups: &HashSet<GroupId>,
    opposing_spotted_groups: &HashSet<GroupId>,
    area_to_group: &HashMap<u32, GroupId>,
    perceiption: &Perceivability,
) -> Vec<&'a SpawnDistance> {
    previous_opposing_areas
        .par_iter()
        .filter(|opposing_area| {
            !(own_spotted_groups.contains(&area_to_group[&current_area.area.area_id])
                && opposing_spotted_groups.contains(&area_to_group[&opposing_area.area.area_id]))
                && match perceiption {
                    Perceivability::Visibility(visibility_cache) => {
                        visibility_cache
                            [&(current_area.area.area_id(), opposing_area.area.area_id())]
                    }
                    Perceivability::Audibility => {
                        areas_audible(&current_area.area, &opposing_area.area)
                    }
                }
        })
        .copied()
        .collect()
}

fn round_up_to_next_100(value: f64) -> f64 {
    (value / 100.0).ceil() * 100.0
}
