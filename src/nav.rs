/// Module for navigation capabilities aimed at CS2.
///
/// Core taken from: <https://github.com/pnxenopoulos/awpy/blob/main/awpy/nav.py>
use crate::collisions::{CollisionChecker, CollisionCheckerStyle, load_collision_checker};
use crate::constants::{
    CROUCHING_ATTRIBUTE_FLAG, CROUCHING_SPEED, JUMP_HEIGHT, PLAYER_CROUCH_HEIGHT, PLAYER_EYE_LEVEL,
    PLAYER_HEIGHT, PLAYER_WIDTH, RUNNING_SPEED,
};
use crate::position::{Position, inverse_distance_weighting};
use crate::utils::create_file_with_parents;

use bincode::{deserialize_from, serialize_into};
use geo::algorithm::line_measures::metric_spaces::Euclidean;
use geo::geometry::{LineString, Point, Polygon};
use geo::{Centroid, Contains, Distance, Intersects};
use itertools::{Itertools, iproduct};
use petgraph::algo::astar;
use petgraph::graphmap::DiGraphMap;
use petgraph::visit::EdgeRef;
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use rustc_hash::FxHashMap as HashMap;
use rustc_hash::FxHashSet as HashSet;
use serde::de::{self, MapAccess, Visitor};
use serde::{Deserialize, Deserializer, Serialize};
use simple_tqdm::{Config, ParTqdm, Tqdm};
use std::cmp::Ordering;
use std::f64;
use std::fmt;
use std::fs::File;
use std::hash::Hash;
use std::path::Path;

// --- DynamicAttributeFlags ---
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Deserialize, Serialize)]
pub struct DynamicAttributeFlags(u32);

impl DynamicAttributeFlags {
    pub const fn new(value: u32) -> Self {
        Self(value)
    }
}

impl From<DynamicAttributeFlags> for u32 {
    fn from(flag: DynamicAttributeFlags) -> Self {
        flag.0
    }
}

pub trait AreaLike {
    fn centroid(&self) -> Position;
    fn requires_crouch(&self) -> bool;
    fn area_id(&self) -> u32;
}

/// A navigation area in the map.
#[derive(Debug, Clone, Serialize)]
pub struct NavArea {
    /// Unique ID of the area.
    ///
    /// Only unique for a given mesh
    pub area_id: u32,
    pub hull_index: u32,
    pub dynamic_attribute_flags: DynamicAttributeFlags,
    /// Corners of the polygon making up the area.
    pub corners: Vec<Position>,
    /// IDs of areas this one is connected to.
    ///
    /// Connections are not necessarily symmetric.
    pub connections: Vec<u32>,
    /// IDs of ladders above this area.
    pub ladders_above: Vec<u32>,
    /// IDs of ladders below this area.
    pub ladders_below: Vec<u32>,
    /// Precomputed centroid of the area.
    centroid: Position,
}

/// Equality is purely done through the `area_id`.
impl PartialEq for NavArea {
    fn eq(&self, other: &Self) -> bool {
        self.area_id == other.area_id
    }
}

#[allow(clippy::cast_precision_loss)]
/// Computes the centroid of the polygon (averaging all corners).
pub fn centroid(corners: &[Position]) -> Position {
    if corners.is_empty() {
        return Position::new(0.0, 0.0, 0.0);
    }
    let (sum_x, sum_y, sum_z) = corners.iter().fold((0.0, 0.0, 0.0), |(sx, sy, sz), c| {
        (sx + c.x, sy + c.y, sz + c.z)
    });
    let count = corners.len() as f64;
    Position::new(sum_x / count, sum_y / count, sum_z / count)
}

impl NavArea {
    pub fn new(
        area_id: u32,
        dynamic_attribute_flags: DynamicAttributeFlags,
        corners: Vec<Position>,
        connections: Vec<u32>,
        ladders_above: Vec<u32>,
        ladders_below: Vec<u32>,
    ) -> Self {
        let centroid = centroid(&corners);
        Self {
            area_id,
            hull_index: 0,
            dynamic_attribute_flags,
            corners,
            connections,
            ladders_above,
            ladders_below,
            centroid,
        }
    }

    /// Compute the 2D polygon area (ignoring z) using the shoelace formula.
    pub fn size(&self) -> f64 {
        if self.corners.len() < 3 {
            return 0.0;
        }
        let mut x: Vec<f64> = self.corners.iter().map(|c| c.x).collect();
        let mut y: Vec<f64> = self.corners.iter().map(|c| c.y).collect();
        // close polygon loop
        x.push(x[0]);
        y.push(y[0]);

        let mut area = 0.0;
        for i in 0..self.corners.len() {
            area += x[i] * y[i + 1] - y[i] * x[i + 1];
        }
        area.abs() / 2.0
    }

    /// Returns a 2D Shapely Polygon using the (x,y) of the corners.
    pub fn to_polygon_2d(&self) -> Polygon {
        let coords: Vec<(f64, f64)> = self.corners.iter().map(|c| (c.x, c.y)).collect();
        Polygon::new(LineString::from(coords), vec![])
    }

    /// Checks if a point is inside the area by converting to 2D.
    pub fn contains(&self, point: &Position) -> bool {
        self.to_polygon_2d().contains(&point.to_point_2d())
    }

    pub fn centroid_distance(&self, point: &Position) -> f64 {
        self.centroid().distance(point)
    }
}

/// Custom deserialization for `NavArea`
///
/// Can handle a lack of the centroid and calculates it on `NavArea` creation.
impl<'de> Deserialize<'de> for NavArea {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct NavAreaVisitor;

        impl<'de> Visitor<'de> for NavAreaVisitor {
            type Value = NavArea;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("a NavArea struct")
            }

            fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
            where
                A: MapAccess<'de>,
            {
                let mut area_id = None;
                let mut hull_index = None;
                let mut dynamic_attribute_flags = None;
                let mut corners: Option<Vec<Position>> = None;
                let mut connections = None;
                let mut ladders_above = None;
                let mut ladders_below = None;
                let mut nav_centroid = None;

                while let Some(key) = map.next_key::<String>()? {
                    match key.as_str() {
                        "area_id" => area_id = Some(map.next_value()?),
                        "hull_index" => hull_index = Some(map.next_value()?),
                        "dynamic_attribute_flags" => {
                            dynamic_attribute_flags = Some(map.next_value()?);
                        }
                        "corners" => corners = Some(map.next_value()?),
                        "connections" => connections = Some(map.next_value()?),
                        "ladders_above" => ladders_above = Some(map.next_value()?),
                        "ladders_below" => ladders_below = Some(map.next_value()?),
                        "centroid" => nav_centroid = Some(map.next_value()?),
                        _ => {
                            let _: serde::de::IgnoredAny = map.next_value()?;
                        }
                    }
                }

                let area_id = area_id.ok_or_else(|| de::Error::missing_field("area_id"))?;
                let hull_index = hull_index.unwrap_or(0); // Default value
                let dynamic_attribute_flags = dynamic_attribute_flags
                    .ok_or_else(|| de::Error::missing_field("dynamic_attribute_flags"))?;
                let corners = corners.ok_or_else(|| de::Error::missing_field("corners"))?;
                let connections = connections.unwrap_or_default(); // Default value
                let ladders_above = ladders_above.unwrap_or_default(); // Default value
                let ladders_below = ladders_below.unwrap_or_default(); // Default value
                let nav_centroid = nav_centroid.unwrap_or_else(|| centroid(&corners)); // Calculate centroid if missing

                Ok(NavArea {
                    area_id,
                    hull_index,
                    dynamic_attribute_flags,
                    corners,
                    connections,
                    ladders_above,
                    ladders_below,
                    centroid: nav_centroid,
                })
            }
        }

        deserializer.deserialize_struct(
            "NavArea",
            &[
                "area_id",
                "hull_index",
                "dynamic_attribute_flags",
                "corners",
                "connections",
                "ladders_above",
                "ladders_below",
                "centroid",
            ],
            NavAreaVisitor,
        )
    }
}

impl AreaLike for NavArea {
    fn centroid(&self) -> Position {
        self.centroid
    }
    fn requires_crouch(&self) -> bool {
        self.dynamic_attribute_flags == CROUCHING_ATTRIBUTE_FLAG
    }

    fn area_id(&self) -> u32 {
        self.area_id
    }
}

impl From<NewNavArea> for NavArea {
    fn from(item: NewNavArea) -> Self {
        Self {
            area_id: item.area_id,
            hull_index: 0,
            dynamic_attribute_flags: item.dynamic_attribute_flags,
            corners: item.corners,
            connections: Vec::from_iter(item.connections),
            ladders_above: Vec::from_iter(item.ladders_above),
            ladders_below: Vec::from_iter(item.ladders_below),
            centroid: item.centroid,
        }
    }
}

/// Result of a pathfinding operation.
///
/// Contains the path as a list of `NavArea` objects and the total distance.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct PathResult {
    pub path: Vec<NavArea>,
    pub distance: f64,
}

/// Enum for path finding input.
///
/// Can either be the ID of an area or a position.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub enum AreaIdent {
    Id(u32),
    Pos(Position),
}

#[derive(Debug, Clone, Deserialize, Serialize)]
struct NavSerializationHelperStruct {
    pub version: u32,
    pub sub_version: u32,
    pub is_analyzed: bool,
    pub areas: HashMap<u32, NavArea>,
}

#[derive(Debug, Clone)]
pub struct Nav {
    pub version: u32,
    pub sub_version: u32,
    pub areas: HashMap<u32, NavArea>,
    pub is_analyzed: bool,
    pub graph: DiGraphMap<u32, f64>,
}

impl Nav {
    pub const MAGIC: u32 = 0xFEED_FACE;

    pub fn new(
        version: u32,
        sub_version: u32,
        areas: HashMap<u32, NavArea>,
        is_analyzed: bool,
    ) -> Self {
        let mut graph = DiGraphMap::new();

        // Add nodes
        for (area_id, area) in &areas {
            graph.add_node(*area_id);
        }

        // Add edges
        for (area_id, area) in &areas {
            for connected_area_id in &area.connections {
                let connected_area = &areas[connected_area_id];
                let dx = area.centroid().x - connected_area.centroid().x;
                let dy = area.centroid().y - connected_area.centroid().y;
                let dist_weight = dx.hypot(dy);

                // TODO: For ladder connected areas add an additional distance
                // based on height difference and LADDER_SPEED.
                let area_relative_speed = if area.requires_crouch() {
                    CROUCHING_SPEED
                } else {
                    RUNNING_SPEED
                } / RUNNING_SPEED;

                let connected_area_relative_speed = if connected_area.requires_crouch() {
                    CROUCHING_SPEED
                } else {
                    RUNNING_SPEED
                } / RUNNING_SPEED;

                let area_time_adjusted_distance = dist_weight / area_relative_speed;
                let connected_area_time_adjusted_distance =
                    dist_weight / connected_area_relative_speed;
                let time_adjusted =
                    (area_time_adjusted_distance + connected_area_time_adjusted_distance) / 2.0;

                graph.add_edge(*area_id, *connected_area_id, time_adjusted);
            }
        }

        Self {
            version,
            sub_version,
            areas,
            is_analyzed,
            graph,
        }
    }

    /// Find the area that contains the position and has the closest centroid by z.
    ///
    /// If no area contains the position, then `None` is returned.
    pub fn find_area(&self, position: &Position) -> Option<&NavArea> {
        self.areas
            .values()
            .filter(|area| area.contains(position))
            .min_by(|a, b| {
                ((a.centroid().z - position.z).abs() - (b.centroid().z - position.z).abs())
                    .partial_cmp(&0.0)
                    .unwrap()
            })
    }

    /// Find the area with the closest centroid to the position.
    pub fn find_closest_area_centroid(&self, position: &Position) -> &NavArea {
        self.areas
            .values()
            .min_by(|a, b| {
                a.centroid_distance(position)
                    .partial_cmp(&b.centroid_distance(position))
                    .unwrap()
            })
            .unwrap()
    }

    /// Utility heuristic function for A* using Euclidean distance between node centroids.
    fn dist_heuristic(&self, node_a: u32, node_b: u32) -> f64 {
        let a = &self.areas[&node_a].centroid();
        let b = &self.areas[&node_b].centroid();
        a.distance_2d(b)
    }

    /// Utility function to calculate the cost of a path(segment).
    fn path_cost(&self, path: &[u32]) -> f64 {
        path.iter()
            .tuple_windows()
            .map(|(u, v)| self.graph.edge_weight(*u, *v).unwrap())
            .sum()
    }

    /// Finds the path between two areas or positions.
    pub fn find_path(&self, start: AreaIdent, end: AreaIdent) -> PathResult {
        // Find the start areas for path finding.
        let start_area = match start {
            AreaIdent::Pos(pos) => {
                self.find_area(&pos)
                    .unwrap_or_else(|| self.find_closest_area_centroid(&pos))
                    .area_id
            }
            AreaIdent::Id(id) => id,
        };

        let end_area = match end {
            AreaIdent::Pos(pos) => {
                self.find_area(&pos)
                    .unwrap_or_else(|| self.find_closest_area_centroid(&pos))
                    .area_id
            }

            AreaIdent::Id(id) => id,
        };

        // Perform A* path finding.
        let Some((distance, path_ids)) = astar(
            &self.graph,
            start_area,
            |finish| finish == end_area,
            |e| *e.weight(),
            |node| self.dist_heuristic(node, end_area),
        ) else {
            return PathResult {
                path: Vec::new(),
                distance: f64::MAX,
            };
        };

        // Calculate the total distance.
        // Idea is so take the distance from a starting position to the SECOND node in the path.
        let total_distance = if path_ids.len() <= 2 {
            match (start, end) {
                (AreaIdent::Pos(start_pos), AreaIdent::Pos(end_pos)) => {
                    start_pos.distance_2d(&end_pos)
                }
                (AreaIdent::Id(_), AreaIdent::Id(_)) => distance,
                // When one of them is a vector, assume using Euclidean distance to/from centroid.
                (AreaIdent::Pos(start_pos), AreaIdent::Id(_)) => {
                    start_pos.distance_2d(&self.areas[&end_area].centroid())
                }
                (AreaIdent::Id(_), AreaIdent::Pos(end_pos)) => {
                    self.areas[&start_area].centroid().distance_2d(&end_pos)
                }
            }
        } else {
            // Use windows for middle path distances.
            let start_distance = match start {
                AreaIdent::Pos(start_pos) => {
                    start_pos.distance_2d(&self.areas[&path_ids[1]].centroid())
                }
                AreaIdent::Id(_) => self.path_cost(&path_ids[0..=1]),
            };

            let middle_distance: f64 = self.path_cost(&path_ids[1..path_ids.len() - 1]);

            let end_distance = match end {
                AreaIdent::Pos(end_pos) => self.areas[&path_ids[path_ids.len() - 2]]
                    .centroid()
                    .distance_2d(&end_pos),
                AreaIdent::Id(_) => {
                    self.path_cost(&path_ids[path_ids.len() - 2..path_ids.len() - 1])
                }
            };

            start_distance + middle_distance + end_distance
        };

        // Convert the path_ids to NavArea objects.
        let path = path_ids
            .iter()
            .filter_map(|id| self.areas.get(id).cloned())
            .collect();

        PathResult {
            path,
            distance: total_distance,
        }
    }

    pub fn save_to_json(self, filename: &Path) {
        let mut file = create_file_with_parents(filename);
        let helper = NavSerializationHelperStruct {
            version: self.version,
            sub_version: self.sub_version,
            is_analyzed: self.is_analyzed,
            areas: self.areas,
        };
        serde_json::to_writer(&mut file, &helper).unwrap();
    }

    // Load a struct instance from a JSON file
    pub fn from_json(filename: &Path) -> Self {
        let file = File::open(filename).unwrap();
        let helper: NavSerializationHelperStruct = serde_json::from_reader(file).unwrap();
        Self::new(
            helper.version,
            helper.sub_version,
            helper.areas,
            helper.is_analyzed,
        )
    }
}

/// Checks if two areas are visible to each other.
///
/// Area positions are on the floor, so a height correction to eye level is applied.
/// Note that this is conservative and can have false negatives for "actual" visibility.
/// For example if one player can see the feet of another player, but not the head.
pub fn areas_visible<T: AreaLike>(area1: &T, area2: &T, vis_checker: &CollisionChecker) -> bool {
    let height_correction = PLAYER_EYE_LEVEL;

    let area1_centroid = area1.centroid();
    let area2_centroid = area2.centroid();

    let used_centroid1 = Position::new(
        area1_centroid.x,
        area1_centroid.y,
        area1_centroid.z + height_correction,
    );
    let used_centroid2 = Position::new(
        area2_centroid.x,
        area2_centroid.y,
        area2_centroid.z + height_correction,
    );

    vis_checker.connection_unobstructed(used_centroid1, used_centroid2)
}

/// Get or build a cache of visibility between all area pairs in a nav mesh.
pub fn get_visibility_cache(
    map_name: &str,
    granularity: usize,
    nav: &Nav,
    vis_checker: &CollisionChecker,
    safe_to_file: bool,
) -> HashMap<(u32, u32), bool> {
    let tqdm_config = Config::new().with_leave(true);
    let cache_path_str =
        format!("./data/collisions/{map_name}_{granularity}_visibility_cache.vis_cache");
    let cache_path = Path::new(&cache_path_str);
    if cache_path.exists() {
        println!("Loading visibility cache from binary.");
        let file = File::open(cache_path).unwrap();
        deserialize_from(file).unwrap()
    } else {
        println!("Building visibility cache from scratch.");
        let visibility_cache = iproduct!(&nav.areas, &nav.areas)
            .collect::<Vec<_>>()
            .par_iter()
            .tqdm_config(tqdm_config.with_desc("Building visibility cache"))
            .map(|((area_id, area), (other_area_id, other_area))| {
                let visible = areas_visible(*area, *other_area, vis_checker);
                ((**area_id, **other_area_id), visible)
            })
            .collect();
        if safe_to_file {
            let mut file = create_file_with_parents(cache_path);
            serialize_into(&mut file, &visibility_cache).unwrap();
        }
        visibility_cache
    }
}

/// Checks if two areas are walkable to each other.
///
/// Requires a collision checker that includes player clippings.
/// For walkability we need to account for player width and height.
/// For height we also need to consider crouching.
fn areas_walkable<T: AreaLike>(area1: &T, area2: &T, walk_checker: &CollisionChecker) -> bool {
    let height = if area1.requires_crouch() || area2.requires_crouch() {
        PLAYER_CROUCH_HEIGHT
    } else {
        PLAYER_HEIGHT
    };
    // Using the full width can slightly mess up some tight corners, so use 90% of it.
    let width = 0.9 * PLAYER_WIDTH;

    let area1_centroid = area1.centroid();
    let area2_centroid = area2.centroid();

    let dx = area2_centroid.x - area1_centroid.x;
    let dy = area2_centroid.y - area1_centroid.y;
    let angle = dx.atan2(dy);

    for (width_correction, height_correction) in iproduct!([width / 2.0, -width / 2.0], [height]) {
        let dx_corr = width_correction * angle.cos();
        let dy_corr = width_correction * angle.sin();

        let used_centroid1 = Position::new(
            area1_centroid.x + dx_corr,
            area1_centroid.y + dy_corr,
            area1_centroid.z + height_correction,
        );
        let used_centroid2 = Position::new(
            area2_centroid.x + dx_corr,
            area2_centroid.y + dy_corr,
            area2_centroid.z + height_correction,
        );
        if !walk_checker.connection_unobstructed(used_centroid1, used_centroid2) {
            return false;
        }
    }
    true
}

pub fn get_walkability_cache(
    map_name: &str,
    granularity: usize,
    nav: &Nav,
    walk_checker: &CollisionChecker,
) -> HashMap<(u32, u32), bool> {
    let tqdm_config = Config::new().with_leave(true);
    let cache_path_str =
        format!("./data/collisions/{map_name}_{granularity}_walkability_cache.json");
    let cache_path = Path::new(&cache_path_str);
    if cache_path.exists() {
        let file = File::open(cache_path).unwrap();
        serde_json::from_reader(file).unwrap()
    } else {
        let mut file = create_file_with_parents(cache_path);
        let mut walkability_cache = HashMap::default();
        for ((area_id, area), (other_area_id, other_area)) in iproduct!(&nav.areas, &nav.areas)
            .tqdm_config(tqdm_config.with_desc("Building walkability cache"))
        {
            let visible = areas_walkable(area, other_area, walk_checker);
            walkability_cache.insert((*area_id, *other_area_id), visible);
        }
        serde_json::to_writer(&mut file, &walkability_cache).unwrap();
        walkability_cache
    }
}

/// `NavArea` variant that includes the original area IDs that the new area is based on.
#[derive(Debug, Clone, Deserialize, Serialize)]
struct NewNavArea {
    pub area_id: u32,
    pub dynamic_attribute_flags: DynamicAttributeFlags,
    pub corners: Vec<Position>,
    pub connections: HashSet<u32>,
    pub ladders_above: HashSet<u32>,
    pub ladders_below: HashSet<u32>,
    pub orig_ids: HashSet<u32>,
    centroid: Position,
}

impl NewNavArea {
    pub fn new(
        corners: Vec<Position>,
        orig_ids: HashSet<u32>,
        ladders_above: HashSet<u32>,
        ladders_below: HashSet<u32>,
        dynamic_attribute_flags: DynamicAttributeFlags,
        connections: HashSet<u32>,
    ) -> Self {
        let centroid = centroid(&corners);
        Self {
            area_id: 0,
            dynamic_attribute_flags,
            corners,
            connections,
            ladders_above,
            ladders_below,
            orig_ids,
            centroid,
        }
    }
}

impl AreaLike for NewNavArea {
    fn centroid(&self) -> Position {
        self.centroid
    }
    fn requires_crouch(&self) -> bool {
        self.dynamic_attribute_flags == CROUCHING_ATTRIBUTE_FLAG
    }

    fn area_id(&self) -> u32 {
        self.area_id
    }
}

#[derive(Debug, Clone)]
struct AdditionalNavAreaInfo {
    pub polygon: Polygon,
    pub z_level: f64,
}

/// Generate a grid of new navigation areas based on the original areas.
#[allow(clippy::cast_precision_loss)]
fn create_new_nav_areas(
    nav_areas: &HashMap<u32, NavArea>,
    grid_granularity: usize,
    xs: &[f64],
    ys: &[f64],
    area_extra_info: &HashMap<u32, AdditionalNavAreaInfo>,
    tqdm_config: Config,
) -> (Vec<NewNavArea>, HashMap<u32, HashSet<u32>>) {
    // Get the boundaries of the original areas
    let min_x = *xs.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();
    let max_x = *xs.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();
    let min_y = *ys.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();
    let max_y = *ys.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();

    // Determine cell size of the new areas
    let cell_width = (max_x - min_x) / grid_granularity as f64;
    let cell_height = (max_y - min_y) / grid_granularity as f64;

    let mut new_cells: Vec<NewNavArea> = Vec::new();

    // Build the actual areas for the grid.
    for (i, j) in iproduct!(0..grid_granularity, 0..grid_granularity)
        .tqdm_config(tqdm_config.with_desc("Creating grid cell"))
    {
        // Information for the new cell
        let cell_min_x = min_x + j as f64 * cell_width;
        let cell_min_y = min_y + i as f64 * cell_height;
        let cell_max_x = cell_min_x + cell_width;
        let cell_max_y = cell_min_y + cell_height;
        let center_x = (cell_min_x + cell_max_x) / 2.0;
        let center_y = (cell_min_y + cell_max_y) / 2.0;
        let center_point = Point::new(center_x, center_y);

        let cell_poly = Polygon::new(
            LineString::from(vec![
                (cell_min_x, cell_min_y),
                (cell_max_x, cell_min_y),
                (cell_max_x, cell_max_y),
                (cell_min_x, cell_max_y),
            ]),
            vec![],
        );

        // TODO: Create tiles and their z coordinate by player clipping collisions
        // with heaven to floor rays?
        // Get all the original areas that the centroid of the new cell is in in 2D
        // Also get all cells that the new area intersects with, also in 2D.
        let mut primary_origs: HashSet<u32> = HashSet::default();
        let mut extra_orig_ids: HashSet<u32> = HashSet::default();
        for (area_id, info) in area_extra_info {
            if info.polygon.contains(&center_point) {
                primary_origs.insert(*area_id);
            } else if info.polygon.intersects(&cell_poly) {
                extra_orig_ids.insert(*area_id);
            }
        }

        // Skip cells that are outside the bounds of the original map (Holes or irregular shapes)
        if primary_origs.is_empty() && extra_orig_ids.is_empty() {
            continue;
        }

        // If an area has no old area that its center is in, then assign the closest intersecting one.
        let primary_origs = if primary_origs.is_empty() {
            let min_id = extra_orig_ids.iter().min_by(|a, b| {
                let distance_a = Euclidean::distance(
                    &area_extra_info[*a].polygon.centroid().unwrap(),
                    &center_point,
                );

                let distance_b = Euclidean::distance(
                    &area_extra_info[*b].polygon.centroid().unwrap(),
                    &center_point,
                );
                distance_a
                    .partial_cmp(&distance_b)
                    .unwrap_or(Ordering::Equal)
            });
            HashSet::from_iter([*min_id.unwrap()])
        } else {
            primary_origs
        };

        // Generate one new nav area for each old one that the cell is based on.
        for primary in primary_origs {
            let mut cell_orig_ids = HashSet::from_iter([primary]);

            // The new cell z is based on the inverse distance weighting of the old area corners.
            // Just taking the avg z leads to issues with long tiles on slopes.
            let primary_z =
                inverse_distance_weighting(&nav_areas[&primary].corners, (center_x, center_y));

            for other in &extra_orig_ids {
                if *other != primary
                    && (primary_z - area_extra_info[other].z_level).abs() <= JUMP_HEIGHT
                {
                    cell_orig_ids.insert(*other);
                }
            }

            let rep_level = (primary_z * 100.0).round() / 100.0;
            let corners = vec![
                Position::new(cell_min_x, cell_min_y, rep_level),
                Position::new(cell_max_x, cell_min_y, rep_level),
                Position::new(cell_max_x, cell_max_y, rep_level),
                Position::new(cell_min_x, cell_max_y, rep_level),
            ];

            let primary_area = &nav_areas[&primary];
            new_cells.push(NewNavArea::new(
                corners,
                cell_orig_ids,
                HashSet::from_iter(primary_area.ladders_above.clone()),
                HashSet::from_iter(primary_area.ladders_below.clone()),
                primary_area.dynamic_attribute_flags,
                HashSet::default(),
            ));
        }
    }
    println!(); // Newline after tqdm so bars dont override each other.

    let old_to_new_children = build_old_to_new_mapping(&mut new_cells);

    (new_cells, old_to_new_children)
}

/// Build a mapping of old area IDs to all new areas that they are connected to.
#[allow(clippy::cast_possible_truncation)]
fn build_old_to_new_mapping(new_cells: &mut [NewNavArea]) -> HashMap<u32, HashSet<u32>> {
    let mut old_to_new_children: HashMap<u32, HashSet<u32>> = HashMap::default();

    for (idx, new_cell) in new_cells.iter_mut().enumerate() {
        new_cell.area_id = idx as u32;
        for orig_id in &new_cell.orig_ids {
            old_to_new_children
                .entry(*orig_id)
                .or_default()
                .insert(new_cell.area_id);
        }
    }
    old_to_new_children
}

/// Build a regularized navigation mesh with a fixed granularity from the original navigation areas.
///
/// First build a grid of cells and assign each cell to the closest original area.
/// Also consider other original areas that intersect the cell.
/// Then build connections between the new cells based on physical reachability in the game.
/// Finally ensure that old connections are preserved.
#[allow(clippy::cast_possible_truncation)]
#[allow(clippy::cast_precision_loss)]
pub fn regularize_nav_areas(
    nav_areas: &HashMap<u32, NavArea>,
    grid_granularity: usize,
    map_name: &str,
) -> HashMap<u32, NavArea> {
    println!("Regularizing nav areas for {map_name}");

    let tqdm_config = Config::new().with_leave(true);

    let mut xs: Vec<f64> = Vec::new();
    let mut ys: Vec<f64> = Vec::new();
    let mut area_extra_info: HashMap<u32, AdditionalNavAreaInfo> = HashMap::default();
    let walk_checker = load_collision_checker(map_name, CollisionCheckerStyle::Walkability);

    // Precompute the 2D polygon projection and an average-z for each nav area
    for (area_id, area) in nav_areas {
        let coords: Vec<(f64, f64)> = area.corners.iter().map(|c| (c.x, c.y)).collect();
        let poly = Polygon::new(LineString::from(coords), vec![]);
        let avg_z: f64 =
            area.corners.iter().map(|corner| corner.z).sum::<f64>() / area.corners.len() as f64;
        area_extra_info.insert(
            *area_id,
            AdditionalNavAreaInfo {
                polygon: poly,
                z_level: avg_z,
            },
        );

        for corner in &area.corners {
            xs.push(corner.x);
            ys.push(corner.y);
        }
    }

    if xs.is_empty() || ys.is_empty() {
        return HashMap::default();
    }

    // Get the base grid of the new areas
    let (mut new_nav_areas, old_to_new_children) = create_new_nav_areas(
        nav_areas,
        grid_granularity,
        &xs,
        &ys,
        &area_extra_info,
        tqdm_config.clone(),
    );

    // add_intra_area_connections(
    //     &mut new_nav_areas,
    //     &old_to_new_children,
    //     tqdm_config.clone(),
    // );

    add_connections_by_reachability(&mut new_nav_areas, &walk_checker, tqdm_config.clone());

    ensure_inter_area_connections(
        &mut new_nav_areas,
        nav_areas,
        &old_to_new_children,
        tqdm_config,
    );

    new_nav_areas
        .into_iter()
        .enumerate()
        .map(|(idx, area)| (idx as u32, area.into()))
        .collect()
}

/// Ensure that a previous area A that was connected to another area B still has this connection
/// via at least one new area A' that is based on A being connected to a new area B' that is based on B.
fn ensure_inter_area_connections(
    new_nav_areas: &mut [NewNavArea],
    nav_areas: &HashMap<u32, NavArea>,
    old_to_new_children: &HashMap<u32, HashSet<u32>>,
    tqdm_config: Config,
) {
    // Ensure old connections are preserved
    for (a_idx, area_a) in nav_areas
        .iter()
        .tqdm_config(tqdm_config.with_desc("Ensuring old connections"))
    {
        // These are old areas that have no assigned new ones. This can happen if they are
        // never the primary area AND have too large a height difference with all primaries.
        // Can think if there is a useful way to still incorporate them later.
        let Some(children_of_a) = old_to_new_children.get(a_idx) else {
            continue;
        };
        for neighbor_of_a_idx in &area_a.connections {
            let Some(children_of_neighbor_of_a) = old_to_new_children.get(neighbor_of_a_idx) else {
                continue;
            };

            let mut neighbors_of_children_of_a: HashSet<&u32> = HashSet::from_iter(children_of_a);
            for child_of_a in children_of_a {
                neighbors_of_children_of_a.extend(&new_nav_areas[*child_of_a as usize].connections);
            }

            if children_of_neighbor_of_a
                .iter()
                .any(|x| neighbors_of_children_of_a.contains(x))
            {
                // If there is overlap, continue the outer loop
                continue;
            }

            let pairs_of_children =
                iproduct!(children_of_a.iter(), children_of_neighbor_of_a.iter());

            let pairs_of_children = pairs_of_children.sorted_by(|pair_a, pair_b| {
                new_nav_areas[*pair_a.0 as usize]
                    .centroid()
                    .distance_2d(&new_nav_areas[*pair_a.1 as usize].centroid())
                    .partial_cmp(
                        &new_nav_areas[*pair_b.0 as usize]
                            .centroid()
                            .distance_2d(&new_nav_areas[*pair_b.1 as usize].centroid()),
                    )
                    .unwrap()
            });

            // Ideally we would just take the overall min here instead of sorting
            // and taking 3. But due to map weirdnesses it can happen that exactly
            // this one field does not have the proper connection so we need to
            // have a buffer. Trying 3 for now.
            for pair_of_children in pairs_of_children.take(3) {
                new_nav_areas
                    .get_mut(*pair_of_children.0 as usize)
                    .unwrap()
                    .connections
                    .insert(*pair_of_children.1);
            }
        }
    }
    println!();
    // Newline after tqdm so bars dont override each other.
}

/// Add connections between areas based on walkability (`areas_walkable`)
/// and the ability to physically reach the area via a jump in the game.
/// Also accounts for connections via ladders.
fn add_connections_by_reachability(
    new_nav_areas: &mut Vec<NewNavArea>,
    walk_checker: &CollisionChecker,
    tqdm_config: Config,
) {
    let new_connections: Vec<HashSet<u32>> = new_nav_areas
        .par_iter()
        .tqdm_config(tqdm_config.with_desc("Connections from reachability"))
        .map(|area| {
            let mut conns = HashSet::default();
            for other_area in &*new_nav_areas {
                if area.area_id == other_area.area_id
                    || area.connections.contains(&other_area.area_id)
                {
                    continue;
                }

                if (!area.ladders_above.is_disjoint(&other_area.ladders_below))
                    || (!area.ladders_below.is_disjoint(&other_area.ladders_above))
                    || (area.centroid().can_jump_to(&other_area.centroid())
                        && areas_walkable(area, other_area, walk_checker))
                {
                    conns.insert(other_area.area_id);
                }
            }
            conns
        })
        .collect();
    for (area, conns) in new_nav_areas.iter_mut().zip(new_connections) {
        area.connections.extend(conns);
    }
    println!();
    // Newline after tqdm so bars dont override each other.
}

/// Add connections between new areas that comprise the same old areas.
///
/// Build connectivity based solely on the new cell's `orig_ids`.
/// For a new cell A with orig set `A_orig`, connect to new cell B with orig set `B_orig` if:
/// ∃ a in `A_orig` and b in `B_orig` with a == b or b in `nav_areas`[a].connections
fn add_intra_area_connections(
    new_nav_areas: &mut [NewNavArea],
    old_to_new_children: &HashMap<u32, HashSet<u32>>,
    tqdm_config: Config,
) {
    for new_area in &mut new_nav_areas
        .iter_mut()
        .tqdm_config(tqdm_config.with_desc("Connections from inheritance"))
    {
        let parent_areas = &new_area.orig_ids;
        for parent_area in parent_areas {
            let siblings = &old_to_new_children[parent_area];

            for sibling in siblings {
                if *sibling != new_area.area_id {
                    new_area.connections.insert(*sibling);
                }
            }
        }
    }
    println!(); // Newline after tqdm so bars dont override each other.
}

#[derive(Debug, Clone, Deserialize, Serialize, PartialEq, Eq, Hash, Copy)]
pub struct GroupId(u32);

/// Groups the nav areas into groups of a certain size.
///
/// Only works for meshes that are rectangular and have the same cell size.
///
/// Mainly used for building spreads and plotting them to avoid too many plots
/// for close but not explicitly path connected areas. Reason for that is that
/// paths are likely to skip a lot of areas because of jumpability connections.
///
/// Returns mappings:
/// `GroupID` -> [`AreaID`]
/// `AreaID` -> `GroupID`
#[allow(clippy::cast_possible_truncation)]
#[allow(clippy::cast_sign_loss)]
pub fn group_nav_areas(
    nav_areas: &[&NavArea],
    group_size: usize,
) -> (HashMap<GroupId, Vec<u32>>, HashMap<u32, GroupId>) {
    println!("Grouping areas");
    let mut block_map: HashMap<(usize, usize), Vec<&NavArea>> = HashMap::default();

    // Get row and column number of each area in the grid.
    // For that we first need to get the starting point of the grid (min_x, min_y)
    let min_x = nav_areas
        .iter()
        .min_by(|a, b| a.centroid.x.partial_cmp(&b.centroid.x).unwrap())
        .unwrap()
        .centroid
        .x;
    let min_y = nav_areas
        .iter()
        .min_by(|a, b| a.centroid.y.partial_cmp(&b.centroid.y).unwrap())
        .unwrap()
        .centroid
        .y;

    // And the size of each cell in the grid
    // This requires that all cells are of the same size
    let first_area = nav_areas.first().unwrap();
    let tile_min_x = first_area
        .corners
        .iter()
        .map(|c| c.x)
        .fold(f64::INFINITY, f64::min);
    let tile_min_y = first_area
        .corners
        .iter()
        .map(|c| c.y)
        .fold(f64::INFINITY, f64::min);
    let tile_max_x = first_area
        .corners
        .iter()
        .map(|c| c.x)
        .fold(f64::NEG_INFINITY, f64::max);
    let tile_max_y = first_area
        .corners
        .iter()
        .map(|c| c.y)
        .fold(f64::NEG_INFINITY, f64::max);

    let delta_x = tile_max_x - tile_min_x;
    let delta_y = tile_max_y - tile_min_y;

    // Get the group that each area belongs to based on just x-y coordinates
    for area in nav_areas {
        let cell_x = ((area.centroid.x - min_x) / delta_x).round() as usize;
        let cell_y = ((area.centroid.y - min_y) / delta_y).round() as usize;
        block_map
            .entry((cell_x / group_size, cell_y / group_size))
            .or_default()
            .push(area);
    }

    // Sorting for deterministic results and nicer plotting.
    let sorted_blocks: Vec<Vec<&NavArea>> = block_map
        .into_iter()
        .sorted_by_key(|(k, _v)| *k)
        .map(|(_, v)| v)
        .collect();

    let mut group_to_areas: HashMap<GroupId, Vec<u32>> = HashMap::default();
    let mut area_to_group: HashMap<u32, GroupId> = HashMap::default();
    let mut next_group_id: u32 = 0;

    // Loop over each x-y grid group
    for mut areas in sorted_blocks {
        areas.sort_by_key(|a| {
            let cell_x = ((a.centroid.x - min_x) / delta_x).round() as usize;
            let cell_y = ((a.centroid.y - min_y) / delta_y).round() as usize;
            (cell_x, cell_y, a.area_id)
        });

        // We do not want to have multiple levels of z in any group.
        let mut z_groups: Vec<Vec<&NavArea>> = Vec::new();
        for area in areas {
            let cell_coord = (
                ((area.centroid.x - min_x) / delta_x).round() as usize,
                ((area.centroid.y - min_y) / delta_y).round() as usize,
            );
            let mut found = false;

            for group in &mut z_groups {
                // The new area should not go into this group of there is another area
                // with identical x-y coordinates.
                if group.iter().any(|a| {
                    let ax = ((a.centroid.x - min_x) / delta_x).round() as usize;
                    let ay = ((a.centroid.y - min_y) / delta_y).round() as usize;
                    (ax, ay) == cell_coord
                }) {
                    continue;
                }

                // The area should be within jump height of all other areas in the group.
                if group
                    .iter()
                    .all(|a| (a.centroid.z - area.centroid.z).abs() <= JUMP_HEIGHT)
                {
                    group.push(area);
                    found = true;
                    break;
                }
            }

            // If it can not be added to any existing group and it created a new
            if !found {
                z_groups.push(vec![area]);
            }
        }

        // Build the group to area and area to group mappings
        for group in z_groups {
            let group_id = next_group_id;
            next_group_id += 1;
            group_to_areas.insert(GroupId(group_id), group.iter().map(|a| a.area_id).collect());
            for area in group {
                area_to_group.insert(area.area_id, GroupId(group_id));
            }
        }
    }

    (group_to_areas, area_to_group)
}
