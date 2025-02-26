use std::collections::{HashMap, HashSet};

use crate::constants::{CROUCHING_ATTRIBUTE_FLAG, CROUCHING_SPEED, RUNNING_SPEED};
use crate::position::Position;
use bincode::{deserialize_from, serialize_into};
use geo::Contains;
use geo::geometry::{LineString, Polygon};
use itertools::Itertools;
use petgraph::algo::astar;
use petgraph::graphmap::DiGraphMap;
use petgraph::visit::EdgeRef;
use serde::{Deserialize, Serialize};

use std::fs::File;
// --- DynamicAttributeFlags ---
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Deserialize, Serialize)]
pub struct DynamicAttributeFlags(u32);

impl DynamicAttributeFlags {
    pub fn new<T: Into<u32>>(value: T) -> Self {
        Self(value.into())
    }
}

impl From<DynamicAttributeFlags> for u32 {
    fn from(flag: DynamicAttributeFlags) -> Self {
        flag.0
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct NavArea {
    pub area_id: u32,
    pub hull_index: u32,
    pub dynamic_attribute_flags: DynamicAttributeFlags,
    pub corners: Vec<Position>,
    pub connections: Vec<u32>,
    pub ladders_above: Vec<u32>,
    pub ladders_below: Vec<u32>,
}

impl PartialEq for NavArea {
    fn eq(&self, other: &Self) -> bool {
        self.area_id == other.area_id
    }
}

impl NavArea {
    pub const fn new(area_id: u32, dynamic_attribute_flags: DynamicAttributeFlags) -> Self {
        Self {
            area_id,
            hull_index: 0,
            dynamic_attribute_flags,
            corners: Vec::new(),
            connections: Vec::new(),
            ladders_above: Vec::new(),
            ladders_below: Vec::new(),
        }
    }

    pub fn connected_areas(&self) -> HashSet<u32> {
        self.connections.iter().copied().collect()
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

    #[allow(clippy::cast_precision_loss)]
    /// Computes the centroid of the polygon (averaging all corners).
    pub fn centroid(&self) -> Position {
        if self.corners.is_empty() {
            return Position::new(0.0, 0.0, 0.0);
        }
        let (sum_x, sum_y, sum_z) = self
            .corners
            .iter()
            .fold((0.0, 0.0, 0.0), |(sx, sy, sz), c| {
                (sx + c.x, sy + c.y, sz + c.z)
            });
        let count = self.corners.len() as f64;
        Position::new(sum_x / count, sum_y / count, sum_z / count)
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

impl std::fmt::Display for NavArea {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut conn_ids: Vec<_> = self.connected_areas().into_iter().collect();
        conn_ids.sort_unstable();
        write!(
            f,
            "NavArea(id={}, connected_ids={:?}, points={:?}, size={})",
            self.area_id,
            conn_ids,
            self.corners,
            self.size()
        )
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct PathResult {
    pub path: Vec<NavArea>,
    pub distance: f64,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub enum AreaIdent {
    Id(u32),
    Pos(Position),
}

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

        // Add nodes (with attributes added as node weights)
        for (area_id, area) in &areas {
            graph.add_node(*area_id);
        }

        // Add edges
        for (area_id, area) in &areas {
            for connected_area_id in area.connected_areas() {
                let connected_area = areas
                    .get(&connected_area_id)
                    .expect("Area missing in graph");
                let dx = area.centroid().x - connected_area.centroid().x;
                let dy = area.centroid().y - connected_area.centroid().y;
                let dist_weight = dx.hypot(dy);

                let area_relative_speed = if DynamicAttributeFlags::new(CROUCHING_ATTRIBUTE_FLAG)
                    == area.dynamic_attribute_flags
                {
                    CROUCHING_SPEED
                } else {
                    RUNNING_SPEED
                } / RUNNING_SPEED;

                let connected_area_relative_speed =
                    if DynamicAttributeFlags::new(CROUCHING_ATTRIBUTE_FLAG)
                        == connected_area.dynamic_attribute_flags
                    {
                        CROUCHING_SPEED
                    } else {
                        RUNNING_SPEED
                    } / RUNNING_SPEED;

                let area_time_adjusted_distance = dist_weight / area_relative_speed;
                let connected_area_time_adjusted_distance =
                    dist_weight / connected_area_relative_speed;
                let time_adjusted =
                    (area_time_adjusted_distance + connected_area_time_adjusted_distance) / 2.0;

                graph.add_edge(*area_id, connected_area_id, time_adjusted);
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
        let a = self.areas.get(&node_a).unwrap().centroid();
        let b = self.areas.get(&node_b).unwrap().centroid();
        a.distance_2d(&b)
    }

    fn path_cost(&self, path: &[u32]) -> f64 {
        path.iter()
            .tuple_windows()
            .map(|(u, v)| self.graph.edge_weight(*u, *v).unwrap())
            .sum()
    }

    /// Finds the path between two areas.
    pub fn find_path(&self, start: AreaIdent, end: AreaIdent, weight: Option<&str>) -> PathResult {
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

        // Call astar_path from networkx with our heuristic.
        let Some((distance, path_ids)) = astar(
            &self.graph,
            start_area,
            |finish| finish == end_area,
            |e| *e.weight(),
            |node| self.dist_heuristic(node, end_area),
        ) else {
            return PathResult {
                path: Vec::new(),
                distance: f64::INFINITY,
            };
        };

        // Calculate the total distance.
        let total_distance = if path_ids.len() <= 2 {
            match (start, end) {
                (AreaIdent::Pos(start_pos), AreaIdent::Pos(end_pos)) => {
                    start_pos.distance_2d(&end_pos)
                }
                (AreaIdent::Id(_), AreaIdent::Id(_)) => distance,
                // When one of them is a vector, assume using Euclidean distance to/from centroid.
                (AreaIdent::Pos(start_pos), AreaIdent::Id(_)) => {
                    start_pos.distance_2d(&self.areas.get(&end_area).unwrap().centroid())
                }
                (AreaIdent::Id(_), AreaIdent::Pos(end_pos)) => self
                    .areas
                    .get(&start_area)
                    .unwrap()
                    .centroid()
                    .distance_2d(&end_pos),
            }
        } else {
            // Use windows for middle path distances.
            let start_distance = match start {
                AreaIdent::Pos(start_pos) => {
                    start_pos.distance_2d(&self.areas.get(&path_ids[1]).unwrap().centroid())
                }
                AreaIdent::Id(_) => self.path_cost(&path_ids[0..=1]),
            };

            let middle_distance: f64 = self.path_cost(&path_ids[1..path_ids.len() - 1]);

            let end_distance = match end {
                AreaIdent::Pos(end_pos) => self
                    .areas
                    .get(&path_ids[path_ids.len() - 2])
                    .unwrap()
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

    pub fn save_to_binary(&self, filename: &str) {
        let mut file = File::create(filename).unwrap();
        serialize_into(&mut file, &self.areas).unwrap();
    }

    // Load a struct instance from a JSON file
    pub fn from_binary(filename: &str) -> Self {
        let mut file = File::open(filename).unwrap();
        let areas = deserialize_from(&mut file).unwrap();
        Self::new(0, 0, areas, false)
    }
}
