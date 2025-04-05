/// Module for ray collision detection using a Bounding Volume Hierarchy tree.
///
/// Taken from: <https://github.com/pnxenopoulos/awpy/blob/main/awpy/visibility.py>
use crate::position::{Position, PositionFromInputOptions};
use crate::utils::create_file_with_parents;

use bincode::{deserialize_from, serialize_into};
use pyo3::exceptions::PyValueError;
use pyo3::{PyResult, pyclass, pymethods};
use serde::{Deserialize, Serialize};
use std::fs;
use std::fs::File;
use std::io::Read;
use std::path::{Path, PathBuf};

/// A triangle in 3D space used for ray intersection checks.
#[pyclass(module = "cs2_nav")]
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Triangle {
    pub p1: Position,
    pub p2: Position,
    pub p3: Position,
}

#[pymethods]
impl Triangle {
    #[new]
    #[must_use]
    pub const fn new(p1: Position, p2: Position, p3: Position) -> Self {
        Self { p1, p2, p3 }
    }

    #[must_use]
    pub fn get_centroid(&self) -> Position {
        Position::new(
            (self.p1.x + self.p2.x + self.p3.x) / 3.0,
            (self.p1.y + self.p2.y + self.p3.y) / 3.0,
            (self.p1.z + self.p2.z + self.p3.z) / 3.0,
        )
    }

    /// Check for ray-triangle intersection.
    /// Returns Some(distance) if intersecting; otherwise None.
    #[must_use]
    pub fn ray_intersection(&self, ray_origin: &Position, ray_direction: &Position) -> Option<f64> {
        let epsilon = 1e-6;
        let edge1 = self.p2 - self.p1;
        let edge2 = self.p3 - self.p1;
        let h = ray_direction.cross(&edge2);
        let a = edge1.dot(&h);

        if a.abs() < epsilon {
            return None;
        }

        let f = 1.0 / a;
        let s = *ray_origin - self.p1;
        let u = f * s.dot(&h);
        if !(0.0..=1.0).contains(&u) {
            return None;
        }

        let q = s.cross(&edge1);
        let v = f * ray_direction.dot(&q);
        if v < 0.0 || (u + v) > 1.0 {
            return None;
        }

        let t = f * edge2.dot(&q);
        if t > epsilon { Some(t) } else { None }
    }
}

// ---------- Edge ----------
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Edge {
    pub next: i32,
    pub twin: i32,
    pub origin: i32,
    pub face: i32,
}

/// Axis-Aligned Bounding Box for efficient collision detection.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Aabb {
    pub min_point: Position,
    pub max_point: Position,
}

fn check_axis(origin: f64, direction: f64, min_val: f64, max_val: f64, epsilon: f64) -> (f64, f64) {
    if direction.abs() < epsilon {
        if origin < min_val || origin > max_val {
            return (f64::INFINITY, f64::NEG_INFINITY);
        }
        return (f64::NEG_INFINITY, f64::INFINITY);
    }
    let t1 = (min_val - origin) / direction;
    let t2 = (max_val - origin) / direction;
    (t1.min(t2), t1.max(t2))
}

impl Aabb {
    #[must_use]
    pub const fn from_triangle(triangle: &Triangle) -> Self {
        let min_point = Position::new(
            triangle.p1.x.min(triangle.p2.x).min(triangle.p3.x),
            triangle.p1.y.min(triangle.p2.y).min(triangle.p3.y),
            triangle.p1.z.min(triangle.p2.z).min(triangle.p3.z),
        );
        let max_point = Position::new(
            triangle.p1.x.max(triangle.p2.x).max(triangle.p3.x),
            triangle.p1.y.max(triangle.p2.y).max(triangle.p3.y),
            triangle.p1.z.max(triangle.p2.z).max(triangle.p3.z),
        );
        Self {
            min_point,
            max_point,
        }
    }

    #[must_use]
    pub fn intersects_ray(&self, ray_origin: &Position, ray_direction: &Position) -> bool {
        let epsilon = 1e-6;

        let (tx_min, tx_max) = check_axis(
            ray_origin.x,
            ray_direction.x,
            self.min_point.x,
            self.max_point.x,
            epsilon,
        );
        let (ty_min, ty_max) = check_axis(
            ray_origin.y,
            ray_direction.y,
            self.min_point.y,
            self.max_point.y,
            epsilon,
        );
        let (tz_min, tz_max) = check_axis(
            ray_origin.z,
            ray_direction.z,
            self.min_point.z,
            self.max_point.z,
            epsilon,
        );

        let t_enter = tx_min.max(ty_min).max(tz_min);
        let t_exit = tx_max.min(ty_max).min(tz_max);

        t_enter <= t_exit && t_exit >= 0.0
    }
}

impl std::fmt::Display for Aabb {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "AABB(min_point={:?}, max_point={:?})",
            self.min_point, self.max_point
        )
    }
}

/// Node in the Bounding Volume Hierarchy tree.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct BVHNode {
    pub aabb: Aabb,
    pub triangle: Option<Triangle>,
    pub left: Option<Box<BVHNode>>,
    pub right: Option<Box<BVHNode>>,
}

/// Collision checker using a Bounding Volume Hierarchy tree.
#[pyclass(name = "VisibilityChecker", module = "cs2_nav")]
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct CollisionChecker {
    #[pyo3(get)]
    pub n_triangles: usize,
    pub root: BVHNode,
}

impl CollisionChecker {
    /// Construct a new `CollisionChecker` from a file of triangles or an existing list.
    #[must_use]
    pub fn new(tri_file: &Path) -> Self {
        let triangles = Self::read_tri_file(tri_file, 1000);

        let n_triangles = triangles.len();
        let root = Self::build_bvh(triangles);
        Self { n_triangles, root }
    }

    /// Read a .tri file containing triangles.
    ///
    /// From <https://github.com/pnxenopoulos/awpy/blob/main/awpy/visibility.py#L757>
    /// # Panics
    ///
    /// Will panic if no file exists at the given path or if the file cannot be read.
    pub fn read_tri_file<P: AsRef<Path>>(tri_file: P, buffer_size: usize) -> Vec<Triangle> {
        // 9 f32 values per triangle, each f32 is 4 bytes.
        let chunk_size: usize = buffer_size * 9 * 4;
        let mut triangles = Vec::new();
        let mut file = fs::File::open(tri_file).expect("Unable to open tri file");
        let mut buffer = vec![0u8; chunk_size].into_boxed_slice();

        loop {
            let n = file.read(&mut buffer).expect("Failed to read file");
            if n == 0 {
                break;
            }
            // number of complete triangles in the buffer.
            let num_complete_triangles = n / 36;
            for i in 0..num_complete_triangles {
                let offset = i * 36;
                let slice = &buffer[offset..offset + 36];
                let mut values = [0f32; 9];
                for (i, chunk) in slice.chunks_exact(4).enumerate() {
                    values[i] = f32::from_ne_bytes(chunk.try_into().unwrap());
                }
                triangles.push(Triangle {
                    p1: Position::new(
                        f64::from(values[0]),
                        f64::from(values[1]),
                        f64::from(values[2]),
                    ),
                    p2: Position::new(
                        f64::from(values[3]),
                        f64::from(values[4]),
                        f64::from(values[5]),
                    ),
                    p3: Position::new(
                        f64::from(values[6]),
                        f64::from(values[7]),
                        f64::from(values[8]),
                    ),
                });
            }
        }
        triangles
    }

    /// Build a Bounding Volume Hierarchy tree from a list of triangles.
    ///
    /// # Panics
    ///
    /// Will panic if not triangles were provided or a triangle centroid coordinate comparison fails.
    pub fn build_bvh(triangles: Vec<Triangle>) -> BVHNode {
        assert!(!triangles.is_empty(), "No triangles provided");
        if triangles.len() == 1 {
            return BVHNode {
                aabb: Aabb::from_triangle(&triangles[0]),
                triangle: Some(triangles[0].clone()),
                left: None,
                right: None,
            };
        }
        // Compute centroids.
        let centroids: Vec<Position> = triangles.iter().map(Triangle::get_centroid).collect();

        // Find spread along each axis.
        let (min_x, max_x) = centroids
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), c| {
                (min.min(c.x), max.max(c.x))
            });
        let (min_y, max_y) = centroids
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), c| {
                (min.min(c.y), max.max(c.y))
            });
        let (min_z, max_z) = centroids
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), c| {
                (min.min(c.z), max.max(c.z))
            });
        let x_spread = max_x - min_x;
        let y_spread = max_y - min_y;
        let z_spread = max_z - min_z;

        // Choose split axis: 0 = x, 1 = y, 2 = z.
        let axis = if x_spread >= y_spread && x_spread >= z_spread {
            0
        } else if y_spread >= z_spread {
            1
        } else {
            2
        };

        // Sort triangles based on centroid coordinate.
        let mut triangles_sorted = triangles;
        triangles_sorted.sort_by(|a, b| {
            let ca = a.get_centroid();
            let cb = b.get_centroid();
            let coord_a = if axis == 0 {
                ca.x
            } else if axis == 1 {
                ca.y
            } else {
                ca.z
            };
            let coord_b = if axis == 0 {
                cb.x
            } else if axis == 1 {
                cb.y
            } else {
                cb.z
            };
            coord_a.partial_cmp(&coord_b).unwrap()
        });

        let mid = triangles_sorted.len() / 2;
        let left = Self::build_bvh(triangles_sorted[..mid].to_vec());
        let right = Self::build_bvh(triangles_sorted[mid..].to_vec());

        // Create encompassing AABB from children.
        let min_point = Position::new(
            left.aabb.min_point.x.min(right.aabb.min_point.x),
            left.aabb.min_point.y.min(right.aabb.min_point.y),
            left.aabb.min_point.z.min(right.aabb.min_point.z),
        );
        let max_point = Position::new(
            left.aabb.max_point.x.max(right.aabb.max_point.x),
            left.aabb.max_point.y.max(right.aabb.max_point.y),
            left.aabb.max_point.z.max(right.aabb.max_point.z),
        );

        BVHNode {
            aabb: Aabb {
                min_point,
                max_point,
            },
            triangle: None,
            left: Some(Box::new(left)),
            right: Some(Box::new(right)),
        }
    }

    /// Traverse the BVH tree to check for ray intersections.
    fn traverse_bvh(
        node: &BVHNode,
        ray_origin: &Position,
        ray_direction: &Position,
        max_distance: f64,
    ) -> bool {
        if !node.aabb.intersects_ray(ray_origin, ray_direction) {
            return false;
        }

        if let Some(ref tri) = node.triangle {
            if let Some(t) = tri.ray_intersection(ray_origin, ray_direction) {
                return t <= max_distance;
            }
            return false;
        }

        let left_hit = Self::traverse_bvh(
            node.left.as_ref().unwrap(),
            ray_origin,
            ray_direction,
            max_distance,
        );
        let right_hit = Self::traverse_bvh(
            node.right.as_ref().unwrap(),
            ray_origin,
            ray_direction,
            max_distance,
        );
        left_hit || right_hit
    }

    /// Save the loaded collision checker with the BVH to a file.
    ///
    /// # Panics
    ///
    /// Will panic if the file cannot be created or written to.
    pub fn save_to_binary(&self, filename: &Path) {
        let mut file = create_file_with_parents(filename);
        serialize_into(&mut file, &self).unwrap();
    }

    /// Load a struct instance from a JSON file
    /// # Panics
    ///
    /// Will panic if the file cannot be read or deserialized.
    #[must_use]
    pub fn from_binary(filename: &Path) -> Self {
        let mut file = File::open(filename).unwrap();
        deserialize_from(&mut file).unwrap()
    }

    /// Check if the line segment between start and end is visible.
    /// Returns true if no triangle obstructs the view.
    #[must_use]
    pub fn connection_unobstructed(&self, start: Position, end: Position) -> bool {
        let mut direction = end - start;
        let distance = direction.length();
        if distance < 1e-6 {
            return true;
        }
        direction = direction.normalize();
        // If any intersection is found along the ray, then the segment is not visible.
        !Self::traverse_bvh(&self.root, &start, &direction, distance)
    }
}

#[pymethods]
impl CollisionChecker {
    /// Construct a new `CollisionChecker` from a file of triangles or an existing list.
    ///
    /// # Errors
    ///
    /// Will return an error if both or neither of `tri_file` and `triangles` are provided.
    #[new]
    #[pyo3(signature = (path=None, triangles=None))]
    fn py_new(path: Option<PathBuf>, triangles: Option<Vec<Triangle>>) -> PyResult<Self> {
        let triangles = match (path, triangles) {
            (Some(tri_file), None) => Self::read_tri_file(tri_file, 1000),
            (None, Some(triangles)) => triangles,
            _ => {
                return Err(PyValueError::new_err(
                    "Exactly one of tri_file or triangles must be provided",
                ));
            }
        };

        let n_triangles = triangles.len();
        if n_triangles == 0 {
            return Err(PyValueError::new_err("No triangles provided"));
        }
        let root = Self::build_bvh(triangles);
        Ok(Self { n_triangles, root })
    }

    fn is_visible(
        &self,
        start: PositionFromInputOptions,
        end: PositionFromInputOptions,
    ) -> PyResult<bool> {
        Ok(self.connection_unobstructed(Position::from_input(start)?, Position::from_input(end)?))
    }

    #[must_use]
    fn __repr__(&self) -> String {
        format!("VisibilityChecker(n_triangles={})", self.n_triangles)
    }

    #[must_use]
    #[allow(clippy::needless_pass_by_value)]
    #[staticmethod]
    #[pyo3(name = "read_tri_file")]
    #[pyo3(signature = (tri_file, buffer_size=1000))]
    fn py_read_tri_file(tri_file: PathBuf, buffer_size: usize) -> Vec<Triangle> {
        Self::read_tri_file(tri_file, buffer_size)
    }
}

impl std::fmt::Display for CollisionChecker {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "CollisionChecker(n_triangles={})", self.n_triangles)
    }
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub enum CollisionCheckerStyle {
    Visibility,
    Walkability,
}

/// Load a visibility checker from a pickle file if available; otherwise build from a .tri file.
/// # Panics
///
/// Will panic if the bath path for the tri or vis file cannot be constructed.
/// Is "`CURRENT_FILE_PATH`/../../"
#[must_use]
pub fn load_collision_checker(map_name: &str, style: CollisionCheckerStyle) -> CollisionChecker {
    let postfix = match style {
        CollisionCheckerStyle::Visibility => "",
        CollisionCheckerStyle::Walkability => "-clippings",
    };
    let current_file = PathBuf::from(file!());
    let base = current_file
        .parent()
        .expect("No parent found")
        .parent()
        .unwrap();
    let tri_path = base.join("tri").join(format!("{map_name}{postfix}.tri"));
    let mut binary_path = tri_path.clone();
    binary_path.set_extension("vis");

    if binary_path.exists() {
        println!(
            "Loading collision checker with style {style:?} from binary: {}",
            binary_path.file_stem().unwrap().to_string_lossy()
        );
        return CollisionChecker::from_binary(&binary_path);
    }
    println!("{tri_path:?}");
    println!(
        "Building collision checker with style {style:?} from tri: {}",
        tri_path.file_stem().unwrap().to_string_lossy()
    );
    let vis_checker = CollisionChecker::new(&tri_path);
    vis_checker.save_to_binary(&binary_path);
    vis_checker
}
