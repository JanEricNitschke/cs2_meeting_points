use crate::constants::{CROUCH_JUMP_HEIGHT_GAIN, GRAVITY, PLAYER_WIDTH, RUNNING_SPEED, jump_speed};

use geo::geometry::Point;
use pyo3::{FromPyObject, Py, PyRef, PyRefMut, PyResult, pyclass, pyfunction, pymethods};
use serde::{Deserialize, Serialize};
use std::ops::{Add, Div, Mul, Sub};

#[pyclass(eq, module = "cs2_nav")]
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Position {
    #[pyo3(get, set)]
    pub x: f64,
    #[pyo3(get, set)]
    pub y: f64,
    #[pyo3(get, set)]
    pub z: f64,
}

impl Position {
    #[must_use]
    pub fn to_point_2d(self) -> Point {
        Point::new(self.x, self.y)
    }
}

impl Add for Position {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }
}

impl Sub for Position {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }
}

impl Mul<f64> for Position {
    type Output = Self;

    fn mul(self, other: f64) -> Self {
        Self::new(self.x * other, self.y * other, self.z * other)
    }
}

impl Div<f64> for Position {
    type Output = Self;

    fn div(self, other: f64) -> Self {
        Self::new(self.x / other, self.y / other, self.z / other)
    }
}

#[derive(FromPyObject)]
pub(crate) enum PositionFromInputOptions {
    #[pyo3(transparent)]
    Other(Vec<f64>),
    #[pyo3(transparent)]
    Position(Position),
}

#[pymethods]
impl Position {
    #[must_use]
    #[new]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    #[staticmethod]
    pub(crate) fn from_input(value: PositionFromInputOptions) -> PyResult<Self> {
        match value {
            PositionFromInputOptions::Position(pos) => Ok(pos),
            PositionFromInputOptions::Other(input) => {
                if input.len() != 3 {
                    return Err(pyo3::exceptions::PyValueError::new_err(
                        "Input must be a Vector3 or tuple or list of length 3",
                    ));
                }
                Ok(Self::new(input[0], input[1], input[2]))
            }
        }
    }

    #[must_use]
    pub fn __sub__(&self, other: &Self) -> Self {
        *self - *other
    }

    #[must_use]
    #[allow(clippy::missing_const_for_fn)]
    pub fn __add__(&self, other: &Self) -> Self {
        *self + *other
    }

    #[must_use]
    pub fn __mul__(&self, other: f64) -> Self {
        *self * other
    }

    /// Python division operator
    ///
    /// # Errors
    ///
    /// Errors if `other` is zero.
    pub fn __truediv__(&self, other: f64) -> PyResult<Self> {
        if other == 0.0 {
            return Err(pyo3::exceptions::PyZeroDivisionError::new_err(
                "Division by zero",
            ));
        }
        Ok(*self / other)
    }

    #[must_use]
    pub fn distance(&self, other: &Self) -> f64 {
        (*self - *other).length()
    }

    #[must_use]
    pub fn distance_2d(&self, other: &Self) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }

    #[must_use]
    pub fn dot(&self, other: &Self) -> f64 {
        self.z
            .mul_add(other.z, self.x.mul_add(other.x, self.y * other.y))
    }

    #[must_use]
    pub fn cross(&self, other: &Self) -> Self {
        Self::new(
            self.y.mul_add(other.z, -(self.z * other.y)),
            self.z.mul_add(other.x, -(self.x * other.z)),
            self.x.mul_add(other.y, -(self.y * other.x)),
        )
    }

    #[must_use]
    pub fn length(&self) -> f64 {
        self.z
            .mul_add(self.z, self.y.mul_add(self.y, self.x.powi(2)))
            .sqrt()
    }

    #[must_use]
    pub fn normalize(&self) -> Self {
        let len = self.length();
        if len == 0.0 {
            return Self::new(0.0, 0.0, 0.0);
        }
        Self::new(self.x / len, self.y / len, self.z / len)
    }

    /// Check if a jump from self to other is possible
    #[must_use]
    pub fn can_jump_to(&self, other: &Self) -> bool {
        let mut h_distance = self.distance_2d(other);
        if h_distance <= 0.0 {
            return true;
        }
        // Technically the modification factor to player width should be sqrt(2)
        // But i have found that it can then make jumps that are just too far
        // So i have reduced it.
        let foothold_width_correction = PLAYER_WIDTH * 1.15;
        h_distance = 0_f64.max(h_distance - (foothold_width_correction));

        // Time to travel the horizontal distance between self and other
        // with running speed
        // Or if we are closer than the apex, then take the time to the apex
        // Equivalent to setting z_at_dest = self.z + JUMP_HEIGHT + CROUCH_JUMP_HEIGHT_GAIN
        let t = (h_distance / RUNNING_SPEED).max(jump_speed() / GRAVITY);

        // In my jump, at which height am i when i reach the destination x-y distance.
        let z_at_dest = (0.5 * GRAVITY * t).mul_add(-t, jump_speed().mul_add(t, self.z))
            + CROUCH_JUMP_HEIGHT_GAIN;
        // Am i at or above my target height?
        z_at_dest >= other.z
    }

    #[allow(clippy::needless_pass_by_value)]
    fn __iter__(slf: PyRef<'_, Self>) -> PyResult<Py<Iter>> {
        let iter = Iter {
            inner: vec![slf.x, slf.y, slf.z].into_iter(),
        };
        Py::new(slf.py(), iter)
    }
}

#[pyclass]
struct Iter {
    inner: std::vec::IntoIter<f64>,
}

#[pymethods]
impl Iter {
    #[allow(clippy::self_named_constructors)]
    const fn __iter__(slf: PyRef<'_, Self>) -> PyRef<'_, Self> {
        slf
    }

    fn __next__(mut slf: PyRefMut<'_, Self>) -> Option<f64> {
        slf.inner.next()
    }
}

/// Inverse Distance Weighting interpolation of the positions z-values at the target x-y position
///
/// <https://en.wikipedia.org/wiki/Inverse_distance_weighting>
#[must_use]
pub fn inverse_distance_weighting(points: &[Position], target: (f64, f64)) -> f64 {
    let p = 2.0; // Power parameter
    let mut weighted_sum = 0.0;
    let mut weight_sum = 0.0;

    for &pos in points {
        let dx = target.0 - pos.x;
        let dy = target.1 - pos.y;
        let dist = dx.hypot(dy);

        // Avoid division by zero by setting a small threshold
        let weight = if dist < 1e-10 {
            return pos.z; // If target is exactly on a point, return its value
        } else {
            1.0 / dist.powf(p)
        };

        weighted_sum += weight * pos.z;
        weight_sum += weight;
    }

    weighted_sum / weight_sum
}

#[pyfunction]
#[allow(clippy::needless_pass_by_value)]
#[pyo3(name = "inverse_distance_weighting")]
#[must_use]
pub fn idw_py(points: Vec<Position>, target: (f64, f64)) -> f64 {
    inverse_distance_weighting(&points, target)
}
