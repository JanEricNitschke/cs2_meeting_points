use crate::constants::{CROUCH_JUMP_HEIGHT_GAIN, GRAVITY, PLAYER_WIDTH, RUNNING_SPEED, jump_speed};

use geo::geometry::Point;
use serde::{Deserialize, Serialize};
use std::ops::{Add, Sub};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Position {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Position {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn distance(&self, other: &Self) -> f64 {
        (*self - *other).length()
    }

    pub fn distance_2d(&self, other: &Self) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }

    pub fn dot(&self, other: &Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(&self, other: &Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    pub fn length(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }

    pub fn normalize(&self) -> Self {
        let len = self.length();
        if len == 0.0 {
            return Self::new(0.0, 0.0, 0.0);
        }
        Self::new(self.x / len, self.y / len, self.z / len)
    }

    pub fn to_point_2d(self) -> Point {
        Point::new(self.x, self.y)
    }

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
        let z_at_dest = self.z + jump_speed() * t - 0.5 * GRAVITY * t * t + CROUCH_JUMP_HEIGHT_GAIN;
        // Am i at or above my target height?
        z_at_dest >= other.z
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
