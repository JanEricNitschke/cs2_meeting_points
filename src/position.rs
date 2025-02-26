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
        let h_distance = self.distance_2d(other) - (PLAYER_WIDTH * 2.0_f64.sqrt());
        if h_distance <= 0.0 {
            return true;
        }

        let t = h_distance / RUNNING_SPEED;
        let z_at_dest = self.z + jump_speed() * t - 0.5 * GRAVITY * t * t + CROUCH_JUMP_HEIGHT_GAIN;
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
