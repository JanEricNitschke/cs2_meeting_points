/// Module for constants in Counter-Strike 2

// Sides
pub const CT_SIDE: &str = "ct";
pub const T_SIDE: &str = "t";

// Server
pub const DEFAULT_SERVER_TICKRATE: u32 = 128;

// Rounds
pub const DEFAULT_FREEZE_TIME_IN_SECS: f64 = 20.0;
pub const DEFAULT_ROUND_TIME_IN_SECS: f64 = 115.0;
pub const DEFAULT_BOMB_TIME_IN_SECS: f64 = 40.0;

// Grenades
pub const DEFAULT_SMOKE_DURATION_IN_SECS: f64 = 20.0;
pub const DEFAULT_INFERNO_DURATION_IN_SECS: f64 = 7.03125;

// Movement
pub const RUNNING_SPEED: f64 = 250.0;
pub const GRAVITY: f64 = 800.0;
pub const CROUCHING_SPEED: f64 = 85.0;
pub const CROUCHING_ATTRIBUTE_FLAG: u32 = 65536;
pub const JUMP_HEIGHT: f64 = 55.83;

/// 0.5m * v^2 = m * g * h
/// v = sqrt(2 * g * h)
#[inline(always)]
pub fn jump_speed() -> f64 {
    (2.0 * GRAVITY * JUMP_HEIGHT).sqrt()
}

pub const CROUCH_JUMP_HEIGHT_GAIN: f64 = 66.02 - JUMP_HEIGHT;
pub const PLAYER_WIDTH: f64 = 32.0;

// https://developer.valvesoftware.com/wiki/Counter-Strike:_Global_Offensive/Mapper%27s_Reference
pub const PLAYER_HEIGHT: f64 = 72.0;
pub const PLAYER_EYE_LEVEL: f64 = 64.093811;
