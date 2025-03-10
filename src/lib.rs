#![allow(unknown_lints)]
#![allow(clippy::suboptimal_flops)]
#![allow(clippy::similar_names)]
#![allow(clippy::many_single_char_names)]
#![allow(clippy::multiple_crate_versions)]
#![allow(clippy::implicit_hasher)]
#![allow(clippy::unsafe_derive_deserialize)]
#![allow(clippy::redundant_pub_crate)]

use crate::collisions::{CollisionChecker, Triangle};
use crate::nav::{
    DynamicAttributeFlags, InvalidNavFileError, Nav, NavArea, PathResult, py_group_nav_areas,
    py_regularize_nav_areas,
};
use crate::position::{Position, idw_py};
use pyo3::{
    Python,
    prelude::{Bound, PyModule, PyResult, pymodule},
    types::PyModuleMethods,
    wrap_pyfunction,
};

pub mod collisions;
pub mod constants;
pub mod nav;
pub mod position;
pub mod spread;
pub mod utils;

#[pymodule]
fn cs2_nav(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Position>()?;
    m.add_function(wrap_pyfunction!(idw_py, m)?)?;
    m.add_class::<DynamicAttributeFlags>()?;
    m.add_class::<NavArea>()?;
    m.add_class::<Nav>()?;
    m.add_class::<PathResult>()?;
    m.add_function(wrap_pyfunction!(py_group_nav_areas, m)?)?;
    m.add_function(wrap_pyfunction!(py_regularize_nav_areas, m)?)?;
    m.add_class::<Triangle>()?;
    m.add_class::<CollisionChecker>()?;
    m.add("InvalidNavFileError", py.get_type::<InvalidNavFileError>())?;
    Ok(())
}
