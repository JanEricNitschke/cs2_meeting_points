use std::{fs::File, path::Path};

/// Create a file with all its parent directories.
///
/// # Panics
///
/// Panics if the file cannot be created.
#[must_use]
pub fn create_file_with_parents(path: &Path) -> File {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).unwrap();
    }

    File::create(path).unwrap()
}
