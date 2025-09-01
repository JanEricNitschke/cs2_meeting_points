use image::GenericImageView;
use image::ImageReader;
use serde_json::Value;
use std::fs;
use std::path::Path;
use std::process::Command;

/// Base reference directory
static REF_INPUT: &str = "tests/references/input";
static REF_OUTPUT: &str = "tests/references/output";

/// Helper to either symlink (preferred) or copy a directory
fn link_or_copy_dir(src: &Path, dst: &Path) -> std::io::Result<()> {
    if cfg!(windows) {
        fs::create_dir_all(dst)?;
        for entry in fs::read_dir(src)? {
            let entry = entry?;
            let file_type = entry.file_type()?;
            let src_path = entry.path();
            let dst_path = dst.join(entry.file_name());
            if file_type.is_dir() {
                link_or_copy_dir(&src_path, &dst_path)?;
            } else {
                fs::copy(&src_path, &dst_path)?;
            }
        }
    } else {
        #[cfg(unix)]
        std::os::unix::fs::symlink(src, dst)?;
    }
    Ok(())
}

/// Helper to remove directories or symlinks
fn remove_dir_or_symlink(path: &Path) {
    if path.exists() {
        if path.is_dir() {
            fs::remove_dir_all(path).unwrap();
        } else {
            fs::remove_file(path).unwrap();
        }
    }
}

#[allow(clippy::too_many_lines)]
#[test]
fn integration_process_maps() {
    // setup: link or copy input directories
    let tmp_dirs = [
        (format!("{REF_INPUT}/nav"), "nav"),
        (format!("{REF_INPUT}/spawns"), "spawns"),
        (format!("{REF_INPUT}/maps"), "maps"),
        (format!("{REF_INPUT}/tri"), "tri"),
    ];
    for (src, dst) in &tmp_dirs {
        link_or_copy_dir(Path::new(src), Path::new(dst)).unwrap();
    }

    // 1. Run process-maps
    let output = Command::new("cargo")
        .args(["run", "--release", "--", "process-maps"])
        .output()
        .expect("failed to run process-maps");

    remove_dir_or_symlink(Path::new("hashes/test_good.txt"));

    assert!(output.status.success(), "process-maps failed");

    let maps_json: Value = serde_json::from_slice(&output.stdout)
        .expect("failed to parse process-maps stdout as JSON");
    let maps = maps_json
        .as_array()
        .expect("expected JSON array of map names")
        .iter()
        .map(|v| v.as_str().expect("map name must be a string"))
        .collect::<Vec<_>>();

    assert_eq!(
        maps.as_slice(),
        &["test_good"],
        "process-maps output unexpected"
    );

    let map = maps[0];

    // 2. Run nav-analysis
    let status = Command::new("cargo")
        .args([
            "run",
            "--release",
            "--",
            "nav-analysis",
            "--granularity",
            "100",
            map,
        ])
        .status()
        .expect("failed to run nav-analysis");
    assert!(status.success());

    for path in &["nav", "spawns", "tri"] {
        remove_dir_or_symlink(Path::new(path));
    }

    // 3. Run Python plotting
    let status = Command::new("uv")
        .args([
            "run",
            "-q",
            "--no-project",
            "--with",
            "tqdm",
            "--with",
            "matplotlib",
            "--with",
            "numpy",
            "--with",
            "pillow",
            "scripts/plot_spread.py",
            map,
        ])
        .status()
        .expect("failed to run plot_spread.py");
    assert!(status.success());

    remove_dir_or_symlink(Path::new("result"));
    remove_dir_or_symlink(Path::new("maps"));

    // 4. Compare generated vs reference
    let out_dir = format!("spread_images/{map}");
    let ref_dir = format!("{REF_OUTPUT}/spread_images/{map}");

    let mut entries: Vec<_> = fs::read_dir(&ref_dir)
        .unwrap()
        .map(|e| e.unwrap())
        .collect();

    // sort numerically by the suffix after "spread_test_good_"
    entries.sort_by_key(|e| {
        let fname = e.file_name();
        let fname = fname.to_string_lossy();
        // parse number from the filename
        fname
            .strip_prefix("spread_test_good_")
            .and_then(|s| s.strip_suffix(".png"))
            .and_then(|num| num.parse::<u32>().ok())
            .unwrap_or(u32::MAX) // fallback in case of unexpected file
    });

    // We only check that the same number of files exist and
    // that they have the same dimensions.
    // Because there can actually be slight differences between what i get locally
    // vs what i get in CI, probably due to floating point stuff?
    for entry in entries {
        let ref_path = entry.path();
        let gen_path = Path::new(&out_dir).join(entry.file_name());

        assert!(
            gen_path.exists(),
            "Missing generated file: {}",
            gen_path.display()
        );

        let ref_img = ImageReader::open(&ref_path).unwrap().decode().unwrap();
        let gen_img = ImageReader::open(&gen_path).unwrap().decode().unwrap();

        assert_eq!(
            ref_img.dimensions(),
            gen_img.dimensions(),
            "Dimensions differ in {}",
            ref_path.display()
        );
    }

    remove_dir_or_symlink(Path::new("spread_images/test_good"));
}
