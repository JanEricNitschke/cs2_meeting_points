use std::process::Command;
use std::fs;
use std::path::Path;
use tempfile::TempDir;
use serde_json::json;
use std::io::Write;

// Helper functions to create test data
mod test_utils {
    use super::*;
    
    pub fn create_test_directories(temp_dir: &Path) {
        fs::create_dir_all(temp_dir.join("maps")).unwrap();
        fs::create_dir_all(temp_dir.join("tri")).unwrap();
        fs::create_dir_all(temp_dir.join("nav")).unwrap();
        fs::create_dir_all(temp_dir.join("spawns")).unwrap();
        fs::create_dir_all(temp_dir.join("hashes")).unwrap();
        fs::create_dir_all(temp_dir.join("results")).unwrap();
    }

    pub fn create_map_data_json(temp_dir: &Path, map_names: &[&str]) {
        let map_data = map_names.iter().fold(json!({}), |mut acc, name| {
            acc[*name] = json!({});
            acc
        });
        
        let map_data_path = temp_dir.join("maps/map-data.json");
        let mut file = fs::File::create(map_data_path).unwrap();
        file.write_all(serde_json::to_string_pretty(&map_data).unwrap().as_bytes()).unwrap();
    }

    pub fn create_dummy_png(temp_dir: &Path, map_name: &str) {
        let png_path = temp_dir.join(format!("maps/{}.png", map_name));
        let mut file = fs::File::create(png_path).unwrap();
        // Minimal valid PNG
        let png_data = vec![
            0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, // PNG signature
            0x00, 0x00, 0x00, 0x0D, // IHDR chunk length (13)
            0x49, 0x48, 0x44, 0x52, // IHDR
            0x00, 0x00, 0x00, 0x01, // Width: 1
            0x00, 0x00, 0x00, 0x01, // Height: 1
            0x08, 0x00, 0x00, 0x00, 0x00, // Bit depth, color type, compression, filter, interlace
            0x3a, 0x7e, 0x9b, 0x55, // CRC
            0x00, 0x00, 0x00, 0x0A, // IDAT chunk length (10)
            0x49, 0x44, 0x41, 0x54, // IDAT
            0x78, 0x9c, 0x62, 0x00, 0x00, 0x00, 0x02, 0x00, 0x01, // Minimal image data
            0xe5, 0x27, 0xde, 0xfc, // CRC
            0x00, 0x00, 0x00, 0x00, // IEND chunk length (0)
            0x49, 0x45, 0x4E, 0x44, // IEND
            0xae, 0x42, 0x60, 0x82, // CRC
        ];
        file.write_all(&png_data).unwrap();
    }

    pub fn create_dummy_tri_file(temp_dir: &Path, map_name: &str, postfix: &str) {
        let tri_path = temp_dir.join(format!("tri/{}{}.tri", map_name, postfix));
        let mut file = fs::File::create(tri_path).unwrap();
        
        // Simple triangle
        let triangles = vec![
            [-1.0f32, -1.0, 0.0, 1.0, -1.0, 0.0, 0.0, 1.0, 0.0],
        ];
        
        for triangle in triangles {
            for value in triangle {
                file.write_all(&value.to_ne_bytes()).unwrap();
            }
        }
    }

    pub fn create_nav_json(temp_dir: &Path, map_name: &str) {
        let nav_data = json!({
            "version": 16,
            "sub_version": 0,
            "is_analyzed": true,
            "areas": {
                "1": {
                    "area_id": 1,
                    "attribute_flags": 0,
                    "dynamic_attribute_flags": 1,
                    "corners": [
                        {"x": -10.0, "y": -10.0, "z": 0.0},
                        {"x": 10.0, "y": -10.0, "z": 0.0},
                        {"x": 10.0, "y": 10.0, "z": 0.0},
                        {"x": -10.0, "y": 10.0, "z": 0.0}
                    ],
                    "connections": [],
                    "ladders_above": [],
                    "ladders_below": []
                }
            }
        });
        
        let nav_path = temp_dir.join(format!("nav/{}.json", map_name));
        let mut file = fs::File::create(nav_path).unwrap();
        file.write_all(serde_json::to_string_pretty(&nav_data).unwrap().as_bytes()).unwrap();
    }

    pub fn create_spawns_json(temp_dir: &Path, map_name: &str) {
        let spawns_data = json!({
            "CT": [
                {"x": -5.0, "y": 0.0, "z": 0.0}
            ],
            "T": [
                {"x": 5.0, "y": 0.0, "z": 0.0}
            ]
        });
        
        let spawns_path = temp_dir.join(format!("spawns/{}.json", map_name));
        let mut file = fs::File::create(spawns_path).unwrap();
        file.write_all(serde_json::to_string_pretty(&spawns_data).unwrap().as_bytes()).unwrap();
    }

    pub fn create_complete_test_map(temp_dir: &Path, map_name: &str) {
        create_dummy_png(temp_dir, map_name);
        create_dummy_tri_file(temp_dir, map_name, "");
        create_dummy_tri_file(temp_dir, map_name, "-clippings");
        create_nav_json(temp_dir, map_name);
        create_spawns_json(temp_dir, map_name);
    }
}

#[test]
fn test_cli_process_maps_command() {
    let temp_dir = TempDir::new().unwrap();
    let base_path = temp_dir.path();
    
    test_utils::create_test_directories(base_path);
    test_utils::create_map_data_json(base_path, &["valid_map", "invalid_map"]);
    
    // Create one complete map and one incomplete map
    test_utils::create_complete_test_map(base_path, "valid_map");
    test_utils::create_dummy_png(base_path, "invalid_map"); // Missing other files
    
    // Get the path to the binary we built
    let cargo_dir = std::env::current_dir().unwrap();
    let binary_path = cargo_dir.join("target/debug/cs2_meeting_points");
    
    // Run the process-maps command from the test directory
    let output = Command::new(&binary_path)
        .arg("process-maps")
        .current_dir(base_path)
        .output()
        .expect("Failed to execute command");
    
    // Check that the command executed successfully
    assert!(output.status.success(), 
        "Command failed with stderr: {}", String::from_utf8_lossy(&output.stderr));
    
    // The output should contain the valid map name
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("valid_map"), "Output should contain valid_map: {}", stdout);
}

#[test] 
fn test_cli_nav_analysis_command() {
    let temp_dir = TempDir::new().unwrap();
    let base_path = temp_dir.path();
    
    test_utils::create_test_directories(base_path);
    test_utils::create_complete_test_map(base_path, "test_nav_map");
    
    // Get the path to the binary we built
    let cargo_dir = std::env::current_dir().unwrap();
    let binary_path = cargo_dir.join("target/debug/cs2_meeting_points");
    
    // Run the nav-analysis command with low granularity to make it fast
    let output = Command::new(&binary_path)
        .args(["nav-analysis", "test_nav_map", "--granularity", "10"])
        .current_dir(base_path)
        .output()
        .expect("Failed to execute command");
    
    // Check that the command executed successfully
    if !output.status.success() {
        println!("STDERR: {}", String::from_utf8_lossy(&output.stderr));
        println!("STDOUT: {}", String::from_utf8_lossy(&output.stdout));
        panic!("Command failed");
    }
    
    // Check that output files were created
    assert!(base_path.join("results/test_nav_map.json").exists(), 
        "Results file should be created");
    
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("test_nav_map"), "Output should contain map name: {}", stdout);
    assert!(stdout.contains("granularity: 10"), "Output should show granularity: {}", stdout);
}

#[test]
fn test_cli_help_commands() {
    // Get the path to the binary we built
    let cargo_dir = std::env::current_dir().unwrap();
    let binary_path = cargo_dir.join("target/debug/cs2_meeting_points");
    
    // Test main help
    let output = Command::new(&binary_path)
        .arg("--help")
        .output()
        .expect("Failed to execute command");
    
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("process-maps"));
    assert!(stdout.contains("nav-analysis"));
    
    // Test nav-analysis help
    let output = Command::new(&binary_path)
        .args(["nav-analysis", "--help"])
        .output()
        .expect("Failed to execute command");
    
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("granularity"));
    assert!(stdout.contains("MAP_NAME"));
}

#[test]
fn test_cli_process_maps_empty_directory() {
    let temp_dir = TempDir::new().unwrap();
    let base_path = temp_dir.path();
    
    test_utils::create_test_directories(base_path);
    test_utils::create_map_data_json(base_path, &[]); // Empty map data
    
    // Get the path to the binary we built
    let cargo_dir = std::env::current_dir().unwrap();
    let binary_path = cargo_dir.join("target/debug/cs2_meeting_points");
    
    // Run the process-maps command on empty directory
    let output = Command::new(&binary_path)
        .arg("process-maps")
        .current_dir(base_path)
        .output()
        .expect("Failed to execute command");
    
    // Should succeed but find no maps
    assert!(output.status.success(),
        "Command failed with stderr: {}", String::from_utf8_lossy(&output.stderr));
    let stdout = String::from_utf8_lossy(&output.stdout);
    // Should output empty list
    assert!(stdout.contains("[]"), "Should output empty list: {}", stdout);
}