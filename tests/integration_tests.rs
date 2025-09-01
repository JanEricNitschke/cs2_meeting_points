use std::collections::HashSet;
use std::fs;
use std::path::Path;
use tempfile::TempDir;

// Import functions from the main module
use cs2_nav::nav::Nav;
use cs2_nav::spread::Spawns;

// Create a helper module for test utilities
mod test_utils {
    use super::*;
    use std::io::Write;

    use serde_json::json;

    pub fn create_test_directories(temp_dir: &Path) {
        fs::create_dir_all(temp_dir.join("maps")).unwrap();
        fs::create_dir_all(temp_dir.join("tri")).unwrap();
        fs::create_dir_all(temp_dir.join("nav")).unwrap();
        fs::create_dir_all(temp_dir.join("spawns")).unwrap();
        fs::create_dir_all(temp_dir.join("hashes")).unwrap();
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
        // Create a minimal PNG file (just a placeholder)
        let png_path = temp_dir.join(format!("maps/{}.png", map_name));
        let mut file = fs::File::create(png_path).unwrap();
        // Write minimal PNG header (8 bytes signature + minimal IHDR chunk)
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
        // Create a simple triangular collision mesh
        let tri_path = temp_dir.join(format!("tri/{}{}.tri", map_name, postfix));
        let mut file = fs::File::create(tri_path).unwrap();
        
        // Create a simple U-shaped collision mesh with a few triangles
        // Each triangle is 9 f32 values (3 points * 3 coordinates)
        let triangles = vec![
            // Left arm of U (ground level)
            [-10.0f32, -10.0, 0.0, -5.0, -10.0, 0.0, -10.0, 10.0, 0.0],
            [-5.0f32, -10.0, 0.0, -5.0, 10.0, 0.0, -10.0, 10.0, 0.0],
            
            // Right arm of U (elevated)
            [5.0f32, -10.0, 200.0, 10.0, -10.0, 200.0, 5.0, 10.0, 200.0],
            [10.0f32, -10.0, 200.0, 10.0, 10.0, 200.0, 5.0, 10.0, 200.0],
            
            // Bottom of U (connecting section)
            [-5.0f32, -10.0, 0.0, 5.0, -10.0, 200.0, -5.0, -5.0, 0.0],
            [5.0f32, -10.0, 200.0, 5.0, -5.0, 200.0, -5.0, -5.0, 0.0],

            // Wall in the middle (tall obstacle)
            [0.0f32, 5.0, 0.0, 0.0, 10.0, 0.0, 0.0, 5.0, 500.0],
            [0.0f32, 10.0, 0.0, 0.0, 10.0, 500.0, 0.0, 5.0, 500.0],
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
                        {"x": -5.0, "y": -10.0, "z": 0.0},
                        {"x": -5.0, "y": 10.0, "z": 0.0},
                        {"x": -10.0, "y": 10.0, "z": 0.0}
                    ],
                    "connections": [2],
                    "ladders_above": [],
                    "ladders_below": []
                },
                "2": {
                    "area_id": 2,
                    "attribute_flags": 0,
                    "dynamic_attribute_flags": 1,
                    "corners": [
                        {"x": 5.0, "y": -10.0, "z": 200.0},
                        {"x": 10.0, "y": -10.0, "z": 200.0},
                        {"x": 10.0, "y": 10.0, "z": 200.0},
                        {"x": 5.0, "y": 10.0, "z": 200.0}
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
                {"x": -7.5, "y": 0.0, "z": 0.0},
                {"x": -7.5, "y": 5.0, "z": 0.0}
            ],
            "T": [
                {"x": 7.5, "y": 0.0, "z": 200.0},
                {"x": 7.5, "y": 5.0, "z": 200.0}
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

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use std::io::Write;
    use cs2_nav::collisions::CollisionChecker;
    
    // We need to include the main.rs functions for testing
    // This is a bit of a hack but necessary for integration testing
    fn expected_files(map_name: &str) -> Vec<String> {
        vec![
            format!("maps/{}.png", map_name),
            format!("tri/{}.tri", map_name),
            format!("tri/{}-clippings.tri", map_name),
            format!("nav/{}.json", map_name),
            format!("spawns/{}.json", map_name),
        ]
    }

    fn collect_valid_maps_in_dir(base_dir: &Path) -> HashSet<String> {
        let map_data_path = base_dir.join("maps/map-data.json");
        let keys: HashSet<String> = if map_data_path.exists() {
            let content = fs::read_to_string(map_data_path).unwrap();
            serde_json::from_str::<serde_json::Value>(&content)
                .unwrap()
                .as_object()
                .unwrap()
                .keys()
                .cloned()
                .collect()
        } else {
            HashSet::new()
        };

        let mut valid_maps = HashSet::new();
        let maps_dir = base_dir.join("maps");

        if let Ok(entries) = fs::read_dir(&maps_dir) {
            for entry in entries.flatten() {
                if let Some(file_name) = entry.file_name().to_str() {
                    if let Some((map_name, _)) = file_name.rsplit_once('.') {
                        // Check if all required files exist
                        let all_exist = expected_files(map_name)
                            .iter()
                            .all(|path| base_dir.join(path).exists());

                        if all_exist && keys.contains(map_name) {
                            valid_maps.insert(map_name.to_string());
                        }
                    }
                }
            }
        }

        valid_maps
    }

    #[test]
    fn test_map_filtering_missing_files() {
        let temp_dir = TempDir::new().unwrap();
        let base_path = temp_dir.path();
        
        test_utils::create_test_directories(base_path);
        test_utils::create_map_data_json(base_path, &["complete_map", "incomplete_map1", "incomplete_map2"]);
        
        // Create a complete map with all files
        test_utils::create_complete_test_map(base_path, "complete_map");
        
        // Create incomplete maps with missing files
        test_utils::create_dummy_png(base_path, "incomplete_map1");
        // incomplete_map1 is missing tri, nav, and spawns files
        
        test_utils::create_dummy_png(base_path, "incomplete_map2");
        test_utils::create_dummy_tri_file(base_path, "incomplete_map2", "");
        // incomplete_map2 is missing clippings tri, nav, and spawns files
        
        let valid_maps = collect_valid_maps_in_dir(base_path);
        
        assert_eq!(valid_maps.len(), 1);
        assert!(valid_maps.contains("complete_map"));
        assert!(!valid_maps.contains("incomplete_map1"));
        assert!(!valid_maps.contains("incomplete_map2"));
    }

    #[test]
    fn test_map_filtering_not_in_map_data() {
        let temp_dir = TempDir::new().unwrap();
        let base_path = temp_dir.path();
        
        test_utils::create_test_directories(base_path);
        test_utils::create_map_data_json(base_path, &["registered_map"]);
        
        // Create a complete map that's registered in map-data.json
        test_utils::create_complete_test_map(base_path, "registered_map");
        
        // Create a complete map that's NOT registered in map-data.json
        test_utils::create_complete_test_map(base_path, "unregistered_map");
        
        let valid_maps = collect_valid_maps_in_dir(base_path);
        
        assert_eq!(valid_maps.len(), 1);
        assert!(valid_maps.contains("registered_map"));
        assert!(!valid_maps.contains("unregistered_map"));
    }

    #[test]
    fn test_load_nav_and_spawns_files() {
        let temp_dir = TempDir::new().unwrap();
        let base_path = temp_dir.path();
        
        test_utils::create_test_directories(base_path);
        test_utils::create_nav_json(base_path, "test_map");
        test_utils::create_spawns_json(base_path, "test_map");
        
        // Test loading nav file
        let nav_path = base_path.join("nav/test_map.json");
        let nav = Nav::from_json(&nav_path);
        assert_eq!(nav.version, 16);
        assert_eq!(nav.sub_version, 0);
        assert_eq!(nav.is_analyzed, true);
        assert_eq!(nav.areas.len(), 2);
        
        // Test loading spawns file
        let spawns_path = base_path.join("spawns/test_map.json");
        let _spawns = Spawns::from_json(&spawns_path);
        // We can't directly inspect the spawns fields since they're private,
        // but we can verify the file loads without error
    }

    #[test]
    fn test_tri_file_creation() {
        let temp_dir = TempDir::new().unwrap();
        let base_path = temp_dir.path();
        
        test_utils::create_test_directories(base_path);
        test_utils::create_dummy_tri_file(base_path, "test_map", "");
        test_utils::create_dummy_tri_file(base_path, "test_map", "-clippings");
        
        let tri_path = base_path.join("tri/test_map.tri");
        let clippings_path = base_path.join("tri/test_map-clippings.tri");
        
        assert!(tri_path.exists());
        assert!(clippings_path.exists());
        
        // Verify file sizes are reasonable (each triangle is 36 bytes, we created 8 triangles)
        let tri_metadata = fs::metadata(tri_path).unwrap();
        let clippings_metadata = fs::metadata(clippings_path).unwrap();
        
        assert_eq!(tri_metadata.len(), 8 * 36); // 8 triangles * 36 bytes each
        assert_eq!(clippings_metadata.len(), 8 * 36);
    }

    #[test]
    fn test_full_pipeline_integration() {
        let temp_dir = TempDir::new().unwrap();
        let base_path = temp_dir.path();
        
        // Set up test environment
        test_utils::create_test_directories(base_path);
        test_utils::create_map_data_json(base_path, &["test_u_map"]);
        test_utils::create_complete_test_map(base_path, "test_u_map");
        
        // Change to temp directory to run tests
        let original_dir = std::env::current_dir().unwrap();
        std::env::set_current_dir(base_path).unwrap();
        
        // Test that the map is detected as valid
        let valid_maps = collect_valid_maps_in_dir(base_path);
        assert_eq!(valid_maps.len(), 1);
        assert!(valid_maps.contains("test_u_map"));
        
        // Test loading navigation and spawns data
        let nav = Nav::from_json(&base_path.join("nav/test_u_map.json"));
        let _spawns = Spawns::from_json(&base_path.join("spawns/test_u_map.json"));
        
        // Verify the navigation mesh has the expected structure
        assert_eq!(nav.version, 16);
        assert_eq!(nav.areas.len(), 2);
        
        // Test collision checker creation (this exercises the tri file reading)
        
        // This would normally load from tri files, but we'll test the tri file format is valid
        let triangles = CollisionChecker::read_tri_file(&base_path.join("tri/test_u_map.tri"), 1000);
        assert_eq!(triangles.len(), 8); // We created 8 triangles in our test data
        
        // Verify triangle data is reasonable
        for triangle in &triangles {
            // Basic sanity check - make sure coordinates are finite
            assert!(triangle.p1.x.is_finite());
            assert!(triangle.p1.y.is_finite());
            assert!(triangle.p1.z.is_finite());
            assert!(triangle.p2.x.is_finite());
            assert!(triangle.p2.y.is_finite());
            assert!(triangle.p2.z.is_finite());
            assert!(triangle.p3.x.is_finite());
            assert!(triangle.p3.y.is_finite());
            assert!(triangle.p3.z.is_finite());
        }
        
        // Restore original directory
        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn test_u_shaped_map_creation() {
        let temp_dir = TempDir::new().unwrap();
        let base_path = temp_dir.path();
        
        test_utils::create_test_directories(base_path);
        
        // Create a more detailed U-shaped map as described in the issue:
        // - One part at height 0
        // - Other parts at +200 and -200
        // - Tall wall in the middle
        let create_detailed_u_map = |temp_dir: &Path, map_name: &str| {
            // Create PNG
            test_utils::create_dummy_png(temp_dir, map_name);
            
            // Create detailed tri files for U-shaped map
            let create_detailed_tri = |temp_dir: &Path, map_name: &str, postfix: &str| {
                let tri_path = temp_dir.join(format!("tri/{}{}.tri", map_name, postfix));
                let mut file = fs::File::create(tri_path).unwrap();
                
                let triangles = vec![
                    // Left arm of U at height 0
                    [-20.0f32, -20.0, 0.0, -10.0, -20.0, 0.0, -20.0, 20.0, 0.0],
                    [-10.0f32, -20.0, 0.0, -10.0, 20.0, 0.0, -20.0, 20.0, 0.0],
                    
                    // Right arm of U - upper part at height +200
                    [10.0f32, 0.0, 200.0, 20.0, 0.0, 200.0, 10.0, 20.0, 200.0],
                    [20.0f32, 0.0, 200.0, 20.0, 20.0, 200.0, 10.0, 20.0, 200.0],
                    
                    // Right arm of U - lower part at height -200
                    [10.0f32, -20.0, -200.0, 20.0, -20.0, -200.0, 10.0, 0.0, -200.0],
                    [20.0f32, -20.0, -200.0, 20.0, 0.0, -200.0, 10.0, 0.0, -200.0],
                    
                    // Bottom connecting part (ramp)
                    [-10.0f32, -20.0, 0.0, 10.0, -20.0, -200.0, -10.0, -15.0, 0.0],
                    [10.0f32, -20.0, -200.0, 10.0, -15.0, -200.0, -10.0, -15.0, 0.0],
                    
                    // Central wall (very tall obstacle)
                    [0.0f32, 15.0, 0.0, 0.0, 20.0, 0.0, 0.0, 15.0, 500.0],
                    [0.0f32, 20.0, 0.0, 0.0, 20.0, 500.0, 0.0, 15.0, 500.0],
                    [0.0f32, 15.0, 500.0, 0.0, 20.0, 500.0, -1.0, 15.0, 500.0],
                    [0.0f32, 20.0, 500.0, -1.0, 20.0, 500.0, -1.0, 15.0, 500.0],
                    [-1.0f32, 15.0, 500.0, -1.0, 20.0, 500.0, -1.0, 15.0, 0.0],
                    [-1.0f32, 20.0, 500.0, -1.0, 20.0, 0.0, -1.0, 15.0, 0.0],
                ];
                
                for triangle in triangles {
                    for value in triangle {
                        file.write_all(&value.to_ne_bytes()).unwrap();
                    }
                }
            };
            
            create_detailed_tri(temp_dir, map_name, "");
            create_detailed_tri(temp_dir, map_name, "-clippings");
            
            // Create detailed nav.json with more areas
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
                            {"x": -20.0, "y": -20.0, "z": 0.0},
                            {"x": -10.0, "y": -20.0, "z": 0.0},
                            {"x": -10.0, "y": 20.0, "z": 0.0},
                            {"x": -20.0, "y": 20.0, "z": 0.0}
                        ],
                        "connections": [2, 3],
                        "ladders_above": [],
                        "ladders_below": []
                    },
                    "2": {
                        "area_id": 2,
                        "attribute_flags": 0,
                        "dynamic_attribute_flags": 1,
                        "corners": [
                            {"x": 10.0, "y": 0.0, "z": 200.0},
                            {"x": 20.0, "y": 0.0, "z": 200.0},
                            {"x": 20.0, "y": 20.0, "z": 200.0},
                            {"x": 10.0, "y": 20.0, "z": 200.0}
                        ],
                        "connections": [1],
                        "ladders_above": [],
                        "ladders_below": []
                    },
                    "3": {
                        "area_id": 3,
                        "attribute_flags": 0,
                        "dynamic_attribute_flags": 1,
                        "corners": [
                            {"x": 10.0, "y": -20.0, "z": -200.0},
                            {"x": 20.0, "y": -20.0, "z": -200.0},
                            {"x": 20.0, "y": 0.0, "z": -200.0},
                            {"x": 10.0, "y": 0.0, "z": -200.0}
                        ],
                        "connections": [1],
                        "ladders_above": [],
                        "ladders_below": []
                    }
                }
            });
            
            let nav_path = temp_dir.join(format!("nav/{}.json", map_name));
            let mut file = fs::File::create(nav_path).unwrap();
            file.write_all(serde_json::to_string_pretty(&nav_data).unwrap().as_bytes()).unwrap();
            
            // Create spawns with strategic positions in the U
            let spawns_data = json!({
                "CT": [
                    {"x": -15.0, "y": -10.0, "z": 0.0},
                    {"x": -15.0, "y": 10.0, "z": 0.0}
                ],
                "T": [
                    {"x": 15.0, "y": 10.0, "z": 200.0},
                    {"x": 15.0, "y": -10.0, "z": -200.0}
                ]
            });
            
            let spawns_path = temp_dir.join(format!("spawns/{}.json", map_name));
            let mut file = fs::File::create(spawns_path).unwrap();
            file.write_all(serde_json::to_string_pretty(&spawns_data).unwrap().as_bytes()).unwrap();
        };
        
        test_utils::create_map_data_json(base_path, &["u_shaped_map"]);
        create_detailed_u_map(base_path, "u_shaped_map");
        
        // Verify the map is complete and valid
        let valid_maps = collect_valid_maps_in_dir(base_path);
        assert_eq!(valid_maps.len(), 1);
        assert!(valid_maps.contains("u_shaped_map"));
        
        // Load and verify the navigation mesh
        let nav = Nav::from_json(&base_path.join("nav/u_shaped_map.json"));
        assert_eq!(nav.areas.len(), 3); // Left arm, right upper arm, right lower arm
        
        // Load triangles and verify count
        let triangles = CollisionChecker::read_tri_file(&base_path.join("tri/u_shaped_map.tri"), 1000);
        assert_eq!(triangles.len(), 14); // We created 14 triangles for the detailed U-map
        
        // Test that triangles have the expected height variations
        let mut has_ground_level = false;
        let mut has_elevated = false;
        let mut has_depressed = false;
        let mut has_wall = false;
        
        for triangle in &triangles {
            let avg_z = (triangle.p1.z + triangle.p2.z + triangle.p3.z) / 3.0;
            if avg_z.abs() < 10.0 { has_ground_level = true; }
            if avg_z > 150.0 { has_elevated = true; }
            if avg_z < -150.0 { has_depressed = true; }
            if avg_z > 400.0 { has_wall = true; }
        }
        
        assert!(has_ground_level, "Should have ground level triangles");
        assert!(has_elevated, "Should have elevated triangles");
        assert!(has_depressed, "Should have depressed triangles");
        assert!(has_wall, "Should have wall triangles");
    }
}