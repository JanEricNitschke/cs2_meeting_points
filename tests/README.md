# Integration Tests

This directory contains integration tests for the CS2 meeting points project.

## Test Overview

### `integration_tests.rs`
Contains unit-style integration tests that test the core functionality:

1. **Map Filtering Tests**:
   - `test_map_filtering_missing_files`: Tests that maps with missing required files are filtered out
   - `test_map_filtering_not_in_map_data`: Tests that maps not registered in map-data.json are filtered out

2. **File Loading Tests**:
   - `test_load_nav_and_spawns_files`: Tests loading of navigation mesh and spawn point files
   - `test_tri_file_creation`: Tests creation and validation of triangular collision mesh files

3. **Pipeline Tests**:
   - `test_full_pipeline_integration`: Tests the full data loading pipeline with realistic test data
   - `test_u_shaped_map_creation`: Tests creation of a complex U-shaped map as specified in the requirements

### `cli_integration_tests.rs`
Contains CLI integration tests that exercise the main application commands:

1. **Command Tests**:
   - `test_cli_process_maps_command`: Tests the `process-maps` command with mixed valid/invalid maps
   - `test_cli_nav_analysis_command`: Tests the `nav-analysis` command on a complete test map
   - `test_cli_process_maps_empty_directory`: Tests `process-maps` with no valid maps
   - `test_cli_help_commands`: Tests help output for both main commands

## Test Data Structure

The tests create minimal but valid input files for various scenarios:

### Required Files (per map)
- `maps/{map_name}.png` - Map image (minimal valid PNG)
- `tri/{map_name}.tri` - Collision triangles (binary format)
- `tri/{map_name}-clippings.tri` - Collision triangles with player clippings
- `nav/{map_name}.json` - Navigation mesh data
- `spawns/{map_name}.json` - Spawn point positions
- `maps/map-data.json` - Registry of available maps

### U-Shaped Test Map

The `test_u_shaped_map_creation` test creates a detailed U-shaped map with:
- Left arm at height 0 (ground level)
- Right arm split into two sections at +200 and -200 height
- Central tall wall obstacle (height 500)
- Strategic spawn points for both teams

This matches the requirements specified in the issue for a U-shaped map with height variations and a tall central wall.

## Running Tests

```bash
# Run all integration tests
cargo test

# Run specific test modules
cargo test integration_tests
cargo test cli_integration_tests

# Run with output
cargo test -- --nocapture
```

## Test Data Generation

The tests include helper functions for generating minimal but valid test data:
- `create_dummy_png()` - Creates minimal valid PNG files
- `create_dummy_tri_file()` - Creates binary triangle mesh files
- `create_nav_json()` - Creates navigation mesh JSON files
- `create_spawns_json()` - Creates spawn point JSON files

These helpers can be used to create additional test scenarios as needed.