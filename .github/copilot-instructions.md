# CS2-Nav - CS2 Meeting Points Navigation and Visibility Utilities

**ALWAYS follow these instructions first and only fallback to additional search and context gathering if the information in these instructions is incomplete or found to be in error.**

CS2-Nav is a Rust library with Python bindings that provides navigation and visibility utilities for Counter-Strike 2 maps. It includes both a command-line tool (`cs2_meeting_points`) for processing map data and a Python library (`cs2_nav`) for analysis and visualization.

## Working Effectively

### Bootstrap and Build the Repository

1. **Install Rust toolchain** (if not already installed):
   ```bash
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   source ~/.cargo/env
   rustup component add clippy
   ```

2. **Build the Rust project**:
   ```bash
   cargo build --verbose
   ```
   - **NEVER CANCEL**: Takes approximately 1-2 minutes for initial build with dependencies. Set timeout to 5+ minutes.
   - Subsequent builds are much faster (~5-10 seconds).

3. **Build optimized release version**:
   ```bash
   cargo build --release
   ```
   - **NEVER CANCEL**: Takes approximately 1-2 minutes. Set timeout to 5+ minutes.

### Python Bindings Development

1. **Set up Python virtual environment and install maturin**:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install maturin pytest patchelf mypy ruff
   ```
   **Note**: If pip install fails due to network timeouts, install packages individually or retry.

2. **Build Python bindings**:
   ```bash
   maturin develop
   ```
   - **NEVER CANCEL**: Takes approximately 10-15 seconds. Set timeout to 2+ minutes.

3. **Install plotting dependencies** (for visualization scripts):
   ```bash
   pip install matplotlib numpy tqdm
   ```
   **Note**: If pip install fails due to network timeouts, install packages individually or retry.

### Testing

1. **Run Rust tests**:
   ```bash
   cargo test --verbose
   ```
   - **NEVER CANCEL**: Takes approximately 3-5 seconds. Set timeout to 2+ minutes.

2. **Run Python binding tests**:
   ```bash
   source .venv/bin/activate
   pytest tests/
   ```
   - **NEVER CANCEL**: Takes approximately 1-2 seconds. Set timeout to 2+ minutes.

### Linting and Formatting

1. **Rust linting**:
   ```bash
   cargo clippy -- -D warnings
   ```
   - **NEVER CANCEL**: Takes approximately 10-15 seconds. Set timeout to 2+ minutes.

2. **Rust formatting check**:
   ```bash
   cargo fmt --check
   ```
   - Takes <1 second.

3. **Apply Rust formatting**:
   ```bash
   cargo fmt
   ```

4. **Python linting**:
   ```bash
   source .venv/bin/activate
   ruff check
   ```
   - Takes <1 second.

5. **Python type checking**:
   ```bash
   source .venv/bin/activate
   python3 -m mypy.stubtest cs2_nav --allowlist tests/mypy-stubtest-allowlist.txt --ignore-unused-allowlist
   ```
   - Takes approximately 1-2 seconds.

## Running the Application

### Command-Line Tool

The `cs2_meeting_points` binary provides two main commands:

1. **Process map hashes**:
   ```bash
   ./target/release/cs2_meeting_points process-maps
   ```

2. **Perform navigation analysis for a specific map**:
   ```bash
   ./target/release/cs2_meeting_points nav-analysis <MAP_NAME> --granularity 200
   ```

**Important**: The CLI requires specific data files to operate:
- Map images: `maps/{map_name}.png`
- Collision triangles: `tri/{map_name}.tri` and `tri/{map_name}-clippings.tri`
- Navigation mesh: `nav/{map_name}.json`
- Spawn points: `spawns/{map_name}.json`

These data files are not included in the repository due to size constraints but are required for full functionality.

### Python Library

After building with `maturin develop`, you can import and use the library:

```python
import cs2_nav
from cs2_nav import Position, Nav, NavArea
```

### Visualization Scripts

The `scripts/` directory contains Python scripts for visualization:

1. **Plot spread visualization**:
   ```bash
   source .venv/bin/activate
   python scripts/plot_spread.py <map_name>
   ```
   
2. **Generate GIF animations**:
   ```bash
   ./scripts/generate_gif.sh <map_name>
   ```

**Note**: Plotting scripts require map data files and images which are excluded from the repository.

## Validation

### Pre-commit Validation

**ALWAYS run these commands before committing changes** or the CI will fail:

1. **Format code**:
   ```bash
   cargo fmt
   ```

2. **Lint code**:
   ```bash
   cargo clippy -- -D warnings
   source .venv/bin/activate
   ruff check
   ```

3. **Run all tests**:
   ```bash
   cargo test --verbose
   source .venv/bin/activate
   pytest tests/
   ```

### Manual Validation Scenarios

**ALWAYS test these scenarios after making changes**:

1. **Basic Rust functionality**:
   - Build succeeds: `cargo build`
   - Tests pass: `cargo test`
   - CLI help works: `./target/debug/cs2_meeting_points --help`

2. **Python bindings functionality**:
   - Build succeeds: `maturin develop`
   - Import works: `python -c "import cs2_nav; print('Success')"`
   - Tests pass: `pytest tests/`

3. **Type stubs validation**:
   - Run stubtest: `python3 -m mypy.stubtest cs2_nav --allowlist tests/mypy-stubtest-allowlist.txt`

## Project Structure

### Core Directories

- `src/` - Rust source code
  - `main.rs` - CLI application entry point
  - `lib.rs` - Python binding definitions
  - `nav.rs` - Navigation mesh processing
  - `collisions.rs` - Collision detection
  - `position.rs` - 3D position utilities
  - `spread.rs` - Spread generation algorithms
  - `utils.rs` - Utility functions

- `tests/` - Python binding tests
  - `test_bindings.py` - Comprehensive tests for all Python bindings
  - `data/` - Test data files

- `scripts/` - Visualization and utility scripts
  - `plot_spread.py` - Map spread visualization
  - `generate_gif.sh` - GIF generation from spread images

### Configuration Files

- `Cargo.toml` - Rust project configuration and dependencies
- `pyproject.toml` - Python project configuration and tool settings
- `.pre-commit-config.yaml` - Pre-commit hooks configuration
- `.clippy.toml` - Clippy linting configuration

## Common Patterns and Idioms

1. **Memory Management**: The project uses jemalloc for better performance on non-MSVC targets.

2. **Error Handling**: Uses standard Rust Result types throughout.

3. **Parallelization**: Uses rayon for parallel processing of large datasets.

4. **Serialization**: Uses serde for JSON serialization/deserialization.

5. **Python Integration**: Uses PyO3 for seamless Rust-Python integration.

## Troubleshooting

### Build Issues

1. **Missing Rust toolchain**: Install via rustup as shown above.
2. **Python binding build fails**: Ensure maturin is installed in virtual environment.
3. **Test failures**: Check that all dependencies are installed correctly.
4. **pip install network timeouts**: Retry installation or install packages individually:
   ```bash
   pip install --timeout 1000 maturin
   pip install --timeout 1000 pytest patchelf mypy ruff
   ```

### Runtime Issues

1. **CLI panics on missing files**: Ensure required data files are present.
2. **Import errors in Python**: Ensure `maturin develop` was run successfully.
3. **Plotting script failures**: Install matplotlib, numpy, and tqdm dependencies.

## CI/CD Information

The GitHub Actions workflow (`.github/workflows/build.yaml`) runs:
1. Cargo build and test on Ubuntu
2. Clippy linting with strict warnings
3. Python binding tests across multiple Python versions (3.8-3.13)
4. Type stub validation

**CRITICAL TIMING EXPECTATIONS**:
- Initial build: 2-3 minutes
- Tests: <30 seconds
- Linting: 30 seconds
- **NEVER CANCEL** any build or test command during CI validation

Ensure all local validation passes before pushing to avoid CI failures.