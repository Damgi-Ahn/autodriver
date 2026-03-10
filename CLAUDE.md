# CLAUDE.md — autodriver

This file provides guidance for AI assistants (Claude and others) working in this repository.

---

## Project Overview

**autodriver** is a ROS (Robot Operating System) based autonomous driving / robotics project.

- **Author:** DamGi Ahn (damgi.dev@gmail.com)
- **License:** MIT (2026)
- **Primary language:** C/C++ (with potential Python for tooling/scripting)
- **Framework:** ROS (Robot Operating System)

The repository is in early initialization stage. The `ros/src/` directory follows the standard ROS workspace layout, where individual ROS packages will be added under `ros/src/`.

---

## Repository Structure

```
autodriver/
├── LICENSE               # MIT License
├── .gitignore            # C/C++ build artifact patterns
├── CLAUDE.md             # This file
└── ros/                  # ROS workspace root
    └── src/              # ROS package sources (catkin/colcon workspace)
        └── <package>/    # Individual ROS packages go here (future)
```

### Expected ROS Package Structure (per package)

Each ROS package under `ros/src/` should follow the standard layout:

```
<package_name>/
├── package.xml           # ROS package manifest
├── CMakeLists.txt        # Build configuration
├── include/
│   └── <package_name>/   # Public C++ headers (.h, .hpp)
├── src/                  # C++ source files (.cpp)
├── scripts/              # Python scripts (if any)
├── launch/               # ROS launch files (.launch, .launch.py)
├── config/               # Parameter files (.yaml)
├── msg/                  # Custom message definitions (.msg)
├── srv/                  # Custom service definitions (.srv)
├── action/               # Custom action definitions (.action)
└── test/                 # Unit and integration tests
```

---

## Development Setup

### Prerequisites

- ROS installation (ROS 1 Noetic or ROS 2 Humble/Iron recommended)
- C++ compiler: GCC 9+ or Clang 10+
- CMake 3.16+
- `catkin_tools` (ROS 1) or `colcon` (ROS 2)

### Building the Workspace (ROS 1 — catkin)

```bash
cd ros
catkin init                  # Initialize catkin workspace (first time only)
catkin build                 # Build all packages
source devel/setup.bash      # Source the workspace
```

### Building the Workspace (ROS 2 — colcon)

```bash
cd ros
colcon build                 # Build all packages
source install/setup.bash    # Source the workspace
```

### Running a Node

```bash
# ROS 1
rosrun <package_name> <node_name>

# ROS 2
ros2 run <package_name> <node_name>
```

---

## Key Conventions

### C++ Style

- Follow the [ROS C++ Style Guide](https://wiki.ros.org/CppStyleGuide)
- Use `snake_case` for variables and function names
- Use `PascalCase` for class names
- Use `UPPER_CASE` for constants and macros
- Header files: `.h` or `.hpp` under `include/<package_name>/`
- Source files: `.cpp` under `src/`
- Include guards or `#pragma once` in all header files

### ROS-specific Conventions

- Node names: `snake_case` (e.g., `lidar_processor`)
- Topic names: `snake_case` with hierarchy (e.g., `/sensors/lidar/points`)
- Parameter names: `snake_case` (e.g., `max_velocity`)
- Use `rclcpp` (ROS 2) or `roscpp` (ROS 1) for C++ nodes
- Prefer launch files for configuring and starting nodes
- Store parameters in YAML config files under `config/`, not hardcoded

### File Naming

- C++ source files: `snake_case.cpp`
- C++ header files: `snake_case.h` or `snake_case.hpp`
- Launch files: `snake_case.launch` or `snake_case.launch.py`
- Config files: `snake_case.yaml`

---

## Git Workflow

### Branch Naming

- Feature branches: `feature/<short-description>`
- Bug fixes: `fix/<short-description>`
- Documentation: `docs/<short-description>`
- Claude AI branches: `claude/<task-description>-<session-id>`

### Commit Style

- Use the imperative mood: "Add lidar subscriber" not "Added lidar subscriber"
- Keep the subject line under 72 characters
- Reference issue numbers where applicable: `Fix obstacle detection (#42)`

### Commit Signing

This repository uses **SSH commit signing**. Commits are signed with:
- Signing key: `/home/claude/.ssh/commit_signing_key.pub`
- Format: SSH
- All commits must be signed — do not use `--no-gpg-sign`

### Remote

```
origin: http://local_proxy@127.0.0.1:43111/git/Damgi-Ahn/autodriver
```

---

## Testing

### ROS Testing Frameworks

- **Unit tests:** [Google Test (gtest)](https://github.com/google/googletest) — for testing C++ logic in isolation
- **ROS integration tests:** `rostest` (ROS 1) or `launch_testing` (ROS 2)

### Running Tests

```bash
# ROS 1 (catkin)
catkin test

# ROS 2 (colcon)
colcon test
colcon test-result --verbose
```

### Test File Location

Tests live in `test/` inside each package. Name test files `test_<feature>.cpp`.

---

## AI Assistant Instructions

When working in this repository as an AI assistant, follow these guidelines:

### Do

- Follow the ROS C++ style guide and naming conventions described above
- Place new packages under `ros/src/<package_name>/`
- Always include `package.xml` and `CMakeLists.txt` when creating a new ROS package
- Use standard ROS message types before defining custom ones
- Keep nodes single-purpose and composable
- Write tests for any non-trivial logic
- Sign all commits (SSH signing is pre-configured)
- Push changes to the correct branch as specified in task instructions

### Avoid

- Do not hardcode topic names or parameters in source code — use launch/config files
- Do not add large binary files to the repository
- Do not skip commit signing (`--no-verify`, `--no-gpg-sign`)
- Do not push to `main`/`master` directly — use feature branches and PRs
- Do not create unnecessary files or directories; keep changes minimal and focused

### When Adding a New ROS Package

1. Create directory: `ros/src/<package_name>/`
2. Add `package.xml` with correct dependencies
3. Add `CMakeLists.txt` configured for catkin or ament (ROS 2)
4. Add source files under `src/` and headers under `include/<package_name>/`
5. Build and verify: `catkin build` or `colcon build`

---

## Current State

The repository is in initial setup. No ROS packages have been implemented yet.
The `ros/src/temp.txt` placeholder will be removed when the first real package is added.

Next expected steps:
1. Define the first ROS package (e.g., a sensor driver or core autonomy node)
2. Add `CMakeLists.txt` and `package.xml` to `ros/`
3. Set up CI/CD (GitHub Actions or equivalent)
4. Add a README.md with project goals and architecture overview
