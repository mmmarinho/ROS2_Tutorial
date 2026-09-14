# Issues found in the ROS2 Tutorial

Static audit + manual review of the whole repository (all `docs/source/*.rst` lessons,
`ros2_tutorial_workspace`, `gazebo_tutorial_workspace`, `sas_tutorial_workspace`,
`cmake_tutorial_workspace`, and `docker/`).

Legend — Severity: `HIGH` = a reader following the lesson would hit an error / the build fails;
`MEDIUM` = confusing or misleading, but workarounds exist; `LOW` = cosmetic / dead content / housekeeping.
Status: `[ ]` open · `[x]` fixed · `[-]` won't fix (decided).

Reproduce the full check with: `python3 ../audit_ros2_tutorial.py` (from the repo root's parent dir).

---

## HIGH — breaks the build / a documented command fails

### H1. CMake example references a source file that does not exist
- **Where:** `cmake_tutorial_workspace/src/cpp_cmake_example_qpoases_lib/CMakeLists.txt`
- **Detail:**
  - `project(test_proxsuite)` + `add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)` expanded to building `test_proxsuite.cpp`, **which did not exist**. The only C++ source present is `src/test_qpoases.cpp` (shown to the reader in the `test_qpoases.cpp` tab).
  - `FIND_PACKAGE(Eigen3 REQUIRED)` was present, but the example does **not** use Eigen, so it needlessly forced an Eigen install.
  - Consequence: the "include and link the qpOASES in your project" example in
    `docs/source/cmake/cmake_packages_without_sudo.rst` would fail at configure/build time.
- **Fix:** rename the project so the built source matches the file the reader is shown
  (e.g. `project(test_qpoases)` + `add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)`),
  or add the missing `test_proxsuite.cpp`. Drop `FIND_PACKAGE(Eigen3 REQUIRED)` / the Eigen include
  unless the example actually needs Eigen.
- **Status:** `[x]` fixed in this PR — project renamed to `test_qpoases`, source path set to
  `src/${PROJECT_NAME}.cpp`, Eigen dropped; doc warning + `:emphasize-lines:` offset + the
  `test_dqrobotics.cpp` download label updated to match.

### H2. `rviz.rst`: "Publish sample images" command uses a non-existent executable
- **Where:** `docs/source/transformations/rviz.rst` (~line 114)
- **Detail:** The tab titled *Terminal 1: Publish sample images* instructed
  `ros2 run rqt_image_view image_publisher`. `rqt_image_view` is a visualiser, not a publisher —
  there is no `image_publisher` executable in it, so the command failed. The following sentence even
  said "We will use the visualiser, not the publisher", confirming a publisher was intended.
- **Fix:** use the real ROS2 publisher package, `image_publisher` (ships with `ros-desktop`), whose
  Jazzy executable is `image_publisher_node`. It takes an image path and publishes on `image_raw`,
  so the topic is remapped to `/images` to match the visualiser:
  `ros2 run image_publisher image_publisher_node /path/to/lenna.png --ros-args -r image_raw:=/images`.
  Also corrected the tab title from "Run the bridge" to "Run the visualiser".
- **Status:** `[x]` fixed in this PR.

---

## MEDIUM — misleading / incomplete

### M1. Docker troubleshooting page is dead (duplicate of inline content)
- **Where:** `docs/source/docker/troubleshooting.rst`
- **Detail:** A full page titled *"Troubleshooting docker"* that is **not** referenced by any
  `toctree`, `:doc:` or `include` — the reader can never reach it. Its "interactive shell" topic is
  duplicated inline in `docker/index.rst` under *Tips and troubleshooting*.
- **Fix:** add `docker/troubleshooting` to the root `toctree:: Other content` block, or merge its
  unique content into `docker/index.rst` and delete the file.

### M2. Gazebo "other_content" page is dead
- **Where:** `docs/source/gazebo/other_content.rst`
- **Detail:** Contains a useful *Gazebo and ROS2 structure* sensor/topic mapping table that is
  **not** linked from `gazebo/index.rst` (no toctree/`:doc:`/`include`). Inaccessible to the reader.
- **Fix:** add `gazebo/other_content` to the Gazebo toctree, or delete it if intentionally dropped.

### M3. Unused RST snippet: `the_pycharm_dependencies_warning.rst`
- **Where:** `docs/source/the_pycharm_dependencies_warning.rst`
- **Detail:** A reusable snippet meant to be `.. include::`d, but it is included by **0** files.
  Worse, it references `:ref:`PyCharm is not finding the dependencies``, a label that is not defined
  anywhere — so the moment it is included, the build emits a broken-reference warning.
- **Fix:** either include it where PyCharm dependency warnings are relevant (and define the target
  label), or delete it.

### M4. Unused RST snippet: `the_section_is_optional.rst`
- **Where:** `docs/source/the_section_is_optional.rst`
- **Detail:** A reusable note ("This section is optional, the ROS2 tutorial starts at
  `ROS2 installation`") included by **0** files. (Its target `:ref:`ROS2 installation`` *does* exist,
  so only the "dead file" part applies.)
- **Fix:** include it in the lessons where a section is optional, or delete it.

### M5. Stale ROS distro links (Humble/Foxy) in a Jazzy-focused tutorial
- **Where:** (all should point at `jazzy` unless they are intentional "previous version" pointers)
  - `docs/source/parameters_and_launch.rst` lines 6, 77, 104, 136 — `docs.ros.org/en/humble`
  - `docs/source/python_node_explained.rst` line 42 — `rclpy/blob/humble`; line 63 — `docs.ros2.org/foxy`
  - `docs/source/publishers_and_subscribers.rst` lines 14, 168 — `docs.ros.org/en/humble`
  - `docs/source/create_interface_package.rst` line 115 — `docs.ros.org/en/humble`
  - `docs/source/interfaces.rst` line 10 — `docs.ros.org/en/humble`
  - `docs/source/transformations/tf2.rst` line 173 — `docs.ros2.org/foxy` (API link)
- **Detail:** The tutorial was updated to Jazzy/Ubuntu 24.04, but these concept/API links still point
  at Humble (or Foxy). They still resolve, but readers are sent to the wrong distro's documentation.
- **Note:** `service_servers_and_clients.rst` line 6 and `create_interface_package.rst` line 14 link to
  `ros2-tutorial.readthedocs.io/en/humble/...` **on purpose** ("the previous version") — leave those.
- **Fix:** bump `en/humble` → `en/jazzy` (and `foxy` → `jazzy`) for the links listed above.

---

## LOW — cosmetic / housekeeping

### L1. `cpp_vent.rst` naming
- **Where:** `docs/source/cpp/cpp_vent.rst` (linked from `docs/source/index.rst:235`)
- **Detail:** The filename reads like a typo ("vent" vs "venv"/"intro"), but the page's own title is
  ``#vent`` Demystifying C++ and the content is intentional, so it works. Rename only for clarity.

### L2. `gazebo_nav2` Docker scripts pinned to a `latest` base image
- **Where:** `docker/gazebo_nav2/Dockerfile` (FROM `ghcr.io/uommscrobotics/sfr_ros2:latest`)
  and `docker/gazebo_nav2/compose.yml` (image `sfr_gazebo_nav2:latest`)
- **Detail:** `:latest` tags are not reproducible; the Jazzy-era build could silently change under
  the reader. Not wrong, but a reliability nit.
- **Fix (optional):** pin a digest or dated tag.

---

## Confirmed clean (checked, no problems)

- All Python in the workspaces, the `preamble` package, and every `setup.py` parses cleanly.
- Every `setup.py` entry point maps to an existing module **and** the referenced `main`/function is
  actually defined in it.
- All `toctree` entries, `:doc:` targets, `literalinclude` / `include` files, `image` files, and
  `:download:` links resolve to real files.
- All `package.xml` files are well-formed and their `<name>` matches the directory name.
- Every YAML file (workspaces + `.devcontainer`/compose files) parses.
- Launch `.py` files and C++/CMake packages are internally consistent
  (`RCLCPP_LOCAL_BINARY_NAME` sources exist).
- No broken `:ref:` targets except the one in the dead file M3.

## Suggested next step (not yet done)
A real `colcon build` inside `osrf/ros:jazzy` (or the repo's `murilomarinho/sas:jazzy` CI image) is
the only check not performed here. Given the static audit above, the two HIGH items are the ones that
would actually surface as build/command failures.
