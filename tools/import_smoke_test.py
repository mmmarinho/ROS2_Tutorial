#!/usr/bin/env python3
"""Import every tutorial Python module so def-time errors fail CI.

Annotations written as ``-> Future[...]`` (rclpy.task.Future is not
generic) are evaluated when the ``def`` executes, so the example nodes
crashed at import time while pyright - which cannot resolve the
untyped rclpy imports - stayed green. This script imports each module
found under */src to reproduce that failure in CI.

Run from the repository root, with ROS and the colcon install
space sourced, e.g.::

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 tools/import_smoke_test.py
"""
import importlib.util
import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
WORKSPACES = ("ros2_tutorial_workspace", "gazebo_tutorial_workspace", "sas_tutorial_workspace")
SKIP_DIRS = {"build", "install", "log", "venv", "__pycache__", "test", ".git"}


def import_file(path: Path, index: int) -> None:
    """Executes a Python file as a throwaway module."""
    name = f"import_smoke_{index}_{path.stem}"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot build an import spec for {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    finally:
        sys.modules.pop(name, None)


def collect_files() -> list[Path]:
    files: list[Path] = []
    for workspace in WORKSPACES:
        root = ROOT / workspace
        if not root.is_dir():
            continue
        for path in sorted(root.rglob("*.py")):
            if SKIP_DIRS.intersection(path.parts) or path.name == "setup.py":
                continue
            files.append(path)
    return files


def extend_sys_path() -> None:
    """Makes sibling tutorial packages importable without a colcon build.

    Only ament_python project dirs (those holding a real python package)
    are added; the raw src/ folders are left out so unbuilt ament_cmake
    interface packages are not shadowed by namespace-directory portions.
    """
    for workspace in WORKSPACES:
        src = ROOT / workspace / "src"
        if not src.is_dir():
            continue
        for project in sorted(src.iterdir()):
            if not project.is_dir() or project.name.startswith("."):
                continue
            if any((project / child / "__init__.py").is_file()
                   for child in project.iterdir() if child.is_dir()):
                sys.path.insert(0, str(project))


def main() -> int:
    files = collect_files()
    extend_sys_path()
    failures: list[Path] = []
    skipped: list[tuple[Path, str]] = []
    for index, path in enumerate(files):
        try:
            import_file(path, index)
        except ModuleNotFoundError as e:
            skipped.append((path, str(e)))
            print(f"SKIP {path.relative_to(ROOT)} ({e})")
        except Exception:
            failures.append(path)
            print(f"FAIL {path.relative_to(ROOT)}")
            traceback.print_exc()
    print(f"{len(files) - len(failures) - len(skipped)}/{len(files)} modules imported cleanly,"
          f" {len(skipped)} skipped, {len(failures)} failed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
