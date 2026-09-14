# Contributing to AuNa

Thanks for your interest in AuNa. This document covers the development
workflow, what to test before opening a pull request, and the expectations that
are specific to a multi-robot ROS 2 project.

By contributing, you agree that your contribution is licensed under the
[MIT License](LICENSE), the license of this repository.

## Getting set up

AuNa is developed inside Docker; a local ROS 2 installation is not required.

```bash
git clone https://github.com/YOUR_USERNAME/AuNa.git
cd AuNa
docker compose run development
```

The `development` service builds and sources the workspace on startup. Inside
the container, the workspace lives under `packages/`.

Alternatively, `.devcontainer/` provides the same environment for VS Code.

## Build and test

From inside the development container:

```bash
cd packages
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

To work on a single package:

```bash
colcon build --packages-select auna_wallfollowing
colcon test --packages-select auna_wallfollowing
```

`scripts/run_quality_checks.sh` runs copyright-header insertion, linting, and
tests in one pass; see [`scripts/README.md`](scripts/README.md).

CI runs the same colcon build and test on ROS 2 Humble and separately verifies
that the Docker image builds. Both must pass before a pull request is merged.

## Coding style

- **C++**: C++17, following the
  [ROS 2 C++ style guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).
  Classes in `PascalCase`, functions and variables in `snake_case`, member
  variables with a trailing underscore (`publisher_`), constants in
  `UPPER_SNAKE_CASE`. Formatting is defined by `.clang-format`.
- **Python**: [PEP 8](https://pep8.org/), checked against `.flake8`.
- **Copyright headers**: every source file carries the MIT header. Run
  `scripts/add_copyright.sh` to add it to new files.

Linting runs as part of `colcon test` via `ament_lint_auto`, so a clean test run
also means a clean lint.

## Expectations for ROS 2 package changes

- Node parameters must be declared with `declare_parameter` and given a sensible
  default; add the parameter to the matching file under `auna_common/config/`
  with a comment explaining it.
- Do not hardcode topic names, frame IDs, or robot indices. Topics belong in
  parameters or in `auna_common/config/`, and nodes must work under any robot
  namespace.
- Launch files are Python (`.launch.py`) and should accept the namespace and
  robot index used by the rest of the stack.
- Changing an existing topic, service, or message is a breaking change. Say so
  explicitly in the pull request and explain why it is necessary — other
  packages and saved configurations depend on these interfaces.
- New dependencies go in `package.xml` as well as `CMakeLists.txt`, so that
  `rosdep` can resolve them in CI.

## Testing multi-robot, Gazebo, and Docker changes

Automated tests cannot cover simulation behavior, so describe what you ran.

- **Algorithm changes** (controllers, filters, estimators): add a unit test
  where the logic can be isolated from ROS interfaces. See
  `packages/src/auna_wallfollowing/test/` for an example that tests a control
  law directly.
- **Multi-robot changes**: run the platooning scenario
  (`WORLD_NAME=arena`, `docker compose --profile platooning up`) and confirm all
  three robots spawn, are correctly namespaced, and respond to their own control
  input only.
- **Gazebo or world changes**: launch both worlds and confirm the robots spawn
  at sane poses and that transforms and the map align in RViz.
- **Docker or Compose changes**: rebuild from scratch
  (`docker compose build --no-cache`) and start at least one full scenario
  profile. Note whether any profile membership changed — a service added to a
  scenario profile changes what every user of that profile runs.
- **Parameter changes**: state the previous and new default and why the change
  is safe for existing users.

## Reporting issues

Open issues through
[GitHub Issues](https://github.com/HarunTeper/AuNa/issues) using the bug report
or feature request template. A useful bug report includes:

- ROS 2 distribution, host OS, and Docker version;
- the scenario: world, Compose profile, number of robots, input source;
- the exact commands you ran;
- the relevant ROS 2 log output, not only the final error line;
- what you expected instead.

Questions and open-ended discussion are also welcome as issues; label them as
questions so they are easy to tell apart from defect reports.

## Pull requests

1. Branch off `main`. **Pull requests should target `main`** unless a
   maintainer asks otherwise; other branches track work on specific ROS 2 or
   Gazebo versions and are not the default integration target.
2. Keep the change focused. Unrelated refactoring in the same pull request makes
   review substantially harder.
3. Fill in the pull request template, including which packages are affected and
   what you tested.
4. Make sure `colcon build` and `colcon test` pass and that no new warnings are
   introduced.
5. Reference the issue the change addresses, if there is one.

Behavior changes to controllers or simulation semantics need a stated technical
justification and evidence — a test, a plot, or a description of the scenario
you ran and what you observed.

## Questions

Open an issue, or contact the maintainer at harun.teper@tu-dortmund.de.
