# Development Scripts

This directory contains scripts for code quality, linting, and maintenance of the AuNa ROS2 project.

## Table of Contents

- [Copyright Management](#copyright-management)
- [Linting and Code Quality](#linting-and-code-quality)
- [Quick Start](#quick-start)
- [What Gets Fixed](#what-gets-fixed)
- [Recommended Workflow](#recommended-workflow)

---

## Copyright Management

### `add_copyright.sh`

Add MIT License copyright headers to all ROS2 package files using the official `ament_copyright` tool.

**Usage:**

```bash
# Use default author and license
./scripts/add_copyright.sh

# Specify custom author and license
./scripts/add_copyright.sh "Your Name" "MIT"
```

**Features:**

- Uses official ROS2 `ament_copyright` tool
- Adds copyright headers to C++, Python, and launch files
- Skips files that already have copyright headers
- Automatically installs `ament_copyright` if missing (interactive mode)
- Simple and reliable using standard ROS2 tooling

**Requirements:**

- `python3-ament-copyright` package (auto-installed if missing)

### `add_copyright_ci.sh`

Non-interactive version for CI/CD pipelines. Same functionality as `add_copyright.sh` but suitable for automation.

**Usage:**

```bash
./scripts/add_copyright_ci.sh "Harun Teper" "MIT"
```

---

## Linting and Code Quality

### `docker_lint_runner.sh` ⭐ Recommended

Convenience wrapper script that uses Docker Compose to run linting fixes in an isolated environment.

**Usage:**

```bash
# Run with cached image
./scripts/docker_lint_runner.sh

# Force rebuild of Docker image
./scripts/docker_lint_runner.sh --rebuild
```

**What it does:**

1. Builds the development Docker image with all linting tools pre-installed
2. Runs the comprehensive `fix_all_linting.sh` script inside the container
3. Applies fixes to your local workspace files
4. Provides next-step instructions

**Advantages:**

- ✅ No local installation of linting tools required
- ✅ Consistent environment across all developers
- ✅ Same tools as CI/CD pipeline
- ✅ Easy to use - just one command

### `fix_all_linting.sh`

Comprehensive linting fix script that addresses all types of linting issues found in GitHub Actions CI.

**Usage:**

```bash
# Run from workspace root
./scripts/fix_all_linting.sh

# Or inside Docker container
docker compose run --rm development bash -c "./scripts/fix_all_linting.sh"
```

**What it fixes:**

**Python:**
- Import sorting with `isort` (black-compatible profile)
- Code formatting with `black` (line length: 99)
- Linting fixes with `ruff`
- Unused imports removal with `autoflake`
- Docstring formatting with `docformatter`
- Blank line issues after docstrings (PEP 257 D202)

**C++:**
- Include sorting with `clang-format`
- Code formatting with `uncrustify` (ROS2 style)
- Header guard fixes (PKG__FILE_HPP_ format)
- Auto-insertion of missing `#include` statements (from cpplint hints)

**CMake:**
- Trailing whitespace removal

**General:**
- Ensures all files end with newline

**Requirements:**

The script will attempt to install missing tools, but for best results, run inside Docker:

```bash
python3-pip
black
ruff
isort
autoflake
docformatter
uncrustify
clang-format
clang-tidy
ament-cpplint
```

---

## Quick Start

### Option 1: Docker (Recommended) 🐳

```bash
# One-command fix for everything
./scripts/docker_lint_runner.sh

# Review changes
git diff

# Commit if satisfied
git add .
git commit -m "fix: apply automatic linting fixes"
```

### Option 2: Manual Execution

```bash
# Run locally (requires tools installed)
./scripts/fix_all_linting.sh

# Or run in Docker container manually
docker compose run --rm development bash
cd /home/ubuntu/workspace
./scripts/fix_all_linting.sh
```

### Option 3: Using Docker Compose Profiles

```bash
# Build development image
docker compose build development

# Run linting fixes via docker-compose profile
docker compose --profile lint-fix up lint-fix

# Build and run all tests
docker compose --profile test up test

# For interactive development
docker compose --profile development up -d development
docker compose exec development bash
```

---

## What Gets Fixed

### ✅ Automatic Fixes

These issues are automatically corrected by the scripts:

#### Python

- ❌ E501: Line too long → ✅ Wrapped to 99 characters
- ❌ F401: Unused imports → ✅ Removed
- ❌ E302/E303: Blank line issues → ✅ Fixed
- ❌ W291: Trailing whitespace → ✅ Removed
- ❌ Import order issues → ✅ Sorted with isort
- ❌ D202: Blank line after docstring → ✅ Fixed
- ❌ Inconsistent quotes → ✅ Normalized

#### C++

- ❌ Uncrustify formatting → ✅ Applied ROS2 style
- ❌ Include order → ✅ Sorted by clang-format
- ❌ Header guards → ✅ Standardized to PKG__FILE_HPP_
- ❌ Missing includes → ✅ Auto-inserted from cpplint hints
- ❌ W291: Trailing whitespace → ✅ Removed

#### CMake

- ❌ Trailing whitespace → ✅ Removed

#### General

- ❌ No newline at end of file → ✅ Added

### ⚠️ Manual Fixes Required

Some issues still require human attention:

- Complex C++ include dependencies
- Missing `explicit` keywords for single-argument constructors
- TODO comments without author/username
- Complex docstring content and structure
- Header guard naming edge cases
- Architecture-specific code issues

---

## Recommended Workflow

### For Regular Development

```bash
# 1. Make your code changes
# ... edit files ...

# 2. Run automatic linting fixes
./scripts/docker_lint_runner.sh

# 3. Review the changes
git diff

# 4. Test that code still builds
docker compose run --rm development bash -c "cd packages && colcon build --symlink-install"

# 5. Run tests
docker compose run --rm development bash -c "cd packages && colcon test"
# Or use the test profile for build + test in one command
docker compose --profile test up test

# 6. Check for remaining linting issues
docker compose run --rm development bash -c "cd packages && \
  ament_cpplint src/ && \
  ament_pep257 src/ && \
  ament_copyright src/"

# 7. Fix any remaining issues manually

# 8. Commit your changes
git add .
git commit -m "feat: your feature description

- Applied automatic linting fixes
- Resolved remaining style issues"
```

### Before Creating a Pull Request

```bash
# Run full linting check
./scripts/docker_lint_runner.sh

# Build and test in one command (recommended)
docker compose --profile test up test

# OR do it step by step:
# Verify build
docker compose run --rm development bash -c "cd packages && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"

# Run all tests
docker compose run --rm development bash -c "cd packages && colcon test"

# Check test results
docker compose run --rm development bash -c "cd packages && colcon test-result --verbose"

# Review all changes
git diff

# Commit and push
git add .
git commit -m "fix: apply linting and formatting fixes"
git push
```

### CI/CD Integration

The scripts are designed to work in CI/CD pipelines:

```yaml
# Example GitHub Actions workflow step
- name: Run linting fixes
  run: |
    ./scripts/fix_all_linting.sh
    
- name: Check for changes
  run: |
    if ! git diff --exit-code; then
      echo "Linting changes detected!"
      exit 1
    fi
```

---

## Additional Resources

### Configuration Files

- `uncrustify.cfg` - Uncrustify configuration for C++ formatting
- `.clang-format` (if exists) - Clang-format configuration
- Project uses `black` profile for Python (99 char line length)

### Troubleshooting

**Issue: Tools not found**
```bash
# Use Docker to ensure all tools are available
./scripts/docker_lint_runner.sh
```

**Issue: Permission denied**
```bash
# Make scripts executable
chmod +x scripts/*.sh
```

**Issue: Docker image build fails**
```bash
# Rebuild from scratch
./scripts/docker_lint_runner.sh --rebuild
```

**Issue: Changes aren't applied**
```bash
# Ensure you're running from workspace root
cd /path/to/AuNa
./scripts/docker_lint_runner.sh
```

### Getting Help

For issues or questions:
1. Check this README
2. Review the script output for specific error messages
3. Check GitHub Actions CI logs for examples
4. Open an issue on the repository

## Configuration

The scripts use these configuration files:
- **Uncrustify:** ROS2 standard config (`/opt/ros/humble/share/ament_uncrustify/configuration/ament_code_style.cfg`)
- **Python:** PEP8 with 99-character line limit
- **Flake8:** Standard configuration with ROS2 exclusions

## Troubleshooting

If you encounter issues:

1. **Permission errors:** Make sure scripts are executable:
   ```bash
   chmod +x scripts/*.sh
   ```

2. **Docker issues:** Ensure Docker and Docker Compose are installed and running

3. **Build errors after linting:** Some automatic fixes might introduce issues that need manual correction

4. **Remaining lint errors:** Run the scripts multiple times, as some fixes enable detection of other issues

## Integration with CI/CD

These scripts are designed to fix the same issues detected by the GitHub Actions CI pipeline. Running them should significantly reduce or eliminate CI linting failures.