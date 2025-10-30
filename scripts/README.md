# Development Scripts

This directory contains scripts for code quality, linting, and maintenance of the AuNa ROS2 project.

## Table of Contents

- [Quick Start](#quick-start)
- [Combined Quality Checks](#combined-quality-checks)
- [Copyright Management](#copyright-management)
- [Linting and Code Quality](#linting-and-code-quality)
- [What Gets Fixed](#what-gets-fixed)
- [Recommended Workflow](#recommended-workflow)

---

## Quick Start

### ⭐ All-in-One Quality Checks (Recommended)

Run copyright headers, linting, and tests in one command:

```bash
# Run all quality checks with default settings
./scripts/run_quality_checks.sh

# Specify custom author and license
./scripts/run_quality_checks.sh "Your Name" "MIT"
```

This will:
1. ✅ Add missing copyright headers
2. ✅ Apply all linting fixes
3. ✅ Build and run all tests
4. ✅ Provide a summary of changes

**This is the recommended approach before committing or creating a pull request.**

---

## Combined Quality Checks

### `run_quality_checks.sh` ⭐ Recommended

All-in-one script that runs the complete quality check pipeline: copyright headers → linting → tests.

**Usage:**

```bash
# Use default author and license
./scripts/run_quality_checks.sh

# Specify custom author and license
./scripts/run_quality_checks.sh "Your Name" "MIT"
```

**What it does:**

1. **Step 1: Copyright Headers** - Adds missing MIT License headers using `ament_copyright`
2. **Step 2: Linting** - Fixes Python (PEP8, imports, formatting) and C++ (uncrustify, includes) issues
3. **Step 3: Tests** - Builds all packages and runs the complete test suite

**Features:**

- ✅ Runs everything in the correct order
- ✅ Uses Docker for consistent environment
- ✅ Stops on first failure with clear error messages
- ✅ Provides detailed summary at the end
- ✅ Same tools and checks as CI/CD pipeline

**When to use:**

- Before committing changes
- Before creating a pull request
- After making significant code changes
- When CI/CD checks are failing

---

## Copyright Management

### `add_copyright.sh`

Add MIT License copyright headers to all ROS2 package files using the official `ament_copyright` tool.

**Usage:**

```bash
# Use default author and license
./scripts/add_copyright.sh

# Specify custom author and license
./scripts/add_copyright.sh "Your Name" "mit"
```

**Features:**

- Uses official ROS2 `ament_copyright` tool
- Adds copyright headers to C++, Python, and launch files
- Skips files that already have copyright headers
- Automatically installs `ament_copyright` if missing (interactive mode)
- Simple and reliable using standard ROS2 tooling

**Requirements:**

- `python3-ament-copyright` package (auto-installed if missing)

---

## Linting and Code Quality

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

## Individual Tool Usage

### Option 1: Run Complete Quality Checks (Recommended) �

```bash
# One command for copyright + linting + tests
./scripts/run_quality_checks.sh

# Review changes
git diff

# Commit if satisfied
git add .
git commit -m "chore: apply quality checks and fix linting"
```

### Option 2: Run Linting Only

```bash
# Run linting directly in Docker container
docker compose run --rm development bash -c "/home/ubuntu/workspace/scripts/fix_all_linting.sh"

# OR use the Docker Compose profile
docker compose --profile linting up linting
```

### Option 3: Manual Execution

```bash
# Run locally (requires tools installed)
./scripts/fix_all_linting.sh

# Or run in Docker container manually
docker compose run --rm development bash
cd /home/ubuntu/workspace
./scripts/fix_all_linting.sh
```

### Option 4: Using Docker Compose Profiles

```bash
# Build development image
docker compose build development

# Run linting fixes via docker-compose profile
docker compose --profile linting up linting

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

# 2. Run complete quality checks (copyright + linting + tests)
./scripts/run_quality_checks.sh

# 3. Review the changes
git diff

# 4. Fix any remaining issues manually (if needed)

# 5. Commit your changes
git add .
git commit -m "feat: your feature description

- Applied quality checks
- Fixed linting issues"
```

**Alternative: Step-by-step approach**

If you prefer to run steps individually:

```bash
# 1. Make your code changes
# ... edit files ...

# 2. Add copyright headers
./scripts/add_copyright.sh

# 3. Run linting fixes
docker compose run --rm development bash -c "/home/ubuntu/workspace/scripts/fix_all_linting.sh"

# 4. Run tests
docker compose --profile test up test

# 5. Review and commit
git diff
git add .
git commit -m "chore: apply quality checks"
```

### Before Creating a Pull Request

```bash
# Run complete quality checks (all-in-one)
./scripts/run_quality_checks.sh

# Review all changes
git diff

# Commit and push
git add .
git commit -m "chore: apply quality checks and fix linting"
git push
```

**For more detailed control:**

```bash
# Run individual steps with verification
./scripts/add_copyright.sh
docker compose run --rm development bash -c "/home/ubuntu/workspace/scripts/fix_all_linting.sh"
docker compose --profile test up test

# Check test results in detail
docker compose run --rm development bash -c "cd packages && colcon test-result --verbose"

# Review and push
git diff
git add .
git commit -m "chore: apply quality checks"
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

**Issue: Quality checks fail**
```bash
# Run steps individually to identify the problem
./scripts/add_copyright.sh
docker compose --profile linting up linting
docker compose --profile test up test
```

**Issue: Tools not found**
```bash
**Issue: Tools not found**

```bash
# Use Docker to ensure all tools are available
docker compose run --rm development bash -c "/home/ubuntu/workspace/scripts/fix_all_linting.sh"
```
```

**Issue: Permission denied**
```bash
# Make scripts executable
chmod +x scripts/*.sh
```

**Issue: Docker image build fails**
```bash
# Rebuild from scratch
docker compose build --no-cache development
```

**Issue: Changes aren't applied**
```bash
# Ensure you're running from workspace root
cd /path/to/AuNa
./scripts/run_quality_checks.sh
```

**Issue: Tests fail after linting**
```bash
# Check detailed test results
docker compose run --rm development bash -c "cd packages && colcon test-result --verbose"

# Review what changed
git diff

# You may need to manually fix some issues
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