# Linting Scripts

This directory contains scripts for automatically fixing code style and linting issues in the AuNa ROS2 project.

## Available Scripts

### 1. `docker_lint_runner.sh` (Recommended)
**Convenience script to run linting using Docker Compose**

```bash
./scripts/docker_lint_runner.sh
```

This script:
- Builds the development Docker image with all linting tools
- Runs the comprehensive linting fixes
- Provides commands for additional operations

### 2. `fix_all_linting.sh`
**Comprehensive linting fix script (runs inside Docker container)**

Fixes all types of linting issues found in GitHub Actions CI:
- Python formatting (autopep8, autoflake)
- C++ formatting (uncrustify)
- CMake whitespace issues
- Missing file endings
- Basic docstring issues
- XML formatting

### 3. `enhanced_automatic_lint.sh`
**Enhanced version of the original automatic_lint.txt (runs inside Docker container)**

A cleaner, more robust version of the original automatic linting script with:
- Better error handling
- ROS2-specific configurations
- Additional fixes for common issues

## Docker Compose Integration

The project now includes a `lint-fix` service in `docker-compose.yml`:

### Build and Run Lint Fixes
```bash
# Build the development image with linting tools
docker compose build development

# Run comprehensive linting fixes
docker compose --profile lint-fix up lint-fix

# Alternative: Run enhanced automatic lint
docker compose run --rm development /home/ubuntu/workspace/scripts/enhanced_automatic_lint.sh
```

### Development Environment
```bash
# Start development environment
docker compose --profile development up -d development

# Enter the container for manual fixes
docker compose exec development bash

# Inside container, run scripts manually:
cd /home/ubuntu/workspace
./scripts/fix_all_linting.sh
# or
./scripts/enhanced_automatic_lint.sh
```

## What Gets Fixed

### Automatic Fixes
- **Python Issues:**
  - Line length violations (E501)
  - PEP8 formatting issues
  - Unused imports
  - Basic docstring formatting
  
- **C++ Issues:**
  - Code formatting via uncrustify
  - Missing newlines at end of files
  - Basic header guard formatting
  
- **CMake Issues:**
  - Trailing whitespace
  
- **General:**
  - File ending newlines
  - XML formatting

### Manual Fixes Still Required
Some issues require manual attention:
- Complex C++ include order issues
- Missing `explicit` keywords for constructors
- Missing `#include` statements
- TODO comments without usernames
- Complex docstring formatting
- Header guard naming conventions

## Workflow

1. **Run automatic fixes:**
   ```bash
   ./scripts/docker_lint_runner.sh
   ```

2. **Check results:**
   ```bash
   git diff  # Review changes
   ```

3. **Test the build:**
   ```bash
   docker compose run --rm development bash -c "cd packages && colcon build --symlink-install"
   ```

4. **Run tests to check for remaining issues:**
   ```bash
   docker compose run --rm development bash -c "cd packages && colcon test"
   ```

5. **Fix remaining issues manually** (if any)

6. **Commit changes:**
   ```bash
   git add .
   git commit -m "fix: apply automatic linting fixes"
   ```

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