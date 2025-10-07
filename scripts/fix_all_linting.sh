#!/usr/bin/env bash
set -e pipefail
IFS=$'\n\t'

# Minimal ROS2 lint/fix script
# Uses only the essential tools to pass colcon test:
# - autopep8: fixes PEP8 issues (E501 line length, trailing whitespace, etc.)
# - uncrustify: C++ formatting (required by ament_uncrustify)
#
# Usage: run from workspace root
#   ./scripts/fix_all_linting.sh

WORKSPACE="${WORKSPACE:-$(pwd)}"
echo "Workspace: $WORKSPACE"
cd "$WORKSPACE"

echo "Sourcing ROS (if present)..."
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash || true
fi

echo "Installing / updating minimal Python tools..."
if command -v pip3 >/dev/null 2>&1; then
  pip3 install --upgrade autopep8 >/dev/null || true
else
  echo "pip3 not found: skip pip installs (CI should provide these tools)."
fi

# Try apt installs for system tools if available (non-fatal)
install_apt_if_missing() {
  cmd="$1"; pkg="$2"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    if command -v apt-get >/dev/null 2>&1; then
      echo "Installing $pkg via apt-get..."
      sudo apt-get update -qq
      sudo apt-get install -y -qq "$pkg" || true
    fi
  fi
}

install_apt_if_missing uncrustify uncrustify

echo
echo "1) Python: autopep8 (fixes E501 line-too-long, trailing whitespace, spacing, etc.)"
PY_FILES=$(find packages/src -name "*.py" -type f || true)
if [ -n "$PY_FILES" ]; then
  if command -v autopep8 >/dev/null 2>&1; then
    echo "Running autopep8 with --max-line-length 99..."
    for f in $PY_FILES; do
      # Use --aggressive to fix more issues, especially line length
      autopep8 --in-place --aggressive --aggressive --max-line-length 99 "$f" || true
    done
  else
    echo "autopep8 not found, skipping Python fixes"
  fi
else
  echo "No Python files found"
fi

echo
echo "2) C++: uncrustify (required by ament_uncrustify)"
CPP_FILES=$(find packages/src -name "*.cpp" -o -name "*.hpp" -o -name "*.cxx" -o -name "*.hh" -type f || true)
if [ -n "$CPP_FILES" ]; then
  UNCRUSTIFY_CONFIG="$WORKSPACE/scripts/uncrustify.cfg"
  if [ -f "$UNCRUSTIFY_CONFIG" ] && command -v uncrustify >/dev/null 2>&1; then
    echo "Running uncrustify with config: $UNCRUSTIFY_CONFIG"
    for f in $CPP_FILES; do
      uncrustify -c "$UNCRUSTIFY_CONFIG" --no-backup --replace "$f" || true
    done
  else
    echo "uncrustify config missing or uncrustify not installed; skipping C++ formatting"
  fi
else
  echo "No C/C++ files found"
fi

echo
echo "3) CMake/CMakeLists: trailing whitespace removal and indentation fix"
CMAKE_FILES=$(find packages/src -name "CMakeLists.txt" -type f || true)
if [ -n "$CMAKE_FILES" ]; then
  for f in $CMAKE_FILES; do
    # Remove trailing whitespace
    sed -i 's/[[:space:]]*$//' "$f" || true
    
    # Fix CMake indentation: normalize continuation lines to 2 spaces
    # This handles lines that start with excessive spaces (common in multi-line commands)
    # Match lines with leading spaces (4+ spaces) and normalize to 2 spaces
    sed -i -E 's/^[[:space:]]{4,}/  /' "$f" || true
  done
  echo "Fixed trailing whitespace and indentation in CMakeLists.txt files"
fi

echo
echo "4) Ensure files end with newline"
ALL_SOURCE_FILES=$(find packages/src -type f \( -name "*.py" -o -name "*.cpp" -o -name "*.hpp" -o -name "CMakeLists.txt" \) || true)
if [ -n "$ALL_SOURCE_FILES" ]; then
  for f in $ALL_SOURCE_FILES; do
    if [ -s "$f" ] && [ "$(tail -c1 "$f" | wc -l)" -eq 0 ]; then
      echo >> "$f"
    fi
  done
  echo "Ensured files end with newline"
fi

echo
echo "5) Final validation: remaining issues that need manual review"
echo "Running ROS2 ament linters (matching colcon test)..."

# Run ament linters to show remaining issues (non-fatal)
if command -v ament_flake8 >/dev/null 2>&1; then
  echo ""
  echo "Running ament_flake8..."
  ament_flake8 --linelength 99 packages/src || echo "  ⚠️  Found flake8 issues (see above)"
else
  echo "ament_flake8 not available, skipping validation"
fi

if command -v ament_pep257 >/dev/null 2>&1; then
  echo ""
  echo "Running ament_pep257..."
  ament_pep257 packages/src || echo "  ⚠️  Found pep257 issues (see above)"
else
  echo "ament_pep257 not available, skipping validation"
fi

if command -v ament_copyright >/dev/null 2>&1; then
  echo ""
  echo "Running ament_copyright..."
  ament_copyright packages/src || echo "  ⚠️  Found copyright issues (see above)"
else
  echo "ament_copyright not available, skipping validation"
fi

if command -v ament_cpplint >/dev/null 2>&1; then
  echo ""
  echo "Running ament_cpplint..."
  ament_cpplint packages/src || echo "  ⚠️  Found cpplint issues (see above)"
else
  echo "ament_cpplint not available, skipping validation"
fi

if command -v ament_uncrustify >/dev/null 2>&1; then
  echo ""
  echo "Running ament_uncrustify..."
  ament_uncrustify packages/src || echo "  ⚠️  Found uncrustify issues (see above)"
else
  echo "ament_uncrustify not available, skipping validation"
fi

if command -v ament_lint_cmake >/dev/null 2>&1; then
  echo ""
  echo "Running ament_lint_cmake..."
  ament_lint_cmake packages/src || echo "  ⚠️  Found CMake lint issues (see above)"
else
  echo "ament_lint_cmake not available, skipping validation"
fi

echo ""
echo "Done! To verify all tests pass, run:"
echo " - cd packages && colcon test"
echo " - cd packages && colcon test-result --verbose"
