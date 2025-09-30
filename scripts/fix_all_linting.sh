#!/bin/bash

# Comprehensive ROS2 Linting Fix Script
# This script fixes all linting issues found in the GitHub Actions CI

set -e  # Exit on any error

echo "=========================================="
echo "  ROS2 Comprehensive Linting Fix Script  "
echo "=========================================="

# Navigate to workspace root
cd /home/ubuntu/workspace

# Ensure we have the correct ROS2 environment
source /opt/ros/humble/setup.bash

echo ""
echo "Installing/updating required tools..."
# Install tools with compatible versions to avoid conflicts
pip3 install --upgrade autopep8 autoflake pydocstyle 2>/dev/null || true

# Try to fix flake8/pycodestyle compatibility issues
pip3 install --upgrade "flake8>=4.0.0" "pycodestyle>=2.8.0" 2>/dev/null || true

echo ""
echo "1. Fixing Python formatting and style issues..."
echo "=============================================="

# Find all Python files first
PYTHON_FILES=$(find packages/src -name "*.py" -type f)

if [ -n "$PYTHON_FILES" ]; then
    echo "Running autopep8 on Python files..."
    for file in $PYTHON_FILES; do
        if command -v autopep8 &> /dev/null; then
            autopep8 --in-place --max-line-length=99 --aggressive --aggressive "$file"
        elif python3 -c "import autopep8" 2>/dev/null; then
            python3 -m autopep8 --in-place --max-line-length=99 --aggressive --aggressive "$file"
        else
            echo "Warning: autopep8 not available, skipping $file"
        fi
    done
else
    echo "No Python files found in packages/src"
fi

if [ -n "$PYTHON_FILES" ]; then
    echo "Running autoflake on Python files..."
    for file in $PYTHON_FILES; do
        if command -v autoflake &> /dev/null; then
            autoflake --in-place --remove-all-unused-imports --remove-unused-variables "$file"
        elif python3 -c "import autoflake" 2>/dev/null; then
            python3 -m autoflake --in-place --remove-all-unused-imports --remove-unused-variables "$file"
        else
            echo "Warning: autoflake not available, skipping unused imports for $file"
        fi
    done
fi

echo ""
echo "2. Fixing C++ formatting issues..."
echo "=================================="

# Find all C++ files
CPP_FILES=$(find packages/src -name "*.cpp" -o -name "*.hpp" -type f)

if [ -n "$CPP_FILES" ]; then
    # Get the ROS2 uncrustify configuration
    UNCRUSTIFY_CONFIG="/opt/ros/humble/share/ament_uncrustify/configuration/ament_code_style.cfg"
    
    # If ROS2 config doesn't exist, use our local one
    if [ ! -f "$UNCRUSTIFY_CONFIG" ]; then
        UNCRUSTIFY_CONFIG="/home/ubuntu/workspace/scripts/uncrustify.cfg"
    fi
    
    if [ -f "$UNCRUSTIFY_CONFIG" ]; then
        echo "Running uncrustify with config: $UNCRUSTIFY_CONFIG"
        for file in $CPP_FILES; do
            if command -v uncrustify &> /dev/null; then
                uncrustify -c "$UNCRUSTIFY_CONFIG" --replace --no-backup "$file"
            else
                echo "Warning: uncrustify not available, skipping $file"
            fi
        done
    else
        echo "Warning: No uncrustify config found, skipping C++ formatting"
    fi
else
    echo "No C++ files found in packages/src"
fi

echo ""
echo "3. Fixing CMake formatting issues..."
echo "===================================="

# Find all CMakeLists.txt files
CMAKE_FILES=$(find packages/src -name "CMakeLists.txt" -type f)

if [ -n "$CMAKE_FILES" ]; then
    echo "Removing trailing whitespace from CMakeLists.txt files..."
    for file in $CMAKE_FILES; do
        sed -i 's/[[:space:]]*$//' "$file"
        echo "Fixed: $file"
    done
else
    echo "No CMakeLists.txt files found"
fi

echo ""
echo "4. Fixing file ending issues..."
echo "==============================="

# Add missing newlines at end of files
ALL_SOURCE_FILES=$(find packages/src -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.py" \))

if [ -n "$ALL_SOURCE_FILES" ]; then
    echo "Adding missing newlines at end of files..."
    for file in $ALL_SOURCE_FILES; do
        if [ -s "$file" ] && [ "$(tail -c1 "$file" | wc -l)" -eq 0 ]; then
            echo "" >> "$file"
            echo "Added newline to: $file"
        fi
    done
else
    echo "No source files found"
fi

echo ""
echo "5. Final validation..."
echo "====================="

# Run a quick validation to see remaining issues
echo "Checking for basic Python syntax issues..."

# Use a simple Python syntax check instead of flake8 to avoid version conflicts
PYTHON_FILES=$(find packages/src -name "*.py" -type f)
if [ -n "$PYTHON_FILES" ]; then
    SYNTAX_ERRORS=0
    for file in $PYTHON_FILES; do
        if ! python3 -m py_compile "$file" 2>/dev/null; then
            echo "Syntax error in: $file"
            SYNTAX_ERRORS=$((SYNTAX_ERRORS + 1))
        fi
    done
    
    if [ $SYNTAX_ERRORS -eq 0 ]; then
        echo "All Python files have valid syntax ✓"
    else
        echo "Found $SYNTAX_ERRORS Python files with syntax errors"
    fi
    
    # Try flake8 with error handling
    echo "Running flake8 validation (if available)..."
    if command -v flake8 &> /dev/null; then
        flake8 --max-line-length=99 --exclude=build,install packages/src 2>/dev/null || echo "flake8 found style issues or encountered errors (may need manual fixing)"
    elif python3 -c "import flake8" 2>/dev/null; then
        python3 -m flake8 --max-line-length=99 --exclude=build,install packages/src 2>/dev/null || echo "flake8 found style issues or encountered errors (may need manual fixing)"
    else
        echo "flake8 not available, skipping detailed style validation"
    fi
else
    echo "No Python files found for validation"
fi

echo ""
echo "=========================================="
echo "  Linting fixes completed!                "
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Review the changes with 'git diff'"
echo "2. Test the build with 'colcon build'"
echo "3. Run 'colcon test' to check for remaining issues"
echo ""