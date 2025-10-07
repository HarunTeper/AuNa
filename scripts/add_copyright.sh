#!/usr/bin/env bash
set -e
IFS=$'\n\t'

# Script to add MIT License copyright headers to ROS2 package source files
# Uses ament_copyright tool from ROS2
# Usage: ./scripts/add_copyright.sh [AUTHOR_NAME] [LICENSE]
#
# Examples:
#   ./scripts/add_copyright.sh "Harun Teper" "MIT"
#   ./scripts/add_copyright.sh  # Uses defaults

WORKSPACE="${WORKSPACE:-$(pwd)}"
AUTHOR="${1:-Harun Teper}"
LICENSE="${2:-mit}"

echo "=========================================="
echo "Adding Copyright Headers to ROS2 Packages"
echo "=========================================="
echo "Workspace: $WORKSPACE"
echo "Author: $AUTHOR"
echo "License: $LICENSE"
echo "=========================================="

cd "$WORKSPACE"

# Check if ament_copyright is available
if ! command -v ament_copyright &> /dev/null; then
  echo "ERROR: ament_copyright not found!"
  echo
  echo "Please install it with:"
  echo "  sudo apt-get update"
  echo "  sudo apt-get install python3-ament-copyright"
  echo
  echo "Or if running in Docker:"
  echo "  docker compose exec auna bash"
  echo "  apt-get update && apt-get install -y python3-ament-copyright"
  exit 1
fi

echo
echo "Adding copyright headers to packages/src..."
echo

# Run ament_copyright to add missing headers
cd packages/src
ament_copyright --add-missing "$AUTHOR" "$LICENSE"

echo
echo "=========================================="
echo "Copyright header addition complete!"
echo "=========================================="
echo
echo "Summary:"
echo "  - Used ament_copyright to add missing headers"
echo "  - Author: $AUTHOR"
echo "  - License: $LICENSE"
echo
echo "Next steps:"
echo "  1. Review changes: git diff"
echo "  2. Verify copyright headers: ament_copyright packages/src"
echo "  3. Build and test: cd packages && colcon build"
echo "  4. Commit changes: git add -A && git commit -m 'Add copyright headers'"
echo
