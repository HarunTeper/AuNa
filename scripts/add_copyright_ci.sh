#!/usr/bin/env bash
set -e
IFS=$'\n\t'

# Non-interactive script to add copyright headers using ament_copyright
# This version is suitable for CI/CD pipelines
# Usage: ./scripts/add_copyright_ci.sh [AUTHOR_NAME] [LICENSE]
#
# Examples:
#   ./scripts/add_copyright_ci.sh "Harun Teper" "MIT"
#   ./scripts/add_copyright_ci.sh  # Uses defaults

WORKSPACE="${WORKSPACE:-$(pwd)}"
AUTHOR="${1:-Harun Teper}"
LICENSE="${2:-MIT}"

echo "Adding Copyright Headers (Non-Interactive Mode)"
echo "Workspace: $WORKSPACE"
echo "Author: $AUTHOR"
echo "License: $LICENSE"

cd "$WORKSPACE"

# Check if ament_copyright is available
if ! command -v ament_copyright &> /dev/null; then
  echo "ERROR: ament_copyright not found!"
  echo "Installing ament_copyright..."
  
  # Try to install if running as root or with sudo
  if [ "$EUID" -eq 0 ] || command -v sudo &> /dev/null; then
    if [ "$EUID" -eq 0 ]; then
      apt-get update -qq
      apt-get install -y -qq python3-ament-copyright
    else
      sudo apt-get update -qq
      sudo apt-get install -y -qq python3-ament-copyright
    fi
  else
    echo "ERROR: Cannot install ament_copyright. Please install manually:"
    echo "  apt-get update && apt-get install -y python3-ament-copyright"
    exit 1
  fi
fi

# Run ament_copyright to add missing headers
cd packages/src
ament_copyright --add-missing "$AUTHOR" "$LICENSE"

echo "Copyright header addition complete!"

